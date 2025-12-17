#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{

}

Eigen::VectorXd KDLController::velocity_ctrl_null(const Eigen::Vector3d& p_des,
                                                  double Kp,
                                                  double lambda)
{
    //Stato attuale dal robot
    KDL::Frame F_e   = robot_->getEEFrame();             // pose end-effector (spatial)
    Eigen::Vector3d p_e = toEigen(F_e.p);                // posizione attuale
    Eigen::Vector3d e_p = p_des - p_e;                   // errore di posizione

    //Jacobiano posizionale (prime 3 righe del 6xN)
    Eigen::MatrixXd J  = robot_->getEEJacobian().data;   // 6xN
    J = J.topRows(3);                                    // 3xN

    //Pseudoinversa
    Eigen::MatrixXd Jpinv = pseudoinverse(J);

    //Task primario: IK differenziale su posizione
    Eigen::VectorXd qdot_task = Jpinv * (Kp * e_p);      // N×1

    // Null-space: gradiente repulsivo dai limiti
    double cost_val = 0.0;
    Eigen::MatrixXd jntLim = robot_->getJntLimits();     // N×2 (min, max)
    Eigen::VectorXd q      = robot_->getJntValues();     // N×1
    Eigen::VectorXd grad   = gradientJointLimits(q, jntLim, cost_val); // N×1

    Eigen::VectorXd qdot0  = -(1.0 / lambda) * grad;      // N×1

    // Proiettore nel null-space
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(J.cols(), J.cols()); 
    Eigen::MatrixXd Nproj = I - Jpinv * J;                             

    // Comando finale
    Eigen::VectorXd qdot = qdot_task + Nproj * qdot0;    // N×1
    return qdot;
}

/*This controller calculates the joint velocities required to make the camera's  Z-axis (optical axis) point directly at the ArUco marker.
 @param pose_in_camera_frame The pose of the ArUco marker relative to the camera (T_cam_aruco).
 @param camera_frame The current pose of the camera in the base frame (T_base_cam).
 @param camera_jacobian The camera's geometric Jacobian in the base frame (J_base_cam).
 @param q0_dot The desired joint velocities for the null-space task (e.g., joint limit avoidance).
 */
Eigen::VectorXd KDLController::vision_ctrl(KDL::Frame pose_in_camera_frame, KDL::Frame camera_frame,
                                        KDL::Jacobian camera_jacobian, Eigen::VectorXd q0_dot)
{
    // Convert the camera rotation to Eigen and build the 6x6 spatial rotation matrix
    Matrix6d R = spatialRotation(camera_frame.M);

    // Compute the direction vector s
    Eigen::Vector3d c_P_o = toEigen(pose_in_camera_frame.p);
    Eigen::Vector3d s = c_P_o / c_P_o.norm();

    // Interaction matrix L
    Eigen::Matrix<double, 3, 6> L = Eigen::Matrix<double, 3, 6>::Zero();
    Eigen::Matrix3d L_11 = (-1 / c_P_o.norm()) * (Eigen::Matrix3d::Identity() - s * s.transpose());
    L.block<3, 3>(0, 0) = L_11;
    L.block<3, 3>(0, 3) = skew(s);
    L = L * R;

    Eigen::MatrixXd J_c = camera_jacobian.data; // Camera Jacobian in the camera frame
    Eigen::MatrixXd LJ = L * J_c;              // Combined matrix L * J_c
    Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse(); // Moore-Penrose pseudoinverse of L * J_c

    // Compute null-space projector N
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(J_c.cols(), J_c.cols());
    Eigen::MatrixXd N = I - (LJ_pinv * LJ);

    Eigen::Vector3d s_d(0, 0, 1); // Desired unit vector pointing forward
    double k = -2;               // Gain for the task
    Eigen::VectorXd joint_velocities = k * LJ_pinv * s_d + N * q0_dot;

    Eigen::Vector3d s_error = s - s_d;
    std::cout <<"Error norm :  " << s_error.norm() << std::endl;

    // Return computed joint velocities
    return joint_velocities;

}
