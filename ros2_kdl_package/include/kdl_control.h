#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"

class KDLController
{

public:

    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo);
                           
    //velocity_ctrl_null è il nome della funzione che stiamo andando ad implementare alla quale passiamo il vettore di posizione desiderata per calcolare poi ep, lambda e il guadagno kp.
    Eigen::VectorXd velocity_ctrl_null(const Eigen::Vector3d& p_des,    
                                   double Kp,
                                   double lambda);
                           
    Eigen::VectorXd vision_ctrl(KDL::Frame pose_in_camera_frame, 
                                KDL::Frame camera_frame,
                                KDL::Jacobian camera_jacobian,
                                Eigen::VectorXd q0_dot);

private:

    KDLRobot* robot_;

};

#endif
