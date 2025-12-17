#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- SOTTOSCRIZIONE ---
        
        self.subscription = self.create_subscription(
            Image,
            '/iiwa/stereo/left/image_rect_color', 
            self.image_callback,
            qos_profile) 
        
        # Publisher verso il "Brain" del robot
        self.publisher_ = self.create_publisher(String, '/iiwa/detected_color', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("Nodo Color Detector avviato.")

    def image_callback(self, msg):
        try:
            # Conversione ROS -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Errore conversione: {e}")
            return

        # Conversione in HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])

        # Creazione maschere
        mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), cv2.inRange(hsv, lower_red2, upper_red2))
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        detected_color = "none"
        
        # --- SOGLIA AREA MINIMA ---
        # Impostata a 50 pixel 
        MIN_AREA = 50 

        # 1. Controllo ROSSO
        contours_r, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        found_red = False
        for contour in contours_r:
            if cv2.contourArea(contour) > MIN_AREA:
                found_red = True
                detected_color = "red"
                # Disegna contorno rosso (spessore 2)
                cv2.drawContours(cv_image, [contour], -1, (0, 0, 255), 2)
                
                # Disegna pallino al centro
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(cv_image, (cX, cY), 5, (255, 255, 255), -1)
                break 

        # 2. Controllo VERDE 
        if not found_red:
            contours_g, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours_g:
                if cv2.contourArea(contour) > MIN_AREA:
                    detected_color = "green"
                    
                    cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
                    
                    
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        cv2.circle(cv_image, (cX, cY), 5, (255, 255, 255), -1)
                    break

        # Pubblicazione
        msg_out = String()
        msg_out.data = detected_color
        self.publisher_.publish(msg_out)

        # Visualizzazione a schermo
        cv2.imshow("IIWA Camera View", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
