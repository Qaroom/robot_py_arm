#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
from sensor_msgs.msg import JointState


class RobotInverseKinematics(Node):
    def __init__(self):
        super().__init__("robot_inverse_kinematics")
        self.angle_value_publisher=self.create_publisher(Float64MultiArray,"target_angle_position",10)
        self.targets_no=self.create_subscription(Float64MultiArray,'target_position',self.target_callback,10)
        self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )

        self.get_logger().info("robot_inverse_kinematics node has been started")

    
        self.L1=331.24
        self.L2=300.0

        self.end_effector_x = 0.0
        self.end_effector_y = 0.0
        
        # Inverse kinematics için hedef pozisyonlar
        self.target_x = 0.0
        self.target_y = 0.0



    def target_callback(self, msg):
        """Hedef pozisyonları al"""

        self.target_x=msg.data[0]
        self.target_y=msg.data[1]


    def joint_state_callback(self, msg):
        """Mevcut joint pozisyonlarını al"""
        joint_positions = {}
        L1 = 331.24
        L2 = 300.0

        for i, name in enumerate(msg.name):
            joint_positions[name] = msg.position[i]
        
        # Joint açılarını al
        self.current_1 = math.degrees(joint_positions.get("First_axis_joint", 0.0)) 
        self.current_2 = math.degrees(joint_positions.get("Second_axis_joint", 0.0)) 
        self.current_3 = math.degrees(joint_positions.get("Third_axis_joint", 0.0))
        self.current_4 = joint_positions.get("left_gripper_hand_joint", 0.0)
        self.current_5 = joint_positions.get("right_gripper_hand_joint", 0.0)

        # Forward Kinematics - Mevcut end-effector pozisyonu
        q1_rad = math.radians(self.current_1)
        q2_rad = math.radians(self.current_2)

        x_current = L1 * math.cos(q1_rad) + L2 * math.cos(q1_rad + q2_rad)  
        y_current = L1 * math.sin(q1_rad) + L2 * math.sin(q1_rad + q2_rad) 
        
        

        # Inverse Kinematics - Hedef açıları hesapla
        x_d = self.target_x  # Hedef X (başka fonksiyondan geliyor)
        y_d = -1*self.target_y  # Hedef Y (başka fonksiyondan geliyor)
        
        # 1. r² hesapla
        r_squared = x_d**2 + y_d**2
        r = math.sqrt(r_squared)
        
        # 2. D değişkeni
        D = (r_squared - L1**2 - L2**2) / (2 * L1 * L2)
        
        # 3. Erişim kontrolü
        if abs(D) > 1:
            self.get_logger().warn(f"⚠️ OUT_OF_REACH: Target({x_d:.1f}, {y_d:.1f}), |D|={abs(D):.3f}")
            return
        
        # 4. İki çözümü hesapla
        # Elbow-up
        q2_up = math.atan2(+math.sqrt(1 - D**2), D)
        q1_up = math.atan2(y_d, x_d) - math.atan2(L2 * math.sin(q2_up), L1 + L2 * math.cos(q2_up))
        
        # Elbow-down
        q2_down = math.atan2(-math.sqrt(1 - D**2), D)
        q1_down = math.atan2(y_d, x_d) - math.atan2(L2 * math.sin(q2_down), L1 + L2 * math.cos(q2_down))
        
        # 5. Mevcut açıya en yakın olanı seç
        dist_up = math.sqrt((q1_up - q1_rad)**2 + (q2_up - q2_rad)**2)
        dist_down = math.sqrt((q1_down - q1_rad)**2 + (q2_down - q2_rad)**2)
        
        if dist_up < dist_down:
            q1_target = q1_up
            q2_target = q2_up
            config = "elbow-up"
        else:
            q1_target = q1_down
            q2_target = q2_down
            config = "elbow-down"
        
        # 6. Hedef açıları dereceye çevir ve kaydet
        q3_target=math.atan2(y_d, x_d+39.0)-q1_target-q2_target+1.5709

        x_current = L1 * math.cos(q1_rad) + L2 * math.cos(q1_rad + q2_rad)  
        y_current = L1 * math.sin(q1_rad) + L2 * math.sin(q1_rad + q2_rad) 
        
        self.end_effector_x = x_current
        self.end_effector_y = y_current

        # self.get_logger().info(f"robot x {self.end_effector_x} robot_y {self.end_effector_y}")
        # self.target_1 = math.degrees(q1_target)
        # self.target_2 = math.degrees(q2_target)


        angle_msg = Float64MultiArray()
        angle_msg.data = [q1_target,q2_target,q3_target,0.0,0.0]
        self.angle_value_publisher.publish(angle_msg)
    



def main():
    rclpy.init()
    node = RobotInverseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
