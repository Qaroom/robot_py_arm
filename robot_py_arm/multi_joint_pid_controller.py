#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np
import threading
import time

TIME_STEP = 0.0167
MAX_OUTPUT = 7.0
MIN_OUTPUT = -7.0
DEADZONE = 0.001 

# PID parameters for each joint
KP = [4.0, 4.0, 4.0, 2.0, 2.0]
KI = [0.0, 0.0, 0.0, 0.0, 0.0] 
KD = [0.0, 0.0, 0.0, 0.0, 0.0]   

JOINT_NAMES = [
    "First_axis_joint", 
    "Second_axis_joint",
    "Third_axis_joint",
    "left_gripper_hand_joint",
    "right_gripper_hand_joint"
]


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0.0
        self.error_last = 0.0
        self.integral = 0.0
        self.output = 0.0

    def update(self, error, dt):
        # Deadzone uygulaması
        if abs(error) < DEADZONE:
            error = 0.0

        self.error = error
        derivative = (error - self.error_last) / dt
        self.integral += error * dt
        self.integral = np.clip(self.integral, -10, 10)  # anti-windup artırıldı
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.error_last = error
        return np.clip(output, MIN_OUTPUT, MAX_OUTPUT)


class MultiJointPIDNode(Node):
    def __init__(self):
        super().__init__('multi_joint_pid_controller')

        # 5 adet PID controller oluştur
        self.pids = [PID(KP[i], KI[i], KD[i]) for i in range(5)]
        
        # Hedef ve mevcut pozisyonlar
        self.target_positions = [0.0] * 5
        self.current_positions = [0.0] * 5
        self.errors = [0.0] * 5
        self.outputs = [0.0] * 5
        
        # Joint state alındı mı kontrolü
        self.joint_state_received = False

        # Subscribers
        self.create_subscription(
            Float64MultiArray, 
            'target_angle_position', 
            self.target_callback, 
            10
        )
        
        self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )

        # Publisher
        self.pub_velocity = self.create_publisher(
            Float64MultiArray, 
            'forward_velocity_controller/commands', 
            10
        )

        # Grafik verileri
        self.time_data = []
        self.error_data = [[] for _ in range(5)]
        self.output_data = [[] for _ in range(5)]
        self.start_time = time.time()

        # Debug counter
        self.debug_counter = 0

        # Timer
        self.create_timer(TIME_STEP, self.update)
        
        # Grafik thread'i
        threading.Thread(target=self.plot_live, daemon=True).start()
        
        self.get_logger().info("Multi-Joint PID controller started.")
        self.get_logger().info(f"Controlling joints: {JOINT_NAMES}")
        self.get_logger().info(f"KP: {KP}")
        self.get_logger().info(f"KI: {KI}")
        self.get_logger().info(f"KD: {KD}")

    def target_callback(self, msg):
        """Hedef pozisyonları al"""
        if len(msg.data) >= 5:
            self.target_positions = list(msg.data[:5])
            self.get_logger().info(f"🎯 New target: {[f'{x:.3f}' for x in self.target_positions]}")
        else:
            self.get_logger().warn(f"Invalid target position array length: {len(msg.data)}")

    def joint_state_callback(self, msg):
        """Mevcut joint pozisyonlarını al"""
        self.joint_state_received = True
        
        # DEBUG: Joint states mesajını kontrol et
        if self.debug_counter % 100 == 0:
            self.get_logger().info(f"📊 Joint names in message: {msg.name}")
        
        for i, joint_name in enumerate(JOINT_NAMES):
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                self.current_positions[i] = msg.position[idx]
            else:
                if self.debug_counter % 100 == 0:
                    self.get_logger().warn(f"⚠️  Joint '{joint_name}' NOT FOUND in joint_states!")

    def update(self):
        """PID güncellemesi ve hız komutlarını yayınla"""
        
        # Joint state alınmadıysa bekle
        if not self.joint_state_received:
            if self.debug_counter % 50 == 0:
                self.get_logger().warn("Waiting for joint_states...")
            self.debug_counter += 1
            return
        
        # Her joint için hata hesapla ve PID güncelle
        for i in range(5):
            self.errors[i] = self.target_positions[i] - self.current_positions[i]
            self.outputs[i] = self.pids[i].update(self.errors[i], TIME_STEP)

        # DEBUG: Her 100 iterasyonda bir detaylı log
        self.debug_counter += 1
        if self.debug_counter % 100 == 0:
            self.get_logger().info("=" * 80)
            self.get_logger().info("📍 CURRENT STATE:")
            for i in range(5):
                self.get_logger().info(
                    f"  {JOINT_NAMES[i]:25} | "
                    f"Target: {self.target_positions[i]:7.3f} | "
                    f"Current: {self.current_positions[i]:7.3f} | "
                    f"Error: {self.errors[i]:7.3f} | "
                    f"Output: {self.outputs[i]:7.3f}"
                )
            self.get_logger().info("=" * 80)

        # Hız komutlarını yayınla
        vel_msg = Float64MultiArray()
        vel_msg.data = self.outputs.copy()  # Copy kullan
        self.pub_velocity.publish(vel_msg)
        
        # DEBUG: Gönderilen mesajı kontrol et
        if self.debug_counter % 100 == 0:
            self.get_logger().info(f"🚀 Published velocities: {[f'{x:.3f}' for x in vel_msg.data]}")

        # Grafik verileri
        t = time.time() - self.start_time
        self.time_data.append(t)
        for i in range(5):
            self.error_data[i].append(self.errors[i])
            self.output_data[i].append(self.outputs[i])

    def plot_live(self):
        """Canlı grafik çizimi - Node çalıştığı sürece KESINLIKLE devam eder"""
        import matplotlib
        matplotlib.use('TkAgg')
        
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        colors = ['blue', 'red', 'green', 'orange', 'purple']
        
        # Hata grafikleri
        error_lines = []
        for i in range(5):
            line, = ax1.plot([], [], label=f'{JOINT_NAMES[i]}', color=colors[i], linewidth=2)
            error_lines.append(line)
        
        # Output grafikleri
        output_lines = []
        for i in range(5):
            line, = ax2.plot([], [], label=f'{JOINT_NAMES[i]}', color=colors[i], linewidth=2)
            output_lines.append(line)
        
        ax1.set_xlabel('Time (s)', fontsize=10)
        ax1.set_ylabel('Error (rad)', fontsize=10)
        ax1.set_title('Position Errors - LIVE', fontsize=12, fontweight='bold')
        ax1.legend(loc='upper right', fontsize=8)
        ax1.grid(True, alpha=0.3)
        
        ax2.set_xlabel('Time (s)', fontsize=10)
        ax2.set_ylabel('Velocity Output', fontsize=10)
        ax2.set_title('PID Velocity Outputs - LIVE', fontsize=12, fontweight='bold')
        ax2.legend(loc='upper right', fontsize=8)
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()

        def onclick(event):
            if event.inaxes in [ax1, ax2]:
                if event.xdata and event.ydata:
                    print(f"\n🟩 Time: {event.xdata:.3f} s | Value: {event.ydata:.3f}\n")

        fig.canvas.mpl_connect('button_press_event', onclick)
        
        self.get_logger().info("🔴 LIVE PLOTTING ACTIVE")

        update_counter = 0
        
        while True:
            try:
                if not rclpy.ok():
                    self.get_logger().info("ROS shutdown detected, stopping plot")
                    break
                
                if len(self.time_data) > 10:
                    for i in range(5):
                        error_lines[i].set_xdata(self.time_data)
                        error_lines[i].set_ydata(self.error_data[i])
                        
                        output_lines[i].set_xdata(self.time_data)
                        output_lines[i].set_ydata(self.output_data[i])
                    
                    ax1.relim()
                    ax1.autoscale_view()
                    ax2.relim()
                    ax2.autoscale_view()
                    
                    update_counter += 1
                
                try:
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
                except:
                    pass
                
                time.sleep(0.05)
                    
            except KeyboardInterrupt:
                self.get_logger().info("Plot interrupted by Ctrl+C")
                break
            except Exception as e:
                self.get_logger().debug(f"Minor plot error (continuing): {e}")
                time.sleep(0.1)
        
        plt.ioff()
        plt.close(fig)
        self.get_logger().info("Plotting thread ended")


def main(args=None):
    rclpy.init(args=args)
    node = MultiJointPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()