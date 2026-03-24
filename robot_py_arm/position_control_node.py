#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk
import threading
import math


class PositionControlNode(Node):
    def __init__(self):
        super().__init__('position_control_node')
        
        # Publisher oluştur
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'target_position',
            10
        )
        
        # Başlangıç pozisyonları (mm)
        self.x = 505.0
        self.y = 376.0
        self.max_radius = 630.0  # Maksimum yarıçap
        
        # Hareket parametreleri
        self.step_size = 5.0  # mm
        self.update_rate = 0.05  # saniye (20 Hz)
        
        # Tuş durumları
        self.key_states = {
            'w': False, 's': False,
            'a': False, 'd': False,
            'Up': False, 'Down': False,
            'Left': False, 'Right': False
        }
        
        # Yol takibi için
        self.path_points = []  # Çizilen yol noktaları (mm cinsinden)
        self.path_following = False
        self.current_path_index = 0
        
        # Timer ile sürekli güncelleme
        self.timer = self.create_timer(self.update_rate, self.update_position)
        self.timer1=self.create_timer(2.0, self.go_to_zero_positoin)

        self.get_logger().info('Position Control Node başlatıldı')
        self.get_logger().info(f'Başlangıç pozisyonu: x={self.x:.1f}mm, y={self.y:.1f}mm')
    def go_to_zero_positoin(self):
        self.x=630.0
        self.y=0.0
        self.timer1.destroy()
        
    def update_position(self):
        """Pozisyonu güncelle ve yayınla"""
        moved = False
        new_x = self.x
        new_y = self.y
        
        # Yol takibi aktifse
        if self.path_following and self.path_points:
            if self.current_path_index < len(self.path_points):
                target_x, target_y = self.path_points[self.current_path_index]
                
                # Hedefe ulaştık mı kontrol et
                distance = math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)
                if distance < self.step_size * 2:  # Hedefe yaklaştık
                    self.current_path_index += 1
                    if self.current_path_index >= len(self.path_points):
                        self.path_following = False
                        self.get_logger().info('Yol takibi tamamlandı')
                else:
                    # Hedefe doğru hareket et
                    dx = target_x - self.x
                    dy = target_y - self.y
                    angle = math.atan2(dy, dx)
                    new_x = self.x + self.step_size * math.cos(angle)
                    new_y = self.y + self.step_size * math.sin(angle)
                    moved = True
            else:
                self.path_following = False
        
        # Manuel klavye kontrolleri (yol takibi yoksa)
        if not self.path_following:
            if self.key_states['d'] or self.key_states['Right']:
                new_x += self.step_size
                moved = True
            if self.key_states['a'] or self.key_states['Left']:
                new_x -= self.step_size
                moved = True
            if self.key_states['w'] or self.key_states['Up']:
                new_y += self.step_size
                moved = True
            if self.key_states['s'] or self.key_states['Down']:
                new_y -= self.step_size
                moved = True
        
        # Yeni pozisyonu kontrol et
        if moved:
            if self.check_limits(new_x, new_y):
                self.x = new_x
                self.y = new_y
            else:
                # Limit aşıldı, maksimum izin verilen noktaya ayarla
                self.x, self.y = self.get_max_allowed_position(new_x, new_y)
        
        # Pozisyonu yayınla
        self.publish_position()
    
    def check_limits(self, x, y):
        """Pozisyonun limit içinde olup olmadığını kontrol et"""
        radius = math.sqrt(x**2 + y**2)
        return radius <= self.max_radius
    
    def get_max_allowed_position(self, x, y):
        """Limit aşıldığında maksimum izin verilen pozisyonu hesapla"""
        radius = math.sqrt(x**2 + y**2)
        if radius > self.max_radius:
            # Pozisyonu maksimum yarıçapa normalize et
            scale = self.max_radius / radius
            return x * scale, y * scale
        return x, y
    
    def publish_position(self):
        """Mevcut pozisyonu yayınla"""
        msg = Float64MultiArray()
        msg.data = [self.x, self.y]
        self.publisher.publish(msg)
    
    def set_step_size(self, value):
        """Adım boyutunu ayarla"""
        self.step_size = float(value)
        self.get_logger().info(f'Adım boyutu: {self.step_size:.1f}mm')
    
    def set_position(self, x, y):
        """Pozisyonu direkt ayarla"""
        if self.check_limits(x, y):
            self.x = x
            self.y = y
            self.get_logger().info(f'Pozisyon ayarlandı: x={x:.1f}mm, y={y:.1f}mm')
        else:
            self.x, self.y = self.get_max_allowed_position(x, y)
            self.get_logger().warn(f'Limit aşıldı, pozisyon ayarlandı: x={self.x:.1f}mm, y={self.y:.1f}mm')
    
    def start_path_following(self):
        """Yol takibini başlat"""
        if self.path_points:
            self.path_following = True
            self.current_path_index = 0
            self.get_logger().info(f'Yol takibi başlatıldı: {len(self.path_points)} nokta')
        else:
            self.get_logger().warn('Takip edilecek yol yok!')
    
    def stop_path_following(self):
        """Yol takibini durdur"""
        self.path_following = False
        self.current_path_index = 0
        self.get_logger().info('Yol takibi durduruldu')
    
    def set_path(self, points):
        """Yol noktalarını ayarla"""
        self.path_points = points
        self.current_path_index = 0
        self.get_logger().info(f'Yol ayarlandı: {len(points)} nokta')
    
    def clear_path(self):
        """Yolu temizle"""
        self.path_points = []
        self.path_following = False
        self.current_path_index = 0
        self.get_logger().info('Yol temizlendi')


class ControlGUI:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("ROS2 Position Control")
        self.root.geometry("900x650")
        
        # Canvas parametreleri
        self.canvas_size = 400  # piksel
        self.mm_to_pixel = self.canvas_size / (2 * 630)  # 1260mm -> 400 piksel
        
        # Çizim modu
        self.drawing_mode = tk.StringVar(value="continuous")  # continuous veya point
        self.is_drawing = False
        self.temp_path = []  # Geçici çizilen yol
        
        self.setup_gui()
        
        # Klavye event'leri
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        
        # GUI güncelleme timer'ı
        self.update_gui()
    
    def setup_gui(self):
        """GUI öğelerini oluştur"""
        # Ana container
        main_container = ttk.Frame(self.root, padding="10")
        main_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Sol panel (mevcut kontroller)
        left_panel = ttk.Frame(main_container)
        left_panel.grid(row=0, column=0, padx=(0, 10), sticky=(tk.N, tk.S))
        
        # Sağ panel (canvas ve yol kontrolleri)
        right_panel = ttk.Frame(main_container)
        right_panel.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        self.setup_left_panel(left_panel)
        self.setup_right_panel(right_panel)
    
    def setup_left_panel(self, parent):
        """Sol panel - Mevcut kontroller"""
        # Başlık
        title_label = ttk.Label(
            parent,
            text="Position Control",
            font=('Arial', 14, 'bold')
        )
        title_label.grid(row=0, column=0, columnspan=2, pady=10)
        
        # Pozisyon gösterimi
        position_frame = ttk.LabelFrame(parent, text="Current Position", padding="10")
        position_frame.grid(row=1, column=0, columnspan=2, pady=10, sticky=(tk.W, tk.E))
        
        ttk.Label(position_frame, text="X:", font=('Arial', 9)).grid(row=0, column=0, sticky=tk.W)
        self.x_label = ttk.Label(position_frame, text="630.0 mm", font=('Arial', 10, 'bold'))
        self.x_label.grid(row=0, column=1, sticky=tk.W, padx=5)
        
        ttk.Label(position_frame, text="Y:", font=('Arial', 9)).grid(row=1, column=0, sticky=tk.W)
        self.y_label = ttk.Label(position_frame, text="0.0 mm", font=('Arial', 10, 'bold'))
        self.y_label.grid(row=1, column=1, sticky=tk.W, padx=5)
        
        ttk.Label(position_frame, text="R:", font=('Arial', 9)).grid(row=2, column=0, sticky=tk.W)
        self.radius_label = ttk.Label(position_frame, text="630.0 mm", font=('Arial', 10, 'bold'))
        self.radius_label.grid(row=2, column=1, sticky=tk.W, padx=5)
        
        # Manuel koordinat girişi
        coord_frame = ttk.LabelFrame(parent, text="Manual Position", padding="10")
        coord_frame.grid(row=2, column=0, columnspan=2, pady=10, sticky=(tk.W, tk.E))
        
        ttk.Label(coord_frame, text="X (mm):").grid(row=0, column=0, sticky=tk.W)
        self.manual_x = tk.DoubleVar(value=630.0)
        ttk.Entry(coord_frame, textvariable=self.manual_x, width=10).grid(row=0, column=1, padx=5)
        
        ttk.Label(coord_frame, text="Y (mm):").grid(row=1, column=0, sticky=tk.W)
        self.manual_y = tk.DoubleVar(value=0.0)
        ttk.Entry(coord_frame, textvariable=self.manual_y, width=10).grid(row=1, column=1, padx=5)
        
        ttk.Button(coord_frame, text="Go", command=self.go_to_position).grid(row=2, column=0, columnspan=2, pady=5)
        
        # Hız kontrolü
        speed_frame = ttk.LabelFrame(parent, text="Speed (Step)", padding="10")
        speed_frame.grid(row=3, column=0, columnspan=2, pady=10, sticky=(tk.W, tk.E))
        
        self.step_var = tk.DoubleVar(value=5.0)
        ttk.Entry(speed_frame, textvariable=self.step_var, width=8).grid(row=0, column=0, padx=5)
        ttk.Button(speed_frame, text="Apply", command=self.apply_step_size).grid(row=0, column=1)
        
        quick_frame = ttk.Frame(speed_frame)
        quick_frame.grid(row=1, column=0, columnspan=2, pady=5)
        for speed in [1, 5, 10, 20, 50]:
            ttk.Button(quick_frame, text=f"{speed}", command=lambda s=speed: self.set_quick_speed(s), width=4).pack(side=tk.LEFT, padx=1)
        
        # Kontrol talimatları
        control_frame = ttk.LabelFrame(parent, text="Keyboard", padding="10")
        control_frame.grid(row=4, column=0, columnspan=2, pady=10, sticky=(tk.W, tk.E))
        
        instructions = [
            "W/↑ : Y +",
            "S/↓ : Y -",
            "D/→ : X +",
            "A/← : X -"
        ]
        
        for i, text in enumerate(instructions):
            ttk.Label(control_frame, text=text, font=('Courier', 8)).grid(row=i, column=0, sticky=tk.W)
        
        # Sıfırlama
        ttk.Button(parent, text="Reset (630,0)", command=self.reset_position).grid(row=5, column=0, columnspan=2, pady=5)
        
        # Çıkış
        ttk.Button(parent, text="Exit", command=self.on_closing).grid(row=6, column=0, columnspan=2, pady=5)
    
    def setup_right_panel(self, parent):
        """Sağ panel - Canvas ve yol kontrolleri"""
        # Canvas frame
        canvas_frame = ttk.LabelFrame(parent, text="Path Planning (630x630mm)", padding="10")
        canvas_frame.grid(row=0, column=0, pady=5)
        
        # Canvas
        self.canvas = tk.Canvas(canvas_frame, width=self.canvas_size, height=self.canvas_size, bg='white', highlightthickness=1, highlightbackground='gray')
        self.canvas.pack()
        
        # Canvas event'leri
        self.canvas.bind('<Button-1>', self.on_canvas_click)
        self.canvas.bind('<B1-Motion>', self.on_canvas_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_canvas_release)
        
        # İlk çizim
        self.draw_workspace()
        
        # Çizim modu seçimi
        mode_frame = ttk.LabelFrame(parent, text="Drawing Mode", padding="10")
        mode_frame.grid(row=1, column=0, pady=10, sticky=(tk.W, tk.E))
        
        ttk.Radiobutton(mode_frame, text="Continuous Drawing", variable=self.drawing_mode, value="continuous").pack(anchor=tk.W)
        ttk.Radiobutton(mode_frame, text="Point Drawing", variable=self.drawing_mode, value="point").pack(anchor=tk.W)
        
        # Yol kontrolleri
        path_control_frame = ttk.LabelFrame(parent, text="Path Controls", padding="10")
        path_control_frame.grid(row=2, column=0, pady=10, sticky=(tk.W, tk.E))
        
        self.path_status_label = ttk.Label(path_control_frame, text="Point: 0", font=('Arial', 9))
        self.path_status_label.pack()
        
        btn_frame = ttk.Frame(path_control_frame)
        btn_frame.pack(pady=5)
        
        self.start_btn = ttk.Button(btn_frame, text="Start", command=self.start_path, state='disabled')
        self.start_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = ttk.Button(btn_frame, text="stop", command=self.stop_path, state='disabled')
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(btn_frame, text="clear", command=self.clear_path).pack(side=tk.LEFT, padx=5)
    
    def mm_to_canvas(self, x_mm, y_mm):
        """mm koordinatlarını canvas piksellerine çevir"""
        # Merkez: (200, 200), Y yukarı pozitif
        x_pixel = self.canvas_size / 2 + x_mm * self.mm_to_pixel
        y_pixel = self.canvas_size / 2 - y_mm * self.mm_to_pixel
        return x_pixel, y_pixel
    
    def canvas_to_mm(self, x_pixel, y_pixel):
        """Canvas piksellerini mm koordinatlarına çevir"""
        x_mm = (x_pixel - self.canvas_size / 2) / self.mm_to_pixel
        y_mm = (self.canvas_size / 2 - y_pixel) / self.mm_to_pixel
        return x_mm, y_mm
    
    def draw_workspace(self):
        """Çalışma alanını çiz"""
        self.canvas.delete('all')
        
        # Limit dairesi çiz
        center = self.canvas_size / 2
        radius_pixel = 630 * self.mm_to_pixel
        self.canvas.create_oval(
            center - radius_pixel, center - radius_pixel,
            center + radius_pixel, center + radius_pixel,
            outline='red', width=2, dash=(5, 3)
        )
        
        # Merkez çizgileri
        self.canvas.create_line(0, center, self.canvas_size, center, fill='lightgray', dash=(2, 2))
        self.canvas.create_line(center, 0, center, self.canvas_size, fill='lightgray', dash=(2, 2))
        
        # Çizilen yolu göster
        if self.temp_path:
            for i in range(len(self.temp_path) - 1):
                x1, y1 = self.mm_to_canvas(*self.temp_path[i])
                x2, y2 = self.mm_to_canvas(*self.temp_path[i + 1])
                self.canvas.create_line(x1, y1, x2, y2, fill='blue', width=2)
            
            # Noktaları göster
            for x_mm, y_mm in self.temp_path:
                x, y = self.mm_to_canvas(x_mm, y_mm)
                self.canvas.create_oval(x-2, y-2, x+2, y+2, fill='blue', outline='blue')
        
        # Robot pozisyonunu göster
        robot_x, robot_y = self.mm_to_canvas(self.node.x, self.node.y)
        self.canvas.create_oval(robot_x-5, robot_y-5, robot_x+5, robot_y+5, fill='green', outline='darkgreen', width=2)
        self.canvas.create_text(robot_x, robot_y-15, text='Robot', fill='green', font=('Arial', 8, 'bold'))
    
    def on_canvas_click(self, event):
        """Canvas'a tıklandığında"""
        x_mm, y_mm = self.canvas_to_mm(event.x, event.y)
        
        # Limit kontrolü
        if not self.node.check_limits(x_mm, y_mm):
            return
        
        if self.drawing_mode.get() == "point":
            # Noktasal mod - her tıklamada nokta ekle
            self.temp_path.append((x_mm, y_mm))
            self.draw_workspace()
            self.update_path_status()
        else:
            # Sürekli çizim - başlat
            self.is_drawing = True
            self.temp_path = [(x_mm, y_mm)]
            self.draw_workspace()
    
    def on_canvas_drag(self, event):
        """Canvas'ta sürüklendiğinde"""
        if not self.is_drawing or self.drawing_mode.get() != "continuous":
            return
        
        x_mm, y_mm = self.canvas_to_mm(event.x, event.y)
        
        # Limit kontrolü
        if not self.node.check_limits(x_mm, y_mm):
            return
        
        # Son noktadan uzaklık kontrolü (çok sık nokta eklemeyi önle)
        if self.temp_path:
            last_x, last_y = self.temp_path[-1]
            distance = math.sqrt((x_mm - last_x)**2 + (y_mm - last_y)**2)
            if distance < 5:  # 5mm'den yakınsa ekleme
                return
        
        self.temp_path.append((x_mm, y_mm))
        self.draw_workspace()
        self.update_path_status()
    
    def on_canvas_release(self, event):
        """Mouse bırakıldığında"""
        if self.is_drawing:
            self.is_drawing = False
            self.update_path_status()
    
    def update_path_status(self):
        """Yol durumunu güncelle"""
        count = len(self.temp_path)
        self.path_status_label.config(text=f"Point: {count}")
        
        if count > 0:
            self.start_btn.config(state='normal')
        else:
            self.start_btn.config(state='disabled')
    
    def start_path(self):
        """Yol takibini başlat"""
        if self.temp_path:
            self.node.set_path(self.temp_path.copy())
            self.node.start_path_following()
            self.start_btn.config(state='disabled')
            self.stop_btn.config(state='normal')
    
    def stop_path(self):
        """Yol takibini durdur"""
        self.node.stop_path_following()
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')
    
    def clear_path(self):
        """Yolu temizle"""
        self.temp_path = []
        self.node.clear_path()
        self.draw_workspace()
        self.update_path_status()
        self.start_btn.config(state='disabled')
        self.stop_btn.config(state='disabled')
    
    def go_to_position(self):
        """Manuel girilen pozisyona git"""
        try:
            x = self.manual_x.get()
            y = self.manual_y.get()
            self.node.set_position(x, y)
        except:
            pass
    
    def on_key_press(self, event):
        """Klavye tuşuna basıldığında"""
        key = event.keysym
        if key in self.node.key_states:
            self.node.key_states[key] = True
    
    def on_key_release(self, event):
        """Klavye tuşu bırakıldığında"""
        key = event.keysym
        if key in self.node.key_states:
            self.node.key_states[key] = False
    
    def apply_step_size(self):
        """Adım boyutunu uygula"""
        try:
            value = self.step_var.get()
            if value > 0:
                self.node.set_step_size(value)
        except:
            pass
    
    def set_quick_speed(self, speed):
        """Hızlı hız seçimi"""
        self.step_var.set(speed)
        self.apply_step_size()
    
    def reset_position(self):
        """Pozisyonu başlangıca döndür"""
        self.node.x = 630.0
        self.node.y = 0.0
        self.node.get_logger().info('Pozisyon başlangıca döndürüldü')
    
    def update_gui(self):
        """GUI'yi güncelle"""
        # Pozisyon bilgilerini güncelle
        self.x_label.config(text=f"{self.node.x:.1f} mm")
        self.y_label.config(text=f"{self.node.y:.1f} mm")
        
        radius = math.sqrt(self.node.x**2 + self.node.y**2)
        self.radius_label.config(text=f"{radius:.1f} mm")
        
        # Renk kodlaması
        if radius > 620:
            self.radius_label.config(foreground='red')
        elif radius > 600:
            self.radius_label.config(foreground='orange')
        else:
            self.radius_label.config(foreground='green')
        
        # Canvas'ı güncelle
        self.draw_workspace()
        
        # Yol takibi durumunu kontrol et
        if not self.node.path_following and self.stop_btn['state'] == 'normal':
            self.stop_btn.config(state='disabled')
            if self.temp_path:
                self.start_btn.config(state='normal')
        
        # 50ms sonra tekrar güncelle
        self.root.after(50, self.update_gui)
    
    def on_closing(self):
        """Pencere kapatıldığında"""
        self.root.quit()
        self.root.destroy()
    
    def run(self):
        """GUI'yi başlat"""
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    
    # Node'u oluştur
    node = PositionControlNode()
    
    # GUI'yi ayrı thread'de çalıştır
    def spin_node():
        rclpy.spin(node)
    
    thread = threading.Thread(target=spin_node, daemon=True)
    thread.start()
    
    # GUI'yi ana thread'de çalıştır
    gui = ControlGUI(node)
    gui.run()
    
    # Temizlik
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()