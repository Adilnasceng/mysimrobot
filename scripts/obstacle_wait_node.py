#!/usr/bin/env python3
"""
obstacle_wait_node.py
---------------------
Robot Nav2 ile bir hedefe giderken önüne engel çıktığında:
  - Robotu durdurur (twist_mux üzerinden sıfır hız override)
  - 15 saniye boyunca engelin kalkmasını bekler
  - Engel kalkarsa → Nav2 kaldığı yerden devam eder
  - 15 saniye geçerse → override bırakılır, Nav2 yeni rota hesaplar
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleWaitNode(Node):
    def __init__(self):
        super().__init__('obstacle_wait_node')

        # --- Parametreler ---
        self.declare_parameter('obstacle_distance', 1.0)   # metre
        self.declare_parameter('obstacle_angle',    0.524) # ±30° (radyan)
        self.declare_parameter('wait_duration',    15.0)   # saniye

        self.obs_dist  = self.get_parameter('obstacle_distance').value
        self.obs_angle = self.get_parameter('obstacle_angle').value
        self.wait_dur  = self.get_parameter('wait_duration').value

        # --- Durum ---
        self.obstacle_in_front = False
        self.blocking          = False
        self.block_start       = None

        # --- ROS arayüzleri ---
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_obstacle', 10)
        self.create_timer(0.1, self.logic_cb)   # 10 Hz

        self.get_logger().info(
            f'obstacle_wait_node başlatıldı | '
            f'mesafe={self.obs_dist}m | '
            f'açı=±{math.degrees(self.obs_angle):.0f}° | '
            f'bekleme={self.wait_dur}s'
        )

    # ------------------------------------------------------------------
    def scan_cb(self, msg: LaserScan):
        """Önündeki ±obs_angle konisinde obs_dist'ten yakın engel var mı?"""
        self.obstacle_in_front = False
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= self.obs_angle and msg.range_min < r < self.obs_dist:
                self.obstacle_in_front = True
                break

    # ------------------------------------------------------------------
    def logic_cb(self):
        """10 Hz'de çalışır; engel durumuna göre sıfır hız yayınlar veya bırakır."""
        now = self.get_clock().now()

        if self.obstacle_in_front:
            if not self.blocking:
                # Engel yeni algılandı — bloğu başlat
                self.blocking    = True
                self.block_start = now
                self.get_logger().warn(
                    f'Engel algılandı (önde <{self.obs_dist}m)! '
                    f'En fazla {self.wait_dur:.0f}s bekleniyor...'
                )

            elapsed = (now - self.block_start).nanoseconds / 1e9

            if elapsed < self.wait_dur:
                # Süre dolmadı — robotu durdur (twist_mux priority 15 override)
                self.cmd_pub.publish(Twist())
            else:
                # 15 saniye doldu, engel hâlâ orada → override'ı bırak
                self.blocking = False
                self.get_logger().warn(
                    f'{self.wait_dur:.0f}s geçti, engel kalkmadı. '
                    'Nav2 güncel costmap ile yeni rota hesaplayacak...'
                )
                # Override bitti → Nav2 costmap'teki engelle yeni yol bulur

        elif self.blocking:
            # Engel kalktı
            elapsed = (now - self.block_start).nanoseconds / 1e9
            self.blocking = False
            self.get_logger().info(
                f'Engel {elapsed:.1f}s sonra kalktı. '
                'Nav2 kaldığı yerden devam ediyor...'
            )
            # Override bitti → Nav2 mevcut plan üzerinden devam eder


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleWaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
