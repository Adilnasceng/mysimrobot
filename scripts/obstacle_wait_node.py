#!/usr/bin/env python3
"""
Nav2 dinamik engel bekleme node'u.

Mantık:
  - Nav2'nin hız komutundan robotun ileri mi geri mi gittiğini anlar.
  - LiDAR'da sadece hareket yönündeki yakın engelleri inceler.
  - Scan noktasını /map'e taşır ve kayıtlı haritadaki dolu hücrelere yakınsa
    bunu statik duvar/engel sayıp yok sayar.
  - Haritada olmayan yeni bir engel varsa /cmd_vel_obstacle üzerinden sıfır hız
    yayınlayarak twist_mux ile robotu durdurur.
  - Engel kalkınca override biter; Nav2 mevcut rotadan devam eder.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

try:
    from tf2_ros import TransformException
except ImportError:  # pragma: no cover - older tf2_ros fallback
    TransformException = Exception


class ObstacleWaitNode(Node):
    def __init__(self):
        super().__init__('obstacle_wait_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('obstacle_cmd_topic', '/cmd_vel_obstacle')
        self.declare_parameter('map_frame', 'map')

        self.declare_parameter('obstacle_distance', 1.0)
        self.declare_parameter('obstacle_angle', 0.524)  # +/-30 derece
        self.declare_parameter('min_linear_speed', 0.03)
        self.declare_parameter('cmd_timeout', 1.0)

        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('map_obstacle_padding', 0.12)
        self.declare_parameter('ignore_unknown_cells', True)
        self.declare_parameter('ignore_outside_map', True)
        self.declare_parameter('require_map', True)
        self.declare_parameter('wait_log_period', 2.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.obstacle_cmd_topic = self.get_parameter('obstacle_cmd_topic').value
        self.map_frame = self.get_parameter('map_frame').value

        self.obs_dist = float(self.get_parameter('obstacle_distance').value)
        self.obs_angle = float(self.get_parameter('obstacle_angle').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.map_obstacle_padding = float(self.get_parameter('map_obstacle_padding').value)
        self.ignore_unknown_cells = bool(self.get_parameter('ignore_unknown_cells').value)
        self.ignore_outside_map = bool(self.get_parameter('ignore_outside_map').value)
        self.require_map = bool(self.get_parameter('require_map').value)
        self.wait_log_period = float(self.get_parameter('wait_log_period').value)

        self.map_msg = None
        self.last_linear_x = 0.0
        self.last_cmd_time = None

        self.dynamic_obstacle = False
        self.obstacle_direction = None
        self.nearest_obstacle = None
        self.blocking = False
        self.last_wait_log_time = None
        self.warn_times = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, map_qos)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.obstacle_cmd_topic, 10)
        self.create_timer(0.1, self.logic_cb)

        self.get_logger().info(
            'obstacle_wait_node başlatıldı | '
            f'scan={self.scan_topic} | map={self.map_topic} | '
            f'cmd={self.cmd_vel_topic} | mesafe={self.obs_dist:.2f}m | '
            f'açı=±{math.degrees(self.obs_angle):.0f}°'
        )

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

    def cmd_cb(self, msg: Twist):
        self.last_linear_x = msg.linear.x
        self.last_cmd_time = self.get_clock().now()

    def scan_cb(self, msg: LaserScan):
        direction = self._motion_direction()
        if direction is None:
            self._set_obstacle(False, None, None)
            return

        if self.require_map and self.map_msg is None:
            self._warn_throttled(
                'map_missing',
                5.0,
                'Harita bekleniyor; statik duvarları ayırt edememek için engel bekleme pasif.',
            )
            self._set_obstacle(False, None, None)
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self._warn_throttled(
                'tf_missing',
                5.0,
                f'{msg.header.frame_id} -> {self.map_frame} TF bulunamadı: {exc}',
            )
            self._set_obstacle(False, None, None)
            return

        nearest = None
        for i, scan_range in enumerate(msg.ranges):
            if not self._valid_range(scan_range, msg):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            if not self._angle_in_motion_sector(angle, direction):
                continue

            scan_x = scan_range * math.cos(angle)
            scan_y = scan_range * math.sin(angle)
            map_x, map_y = self._transform_xy(transform, scan_x, scan_y)

            if self._is_static_map_obstacle(map_x, map_y):
                continue

            nearest = scan_range if nearest is None else min(nearest, scan_range)

        self._set_obstacle(nearest is not None, direction, nearest)

    def logic_cb(self):
        if self.dynamic_obstacle:
            self.cmd_pub.publish(Twist())

            if not self.blocking:
                self.blocking = True
                self.last_wait_log_time = self.get_clock().now()
                where = 'önde' if self.obstacle_direction == 'front' else 'arkada'
                self.get_logger().warn(
                    f'Haritada olmayan engel {where} algılandı '
                    f'({self.nearest_obstacle:.2f}m). Engel kalkana kadar bekleniyor.'
                )
                return

            if self._seconds_since(self.last_wait_log_time) >= self.wait_log_period:
                self.last_wait_log_time = self.get_clock().now()
                where = 'önde' if self.obstacle_direction == 'front' else 'arkada'
                self.get_logger().warn(
                    f'Engel hala {where} ({self.nearest_obstacle:.2f}m); bekleniyor.'
                )
            return

        if self.blocking:
            self.blocking = False
            self.last_wait_log_time = None
            self.get_logger().info('Engel kalktı; Nav2 mevcut rotadan devam ediyor.')

    def _set_obstacle(self, active, direction, nearest):
        self.dynamic_obstacle = active
        self.obstacle_direction = direction
        self.nearest_obstacle = nearest

    def _motion_direction(self):
        if self.last_cmd_time is None:
            return self.obstacle_direction if self.blocking else None
        if self._seconds_since(self.last_cmd_time) > self.cmd_timeout:
            return self.obstacle_direction if self.blocking else None
        if self.last_linear_x > self.min_linear_speed:
            return 'front'
        if self.last_linear_x < -self.min_linear_speed:
            return 'rear'
        if self.blocking:
            return self.obstacle_direction
        return None

    def _angle_in_motion_sector(self, angle, direction):
        angle = self._normalize_angle(angle)
        if direction == 'front':
            return abs(angle) <= self.obs_angle
        return abs(math.pi - abs(angle)) <= self.obs_angle

    def _valid_range(self, scan_range, msg):
        return (
            math.isfinite(scan_range)
            and msg.range_min < scan_range < min(self.obs_dist, msg.range_max)
        )

    def _is_static_map_obstacle(self, map_x, map_y):
        if self.map_msg is None:
            return False

        center = self._world_to_map(map_x, map_y)
        if center is None:
            return self.ignore_outside_map

        info = self.map_msg.info
        padding_cells = max(0, math.ceil(self.map_obstacle_padding / info.resolution))
        cx, cy = center

        for y in range(cy - padding_cells, cy + padding_cells + 1):
            for x in range(cx - padding_cells, cx + padding_cells + 1):
                if not (0 <= x < info.width and 0 <= y < info.height):
                    if self.ignore_outside_map:
                        return True
                    continue

                value = self.map_msg.data[y * info.width + x]
                if value < 0 and self.ignore_unknown_cells:
                    return True
                if value >= self.occupied_threshold:
                    return True

        return False

    def _world_to_map(self, x, y):
        info = self.map_msg.info
        origin = info.origin
        yaw = self._yaw_from_quaternion(origin.orientation)

        dx = x - origin.position.x
        dy = y - origin.position.y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        map_local_x = cos_yaw * dx + sin_yaw * dy
        map_local_y = -sin_yaw * dx + cos_yaw * dy

        mx = math.floor(map_local_x / info.resolution)
        my = math.floor(map_local_y / info.resolution)

        if not (0 <= mx < info.width and 0 <= my < info.height):
            return None
        return int(mx), int(my)

    def _transform_xy(self, transform, x, y):
        yaw = self._yaw_from_quaternion(transform.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        return (
            tx + cos_yaw * x - sin_yaw * y,
            ty + sin_yaw * x + cos_yaw * y,
        )

    def _warn_throttled(self, key, period, message):
        now = self.get_clock().now()
        previous = self.warn_times.get(key)
        if previous is None or (now - previous).nanoseconds / 1e9 >= period:
            self.warn_times[key] = now
            self.get_logger().warn(message)

    def _seconds_since(self, stamp):
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )


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
