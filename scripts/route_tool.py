#!/usr/bin/env python3
"""Save and replay simple Nav2 routes."""

import argparse
import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

try:
    from tf2_ros import TransformException
except ImportError:  # pragma: no cover - older tf2_ros fallback
    TransformException = Exception


DEFAULT_FRAME = 'map'
DEFAULT_BASE_FRAME = 'base_link'


def default_routes_file():
    cwd_routes = Path.cwd() / 'config' / 'routes.yaml'
    if cwd_routes.parent.exists():
        return cwd_routes
    return Path.home() / '.mysimrobot' / 'routes.yaml'


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(half),
        'w': math.cos(half),
    }


def load_routes(path):
    if not path.exists():
        return {'routes': {}}
    data = yaml.safe_load(path.read_text()) or {}
    if 'routes' not in data or data['routes'] is None:
        data['routes'] = {}
    return data


def write_routes(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
        encoding='utf-8',
    )


class RouteSaver(Node):
    def __init__(self, target_frame, base_frame):
        super().__init__('route_saver')
        self.target_frame = target_frame
        self.base_frame = base_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def current_pose(self, timeout):
        deadline = self.get_clock().now() + Duration(seconds=timeout)
        last_error = None

        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.base_frame,
                    Time(),
                    timeout=Duration(seconds=0.1),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                return {
                    'x': float(translation.x),
                    'y': float(translation.y),
                    'yaw': float(yaw_from_quaternion(rotation)),
                }
            except TransformException as exc:
                last_error = exc

        raise RuntimeError(
            f'{self.target_frame} -> {self.base_frame} TF bulunamadı: {last_error}'
        )


class RouteRunner(Node):
    def __init__(self):
        super().__init__('route_runner')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def run_route(self, route_name, route, server_timeout):
        poses = route.get('poses') or []
        if not poses:
            raise RuntimeError(f'"{route_name}" rotasında hedef pose yok.')

        self.get_logger().info('Nav2 navigate_to_pose action server bekleniyor...')
        if not self.nav_client.wait_for_server(timeout_sec=server_timeout):
            raise RuntimeError('navigate_to_pose action server bulunamadı.')

        frame_id = route.get('frame_id', DEFAULT_FRAME)
        for index, pose_data in enumerate(poses, start=1):
            point_name = pose_data.get('name', f'wp_{index}')
            self.get_logger().info(
                f'{route_name}: {index}/{len(poses)} hedefe gidiliyor: {point_name}'
            )
            result_status = self._send_pose_goal(frame_id, pose_data)
            if result_status != 4:
                raise RuntimeError(
                    f'{point_name} hedefine gidilemedi. Action status={result_status}'
                )

        self.get_logger().info(f'"{route_name}" rotası tamamlandı.')

    def _send_pose_goal(self, frame_id, pose_data):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._pose_stamped(frame_id, pose_data)

        send_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('Nav2 hedefi reddetti.')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().status

    def _pose_stamped(self, frame_id, pose_data):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(pose_data['x'])
        pose.pose.position.y = float(pose_data['y'])
        pose.pose.position.z = 0.0

        q = quaternion_from_yaw(float(pose_data.get('yaw', 0.0)))
        pose.pose.orientation.x = q['x']
        pose.pose.orientation.y = q['y']
        pose.pose.orientation.z = q['z']
        pose.pose.orientation.w = q['w']
        return pose


def save_command(args):
    route_file = Path(args.file).expanduser()
    rclpy.init()
    node = RouteSaver(args.frame, args.base_frame)
    try:
        pose = node.current_pose(args.timeout)
        point = {
            'name': args.point,
            'x': round(pose['x'], 4),
            'y': round(pose['y'], 4),
            'yaw': round(pose['yaw'], 4),
        }

        data = load_routes(route_file)
        route = data['routes'].get(args.route, {'frame_id': args.frame, 'poses': []})
        route['frame_id'] = args.frame
        if args.append:
            route.setdefault('poses', []).append(point)
        else:
            route['poses'] = [point]
        data['routes'][args.route] = route
        write_routes(route_file, data)

        node.get_logger().info(
            f'"{args.route}" rotasına "{args.point}" kaydedildi: '
            f'x={point["x"]:.4f}, y={point["y"]:.4f}, yaw={point["yaw"]:.4f}'
        )
        node.get_logger().info(f'Rota dosyası: {route_file}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_command(args):
    route_file = Path(args.file).expanduser()
    data = load_routes(route_file)
    route = data['routes'].get(args.route)
    if route is None:
        known = ', '.join(sorted(data['routes'])) or 'yok'
        raise RuntimeError(f'"{args.route}" rotası bulunamadı. Kayıtlı rotalar: {known}')

    rclpy.init()
    node = RouteRunner()
    try:
        node.run_route(args.route, route, args.server_timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def list_command(args):
    route_file = Path(args.file).expanduser()
    data = load_routes(route_file)
    routes = data.get('routes', {})
    if not routes:
        print(f'Kayıtlı rota yok. Dosya: {route_file}')
        return

    print(f'Rota dosyası: {route_file}')
    for name, route in routes.items():
        print(f'- {name}: {len(route.get("poses") or [])} hedef')


def build_parser():
    parser = argparse.ArgumentParser(
        description='Save current robot pose and replay routes with Nav2.'
    )
    parser.set_defaults(file=str(default_routes_file()))
    subparsers = parser.add_subparsers(dest='command', required=True)

    save = subparsers.add_parser('save', help='Save current robot pose into a route.')
    save.add_argument('route', help='Route name, e.g. baslangic')
    save.add_argument('--point', default='goal', help='Saved point name')
    save.add_argument('--append', action='store_true', help='Append instead of overwrite')
    save.add_argument('--file', default=str(default_routes_file()), help='Routes YAML file')
    save.add_argument('--frame', default=DEFAULT_FRAME, help='Target frame for saved pose')
    save.add_argument('--base-frame', default=DEFAULT_BASE_FRAME, help='Robot base frame')
    save.add_argument('--timeout', type=float, default=5.0, help='TF wait timeout seconds')
    save.set_defaults(func=save_command)

    run = subparsers.add_parser('run', help='Run a saved route using Nav2.')
    run.add_argument('route', help='Route name to run')
    run.add_argument('--file', default=str(default_routes_file()), help='Routes YAML file')
    run.add_argument(
        '--server-timeout',
        type=float,
        default=10.0,
        help='Nav2 action server wait timeout seconds',
    )
    run.set_defaults(func=run_command)

    list_routes = subparsers.add_parser('list', help='List saved routes.')
    list_routes.add_argument('--file', default=str(default_routes_file()), help='Routes YAML file')
    list_routes.set_defaults(func=list_command)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    try:
        args.func(args)
    except RuntimeError as exc:
        print(f'Hata: {exc}')
        raise SystemExit(1) from exc


if __name__ == '__main__':
    main()
