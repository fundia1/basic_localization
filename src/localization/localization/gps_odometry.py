"""
gps_odometry.py — GPS → Odometry 변환 노드

map_anchor.yaml의 datum 좌표를 원점(0,0)으로 하여
GPS NavSatFix를 UTM 로컬 좌표의 Odometry 메시지로 변환합니다.

  /ublox_gps_node/fix (NavSatFix) → /odometry/gps (Odometry)
"""
import os

import utm
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion


class GpsOdometryNode(Node):

    def __init__(self):
        super().__init__('gps_odometry')

        # map_anchor.yaml에서 datum 로드
        pkg_share = get_package_share_directory('localization')
        anchor_path = os.path.join(pkg_share, 'config', 'map_anchor.yaml')
        with open(anchor_path, 'r') as f:
            anchor = yaml.safe_load(f)

        datum_lat = anchor['latitude']
        datum_lon = anchor['longitude']

        # datum → UTM 원점
        self._datum_e, self._datum_n, self._zone_number, self._zone_letter = \
            utm.from_latlon(datum_lat, datum_lon)

        self.get_logger().info(
            f'Datum: lat={datum_lat:.6f}, lon={datum_lon:.6f} '
            f'→ UTM {self._zone_number}{self._zone_letter} '
            f'E={self._datum_e:.2f}, N={self._datum_n:.2f}'
        )

        # Subscriber / Publisher
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self._on_fix, 10)
        self._odom_pub = self.create_publisher(Odometry, '/odometry/gps', 10)

    def _on_fix(self, msg: NavSatFix):
        if msg.status.status < 0:
            return

        # GPS → UTM → datum 기준 로컬 좌표
        e, n, _, _ = utm.from_latlon(msg.latitude, msg.longitude,
                                     force_zone_number=self._zone_number,
                                     force_zone_letter=self._zone_letter)
        x = e - self._datum_e   # 동쪽
        y = n - self._datum_n   # 북쪽

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # 공분산
        cov36 = [0.0] * 36
        if len(msg.position_covariance) == 9:
            cov36[0] = msg.position_covariance[4]   # 동(x)
            cov36[7] = msg.position_covariance[0]   # 북(y)
        else:
            cov36[0] = 10.0
            cov36[7] = 10.0
        cov36[14] = 1e6
        cov36[21] = 1e6
        cov36[28] = 1e6
        cov36[35] = 1e6
        odom.pose.covariance = cov36

        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GpsOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
