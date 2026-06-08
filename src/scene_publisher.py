#!/usr/bin/env python3
"""
scene_publisher.py

Loads desk.stl and publishes it as a permanent MoveIt2 collision object
fixed to the 'base' frame. Republishes every 5 s to survive move_group
restarts.

Adjust DESK_POSITION to shift the desk in the 'base' frame (metres).
The STL is in millimetres  converted automatically.
"""

import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, PlanningScene

STL_PATH      = '/home/lmh/desk.stl'
MM_TO_M       = 0.001
PUBLISH_EVERY = 5.0   # seconds

# ── Adjust these to position the desk relative to 'base'(metres) ────────────
DESK_POSITION   = (0.54,  0.0,  -0.45)      # x, y, z
DESK_QUATERNION = (0.0,  0.0,  0.7071, 0.7071)  # z축 90도 회전


def _parse_binary_stl(path: str) -> Mesh:
    with open(path, 'rb') as f:
        data = f.read()
    num_tri = struct.unpack_from('<I', data, 80)[0]

    vertices = []
    vert_idx = {}
    triangles = []

    for i in range(num_tri):
        off = 84 + i * 50
        tri = []
        for j in range(3):
            vx, vy, vz = struct.unpack_from('<fff', data, off + 12 + j * 12)
            key = (vx, vy, vz)
            if key not in vert_idx:
                vert_idx[key] = len(vertices)
                vertices.append(key)
            tri.append(vert_idx[key])
        triangles.append(tri)

    mesh = Mesh()
    mesh.vertices  = [Point(x=v[0]*MM_TO_M, y=v[1]*MM_TO_M, z=v[2]*MM_TO_M)
                      for v in vertices]
    mesh.triangles = [MeshTriangle(vertex_indices=[t[0], t[1], t[2]])
                      for t in triangles]
    return mesh


class ScenePublisher(Node):
    def __init__(self):
        super().__init__('scene_publisher')
        self._pub  = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self._mesh = _parse_binary_stl(STL_PATH)
        self.get_logger().info(
            f'Desk: {len(self._mesh.vertices)} vertices, '
            f'{len(self._mesh.triangles)} triangles')

        # First publish after 1 s (move_group may not be up yet), then every 5 s
        self.create_timer(1.0, self._publish)
        self.create_timer(PUBLISH_EVERY, self._publish)

    def _publish(self):
        pose = Pose(
            position=Point(x=DESK_POSITION[0],
                           y=DESK_POSITION[1],
                           z=DESK_POSITION[2]),
            orientation=Quaternion(x=DESK_QUATERNION[0],
                                   y=DESK_QUATERNION[1],
                                   z=DESK_QUATERNION[2],
                                   w=DESK_QUATERNION[3]),
        )

        co = CollisionObject()
        co.header.frame_id = 'base'
        co.header.stamp    = self.get_clock().now().to_msg()
        co.id              = 'desk'
        co.operation       = CollisionObject.ADD
        co.meshes          = [self._mesh]
        co.mesh_poses      = [pose]

        ps = PlanningScene()
        ps.is_diff                 = True
        ps.world.collision_objects = [co]
        self._pub.publish(ps)
        self.get_logger().debug('Desk published to /planning_scene')


def main():
    rclpy.init()
    node = ScenePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
