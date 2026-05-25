#!/usr/bin/env python3
"""
command_server.py

다른 컴퓨터에서 TCP로 JSON 명령을 받아 /scenario_trigger 토픽으로 변환한다.

수신 JSON 예시:
  {"command": "package_start"}    → 시나리오 1
  {"command": "package_complete"} → 시나리오 2

송신 응답:
  {"status": "ok", "scenario": 1}
  {"status": "error", "message": "..."}

다른 컴퓨터에서 보내는 예시 (Python):
  import socket, json
  with socket.socket() as s:
      s.connect(('로봇PC_IP', 8765))
      s.sendall(json.dumps({"command": "package_start"}).encode())
      print(s.recv(1024).decode())
"""

import json
import socket
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

HOST = '0.0.0.0'
PORT = 8765


def normalize_command(data_text: str) -> str:
    """JSON 또는 plain text에서 명령어 문자열을 추출한다."""
    data_text = data_text.strip()
    try:
        data = json.loads(data_text)
        if isinstance(data, dict):
            if data.get('command'):
                return data['command'].strip().lower()
            if data.get('color') not in [None, '', 'none']:
                return data['color'].strip().lower()
            if data.get('raw_text'):
                return data['raw_text'].strip().lower()
            if data.get('mode'):
                return data['mode'].strip().lower()
    except json.JSONDecodeError:
        pass
    return data_text.lower()


class CommandServer(Node):
    def __init__(self):
        super().__init__('command_server')
        self._pub = self.create_publisher(Int32, '/scenario_trigger', 10)

        t = threading.Thread(target=self._run_server, daemon=True)
        t.start()
        self.get_logger().info(f'✅ Command server 시작 — {HOST}:{PORT} 대기 중')

    def _run_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((HOST, PORT))
            srv.listen(5)
            while True:
                conn, addr = srv.accept()
                threading.Thread(target=self._handle, args=(conn, addr), daemon=True).start()

    def _handle(self, conn, addr):
        with conn:
            try:
                data = b''
                while True:
                    chunk = conn.recv(1024)
                    if not chunk:
                        break
                    data += chunk
                    if b'}' in data or b'\n' in data:
                        break

                command = normalize_command(data.decode('utf-8'))
                self.get_logger().info(f'[{addr[0]}] 수신: {command}')

                if command == 'package_start':
                    self._pub.publish(Int32(data=1))
                    self.get_logger().info(f'[{addr[0]}] package_start → 시나리오 1 트리거')
                    conn.sendall(json.dumps({'status': 'ok', 'scenario': 1}).encode())

                elif command == 'package_complete':
                    self._pub.publish(Int32(data=2))
                    self.get_logger().info(f'[{addr[0]}] package_complete → 시나리오 2 트리거')
                    conn.sendall(json.dumps({'status': 'ok', 'scenario': 2}).encode())

                else:
                    self.get_logger().warn(f'[{addr[0]}] 알 수 없는 명령: {command}')
                    conn.sendall(json.dumps({'status': 'error', 'message': f'unknown command: {command}'}).encode())

            except Exception as e:
                self.get_logger().error(f'[{addr[0]}] 오류: {e}')
                try:
                    conn.sendall(json.dumps({'status': 'error', 'message': str(e)}).encode())
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = CommandServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
