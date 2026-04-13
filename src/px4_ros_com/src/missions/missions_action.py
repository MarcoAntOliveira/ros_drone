 #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.action import ExecuteMission
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
import imutils
import numpy as np
import cv2
class MissionAction(Node):

	def __init__(self):
		super().__init__('mission_action')
		self.camera = cv2.VideoCapture(2)  # Acessa a câmera especificada na requisição

		if not self.camera.isOpened():
			self.get_logger().error("Erro ao abrir câmera")
			exit()

		self.timer = self.create_timer(0.03, self.process_frame)

		self._action_server = ActionServer(
			self,
			ExecuteMission,
			'execute_mission',
			self.execute_callback,
			goal_callback=self.goal_callback,
			)
	def goal_callback(self, goal_request: ExecuteMission.Goal):
		self.get_logger().info("Received a goal")

		# Policy: refuse new goal if current goal still active
		# with self.goal_lock_:
		#     if self.goal_handle_ is not None and self.goal_handle_.is_active:
		#         self.get_logger().info("A goal is already active, rejecting new goal")
		#         return GoalResponse.REJECT

		# Validate the goal request
		if goal_request.target_number <= 0:
			self.get_logger().info("Rejecting the goal")
			return GoalResponse.REJECT

			# Policy: preempt existing goal when receiving new goal
			# with self.goal_lock_:
			#     if self.goal_handle_ is not None and self.goal_handle_.is_active:
			#         self.get_logger().info("Abort current goal and accept new goal")
			#         self.goal_handle_.abort()

		self.get_logger().info("Accepting the goal")
		return GoalResponse.ACCEPT

	async def execute_callback(self, goal_handle):

		self.takeoff()
		self.search_aruco()
		aruco_id = self.detect_id()

		target = self.decide_target(aruco_id)

		self.go_to_target(target)
		self.land()

		goal_handle.succeed()

		return ExecuteMission.Result(success=True, message="Mission complete")

	def search_aruco(self):
		# Implementar a lógica para procurar o ArUco
		pass
def main(args=None):
    rclpy.init(args=args)
    node = MissionAction()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
