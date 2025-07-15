#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.executors import MultiThreadedExecutor
from coordinate_follower.action import MoveToPos

class MoveToPosClientNode(Node):
    def __init__(self, coord):
        super().__init__("move_to_pos_client")
        self.get_logger().info("MoveToPosClientNode initialized.")
        self.move_to_pos_client = ActionClient(self, MoveToPos, "move_to_pos")
        self.coord = coord
        self._result_future = None
        self.get_logger().info("Client node ready.")

    def send_goal(self, target_x, target_y):
        self.get_logger().info(f"Sending goal: x={target_x}, y={target_y}")
        self.target_x = target_x
        self.target_y = target_y

        self.get_logger().info("Waiting for server...")
        if not self.move_to_pos_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available after waiting. Aborting goal send.")
            return
        self.get_logger().info("Server available, creating goal.")

        goal = MoveToPos.Goal()
        goal.x = target_x
        goal.y = target_y

        try:
            self.get_logger().info("Calling send_goal_async.")
            future = self.move_to_pos_client.send_goal_async(
                goal, feedback_callback=self.feedback_callback
            )
            self.get_logger().info("Adding goal_response_callback.")
            future.add_done_callback(self.goal_response_callback)
        except Exception as e:
            self.get_logger().error(f"Exception during send_goal_async: {e}")

    def feedback_callback(self, feedback_msg):
        pass  # No logging in feedback callback

    def goal_response_callback(self, future):
        self.get_logger().info("Entered goal_response_callback.")
        try:
            self.goal_handle: ClientGoalHandle = future.result()
            self.get_logger().info("goal_handle acquired.")
        except Exception as e:
            self.get_logger().error(f"Exception retrieving goal_handle from future: {e}")
            return

        if self.goal_handle is None:
            self.get_logger().error("goal_handle is None! Goal may not have been sent.")
            return

        if self.goal_handle.accepted:
            self.get_logger().info("Goal accepted by server.")
            try:
                self.get_logger().info("Calling get_result_async.")
                self._result_future = self.goal_handle.get_result_async()
                self.get_logger().info("Adding goal_result_callback.")
                self._result_future.add_done_callback(self.goal_result_callback)

            except Exception as e:
                self.get_logger().error(f"Exception during get_result_async: {e}")
        else:
            self.get_logger().warn("Goal rejected by server.")

    def goal_result_callback(self, future):
        self.get_logger().info("Entered goal_result_callback.")
        try:
            result = future.result().result
            self.get_logger().info(f"Result received: {result}")
        except Exception as e:
            self.get_logger().error(f"Exception retrieving result from future: {e}")
            return

        if result is None:
            self.get_logger().error("Result is None! Something went wrong in the server or communication.")
            return

        if getattr(result, "success", False):
            self.get_logger().info("Robot reached the goal.")

            if len(self.coord) > 0:
                next_x, next_y = self.coord.pop()
                self.get_logger().info(f"Next goal: x={next_x}, y={next_y}")
                timer = self.create_timer(0.5, lambda: self._send_and_cancel(timer, next_x, next_y))

            else:
                self.get_logger().info("All goals completed.")

        else:
            self.get_logger().warn("Goal failed, retrying same goal.")
            timer = self.create_timer(0.5, lambda: self._send_and_cancel(timer, self.target_x, self.target_y))


    def _send_and_cancel(self, timer, x, y):
        self.get_logger().info(f"_send_and_cancel: Canceling timer and sending goal x={x}, y={y}")
        timer.cancel()
        self.send_goal(x, y)

def main(args=None):
    rclpy.init(args=args)
    print("rclpy initialized.")
    try:
        with open("/home/asvin/projectKratos/src/coordinate_follower/scripts/coordinates.txt", 'r') as f:
            coord = []

            for line in f:
                try:
                    parts = line.strip().split()
                    if len(parts) == 2:
                        x, y = float(parts[0]), float(parts[1])
                        coord.append((x, y))
                except Exception as e:
                    print(f"Error parsing line '{line.strip()}': {e}")
        print("Coordinates loaded.")
        coord.reverse()

        if not coord:
            print("No coordinates to follow. Exiting.")
            return
        node = MoveToPosClientNode(coord[:-1])
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        node.get_logger().info("Starting first goal send.")
        node.send_goal(*coord.pop())
        try:
            executor.spin()

        except KeyboardInterrupt:
            node.get_logger().info("KeyboardInterrupt received, shutting down.")

        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
            node.get_logger().info("Shutdown complete.")
            
    except Exception as e:
        print(f"Exception during client main: {e}")

if __name__ == "__main__":
    main()