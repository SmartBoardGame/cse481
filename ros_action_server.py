import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse

class WasteDisposal(Node):
    def __init__(self):
        super().__init__('waste_disposal')

        # Action Server
        self._action_server = ActionServer(
            self,
            None, # Placeholder Action Type
            'task_execution',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback
        )
        self.get_logger().info("Action server started.")

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        """route incoming goals to correct handler"""
        task_type = goal_handle.request.task_type
        # navigation, extraction, etc.
        handler = getattr(self, f"execute_{task_type}", None)
        
        if not handler:
            self.get_logger().error(f"Unknown task type: {task_type}")
            goal_handle.abort()
            return None
        
        handler(goal_handle)
        return None # should return result
    
    def execute_navigation(self, goal_handle):
        self.get_logger().info("Executing navigation task ...")


        goal_handle.succeed()
        return None

    def execute_extraction(self, goal_handle):
        self.get_logger().info("Executing extraction task...")


        goal_handle.succeed()
        return None

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WasteDisposal())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
