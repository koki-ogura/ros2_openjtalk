import traceback
from typing import Optional
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros2_openjtalk_interfaces.srv import Talk

# ----------------------------------------------------------------------------------------------------------------------------
class OpenJTalkHelper(Node):
    def __init__(self):
        super().__init__('ros2_openjtalk_client')
        self._talk_cli = self.create_client(Talk, '/ros2_openjtalk_node/talk')

    #-----------------------------------------------------------------------------------------------
    def call_service(self, req, cli, name):
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().info(f'{name} timeout')
            raise TechmagicLotusGripperError(f'{name} timeout')
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        res = future.result()
        if (res is None) or (not res.success):
            self.get_logger().info(f'{name} rejected')
            raise TechmagicLotusGripperError(f'{name} rejected')
        return res

    def talk(self, message, speed):
        self.get_logger().info(f'talk({message}, {speed})')
        req = Talk.Request()
        req.message = message
        req.speed = speed
        res = self.call_service(req, self._talk_cli, 'talk')
        return res.success

# ----------------------------------------------------------------------------------------------------------------------------
def spin_sleep(executor: Optional[MultiThreadedExecutor], sec):
    future = executor._executor.submit(lambda: time.sleep(sec))
    executor.spin_until_future_complete(future)

# ----------------------------------------------------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    talker = OpenJTalkHelper()
    executor.add_node(talker)

    try:
        talker.talk('こんばんは', 0.5)
        spin_sleep(executor, 1.0)
        talker.talk('お元気ですか？', 1.5)
        spin_sleep(executor, 1.0)
        talker.destroy_node()

    except Exception as e:
        talker.get_logger().info(f'exception: {e}')
        talker.get_logger().error(traceback.format_exc(limit=0))

if __name__ == '__main__':
    main()
