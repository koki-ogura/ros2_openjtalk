import time
import numpy as np
import pyaudio
import pyopenjtalk
import rclpy
from rclpy.service import Service
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from ros2_openjtalk_interfaces.srv import Talk

#---------------------------------------------------------------------------------------
class BaseTalk():
    def __init__(self):
        self._data = None
        self._length = 0
        self._top = 0
        self._p = pyaudio.PyAudio()

    def __del__(self):
        self._p.terminate()

    def callback(self, in_data, frame_count, time_info, status):
        len = self._length
        if len > frame_count:
            len = frame_count
        cdata = self._data[self._top:self._top+len]
        self._length -= len
        self._top += len
        return (cdata, pyaudio.paContinue)

    def text2wav16(self, text):
        return (None, None)

    def talk(self, text):
        self._data, sr = self.text2wav16(text)
        self._length = len(self._data)
        self._top = 0
        stream = self._p.open(format=pyaudio.paInt16,
                              channels=1,
                              rate=sr,
                              output=True,
                              stream_callback=self.callback)
        while stream.is_active():
            time.sleep(0.1)
        self._data = None
        stream.close()

class OpenJTalk(BaseTalk):
    def text2wav16(self, text):
        x, sr = pyopenjtalk.tts(text)
        return (x.astype(np.int16), sr)

#---------------------------------------------------------------------------------------
class OpenJTalkNode(Node):
    def __init__(self, **kwargs):
        self._talker: Optional[OpenJtalk] = None
        self._talk_srv: Optional[Service] = None
        super().__init__('ros2_openjtalk_node', **kwargs)

    def setup_params(self):
        self.declare_parameter('device_name', 'default')
        self._device_name = self.get_parameter('device_name').value
        self.get_logger().info(f'params: device_name={self._device_name}')

    #-----------------------------------------------------------------------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('lifecycle:on_configure()')
        self._talker = OpenJTalk()
        self._talk_srv = self.create_service(Talk, '~/talk', self.talk_callback)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('lifecycle:on_activate()')
        return super().on_activate(state)
            
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('lifecycle:on_deactivate()')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('lifecycle:on_cleanup()')
        self.manual_shutdown()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('lifecycle:on_shutdown()')
        self.manual_shutdown()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('lifecycle:on_error()')
        self.manual_shutdown()
        return TransitionCallbackReturn.SUCCESS

    #-----------------------------------------------------------------------------
    def manual_shutdown(self):
        self._talk_srv.destroy()
        self._talker = None

    #-----------------------------------------------------------------------------
    def talk_callback(self, request, response):
        msg = request.message
        self.get_logger().info(f'talk({msg})')
        self._talker.talk(msg)
        response.success = True
        return response

#---------------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node = OpenJTalkNode()
    executor.add_node(node)
    node.setup_params()
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.manual_shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()
