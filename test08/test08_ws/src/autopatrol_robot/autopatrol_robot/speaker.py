import rclpy
from rclpy.node import Node
from autopatrol_interfaces.srv import SpeachText
import espeaking

class  Speaker(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.speech_service=self.create_service(
            SpeachText,"speech_text",self.speak_text_callback
        )
        self.speaker=espeaking.Speaker()
        self.speaker.voice='zh'
    def speaker_text_callback(self,request,response):
        self.get_logger().info('正在朗读%s'% request.text)
        self.speaker.say(request.text)
        self.speaker.wait()
        response.result=True
        return response

def main(args=Node):
    rclpy.init(args=args)
    node=Speaker('speaker')
    rclpy.spin(node)
    rclpy.shutdown()