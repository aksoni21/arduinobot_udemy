#!/usr/bin/env python3
import signal
import threading
from ask_sdk_core.dispatch_components import AbstractExceptionHandler
from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from arduino_bot_msgs.action import ArduinobotTask

app = Flask(__name__)
ros_node = None
action_client = None

class ROSNode(Node):
    def __init__(self):
        super().__init__("alexa_interface")
        self._action_client = ActionClient(self, ArduinobotTask, "task_server")

    def send_goal(self, task_number):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available")
            return

        goal_msg = ArduinobotTask.Goal()
        goal_msg.task_number = task_number

        self.get_logger().info(f"Sending goal: task_number={task_number}")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result received: {result}")

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        speech_text = "Hi, how can I help you?"
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)).set_should_end_session(False)
        
        ros_node.send_goal(0)
        return handler_input.response_builder.response

class PickIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("PickIntent")(handler_input)

    def handle(self, handler_input):
        speech_text = "Ok, I will pick up"
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(True)
        
        ros_node.send_goal(1)
        return handler_input.response_builder.response

class SleepIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("SleepIntent")(handler_input)

    def handle(self, handler_input):
        speech_text = "Sleeping ... zzz"
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Sleep", speech_text)).set_should_end_session(True)
        
        ros_node.send_goal(2)
        return handler_input.response_builder.response

class WakeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("WakeIntent")(handler_input)

    def handle(self, handler_input):
        speech_text = "Waking up ... aaa"
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Wake", speech_text)).set_should_end_session(True)
        
        ros_node.send_goal(0)
        return handler_input.response_builder.response

class AllExceptionHandler(AbstractExceptionHandler):
    def can_handle(self, handler_input, exception):
        return True

    def handle(self, handler_input, exception):
        print(exception)
        speech = "Sorry, I didn't get it. Can you please say it again!!"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response

def init_ros():
    global ros_node
    rclpy.init()
    ros_node = ROSNode()
    rclpy.spin(ros_node)

def shutdown_ros(signum, frame):
    rclpy.shutdown()
    threading.current_thread().join()

skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PickIntentHandler())
skill_builder.add_request_handler(SleepIntentHandler())
skill_builder.add_request_handler(WakeIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())

skill_adapter = SkillAdapter(
    skill=skill_builder.create(), skill_id="amzn1.ask.skill.ed24b99d-cbe2-4e54-9e60-030e2342c266", app=app)

@app.route("/")
def invoke_skill():
    return skill_adapter.dispatch_request()

skill_adapter.register(app=app, route="/")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, shutdown_ros)
    signal.signal(signal.SIGTERM, shutdown_ros)
    threading.Thread(target=init_ros).start()
    app.run()
