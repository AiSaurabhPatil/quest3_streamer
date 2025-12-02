import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
import time

class QuestROSNode(Node):
    def __init__(self):
        super().__init__('quest_openxr_streamer')
        
        # Publishers
        self.pub_left_pose = self.create_publisher(PoseStamped, '/quest/left_hand/pose', 10)
        self.pub_right_pose = self.create_publisher(PoseStamped, '/quest/right_hand/pose', 10)
        self.pub_left_input = self.create_publisher(Joy, '/quest/left_hand/inputs', 10)
        self.pub_right_input = self.create_publisher(Joy, '/quest/right_hand/inputs', 10)
        
        self.get_logger().info("Quest ROS Node Initialized")

    def publish_data(self, left_pose_data, right_pose_data, left_inputs, right_inputs):
        timestamp = self.get_clock().now().to_msg()
        
        # Publish Poses
        if left_pose_data:
            msg = self._to_ros_pose(left_pose_data, timestamp, "quest_stage")
            self.pub_left_pose.publish(msg)
            
        if right_pose_data:
            msg = self._to_ros_pose(right_pose_data, timestamp, "quest_stage")
            self.pub_right_pose.publish(msg)
            
        # Publish Inputs
        if left_inputs:
            msg = self._to_ros_joy(left_inputs, timestamp)
            self.pub_left_input.publish(msg)
            
        if right_inputs:
            msg = self._to_ros_joy(right_inputs, timestamp)
            self.pub_right_input.publish(msg)

    def _to_ros_pose(self, xr_pose_dict, timestamp, frame_id):
        # xr_pose_dict is expected to be {'position': {'x':..., 'y':..., 'z':...}, 'orientation': {'x':..., 'y':..., 'z':..., 'w':...}}
        # or a direct object. Let's assume we pass the OpenXR pose object or a dict.
        # For modularity, let's assume we extract the values before calling this, or handle the object.
        # In input_manager.py, we print the values. We should probably return them.
        
        msg = PoseStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id
        
        msg.pose.position.x = float(xr_pose_dict['position'].x)
        msg.pose.position.y = float(xr_pose_dict['position'].y)
        msg.pose.position.z = float(xr_pose_dict['position'].z)
        
        msg.pose.orientation.x = float(xr_pose_dict['orientation'].x)
        msg.pose.orientation.y = float(xr_pose_dict['orientation'].y)
        msg.pose.orientation.z = float(xr_pose_dict['orientation'].z)
        msg.pose.orientation.w = float(xr_pose_dict['orientation'].w)
        
        return msg

    def _to_ros_joy(self, input_dict, timestamp):
        msg = Joy()
        msg.header.stamp = timestamp
        
        # Map inputs to Joy axes and buttons
        # This mapping is arbitrary and should be documented
        # Axes: [Trigger, Squeeze, StickX, StickY]
        # Buttons: [A/X, B/Y, Menu, StickClick]
        
        msg.axes = [
            float(input_dict.get('trigger', 0.0)),
            float(input_dict.get('squeeze', 0.0)),
            float(input_dict.get('thumbstick_x', 0.0)),
            float(input_dict.get('thumbstick_y', 0.0))
        ]
        
        msg.buttons = [
            int(input_dict.get('button_a_x', False)),
            int(input_dict.get('button_b_y', False)),
            int(input_dict.get('menu', False)),
            int(input_dict.get('thumbstick_click', False))
        ]
        
        return msg
