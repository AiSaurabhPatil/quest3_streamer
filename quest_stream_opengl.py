import xr
import ctypes
import time
import traceback
import graphics
import openxr_core
import input_manager
import rclpy
import ros_interface

class QuestStreamer:
    def __init__(self):
        self.window = graphics.init_glfw()
        self.instance = openxr_core.create_openxr_instance()
        self.system_id = openxr_core.get_openxr_system(self.instance)
        
        self.session = None
        self.graphics_binding = None
        self.input_mgr = None
        self.session_running = False
        self.session_focused = False
        
        # ROS Node
        rclpy.init()
        self.ros_node = ros_interface.QuestROSNode()

    def init_openxr_session(self):
        # Graphics Requirements
        reqs = xr.get_opengl_graphics_requirements_khr(self.instance, self.system_id)
        print(f"OpenGL Requirements: Min {reqs.min_api_version_supported} Max {reqs.max_api_version_supported}")

        # Create Graphics Binding
        self.graphics_binding = graphics.create_graphics_binding(graphics.libglfw, graphics.libGL)

        # Create Session
        self.session = xr.create_session(
            self.instance,
            xr.SessionCreateInfo(
                system_id=self.system_id,
                next=ctypes.cast(ctypes.byref(self.graphics_binding), ctypes.c_void_p),
            )
        )

    def setup_actions(self):
        self.input_mgr = input_manager.InputManager(self.instance, self.session)
        self.input_mgr.setup_actions()

    def setup_spaces(self):
        self.input_mgr.setup_spaces()

    def poll_events(self):
        try:
            event_buffer = xr.poll_event(self.instance)
            while event_buffer is not None:
                if event_buffer.type == xr.StructureType.EVENT_DATA_SESSION_STATE_CHANGED:
                    event = ctypes.cast(ctypes.byref(event_buffer), ctypes.POINTER(xr.EventDataSessionStateChanged)).contents
                    if event.session == self.session:
                        self._handle_session_state_change(event.state)
                event_buffer = xr.poll_event(self.instance)
        except xr.EventUnavailable:
            pass

    def _handle_session_state_change(self, state):
        if state == xr.SessionState.READY:
            xr.begin_session(self.session, xr.SessionBeginInfo(primary_view_configuration_type=xr.ViewConfigurationType.PRIMARY_STEREO))
            self.session_running = True
            print("Session Running!")
        elif state == xr.SessionState.STOPPING:
            xr.end_session(self.session)
            self.session_running = False
            print("Session Stopped.")
        elif state == xr.SessionState.EXITING:
            print("Exiting...")
            self.session_running = False
        elif state == xr.SessionState.FOCUSED:
            self.session_focused = True
            print("Session Focused!")
        elif state == xr.SessionState.VISIBLE:
            self.session_focused = False
            print("Session Visible (Lost Focus)")

    def run(self):
        print("Starting loop...")
        while True:
            graphics.glfw.poll_events()
            graphics.glfw.swap_buffers(self.window)
            if graphics.glfw.window_should_close(self.window):
                break

            self.poll_events()

            if not self.session_running:
                time.sleep(0.1)
                continue

            frame_state = xr.wait_frame(self.session, xr.FrameWaitInfo())
            xr.begin_frame(self.session, xr.FrameBeginInfo())

            if self.session_focused:
                try:
                    self.input_mgr.sync_actions()
                except xr.ResultError as e:
                    print(f"Sync Actions Failed: {e}")
                
                display_time = frame_state.predicted_display_time
                
                # Get Data
                left_aim_data = self.input_mgr.get_pose_data(self.input_mgr.left_aim_space, display_time)
                right_aim_data = self.input_mgr.get_pose_data(self.input_mgr.right_aim_space, display_time)
                
                # Publish to ROS
                self.ros_node.publish_data(left_aim_data, right_aim_data, {}, {})
                rclpy.spin_once(self.ros_node, timeout_sec=0)

                self.input_mgr.print_pose("Left Aim ", self.input_mgr.left_aim_space, display_time)
                self.input_mgr.print_pose("Right Aim", self.input_mgr.right_aim_space, display_time)
                self.input_mgr.print_pose("Left Grip", self.input_mgr.left_grip_space, display_time)
                self.input_mgr.print_pose("Right Grip", self.input_mgr.right_grip_space, display_time)
            else:
                print("Waiting for session focus...")

            xr.end_frame(
                self.session,
                xr.FrameEndInfo(
                    display_time=frame_state.predicted_display_time,
                    environment_blend_mode=xr.EnvironmentBlendMode.OPAQUE,
                    layers=[]
                )
            )

    def cleanup(self):
        if self.session:
            xr.destroy_session(self.session)
        if self.instance:
            xr.destroy_instance(self.instance)
        if self.window:
            graphics.glfw.terminate()
        if rclpy.ok():
            self.ros_node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    streamer = QuestStreamer()
    try:
        streamer.init_openxr_session()
        streamer.setup_actions()
        streamer.setup_spaces()
        streamer.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        traceback.print_exc()
        print(f"An error occurred: {e}")
    finally:
        streamer.cleanup()
