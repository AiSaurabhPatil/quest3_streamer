import xr
import ctypes

class InputManager:
    def __init__(self, instance, session):
        self.instance = instance
        self.session = session
        self.action_set = None
        self.action_aim_pose = None
        self.action_grip_pose = None
        
        self.ref_space = None
        self.left_aim_space = None
        self.right_aim_space = None
        self.left_grip_space = None
        self.right_grip_space = None
        
        # Get xrLocateSpace function pointer
        self.xrLocateSpace = ctypes.cast(
            xr.get_instance_proc_addr(self.instance, "xrLocateSpace"),
            xr.PFN_xrLocateSpace
        )

    def setup_actions(self):
        self.action_set = xr.create_action_set(
            self.instance,
            xr.ActionSetCreateInfo(
                action_set_name="controller_data",
                localized_action_set_name="Controller Data",
                priority=1,
            )
        )
        
        # Create Actions
        self.action_aim_pose = xr.create_action(
            self.action_set,
            xr.ActionCreateInfo(
                action_type=xr.ActionType.POSE_INPUT,
                action_name="aim_pose",
                localized_action_name="Aim Pose",
                count_subaction_paths=2,
                subaction_paths=[
                    xr.string_to_path(self.instance, "/user/hand/left"),
                    xr.string_to_path(self.instance, "/user/hand/right"),
                ],
            )
        )
        
        self.action_grip_pose = xr.create_action(
            self.action_set,
            xr.ActionCreateInfo(
                action_type=xr.ActionType.POSE_INPUT,
                action_name="grip_pose",
                localized_action_name="Grip Pose",
                count_subaction_paths=2,
                subaction_paths=[
                    xr.string_to_path(self.instance, "/user/hand/left"),
                    xr.string_to_path(self.instance, "/user/hand/right"),
                ],
            )
        )

        # Suggest Bindings
        self._suggest_bindings()

        # Attach Action Set
        xr.attach_session_action_sets(
            self.session,
            xr.SessionActionSetsAttachInfo(action_sets=[self.action_set])
        )

    def _suggest_bindings(self):
        # Oculus Touch
        profile_path = xr.string_to_path(self.instance, "/interaction_profiles/oculus/touch_controller")
        xr.suggest_interaction_profile_bindings(
            self.instance,
            xr.InteractionProfileSuggestedBinding(
                interaction_profile=profile_path,
                suggested_bindings=[
                    xr.ActionSuggestedBinding(action=self.action_aim_pose, binding=xr.string_to_path(self.instance, "/user/hand/left/input/aim/pose")),
                    xr.ActionSuggestedBinding(action=self.action_aim_pose, binding=xr.string_to_path(self.instance, "/user/hand/right/input/aim/pose")),
                    xr.ActionSuggestedBinding(action=self.action_grip_pose, binding=xr.string_to_path(self.instance, "/user/hand/left/input/grip/pose")),
                    xr.ActionSuggestedBinding(action=self.action_grip_pose, binding=xr.string_to_path(self.instance, "/user/hand/right/input/grip/pose")),
                ],
            )
        )

    def setup_spaces(self):
        self.ref_space = xr.create_reference_space(
            self.session,
            xr.ReferenceSpaceCreateInfo(
                reference_space_type=xr.ReferenceSpaceType.VIEW,
                pose_in_reference_space=xr.Posef(),
            )
        )
        
        left_hand = xr.string_to_path(self.instance, "/user/hand/left")
        right_hand = xr.string_to_path(self.instance, "/user/hand/right")
        
        self.left_aim_space = xr.create_action_space(self.session, xr.ActionSpaceCreateInfo(action=self.action_aim_pose, subaction_path=left_hand))
        self.right_aim_space = xr.create_action_space(self.session, xr.ActionSpaceCreateInfo(action=self.action_aim_pose, subaction_path=right_hand))
        self.left_grip_space = xr.create_action_space(self.session, xr.ActionSpaceCreateInfo(action=self.action_grip_pose, subaction_path=left_hand))
        self.right_grip_space = xr.create_action_space(self.session, xr.ActionSpaceCreateInfo(action=self.action_grip_pose, subaction_path=right_hand))

    def sync_actions(self):
        active_action_set = xr.ActiveActionSet(action_set=self.action_set, subaction_path=xr.NULL_PATH)
        xr.sync_actions(self.session, xr.ActionsSyncInfo(active_action_sets=[active_action_set]))

    def print_pose(self, name, space, display_time):
        velocity = xr.SpaceVelocity(type=xr.StructureType.SPACE_VELOCITY)
        location = xr.SpaceLocation(
            type=xr.StructureType.SPACE_LOCATION,
            next=ctypes.cast(ctypes.byref(velocity), ctypes.c_void_p)
        )
        
        result = self.xrLocateSpace(space, self.ref_space, display_time, ctypes.byref(location))
        
        if result != xr.Result.SUCCESS:
            print(f"{name} Locate Failed: {result}")
            return

        if location.location_flags & xr.SPACE_LOCATION_POSITION_VALID_BIT:
            pos = location.pose.position
            rot = location.pose.orientation
            print(f"{name} Pos: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) "
                  f"Rot: ({rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f})")
            
            if velocity.velocity_flags & xr.SPACE_VELOCITY_LINEAR_VALID_BIT:
                lin_vel = velocity.linear_velocity
                print(f"    Lin Vel: ({lin_vel.x:.3f}, {lin_vel.y:.3f}, {lin_vel.z:.3f})")
            
    def get_pose_data(self, space, display_time):
        velocity = xr.SpaceVelocity(type=xr.StructureType.SPACE_VELOCITY)
        location = xr.SpaceLocation(
            type=xr.StructureType.SPACE_LOCATION,
            next=ctypes.cast(ctypes.byref(velocity), ctypes.c_void_p)
        )
        
        result = self.xrLocateSpace(space, self.ref_space, display_time, ctypes.byref(location))
        
        if result != xr.Result.SUCCESS:
            return None

        data = {}
        if location.location_flags & xr.SPACE_LOCATION_POSITION_VALID_BIT:
            data['position'] = location.pose.position
            data['orientation'] = location.pose.orientation
        
        # We could also return velocity if needed, but let's stick to pose for now or add it to the dict
        # The ROS interface expects 'position' and 'orientation' keys with objects having x,y,z attributes
        
        return data if data else None
