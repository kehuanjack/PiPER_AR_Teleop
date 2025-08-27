import numpy as np
from scipy.spatial.transform import Rotation as R
from .pos_grpc_server import start_grpc_server


class Teleop:
    """Teleoperation class to map phone movements to robot end-effector movements.

        Args:
            init_end_pose: [x,y,z,roll,pitch,yaw] in robot frame, xyz(m) rpy(deg)

    """
    def __init__(self, init_end_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) -> None:
        self.INITIAL_POS_ROBOT = np.array(init_end_pose[0:3], float)
        self.INITIAL_WXYZ_ROBOT = R.from_euler('xyz', init_end_pose[3:6], degrees=True).as_quat(scalar_first=True)

        self.translation_sensitivity = 1.0
        self.rotation_sensitivity = 1.0

        self.start_teleop = False
        self._phone_connected = False

        self.current_t_R: np.ndarray = self.INITIAL_POS_ROBOT
        self.current_q_R: np.ndarray = self.INITIAL_WXYZ_ROBOT

        self._start_grpc_server()

    def _start_grpc_server(self) -> None:
        """Start the gRPC server for phone pose streaming."""
        self.server, self.pose_service = start_grpc_server()

    def _open_phone_connection(self) -> tuple[np.ndarray, np.ndarray, bool]:
        """Wait for phone to connect and set initial mapping."""
        self.current_t_R: np.ndarray = self.INITIAL_POS_ROBOT
        self.current_q_R: np.ndarray = self.INITIAL_WXYZ_ROBOT
        init_rot_robot = R.from_quat(self.current_q_R, scalar_first=True)

        while not self.start_teleop:
            data = self.pose_service.get_latest_pose(block=True, timeout=1.0)
            if data is not None:
                self.start_teleop = data["switch"]

        pos, quat, gripper = data["position"], data["rotation"], data["gripper_open"]
        initial_rot_phone = R.from_quat(quat, scalar_first=True)
        initial_pos_phone = np.array(pos)

        self.initial_phone_quat = quat.copy()
        self.initial_phone_pos = initial_pos_phone.copy()

        quat_RP = init_rot_robot * initial_rot_phone.inv()
        translation_RP = self.current_t_R - quat_RP.apply(initial_pos_phone)
        return quat_RP, translation_RP, gripper

    def _map_phone_to_robot(
        self, phone_pos: np.ndarray, phone_quat: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Map phone translation and rotation to robot's coordinate frame."""
        phone_pos = np.array(phone_pos, float)
        phone_quat = np.array(phone_quat, float)

        # Translate
        delta = (phone_pos - self.initial_phone_pos) * self.translation_sensitivity
        scaled_pos = self.initial_phone_pos + delta

        # Rotate
        init_rot = R.from_quat(self.initial_phone_quat, scalar_first=True)
        curr_rot = R.from_quat(phone_quat, scalar_first=True)
        relative_rot = init_rot.inv() * curr_rot
        rotvec = relative_rot.as_rotvec() * self.rotation_sensitivity
        scaled_rot = R.from_rotvec(rotvec)
        quat_scaled = init_rot * scaled_rot

        # Apply mapping
        quat_robot = self.quat_RP * quat_scaled
        self.current_q_R = quat_robot.as_quat(scalar_first=True)
        self.current_t_R = self.quat_RP.apply(scaled_pos) + self.translation_RP
        return self.current_t_R, self.current_q_R

    def send_to_robot(self):
        """Process phone input, and send commands to the robot."""
        if not self._phone_connected:
            self.quat_RP, self.translation_RP, gripper = self._open_phone_connection()
            self._phone_connected = True

        # Map phone pose to robot pose
        data = self.pose_service.get_latest_pose(block=False)
        pos, quat, gripper = data["position"], data["rotation"], data["gripper_open"]
        t_robot, q_robot = self._map_phone_to_robot(pos, quat)

        self.start_teleop = data["switch"]
        if not self.start_teleop:
            self._phone_connected = False

        return t_robot, q_robot, gripper
