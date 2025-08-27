import grpc
import socket
import threading
import numpy as np
from concurrent import futures
from scipy.spatial.transform import Rotation as R
from . import pose_telemetry_pb2, pose_telemetry_pb2_grpc


class PoseTelemetryService(pose_telemetry_pb2_grpc.PoseTelemetryServicer):
    def __init__(self):
        self._latest_pose = None                # single shared slot
        self._pose_lock = threading.Lock()      # protect access to it
        self._new_pose_evt = threading.Event()  # signal consumer

        # Calculate the transformation matrix to align phone coordinate system with robot end-effector coordinate system
        # Original phone coordinate system: screen out is +Y, right is +X, down is +Z
        # Desired phone coordinate system: screen in is +Z, left is +Y, down is +X
        # This corresponds to a rotation of 90 degrees around X axis, then -90 degrees around Y axis
        rotation_x = R.from_euler('x', 90, degrees=True)
        rotation_y = R.from_euler('y', -90, degrees=True)
        rotation_z = R.from_euler('z', 0, degrees=True)
        R_phone_new = (rotation_z * rotation_y * rotation_x).as_matrix()
        self.T_phone_new = np.eye(4)
        self.T_phone_new[0:3, 0:3] = R_phone_new

    def StreamPoses(self, request_iterator, context):
        """Handle streaming poses from client"""
        print("Device connected to gRPC streaming")
        try:
            for pose_data in request_iterator:

                # Extract data
                translation = list(pose_data.translation)
                rotation = list(pose_data.rotation)
                is_open = pose_data.gripper_open
                stream_on = pose_data.start_stream
                precision_mode = pose_data.precision_mode

                # Reorder quaternion from ARCore [qx,qy,qz,qw] to [qw,qx,qy,qz]
                if len(rotation) == 4:
                    wxyz = [rotation[3], rotation[1], rotation[2], rotation[0]]
                else:
                    wxyz = rotation

                # Reorder position if needed (keeping your original logic)
                if len(translation) == 3:
                    pos = [translation[1], translation[2], translation[0]]
                else:
                    pos = translation

                # T_base_phone
                original_rot = R.from_quat(wxyz, scalar_first=True)
                T_base_phone = np.eye(4)
                T_base_phone[0:3, 3] = pos
                T_base_phone[0:3, 0:3] = original_rot.as_matrix()

                # T_base_new
                T_base_new = T_base_phone @ self.T_phone_new
                rotated_pos = T_base_new[0:3, 3].tolist()
                rotated_rot = R.from_matrix(T_base_new[0:3, 0:3])

                # Update position and rotation
                pos = rotated_pos
                wxyz = rotated_rot.as_quat(scalar_first=True).tolist()

                # Store in queue for processing
                pose_data_processed = {
                    "position": pos,
                    "rotation": wxyz,
                    "gripper_open": is_open,
                    "switch": stream_on,
                    "precision": precision_mode,
                }

                # atomically overwrite the shared slot
                with self._pose_lock:
                    self._latest_pose = pose_data_processed
                    # wake up any waiting consumer
                    self._new_pose_evt.set()

            return pose_telemetry_pb2.PoseResponse(
                success=True, message="Poses received successfully"
            )

        except Exception as e:
            return pose_telemetry_pb2.PoseResponse(
                success=False, message=f"Error: {str(e)}"
            )

    def get_latest_pose(self, block: bool = True, timeout: float = None):
        """
        Wait for a new pose (optionally), then return the most recent one.
        If block=False, returns immediately (or None if none seen yet).
        """
        if block:
            got = self._new_pose_evt.wait(timeout)
            if not got:
                return None
            # clear the flag so next wait blocks until another set()
            self._new_pose_evt.clear()

        with self._pose_lock:
            return self._latest_pose


def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

def start_grpc_server(host="0.0.0.0", port=8765):
    IPAddr = get_ip()
    print("*"*60)
    print("* Open the Daxie app.")
    print("* Enter {0}:{1} in the 'Server settings'.".format(IPAddr, port))
    print("*"*60)
    print()
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    pose_service = PoseTelemetryService()
    pose_telemetry_pb2_grpc.add_PoseTelemetryServicer_to_server(pose_service, server)
    server.add_insecure_port(f"{host}:{port}")
    server.start()
    return server, pose_service