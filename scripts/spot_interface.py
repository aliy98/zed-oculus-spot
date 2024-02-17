import logging
import time

import cv2
import numpy as np
from scipy import ndimage

import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
import bosdyn.client.util
from bosdyn import geometry
from bosdyn.api import geometry_pb2, trajectory_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.client import create_standard_sdk
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.time_sync import TimedOutError

LOGGER = logging.getLogger()
VELOCITY_CMD_DURATION = 0.6  # seconds
MAX_YAW = 0.25 # rad (14.3deg)
MAX_PITCH = 0.3 # rad (17.2deg)
MAX_ROLL = 0.1 # rad

VALUE_FOR_Q_KEYSTROKE = 113
VALUE_FOR_ESC_KEYSTROKE = 27

ROTATION_ANGLE = {
    'back_fisheye_image': 0,
    'frontleft_fisheye_image': -90,
    'frontright_fisheye_image': -90,
    'left_fisheye_image': 0,
    'right_fisheye_image': 180
}


class SpotInterface:
    def __init__(self):
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._z = 0.0
        self._v_x = 0.0
        self._v_y = 0.0
        self._v_rot = 0.0
        self._powered_on = False
        self._estop_keepalive = None
        sdk = create_standard_sdk("spot_interface")
        self._robot = sdk.create_robot('10.42.0.211')
        self._robot.authenticate('user', 'wruzvkg4rce4')
        self._robot.sync_with_directory()
        self._robot.time_sync.wait_for_sync()
        self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
        try:
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
            self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
        except:
            self._estop_client = None
            self._estop_endpoint = None
        self._power_client = self._robot.ensure_client(PowerClient.default_service_name)
        self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
        #self._image_client = self._robot.ensure_client(ImageClient.default_service_name)
        self._start()
        
    
    def _toggle_estop(self):
        """toggle estop on/off. Initial state is ON"""
        if self._estop_client is not None and self._estop_endpoint is not None:
            if not self._estop_keepalive:
                self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
            else:
                LOGGER.info('stopping estop')
                self._estop_keepalive.stop()
                self._estop_keepalive.shutdown()
                self._estop_keepalive = None
        
    def _orientation_cmd_helper(self, yaw=0.0, roll=0.0, pitch=0.0, height=0.0):
        orientation = geometry.EulerZXY(yaw, roll, pitch)
        cmd = RobotCommandBuilder.synchro_stand_command(body_height=height, footprint_R_body=orientation)
        self._robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def _velocity_cmd_helper(self, v_x=0.0, v_y=0.0, v_rot=0.0):
        mobility_params = self._set_mobility_params()
        cmd = RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot, params=mobility_params)
        self._robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + VELOCITY_CMD_DURATION)
        
    def _set_mobility_params(self):
        obstacles = spot_command_pb2.ObstacleParams(disable_vision_body_obstacle_avoidance=False,
                                                    disable_vision_foot_obstacle_avoidance=False,
                                                    disable_vision_foot_constraint_avoidance=False,
                                                    disable_vision_foot_obstacle_body_assist= False,
                                                    disable_vision_negative_obstacles=False,
                                                    obstacle_avoidance_padding=0.1)
        if self._v_x == 0 and self._v_y == 0 and self._v_rot == 0:
            footprint_R_body = geometry.EulerZXY(roll=self._roll, pitch=self._pitch, yaw=self._yaw)
        else:
            footprint_R_body = geometry.EulerZXY(roll=0.0, pitch=self._pitch, yaw=0.0)
        position = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
        rotation = footprint_R_body.to_quaternion()
        pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
        point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
        traj = trajectory_pb2.SE3Trajectory(points=[point])
        body_control=spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)
        speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(linear=Vec2(x=1.0, y=1.0), angular=0.7))
        mobility_params = spot_command_pb2.MobilityParams(obstacle_params=obstacles, vel_limit=speed_limit, body_control=body_control, locomotion_hint=spot_command_pb2.HINT_AUTO)
        return mobility_params
    
    def get_body_vel(self):
        robot_state = self._robot_state_client.get_robot_state()
        vis_vel_ang = robot_state.kinematic_state.velocity_of_body_in_vision.angular
        measures = [vis_vel_ang.z, vis_vel_ang.y, vis_vel_ang.x]
        return measures

    def set_controls(self, controls, dt):
        dyaw = controls[0] * dt
        dpitch = controls[1] * dt
        #droll = controls[2] * dt
 
        if abs(self._yaw + dyaw) < MAX_YAW:
            self._yaw = self._yaw + dyaw
        if abs(self._pitch + dpitch) < MAX_PITCH:
            self._pitch = self._pitch + dpitch
        # if abs(self._roll + droll) < MAX_ROLL:
        #     self._roll = self._roll + droll

        self._v_x = controls[3] 
        self._v_y = controls[4]
        self._v_rot = controls[5]
        
        self._velocity_cmd_helper(v_x=self._v_x, v_y=self._v_y, v_rot=self._v_rot)
        
    def _start(self):
        # self._image_sources = ['frontleft_fisheye_image', 'frontright_fisheye_image']
        # self._image_requests = [build_image_request(source, quality_percent=50, resize_ratio=1) for source in self._image_sources]
        # for image_source in self._image_sources:
        #     cv2.namedWindow(image_source, cv2.WINDOW_NORMAL)
        #     cv2.setWindowProperty(image_source, cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_AUTOSIZE)
        self._lease = self._lease_client.take()
        self._lease_keepalive = bosdyn.client.lease.LeaseKeepAlive(self._lease_client)
        if self._estop_endpoint is not None:
            self._estop_endpoint.force_simple_setup(
            )  # Set this endpoint as the robot's sole estop.
        self._toggle_estop()
        self._robot.power_on()
        self._powered_on=True
        time.sleep(5)
        blocking_stand(self._robot_command_client)
        time.sleep(5)
        LOGGER.info("ready to take commands")

    def shutdown(self):
        self._orientation_cmd_helper(roll=0.0, pitch=0.0, yaw=0.0)
        time.sleep(5)
        blocking_sit(self._robot_command_client)
        time.sleep(5)
        safe_power_off_cmd=RobotCommandBuilder.safe_power_off_command()
        self._robot_command_client.robot_command(command= safe_power_off_cmd)
        time.sleep(2.5)
        self._powered_on=False
        LOGGER.info('Shutting down SpotInterface.')
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        if self._lease_keepalive:
            self._lease_keepalive.shutdown()

    def _image_to_opencv(self, image, auto_rotate=True):
        """Convert an image proto message to an openCV image."""
        num_channels = 1  # Assume a default of 1 byte encodings.
        if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            dtype = np.uint16
            extension = '.png'
        else:
            dtype = np.uint8
            if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
                num_channels = 3
            elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
                num_channels = 4
            elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
                num_channels = 1
            elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16:
                num_channels = 1
                dtype = np.uint16
            extension = '.jpg'

        img = np.frombuffer(image.shot.image.data, dtype=dtype)
        if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
            try:
                # Attempt to reshape array into a RGB rows X cols shape.
                img = img.reshape((image.shot.image.rows, image.shot.image.cols, num_channels))
            except ValueError:
                # Unable to reshape the image data, trying a regular decode.
                img = cv2.imdecode(img, -1)
        else:
            img = cv2.imdecode(img, -1)

        if auto_rotate:
            img = ndimage.rotate(img, ROTATION_ANGLE[image.source.name])

        return img, extension


    def _reset_image_client(self):
        """Recreate the ImageClient from the robot object."""
        del self._robot.service_clients_by_name['image']
        del self._robot.channels_by_authority['api.spot.robot']
        return self._robot.ensure_client('image')
    
    def get_image(self, image_timeout_count):
        timeout_count_before_reset = image_timeout_count
        try:
            images_future = self._image_client.get_image_async(self._image_requests, timeout=0.5)
            while not images_future.done():
                keystroke = cv2.waitKey(25)
                #print(keystroke)
            images = images_future.result()
        except TimedOutError as time_err:
            if timeout_count_before_reset == 5:
                # To attempt to handle bad comms and continue the live image stream, try recreating the
                # image client after having an RPC timeout 5 times.
                LOGGER.info('Resetting image client after 5+ timeout errors.')
                self._image_client = self._reset_image_client(self._robot)
                timeout_count_before_reset = 0
            else:
                timeout_count_before_reset += 1
        except Exception as err:
            LOGGER.warning(err)
            
        for i in range(len(images)):
            image, _ = self._image_to_opencv(images[i])
            cv2.imshow(images[i].source.name, image)
        keystroke = cv2.waitKey(100)

        return timeout_count_before_reset


        



