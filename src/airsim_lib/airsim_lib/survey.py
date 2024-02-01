import os
import time
import airsim
import copy
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
import std_msgs
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
import cv_bridge
import shutil

AIRCRAFT_OFF = 0
AIRCRAFT_STARTUP = 13
AIRCRAFT_TAKEOFF = 6
AIRCRAFT_LAND = 7
AIRCRAFT_HOME = 3
AIRCRAFT_HOVER = 11

class SurveyNavigator:
    dist_threshold = 0.5
    dt = 0.06
    accel_threshold = 1
    max_speed = 2
    MAX_DEPTH = 20
    MIN_DEPTH = 0
    def __init__(self, logdir, num_waypoints, show = True):
        self.frame_counter = 0
        self.num_waypoints = num_waypoints
        self.currentUAVMode = AIRCRAFT_OFF
        self.modeSetTime = 0
        self.last_speed = 0.0
        self.goal_index = 0
        self.target_proximity = False
        self.show = show
        self.descend = False
        self.descent_started = False
        self.logdir = logdir
        self.client = airsim.MultirotorClient(ip="127.0.0.0", port = 41452)
        self.currentUAVMode = AIRCRAFT_STARTUP
        self.modeSetTime = copy.deepcopy(self.frame_counter)
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        state = self.client.getMultirotorState()
        landed = state.landed_state
        if(landed == airsim.LandedState.Landed):
            self.takeoff = True
        else:
            self.takeoff = False
        pos = state.kinematics_estimated.position
        gps = state.gps_location
        print("position: x: {} y:{} z:{}".format(pos.x_val, pos.y_val, pos.z_val))
        print("gps position: altitude: {} latitude: {} longitude: {}".format(gps.altitude, gps.latitude, gps.longitude))

    def start(self, step_callback):
        self.client.armDisarm(True)
        # for i in range(40):
        #     obs = self._get_observation()
        #     step_callback(obs)
        #     time.sleep(1/25)
        #     self._last_obs = copy.deepcopy(obs)
        state = self.client.getMultirotorState()
        landed = state.landed_state
        self.start = state.kinematics_estimated.position
        if(landed == airsim.LandedState.Landed):
            self.currentUAVMode = AIRCRAFT_TAKEOFF
            self.modeSetTime = copy.deepcopy(self.frame_counter)
            self.takeoff = True
            print("taking off...")
            self.client.takeoffAsync().join()
            state = self.client.getMultirotorState()
            self.start = state.kinematics_estimated.position
            pos = state.kinematics_estimated.position
            gps = state.gps_location
            print("position: x: {} y:{} z:{}".format(pos.x_val, pos.y_val, pos.z_val))
            print("gps position: altitude: {} latitude: {} longitude: {}".format(gps.altitude, gps.latitude, gps.longitude))
        for i in range(40):
            obs = self._get_observation()
            step_callback(obs)
            time.sleep(1/25)
            self._last_obs = copy.deepcopy(obs)
       
        self.set_path()

        #landed = False
        self.goal_index = 0
        goal = self.waypoints[self.goal_index]
        state = self.client.getMultirotorState()
        pos = state.kinematics_estimated.position
        gps = state.gps_location
        print("position: x: {} y:{} z:{}".format(pos.x_val, pos.y_val, pos.z_val))
        print("gps position: altitude: {} latitude: {} longitude: {}".format(gps.altitude, gps.latitude, gps.longitude))
        self.start = state.kinematics_estimated.position
        print("Flying Off.")
        count = 0
        while not landed:
            time.sleep(1/40)
            state = self.client.getMultirotorState()
            # print("position: ", state.kinematics_estimated.position)
            landed = (state.landed_state == airsim.LandedState.Landed)
            #print(state.landed_state, landed)
            pos = state.kinematics_estimated.position
            vel = state.kinematics_estimated.linear_velocity
            state = np.array([pos.x_val, pos.y_val, pos.z_val, vel.x_val, vel.y_val, vel.z_val]) 
            action = self.sample_action(goal, state)
            obs = self.step(action, self.descend)
            step_callback(obs)
            self._last_obs = copy.deepcopy(obs)
            if self.show and cv2.waitKey(1) & 0xFF == ord('q'):
                break

            dist = np.sqrt(np.square(goal - state[:3]).sum())
            # print(dist)
            if dist < self.dist_threshold:
                if not self.descent_started:
                    print("Reached waypoint ", self.goal_index)
                if self.goal_index == 0:
                    self.currentUAVMode = AIRCRAFT_HOVER
                    self.modeSetTime = copy.deepcopy(self.frame_counter)
                if self.goal_index >= 0:
                    self.goal_index += 1 
                if self.goal_index > self.waypoints.shape[0] - 1 and self.goal_index >= 0:
                    print("Last waypoint, index", self.goal_index)
                    goal = self.waypoints[0]
                    self.goal_index = -1
                elif self.goal_index < 0 and not self.descent_started:
                    self.currentUAVMode = AIRCRAFT_LAND
                    self.modeSetTime = copy.deepcopy(self.frame_counter)
                    print("Landing Sequence Starting")
                    self.descent_started = True
                    goal = np.array([self.start.x_val, self.start.y_val, self.start.z_val])
                elif self.goal_index < 0 and self.descent_started and not self.descend:
                    print("Landing")
                    obs['mode'] = AIRCRAFT_HOME
                    obs = self._get_observation()
                    step_callback(obs)
                    self.descend = True
                else:
                    goal = self.waypoints[self.goal_index]      
            elif self.goal_index < 0 and self.descent_started and self.descend:
                count+=1
                obs = self._get_observation()
                step_callback(obs)
                if(count > 150):
                    print("Breaking Loop")
                    break

        if self.takeoff:
            print("landed")
            state = self.client.getMultirotorState()
            pos = state.kinematics_estimated.position
            gps = state.gps_location
            print("position: x: {} y:{} z:{}".format(pos.x_val, pos.y_val, pos.z_val))
            print("gps position: altitude: {} latitude: {} longitude: {}".format(gps.altitude, gps.latitude, gps.longitude))
            self.client.landAsync().join()
            obs = self._get_observation()
            step_callback(obs)
            #state = self.client.getMultirotorState()
            #print("position: ", state.kinematics_estimated.position)

            print("disarming.")
            self.client.armDisarm(False)
            for i in range(20):
                obs['mode'] = AIRCRAFT_OFF
                obs = self._get_observation()
                step_callback(obs)
                #time.sleep(1/25)
        print('Done Landing')

    def set_path(self):
        print("Setting Path for Sampling")
        xpoints = np.random.uniform(-200.0, 200.0, (self.num_waypoints + 1, 1))
        ypoints = np.random.uniform(-200.0, 200.0, (self.num_waypoints + 1, 1))
        xpoints[0] = self.start.x_val
        ypoints[0] = self.start.y_val
        zpoints = np.random.uniform(-75, -50, (self.num_waypoints + 1, 1))
        self.waypoints = np.concatenate([xpoints, ypoints, zpoints], -1)
        print(self.waypoints)

    def get_speed(self, curr_v, x, goal_speed, goal):
        v = 0
        dx = np.abs(x - goal)
        if dx < ((curr_v ** 2) / (2 * self.accel_threshold)):
            goal_speed = 0
        #print("dx: ", dx, "\nx: ", x, "\ncurr_speed: ", curr_v, "\ngoal: ", goal, "\ngoal_speed: ", goal_speed, "\nstopping distance: ", (curr_v ** 2) / (2 * self.accel_threshold))
        #print(self.accel_threshold * self.dt)
        v = curr_v + self.accel_threshold * self.dt * (goal_speed - curr_v)
        # v = curr_v + self.accel_threshold * self.dt * (-1 if goal_speed - curr_v > 0 else 1)
        return v

    def sample_action(self, goal, state):
        x, y, z, curr_vx, curr_vy, curr_vz = state
        vx = 0
        vy = 0
        vz = 0
        goal_index = copy.deepcopy(self.goal_index)
        if self.goal_index == -1:
            goal_index = 0
        goal = self.waypoints[goal_index]
        if self.descent_started:
            goal = np.array([self.start.x_val, self.start.y_val, self.start.z_val])
        dz = goal[2] - z
        goal_vz = 0
        goal_vx = 0
        goal_vy = 0
        if dz > 0:
            goal_vz = self.max_speed / 5
        else:
            goal_vz = -self.max_speed / 5
        if dz > -self.dist_threshold and dz < self.dist_threshold:
            goal_vz = 0
        vz = self.get_speed(curr_vz, z, goal_vz, goal[2])
        if goal_index != 0 or self.goal_index == -1:
            dx = goal[0] - x
            dy = goal[1] - y
            orientation = np.arctan2(dy, dx)
            goal_vx = self.max_speed * np.cos(orientation)
            goal_vy = self.max_speed * np.sin(orientation)
            vx = self.get_speed(curr_vx, x, goal_vx, goal[0]) 
            vy = self.get_speed(curr_vy, y, goal_vy, goal[1])
        v = np.array([vx, vy, vz])
        #print(v, goal_vx, goal_vy, goal_vz)
        return v

    def step(self, action, land = False):
        # print(action)
        if not land:
            # print("moving")
            vx, vy, vz = action
            self.client.moveByVelocityAsync(
                    vx, vy, vz, self.dt,
                    airsim.DrivetrainType.MaxDegreeOfFreedom).join()
        else:
            self.client.landAsync()
        obs = self._get_observation()
        return obs

    def _get_observation(self):
        self.client.simPause(True)
        # Paused Airsim client to ensure synchronised data timestamps
        responses = self.client.simGetImages([
            airsim.ImageRequest("3", airsim.ImageType.Scene, False, False),
            airsim.ImageRequest("3", airsim.ImageType.DepthPerspective, True, False),
            airsim.ImageRequest("3", airsim.ImageType.Infrared, False, False),
            ])
        state = self.client.getMultirotorState()
        gnss = self.client.getGpsData().gnss
        gps = gnss.geo_point

        pos = state.kinematics_estimated.position
        vel = gnss.velocity
        self.client.simPause(False)
        
        response = responses[0]
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        rgb = img1d.reshape(response.height, response.width, 3)
        rgb_timestamp = response.time_stamp

        # https://github.com/microsoft/AirSim/issues/2835
        # Refer to the aforementioned link for more information about the implementation below
        response = responses[1]
        img1d = airsim.list_to_2d_float_array(response.image_data_float, response.width, response.height)
        # print("Image size: ", response.width, response.height)
        depth_img_in_meters = img1d.reshape(response.height, response.width, 1)
        depth_img_in_meters = np.clip(depth_img_in_meters, self.MIN_DEPTH, self.MAX_DEPTH)
        # print("Minimum distance: ", np.min(depth_img_in_meters))
        # print("Maximum Depth: ", np.max(depth_img_in_meters))
        # depth = np.interp(depth_img_in_meters, (self.MIN_DEPTH, self.MAX_DEPTH), (0, 255))
        # depth = depth.astype(np.uint8)
        depth = np.clip(depth_img_in_meters * 1000, 0, 65535)
        depth = depth.astype(np.uint16)
        depth_timestamp = response.time_stamp
        
        response = responses[2]
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        infrared = img1d.reshape(response.height, response.width, 3)
        infrared_timestamp = response.time_stamp
        #img_rgb = np.flipud(rgb)
        if self.show:
            cv2.imshow("scene", rgb)
            cv2.imshow("depth", depth)
            cv2.imshow("infrared", infrared)
        obs = {
            'scene' : rgb,
            'scene_timestamp': rgb_timestamp, 
            'depth': depth,
            'depth_timestamp': depth_timestamp,
            'infrared': infrared,
            'infrared_timestamp': infrared_timestamp,
            'gps': np.array([gps.latitude, gps.longitude, -gps.altitude]),
            'gps_timestamp': gnss.time_utc,
            'gps_vel': np.array([vel.x_val, vel.y_val, vel.z_val]),
            'gps_err': np.array([gnss.eph, gnss.epv]),
            'mode': copy.deepcopy(self.currentUAVMode),
            'position': np.array([pos.x_val, pos.y_val, -pos.z_val]),
            'frame_counter': copy.deepcopy(self.frame_counter)
        }
        self.frame_counter += 1
        return obs


class AirSimNode(Node):
    def __init__(self, nodeName = "airsim_node", logToFile = True, logDir = "data", num_waypoints = 1):
        super().__init__(nodeName)
        self.logToFile = logToFile
        self.logDir = logDir
        self.switchedToMonocular = False
        if self.logToFile:
            if os.path.exists(self.logDir):
                shutil.rmtree(self.logDir)
            os.mkdir(self.logDir)
            filePath = os.path.join(self.logDir, 'day_gps.txt')
            if os.path.exists(filePath):
                os.remove(filePath)
            self.file = open(filePath, 'a')
            size = (640, 480)
            self.videofilePath = os.path.join(self.logDir, 'frames.avi')
            self.video = cv2.VideoWriter(self.videofilePath, cv2.VideoWriter_fourcc(*'MJPG'), 15, size, True)
            self.depthDatadir = os.path.join(self.logDir, 'Depth')
            if os.path.exists(self.depthDatadir):
                shuitl.rmtree(self.depthDatadir)
            os.mkdir(self.depthDatadir)
        self.bridge = cv_bridge.CvBridge()
        self.navigator = SurveyNavigator(self.logDir, num_waypoints, False)
        obs = self.navigator._get_observation()
        self._publishers = {}

        self._publishers['depth'] = self.create_publisher(Image, "~/camera/bottom_center/depth", 10)
        self._publishers['scene'] = self.create_publisher(Image, "~/camera/bottom_center/rgb", 10)
        self._publishers['gps'] = self.create_publisher(NavSatFix, "~/gps/position", 10)
        self._publishers['gps_vel'] = self.create_publisher(Vector3Stamped, "~/gps/velocity", 10)
        self.navigator.start(self.step_callback)
        print("done with node constructor")


    def step_callback(self, obs):
        header_rgb = Header()
        header_rgb.stamp = rclpy.time.Time(nanoseconds = obs['scene_timestamp']).to_msg()
        rgb = self.bridge.cv2_to_imgmsg(obs['scene'], encoding = 'rgb8', header = header_rgb)
        self._publishers['scene'].publish(rgb)
        
        header_depth = Header()
        header_depth.stamp = rclpy.time.Time(nanoseconds = obs['depth_timestamp']).to_msg()
        depth = self.bridge.cv2_to_imgmsg(obs['depth'], encoding = 'mono16', header = header_depth)
        self._publishers['depth'].publish(depth)

        header_gps = Header()
        header_gps.stamp = rclpy.time.Time(nanoseconds = obs['gps_timestamp']).to_msg()
        gps = NavSatFix()
        gps.header = header_gps
        gps.latitude = obs['gps'][0]
        gps.longitude = obs['gps'][1]
        gps.altitude = obs['gps'][2]
        self._publishers['gps'].publish(gps)


        header_gps_vel = Header()
        header_gps_vel.stamp = rclpy.time.Time(nanoseconds = obs['gps_timestamp']).to_msg()
        gps_vel = Vector3Stamped()
        gps_vel.header = header_gps_vel
        gps_vel.vector.x = obs['gps_vel'][0]
        gps_vel.vector.y = obs['gps_vel'][1]
        gps_vel.vector.z = obs['gps_vel'][2]
        self._publishers['gps_vel'].publish(gps_vel)

        if self.logToFile:
            self.log_to_file(obs)

    def log_to_file(self, obs):
        speed = np.sqrt(np.sum(np.square(obs['gps_vel'])))
        heading = np.degrees(np.arctan2(obs['gps_vel'][1], obs['gps_vel'][0]))
        sensorType = 'RGBD'
        if(obs['position'][2] > 7.5):
            sensorType = 'Monocular'
            if not self.switchedToMonocular:

                print('Switched to Monocular')
                self.switchedToMonocular = True
        data = "{:.7f},{:.7f},{:.7f},{},0,{},{},{:.7f},{:.7f},{:.7f},{},11,0,{},GDN_PROCESS_OK,TRACKING,Sensor_OK,Sensor_OK,{},0,{:.7f},0,0\n".format(
                obs['gps'][0],
                obs['gps'][1],
                obs['position'][2],
                obs['mode'],
                obs['gps_timestamp'],
                obs['gps_timestamp'] // (60 * 60 * 24),
                obs['gps'][2],
                obs['gps_err'][0],
                speed,
                heading,
                sensorType,
                obs['frame_counter'],
                obs['position'][2]
                )
        self.file.write(data)
        self.video.write(obs['scene'])
        cv2.imwrite(os.path.join(self.depthDatadir, 'depth_{}.png'.format(obs['frame_counter'])), obs['depth'])

    def destroy_node(self):
        print("Destroying custom node")
        self.close()
        super().destroy_node()


    def close(self):
        self.file.close()
        self.video.release()


if __name__ == "__main__":
    nav = SurveyNavigator("./", 5)
    nav.start(step_callback)
    nav.close()
