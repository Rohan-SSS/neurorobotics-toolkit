import os
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

AIRCRAFT_OFF = 0
AIRCRAFT_STARTUP = 13
AIRCRAFT_TAKEOFF = 6
AIRCRAFT_LAND = 7
AIRCRAFT_HOME = 3
AIRCRAFT_HOVER = 11

class SurveyNavigator:
    dist_threshold = 10
    def __init__(self, logdir, num_waypoints, show = True):
        self.frame_counter = 0
        self.num_waypoints = num_waypoints
        self.currentUAVMode = AIRCRAFT_OFF
        self.show = show
        self.descend = False
        self.descent_started = False
        self.logdir = logdir
        self.client = airsim.MultirotorClient()
        self.currentUAVMode = AIRCRAFT_STARTUP
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
        for i in range(40):
            obs = self._get_observation()
            step_callback(obs)
        state = self.client.getMultirotorState()
        landed = state.landed_state
        self.start = state.kinematics_estimated.position
        if(landed == airsim.LandedState.Landed):
            self.currentUAVMode = AIRCRAFT_TAKEOFF
            self.takeoff = True
            print("taking off...")
            self.client.takeoffAsync()
            state = self.client.getMultirotorState()
            self.start = state.kinematics_estimated.position
            pos = state.kinematics_estimated.position
            gps = state.gps_location
            print("position: x: {} y:{} z:{}".format(pos.x_val, pos.y_val, pos.z_val))
            print("gps position: altitude: {} latitude: {} longitude: {}".format(gps.altitude, gps.latitude, gps.longitude))
        for i in range(40):
            obs = self._get_observation()
            step_callback(obs)
       
        self.set_path()

        #landed = False
        index = 0
        goal = self.waypoints[index]
        state = self.client.getMultirotorState()
        pos = state.kinematics_estimated.position
        gps = state.gps_location
        print("position: x: {} y:{} z:{}".format(pos.x_val, pos.y_val, pos.z_val))
        print("gps position: altitude: {} latitude: {} longitude: {}".format(gps.altitude, gps.latitude, gps.longitude))
        self.start = state.kinematics_estimated.position
        print("Flying Off.")
        count = 0
        while not landed:
            state = self.client.getMultirotorState()
            # print("position: ", state.kinematics_estimated.position)
            landed = (state.landed_state == airsim.LandedState.Landed)
            #print(state.landed_state, landed)
            pos = state.kinematics_estimated.position
            pose = np.array([pos.x_val, pos.y_val, pos.z_val]) 
            action = self.sample_action(goal, pose)
            obs = self.step(action, self.descend)
            step_callback(obs)
            if self.show and cv2.waitKey(1) & 0xFF == ord('q'):
                break

            dist = np.sqrt(np.square(goal - pose).sum())
            #print(dist)
            if dist < 2:
                if not self.descent_started:
                    print("Reached waypoint ", index)
                if index == 0:
                    self.currentUAVMode = AIRCRAFT_HOVER
                if index >= 0:
                    index += 1 
                if index > self.waypoints.shape[0] - 1 and index >= 0:
                    print("Last waypoint, index", index)
                    goal = self.waypoints[0]
                    index = -1
                elif index < 0 and not self.descent_started:
                    self.currentUAVMode = AIRCRAFT_LAND
                    print("Landing Sequence Starting")
                    self.descent_started = True
                    goal = np.array([self.start.x_val, self.start.y_val, self.start.z_val])
                elif index < 0 and self.descent_started and not self.descend:
                    print("Landing")
                    obs['mode'] = AIRCRAFT_HOME
                    obs = self._get_observation()
                    step_callback(obs)
                    self.descend = True
                elif index < 0 and self.descent_started and self.descend:
                    count+=1
                    if(count % 10 == 0):
                        obs = self._get_observation()
                        step_callback(obs)
                    if(count > 500):
                        print("Breaking Loop")
                        break
                else:
                    goal = self.waypoints[index]      

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

    def set_path(self):
        print("Setting Path for Sampling")
        xpoints = np.random.uniform(-200.0, -50.0, (self.num_waypoints + 1, 1))
        ypoints = np.random.uniform(-200.0, -50.0, (self.num_waypoints + 1, 1))
        xpoints[0] = self.start.x_val
        ypoints[0] = self.start.y_val
        zpoints = np.random.uniform(-75, -50.0, (self.num_waypoints + 1, 1))
        self.waypoints = np.concatenate([xpoints, ypoints, zpoints], -1)
        print(self.waypoints)

    def sample_action(self, goal, pose):
        dist = np.sqrt(np.square(goal - pose).sum())
        orientation = np.arctan2(pose[1] - goal[1], pose[0] - goal[0])
        if(dist < self.dist_threshold):
            speed = dist
        else:
            speed = self.dist_threshold
        vx = -speed * np.cos(orientation)
        vy = -speed * np.sin(orientation)
        z = goal[2]
        return np.array([vx, vy, z])

    def step(self, action, land = False):
        # print(action)
        if not land:
            # print("moving")
            vx, vy, z = action
            self.client.moveByVelocityZAsync(
                    vx, vy, z, 1,
                    airsim.DrivetrainType.MaxDegreeOfFreedom)
        else:
            self.client.landAsync()
        obs = self._get_observation()
        return obs

    def _get_observation(self):
        self.client.simPause(True)
        # Paused Airsim client to ensure synchronised data timestamps
        responses = self.client.simGetImages([
            airsim.ImageRequest("3", airsim.ImageType.Scene, False, False),
            airsim.ImageRequest("3", airsim.ImageType.DepthVis, False, False),
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


        response = responses[1]
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        depth = img1d.reshape(response.height, response.width, 3)
        depth_timestamp = response.time_stamp
        #img_rgb = np.flipud(rgb)
        if self.show:
            cv2.imshow("scene", rgb)
            cv2.imshow("depth", depth)
        obs = {
            'scene' : rgb,
            'scene_timestamp': rgb_timestamp, 
            'depth': depth,
            'depth_timestamp': depth_timestamp,
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
    def __init__(self, nodeName = "airsim_node", logToFile = True):
        super().__init__(nodeName)
        self.logToFile = logToFile
        if self.logToFile:
            if os.path.exists('day_gps.txt'):
                os.remove('day_gps.txt')
            self.file = open("day_gps.txt", 'a')
        self.bridge = cv_bridge.CvBridge()
        self.navigator = SurveyNavigator("./", 2)
        obs = self.navigator._get_observation()
        self._publishers = {}

        self._publishers['depth'] = self.create_publisher(Image, "~/camera/bottom_center/depth", 10)
        self._publishers['scene'] = self.create_publisher(Image, "~/camera/bottom_center/rgb", 10)
        self._publishers['gps'] = self.create_publisher(NavSatFix, "~/gps/position", 10)
        self._publishers['gps_vel'] = self.create_publisher(Vector3Stamped, "~/gps/velocity", 10)
        self.navigator.start(self.step_callback)


    def step_callback(self, obs):
        header_rgb = Header()
        header_rgb.stamp = rclpy.time.Time(nanoseconds = obs['scene_timestamp']).to_msg()
        rgb = self.bridge.cv2_to_imgmsg(obs['scene'], encoding = 'rgb8', header = header_rgb)
        self._publishers['scene'].publish(rgb)
        
        header_depth = Header()
        header_depth.stamp = rclpy.time.Time(nanoseconds = obs['depth_timestamp']).to_msg()
        depth = self.bridge.cv2_to_imgmsg(obs['depth'], encoding = 'rgb8', header = header_depth)
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
     
    def close(self):
        self.file.close()


if __name__ == "__main__":
    nav = SurveyNavigator("./", 1)
    nav.start(step_callback)
    nav.close()
