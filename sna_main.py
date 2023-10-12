'''
@Description:
This code runs the main logic of Sense and Avoid. As of now, only Sense and Stop Logic is included.

@TODO
Fix Get Mission Coordinates function => Should fetch and store only 
Align it with latest CC Code
Add Sense and Avoid Feature
'''
# Import Necessary Modules
#import pymap3d as pm
from os import O_DSYNC
from pickle import NONE
import time
import os
import threading
import numpy as np
import math
import csv

from pymavlink import mavutil
from src.globals.ga_logging import GALogging
from src.globals.globals import Globals
from src.mavlink.mavlink import Mavlink
from src.peripherals.iot.iot_core import IOTCore
from src.util.cpu_monitor import CPUInfo
from src.util.task_scheduler import SchedulerTask
from src.vehicle.vehicle import Vehicle
from src.vehicle.vehicle_flight_mode import CopterMode
from src.vehicle.vehicle_ga3a.arming_check import ArmingCheck
from src.vehicle.vehicle_ga3a.inflight_alerts.inflight_alert import InflightAlert
from src.vehicle.vehicle_ga3a.preflight_checks.preflight_check import PreflightCheck
from src.vehicle.vehicle_gcs_command import VehicleGCSCommand
from datetime import datetime

from src.vehicle.vehicle_status import VehicleStatus
from src.vehicle.vehicle_ga3a.mission_summary import MissionSummary
from src.vehicle.vehicle_flight_mode import CopterMode
from src.vehicle.vehicle_status import VehicleStatus
from src.vehicle.vehicle_command import VehicleCommand
from src.util.geo_util import GeoUtil

# Import Necessary Modules for SnA
from src.vehicle.vehicle_ga3a.obstacle_avoidance.sna_data_handler import SnADataHandler
from src.vehicle.vehicle_ga3a.obstacle_avoidance.vector_math import AngularTransformation
from src.vehicle.vehicle_ga3a.obstacle_avoidance.vector_math import vector
from src.vehicle.vehicle_ga3a.obstacle_avoidance.sna_sensor_driver import SnASensorDriver


# Import Radar Class

np.set_printoptions(suppress = True)

# Import C++ and python code
# from src.vehicle.vehicle_ga3a.sense_and_avoid.udp_arm_code import arm_executable
# from src.vehicle.vehicle_ga3a.sense_and_avoid.udp_arm_code import client.py

class SnAMain:
    
    SENSE_AND_STOP = False
    SENSE_AND_AVOID = False
    

    def __init__(self, vehicle):

        self.__vehicle = vehicle
        self.__angTransform = AngularTransformation()
        self.__vector = vector()
                        
        # Vehicle Status
        self.rc6 = 0
        self.isFlying = False
        self.isArmed = False
        self.__currentWP = 0
        self.__currentMode = "UNKNOWN"


        # Home Position
        self.__home_lat = 0
        self.__home_long = 0
        self.__home_alt = 0
        self.__home = np.array([])

        # Current waypoint
        self.__wp_lat = 0
        self.__wp_long = 0
        self.__wp = np.array([])

        self.__current_waypoint = 0
        self.__next_waypoint = 0
        self.__nextwaypoint = 0
        
        # Mission Related Variables
        self.__waypoints = []
        self.__i_waypoints =  []
        self.__i_current_waypoint = 0
        self.__i_next_waypoint = 0

        self.accelerated = False
        self.acceleratedt1 = 0
        
        self.__temp_wp_count = 0
        self.__waypoint_count = 0

        self.__mission_uploaded = False

        # Vehicle Position
        self.__lat = -200
        self.__lon = -200
        self.__hdop = 100
        self.__globalTime = 0
        self.__globalAlt = -1000      # m
        self.__relativeAlt = -1000    # m
        self.__terrainAlt = 0         # m

        # Vehicle Speed
        self.__vx = 0 # m/s
        self.__vy = 0 # m/s
        self.__vz = 0 # m/s
        self.__speed = 0 # m/s

        #Initiatialising Position Predictor Vectors
        self.__prev_px = 0
        self.__prev_py = 0
        self.__px = 0
        self.__py = 0
        self.__pz = 0

        # Attitude
        self.__pitch = 0 # rad (-pi to pi)
        self.__roll = 0 # rad (-pi to pi)
        self.__yaw = 0 # rad (0 to 2pi)

        # Front Sensor
        self.__front_sensor = SnADataHandler()

        # Map related params
        self.__np_mat_to_list = np.array([])
        self.__matmap = np.array([[]])
        self.obstacle_map_count = []
        self.desired_angle = 0
        self.engaging_distance = 0
        self.static_map = False
        self.counter = 0
        
        # Brake flag
        self.__braked = False

        # Avoidance Flag
        self.__sense_and_stop = False
        self.__sense_and_avoid = False

        # Safety Flags
        
        # Mode Change Params
        self.__mode_changed_to_guided = False
        self.__mode_changed_to_guided_from_loiter = False
        self.__mode_changed_to_guided_from_auto = False

        # Pilot overriding
        self.__overriding = 1    # Initially pilot control

        # Sensor params
        self.__lidar = None
        self.__radar = None
        self.__radRadVel = []
        self.__radLatVel = []
        self.__radRcs = []

        self.__obstacle_data = None
        self.csv_file = None

        self.max_y = 0
        self.max_x = 0

        # Temp roll pitch conf var
        self.rolled_right = False
        self.pitched_forward = False
      

        # Thread Variables
        self.__scheduler_update = SchedulerTask(self.update_mission, name='SnA Update Mission', delay=0.0, time_period=0.1)
        self.__scheduler_receive_radar_data = SchedulerTask(self.__front_sensor.receive_radar_data, name='Receive Radar Data', delay=0.0, time_period=0.08)


    
    def initialize(self):

        print("Initializing sna main code")
        if Globals.SITL:
            #self.create_csv_file()
            #self.get_mission_coordinates()
            self.__lidar = SnASensorDriver('SITL')
            self.__lidar.connect_sitl_sensor()
            self.__schedule_sitl_sensor_update = SchedulerTask(self.__lidar.update_sitl_sensor, name='vehicle_update', delay=0, time_period=0.02)  # 0.02

            self.__schedule_sitl_sensor_update.start() 
            self.__scheduler_update.start()

        if not Globals.SITL:
            #self.get_mission_coordinates()
            self.__scheduler_receive_radar_data.start()
            self.__scheduler_update.start()

        GALogging.info('Initialized Obstacle Sensor')
        print('Initialized Obstacle Sensor')


    def handle_notified_object(self, key, val):

        if key is Mavlink.MSG_LOCAL_POSITION_NED:
            self.__px = val.x
            self.__py = val.y
            self.__vx = val.vx #in m/s
            self.__vy = val.vy #in m/s
            self.__vz = val.vz #in m/s
            self.__speed = np.sqrt(self.__vx**2 + self.__vy**2 + self.__vz**2) 
            #print("px, py while receiving", self.__px, self.__py)
            
            

        if key is Mavlink.MSG_ATTITUDE:
            self.__pitch = val.pitch
            self.__roll = val.roll
            if val.yaw<0:
                self.__yaw = val.yaw + 2*np.pi
            else:
                self.__yaw = val.yaw
            

        if key is Mavlink.MSG_GPS_RAW_INT:
            self.__lat = val.lat * 1e-7
            self.__lon = val.lon * 1e-7
            self.__globalAlt = val.alt/1000 # m/s
            self.__hdop = val.eph/100
            

        # if key is Mavlink.MSG_GLOBAL_POSITION_INT:
        #     self.__vx = 0.01*val.vx #in m/s
        #     self.__vy = 0.01*val.vy #in m/s
        #     self.__vz = 0.01*val.vz #in m/s
        #     self.__speed = np.sqrt(self.__vx**2 + self.__vy**2 + self.__vz**2) #in m/s
        #     self.__relativeAlt = val.relative_alt/1000. #in m
            

        if key is Mavlink.MSG_RANGEFINDER:
            self.__terrainAlt = val.distance
            

        if key is Mavlink.MSG_MISSION_CURRENT:
            self.__currentWP = val.seq
            #self.obtain_current_waypoint_recent()
            #print("mission val inside handle not: ", self.__currentWP)

        

        # if self.__vehicle.mavlink().mav_connection is not None:
        #     self.__vehicle.mavlink().send_message(
        #                     mavutil.mavlink.MAVLink_command_long_message(
        #                         self.__vehicle.mavlink().mav_connection.target_system,
        #                         self.__vehicle.mavlink().mav_connection.target_component,
        #                         mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        #                         0,
        #                         0,
        #                         0,
        #                         0,
        #                         0,
        #                         0,
        #                         0,
        #                         0
        #                     ))
        if key is 'HOME_POSITION':
            self.__home_lat = val.latitude
            self.__home_long = val.longitude
            self.__home_alt = val.altitude

        if key is VehicleStatus.ARM_STATUS_CHANGED:
            if val is True:
                self.get_mission_coordinates()


        '''
        @Description: Gets the whole mission accurately, including takeoff and land if present
        '''

    def get_mission_coordinates(self):

        self.__vehicle.mavlink().send_message(
                mavutil.mavlink.MAVLink_mission_request_list_message(
                    self.__vehicle.mavlink().mav_connection.target_system,
                    self.__vehicle.mavlink().mav_connection.target_component
                ))

        msg = self.__vehicle.mavlink().mav_connection.recv_match(type=['MISSION_COUNT'],blocking=True, timeout=0.3)
        if msg is not None:
            self.__waypoint_count = msg.count

        #print("Waypoint Count: ", self.__waypoint_count)
        home_lat = self.__home_lat * 1e-07
        home_lon = self.__home_long * 1e-07
        home_alt = self.__home_alt * 1e-07
        coordinates = []
        coordinates_ned = []
        self.__temp_wp_count = 0
       
        while self.__temp_wp_count < self.__waypoint_count:
                
            #print("wp temp count: ", self.__temp_wp_count)

            self.__vehicle.mavlink().send_message(
                mavutil.mavlink.MAVLink_mission_request_int_message(
                    self.__vehicle.mavlink().mav_connection.target_system,
                    self.__vehicle.mavlink().mav_connection.target_component,
                    self.__temp_wp_count
                ))

            msg = self.__vehicle.mavlink().mav_connection.recv_match(type=['MISSION_ITEM_INT'], blocking=True, timeout=0.1)
            #print("WP message: ", msg)
            
            if msg is not None:

                if msg.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                    # Get the GPS coordinates of the waypoint
                    lat = msg.x * 1e-07
                    lon = msg.y * 1e-07
                    alt = msg.z * 1e-07
                    # Convert GPS coordinates to NED
                    d_lat = lat - home_lat
                    d_lon = lon - home_lon
                    d_alt = alt - home_alt
                    R = 6371000  # Earth's radius in meters
                    x = d_lat * math.pi / 180 * R
                    y = d_lon * math.pi / 180 * R * math.cos(home_lat * math.pi / 180)
                    z = d_alt
                    
                    # Add the NED coordinates to the list
                    coordinates_ned.append((x, y, z))
                    coordinates.append((lat,lon,alt))

                elif msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    coordinates.append("TakeOff")
                    coordinates_ned.append("TakeOff")
                elif msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                    coordinates.append("land")
                    coordinates_ned.append("land")
                elif msg.command == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    coordinates.append("RTL")
                    coordinates_ned.append("RTL")
                else:
                    coordinates.append("Non-Navigation Command")
                    coordinates_ned.append("Non-Navigation Command")

                self.__temp_wp_count += 1

        self.__waypoints = coordinates
        #self.__i_waypoints = [(14599804578904.467, 86485629599149.92, 887.2000122070312), 'TakeOff', (14599347234171.178, 86485466142607.75, 5.0), (14599189448570.27, 86485410322754.58, 5.0), (14599235928049.605, 86485705656479.73, 5.0), 'RTL']
        self.__i_waypoints = coordinates_ned
        print("Inertial Waypoints: ", self.__i_waypoints)

        # mission_coordinates = []
        # mission_coordinates_ned = []

        # mission_list = self.__vehicle.mission_manager().mission_item_list()
        # if len(mission_list) != 0:
        #     for i in range(len(mission_list)):
        #         if mission_list[i].command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
        #             wp_xy = GeoUtil.convert_geo_to_ned(np.array([[mission_list[i].lat, mission_list[i].lon]]),
        #                                          [self.__vehicle.GPSGlobalOrigin.lat,
        #                                           self.__vehicle.GPSGlobalOrigin.lon]).tolist()
                    
                    
                    
        #             # lat = mission_list[i].lat * 1e-07
        #             # lon = mission_list[i].lat * 1e-07
        #             # alt = msg.z * 1e-07
        #             # # Convert GPS coordinates to NED
        #             # d_lat = lat - home_lat
        #             # d_lon = lon - home_lon
        #             # d_alt = alt - home_alt
        #             # R = 6371000  # Earth's radius in meters
        #             # x = d_lat * math.pi / 180 * R
        #             # y = d_lon * math.pi / 180 * R * math.cos(home_lat * math.pi / 180)
        #             # z = d_alt
                    
        #             mission_coordinates.append((mission_list[i].lat, mission_list[i].lon, mission_list[i].alt))
        #             mission_coordinates_ned.append(wp_xy)
        #         if mission_list[i].command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
        #             mission_coordinates.append("TakeOff")
        #             mission_coordinates_ned.append("TakeOff")
        #         if mission_list[i].command == mavutil.mavlink.MAV_CMD_NAV_LAND:
        #             mission_coordinates.append("Land")
        #             mission_coordinates_ned.append("Land")
        #         if mission_list[i].command == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
        #             mission_coordinates.append("Return_to_Launch")
        #             mission_coordinates_ned.append("Return_to_Launch")

        # self.__waypoints = mission_coordinates
        # self.__i_waypoints = mission_coordinates_ned
        # # print("**********Mission Coordinates********", mission_coordinates) 
        # #print("**********Mission NED Coordinates*************", mission_coordinates_ned)
        
        
    def get_mission_coordinates1(self):
        pass

    def __change_mode_to_guided(self):
        self.__vehicle.vehicle_command().set_flight_mode(mavutil.mavlink.COPTER_MODE_GUIDED)
        print("Mode changed to guided")
    
    def __guided_roll(self, roll_velocity):
        print("Sending roll command")
        self.__vehicle.mavlink().send_message(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                1,
                self.__vehicle.mavlink().mav_connection.target_system, 
                self.__vehicle.mavlink().mav_connection.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
                int(0b0000111111000111),
                0, 0, 0,
                0, roll_velocity, 0,
                0, 0, 0, 0, 0
            )
        )
        print("Rolled right")

    def __guided_pitch(self, pitch_distance):
        print("Sending picth command")
        self.__vehicle.mavlink().send_message(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                1,
                self.__vehicle.mavlink().mav_connection.target_system, 
                self.__vehicle.mavlink().mav_connection.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
                int(0b0000111111111000),
                pitch_distance, 0, 0,
                0, 0, 0,
                0, 0, 0, 0, 0
            )
        )
        print("Pitched Forward")
        
        

    def obtain_current_waypoint_recent(self): # Thread
        try:
            if len(self.__i_waypoints) != 0: 
                self.__current_waypoint=self.__waypoints[self.__currentWP]
                self.__next_waypoint=self.__waypoints[self.__currentWP+1]
                self.__i_current_waypoint = self.__i_waypoints[self.__currentWP]
                self.__i_next_waypoint = self.__i_waypoints[self.__currentWP+1]
                # print("Current wp: ", self.__i_current_waypoint)
                # print("next wp: ", self.__i_next_waypoint)
        except KeyboardInterrupt:
            print("Exception in obtain_Current_Waypoint")

        
    '''
    @Description: Checks for obstacle in the direction
    '''
    def obstacle_in_direction(self,direc_vec,global_map,des_angle,des_distance):
       
        obs_in_direc_marker=0
        obs_min_in_direct=100 #returns 100 if there is nothing within des_distance else gives min_distance
        for i in range(np.size(global_map,axis=0)):
            #Compute Vector
            obstacle_vector = [global_map[i][0]-self.__px,global_map[i][1]-self.__py]
            print("Obstacle vector: ", obstacle_vector)
            obstacle_vector_magnitude=self.__vector.mag2d(obstacle_vector)

            obstacle_mag_in_direc=(np.dot(direc_vec,obstacle_vector)/(self.__vector.mag2d(direc_vec)))
            #print("obstacle mag in direct: ", obstacle_mag_in_direc)
            if(obstacle_vector_magnitude<=des_distance and obstacle_mag_in_direc>0):                    
                obstacle_angle = round(math.acos(np.dot(direc_vec,obstacle_vector)/(self.__vector.mag2d(direc_vec)*self.__vector.mag2d(obstacle_vector)))*180/math.pi,2)
                #print('obstacle angle', obstacle_angle)
                if abs(obstacle_angle)<des_angle :
                    obs_in_direc_marker=1
                    if((obstacle_mag_in_direc)<obs_min_in_direct):
                        obs_min_in_direct=obstacle_mag_in_direc
                        #print("obs min in direct: ", obs_min_in_direct)                
                        GALogging.info(f'SnALogging obs_min_in_direct: {obs_min_in_direct}')
        if(obs_in_direc_marker==1):
            print('found obstacle in direction ',direc_vec,' in global map ',global_map,' from position ',round(self.__px),round(self.__py))
            GALogging.info(f'SnALogging found obstacle in direction {direc_vec}, in global map {global_map}, from position {round(self.__px)}, {round(self.__py)}')
            return True,obs_min_in_direct
            
        else:
            return False,obs_min_in_direct

    
    def update_vars(self):
        """This function acts as a bridge between different class to transfer data. Part of the requirements for 
        developing the algorithm
        """
        #6.198883056640625e-06 seconds
        #self.front_sensor.data = [[self.nanoRadar.obj_x_dis],[self.nanoRadar.obj_y_dis]]
        if Globals.SITL:
            self.__front_sensor.data = self.__lidar.raw_data
            self.__front_sensor.handle_sitl_sensor_data(1,0.03098,40,1,-0.976)

        self.__front_sensor.pitch = self.__pitch

        if Globals.SITL:

            self.__front_sensor.roll = self.__roll + math.pi
        else:
            self.__front_sensor.roll = self.__roll
        

        self.__front_sensor.yaw = math.atan2(math.sin(self.__yaw),math.cos(self.__yaw))
        self.__front_sensor.px = self.__px
        self.__front_sensor.py = self.__py
        self.__front_sensor.pz = self.__relativeAlt
        self.__front_sensor.terrainAlt = self.__terrainAlt
        
    def create_csv_file(self):
        self.log_time = time.time()
        self.log_time1 = datetime.now()
        csv_name = str(self.log_time1) + '.csv'
        self.csv_file = open(csv_name,'w')
        print('made csv file with name', self.csv_file)
        
    def log_data(self):
        
        roll_pitch_yaw_string = str(round(self.__roll,4))+','+str(round(self.__pitch,4))+','+str(round(self.__yaw,4))+"{}"
        terrain_alt_str = str(round(self.__terrainAlt,2))+"{}"
        selfpx_and_selfpy_and_selfpz = str(round(self.__px,2))+','+str(round(self.__py,2))+','+str(round(self.__pz,2))+'{}'
        gps_lat_lon_of_drone = '('+str(self.__lat)+','+str(self.__lon)+')'+'{}'        
    
        

        # print('front sensor x', self.front_sensor.X)
        # print('terain alt',round(self.terrainAlt,2))

        try:
            if Globals.SITL:
                frontSensorX = ','.join([str(round(elem,2)) for elem in self.__front_sensor.x]) + '{}'
                frontSensorY = ','.join([str(round(elem,2)) for elem in self.__front_sensor.y])+"{}"
            if not Globals.SITL:
                frontSensorX = ','.join([str(round(elem,2)) for elem in self.__front_sensor.x]) + '{}'
                frontSensorY = ','.join([str(round(elem,2)) for elem in self.__front_sensor.y])+"{}"
                # radialVel = ','.join([str(round(elem,2)) for elem in self.__radRadVel])+"{}"
                # latvel = ','.join([str(round(elem,2)) for elem in self.__radLatVel])+"{}"
                # rcs = ','.join([str(round(elem,2)) for elem in self.__radRcs])+"{}"
                # rawRadx = ','.join([str(round(elem,2)) for elem in self.__radar.final_x]) + '{}'
                # rawRady = ','.join([str(round(elem,2)) for elem in self.__radar.final_y]) + '{}'
        except Exception as e:
            GALogging.warn(f"SnALogging: Error in logging radar data {e}")
            
    
        vx_vy_vz = str(round(self.__vx,2))+','+str(round(self.__vy,2))+'{}'

        #number=str(self.__front_sensor.X.shape[0])+'{}'
        
        #final_data = "Time," + str(time.time())+'{}' + str(self.log_time1) + '{}' + roll_pitch_yaw_string + terrain_alt_str + selfpx_and_selfpy_and_selfpz +vx_vy_vz  + gps_lat_lon_of_drone  + self.__currentMode + '\n'

        GALogging.info(f'SnALogging : {roll_pitch_yaw_string} {terrain_alt_str} {selfpx_and_selfpy_and_selfpz} {vx_vy_vz}  {self.__currentMode} {frontSensorX} {frontSensorY}')

        
        # self.csv_file.write(final_data)
        # if time.time()-self.log_time>=10000:
        #     self.csv_file.close()
        
    def direction(self):
        #self.heading_vector
        pass

    def heading_vector(self):
            #todo separate logic for rtl
        # siny=np.sin(self.__yaw)

        # cosy=np.cos(self.__yaw)

        # if (self.__currentMode is CopterMode.GUIDED):
        #     self.wpvec = [cosy, siny]
        # elif (self.__currentMode is CopterMode.LOITER or self.__currentMode is CopterMode.AUTO):
        #     self.wpvec = [self.__vx, self.__vy]        
        
        self.wpvec = [self.__vx, self.__vy]
       
        
        # try:
        #     if self.__speed > 1:
        #         self.wpvec=[self.__vx,self.__vy]
        #     elif self.__i_current_waypoint is not 'TakeOff':
        #         self.wpvec=[self.__i_current_waypoint[0],self.__i_current_waypoint[1]] # wpvec is direction of moving
        #     #print("WPvec: ", self.wpvec)
        # except Exception as e:
        #     print("Exception in wpvec: ", e)
        
        
        
        
        
        
        
        
        #print(self.wpvec)
        # try:
        #     if self.__i_next_waypoint is not 'TakeOff':
        #         self.wpvec = [self.__i_next_waypoint[0], self.__i_next_waypoint[1]]
        #         print("wpvec", self.__i_next_waypoint)
        #     else:
        #         self.wpvec = [0, 0]
        # except Exception as e:
        #     print("Exception while asigning wpvec")
        # try:
            
        #     self.wpvec=[self.i_current_waypoint[0]-self.i_previous_waypoint[0],self.i_current_waypoint[1]-self.i_previous_waypoint[1]] # wpvec is direction of moving
            
        # except:       
        #     self.wpvec= [0,-1]
            
            
            # print('waypoints not fetched, erorr in finding heading vector')
            # print('current waypoint',self.i_current_waypoint)
            # print('previous waypoint',self.i_previous_waypoint)

            #todo add some logic here

    def memory_map(self):
        #if not self.static_map:
        obs = np.array([])
        
        if self.__front_sensor.obstacle_vector_inertial.size != 0: #inertial map updation logic
            obs=np.array(self.__front_sensor.obstacle_vector_inertial[0:2,:])

        else:
            obs = np.array([])

        if np.size(obs) != 0:
            self.__np_mat_to_list=obs.T+np.array([self.__px,self.__py]) #array in inertial coordinates
            #print('np mat to list: ', self.__np_mat_to_list)
            if np.size(self.__matmap) == 0:
                print('MATMAP zero')
                self.__matmap = self.__np_mat_to_list
            else:
                for obs in self.__np_mat_to_list:
                    is_new_obstacle = True
                    for obs_inert in self.__matmap:
                        if np.allclose(obs, obs_inert, atol=0.8):
                            #print('obs already in map')
                            is_new_obstacle = False
                            break

                    if is_new_obstacle:
                        self.obstacle_map_count.append(1)
                        if len(self.obstacle_map_count) >= 5:
                            self.__matmap = np.vstack((self.__matmap, obs))
                            self.obstacle_map_count = []

        # if self.static_map:
        #     self.__np_mat_to_list=[[-35, 0],
        #                             [-35, 5],
        #                             [-35, -5],
        #                             [-35, -10],
        #                             [-35, 10]] #array in inertial coordinates
        #     #print('np mat to list: ', self.__np_mat_to_list)
        #     if np.size(self.__matmap) == 0:
        #         print('MATMAP zero')
        #         self.__matmap = self.__np_mat_to_list
        #     else:
                
        #         for obs in self.__np_mat_to_list:
        #             is_new_obstacle = True
        #             for obs_inert in self.__matmap:
        #                 if np.allclose(obs, obs_inert, atol=0.8):
        #                     #print('obs already in map')
        #                     is_new_obstacle = False
        #                     break

        #             if is_new_obstacle:
        #                 self.obstacle_map_count.append(1)
        #                 if len(self.obstacle_map_count) >= 5:
        #                     self.__matmap = np.vstack((self.__matmap, obs))
        #                     self.obstacle_map_count = []
        #             # else:
        #             #     idx = np.where(np.allclose(obs, self.__matmap, atol=0.5))
        #             #     self.obstacle_map_count[idx] += 1
        #             #     if self.obstacle_map_count[idx] >= SnAMain.OBSTACLE_MIN_OBS_COUNT:
        #             #         self.__matmap = np.vstack((self.__matmap, obs))

        #             #         self.obstacle_map_count.append(0)

        #     #print('mat map: ', self.__matmap, np.shape(self.__matmap))
        #     #return self.__matmap
    def guided_navigation(self):

        if(self.__currentMode == CopterMode.GUIDED and self.__speed < 0.5):
            #while self.__currentMode is CopterMode.GUIDED:
            
            # temp = self.__i_waypoints[0]
            # temp_wp_dist_vec=[self.__i_current_waypoint[0]-self.__px-temp[0],self.__i_current_waypoint[1]-self.__py-temp[1]] #used to find if the current vector to next waypoint i
            heading = [np.cos(self.__yaw), np.sin(self.__yaw)]
            bool_temp,self.dist_temp=self.obstacle_in_direction(heading,self.__np_mat_to_list,self.desired_angle,self.engaging_distance) # if any obstacle on the way
            #print('bool_temp, dist_temp',bool_temp, self.dist_temp, temp_wp_dist_vec)
            if(bool_temp and not self.rolled_right):#and (self.__vx*self.__vx+self.__vy*self.__vy)>2): #if obstacle needs to be avoided
                self.max_x = self.dist_temp + 8
                self.init_px = self.__px
                self.init_py = self.__py
                
                print("sending roll command")
                self.__guided_roll(2)
            else:
                self.rolled_right = True
                self.pitched_forward = False
        
            if self.rolled_right and not self.pitched_forward:
                print("############### max x #################", self.max_x)
                self.__guided_pitch(self.max_x)
                self.rolled_right = False
                #if(bool_temp and self.__vector.mag2d(temp_wp_dist_vec)>abs(self.dist_temp) and not self.pitched_forward):
                #self.__guided_pitch(0)
                self.pitched_forward = True
                self.__mode_changed_to_guided_from_auto = False        

    def update_mission(self):
        
        if Globals.get_param_val('SNS_ENABLE') == 1:
            self.__sense_and_stop = True
            #print(f"sns: {self.__sense_and_stop}")
        if Globals.get_param_val('SNA_ENABLE') == 1:
            self.__sense_and_avoid = True
            #print(f"sna: {self.__sense_and_avoid}")
        
        
        #self.get_mission_coordinates()
        
        self.update_vars() #for transferign variable values across classes clear

        if Globals.SITL:
            
            self.__front_sensor.x = self.__front_sensor.Y  # Gazebo axes are different from real world axes
            self.__front_sensor.y = self.__front_sensor.X
        
        #print("Front sensor: ", self.__front_sensor.x)
        self.__currentMode = self.__vehicle.vehicle_status().flight_mode()
        if self.__currentMode == CopterMode.GUIDED:
            self.__braked = 1
        else:
            self.__braked = 0

    
        self.__front_sensor.update_vehicle_states()#gets heading and rotation matrix
        self.heading_vector() #gives heading of drone based on velocity vector some other logic can also be interpreted
        self.__front_sensor.rotate_body_to_inertial_frame() #returns 3xn matrix containing n obstacles, in inertial frame, on drone origin
        
        
        # General realtime mapping
        if (self.__sense_and_avoid or self.__sense_and_stop):
            obs = np.array([])
                
            if self.__front_sensor.obstacle_vector_inertial.size != 0: #inertial map updation logic
                obs=np.array(self.__front_sensor.obstacle_vector_inertial[0:2,:])

            else:
                obs = np.array([])

            if np.size(obs) != 0:
                self.__np_mat_to_list=obs.T+np.array([self.__px,self.__py]) #array in inertial coordinates
            else:
                self.__np_mat_to_list=np.array([])

        if self.counter == 2:
            self.log_data()
            self.counter = 0
        else:
            self.counter+=1
        # self.__np_mat_to_list=[[10, -60],
        #                             [5, -60],
        #                             [0, -60],
        #                             [-5, -60],
        #                             [-10, -60]]

        # ########### Loiter Mode Stopping #############

        # if(self.__currentMode==CopterMode.LOITER):#and self.i_current_waypoint!=0 and self.terrainAlt>2 and self.i_current_waypoint!="TakeOff"):
        #     #if something is in heading direction, within desired angle and engaging distance, then engage brake/switch to guided
        #     #print('inside loop')
        #     #temp_wp_dist_vec=[self.i_current_waypoint[0]-self.px-temp[0],self.i_current_waypoint[1]-self.py-temp[1]] #used to find if the current vector to next waypoint i
        #     #print('inside check loop')
        #     self.desired_angle=15
        #     self.engaging_distance=20
        #     bool_temp,self.dist_temp=self.obstacle_in_direction(self.wpvec,self.__np_mat_to_list, self.desired_angle,self.engaging_distance) # if any obstacle on the way
        #     #print('wpvec',self.wpvec,'bool',bool_temp,'distance',self.dist_temp)
        #     #self.log_data()
        #     #time.sleep(0.1)
        #     if(bool_temp and (self.__vx*self.__vx+self.__vy*self.__vy)>2):# and self.vec.mag2d(temp_wp_dist_vec)>abs(self.dist_temp)): #if obstacle needs to be avoided
        #     #if(True):
        #         if(self.__currentMode==CopterMode.LOITER and self.__currentMode!= CopterMode.GUIDED):
        #             print('sent brake mode request')
        #             # self.__vehicle.mavlink().send_message(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,4))
        #             self.__change_mode_to_guided()
        #             print('vx,vy',self.__vx,',',self.__vy)
        #             print("Braked with Guided Mode Logic")
        #             #time.sleep(0.1)
                    

    ############## Auto mode logic ###############

        # print('px py in mission', self.__px, self.__py)
        if(self.__currentMode==CopterMode.AUTO and (self.__sense_and_stop or self.__sense_and_avoid)):#and self.i_current_waypoint!=0 and self.terrainAlt>2 and self.i_current_waypoint!="TakeOff"):
                #if something is in heading direction, within desired angle and engaging distance, then engage brake/switch to guided
                #self.log_data()
                #temp=self.__i_waypoints[0] # means inertial home
                #print("Error here")
                # print('inside loop')
                if(self.__speed>7 and not self.accelerated):
                    self.accelerated = True
                    self.acceleratedt1 = time.time()
            
                #temp_wp_dist_vec=[self.__i_current_waypoint[0]-self.__px-temp[0],self.__i_current_waypoint[1]-self.__py-temp[1]] #used to find if the current vector to next waypoint i
                #print('temp wp dist vec: ', temp_wp_dist_vec)
                #print('values are',temp,temp_wp_dist_vec)
                self.desired_angle=15
                self.engaging_distance=30
                bool_temp,self.dist_temp=self.obstacle_in_direction(self.wpvec,self.__np_mat_to_list,self.desired_angle,self.engaging_distance) # if any obstacle on the way
                #print('wpvec',self.wpvec,'bool',bool_temp,'distance',self.dist_temp)
                # print('temp_wp_dist+vec',temp_wp_dist_vec)
                # print('current_waypoint',self.__i_current_waypoint)
                # print('temp,',temp)
                # print('dist_temp',self.dist_temp)
                t1 = time.time()
                #print(bool_temp,self.__vector.mag2d(temp_wp_dist_vec),self.dist_temp,self.__speed)
                if (bool_temp and self.__speed > 2 and self.accelerated and t1-self.acceleratedt1>1): #if obstacle needs to be avoided: #if obstacle needs to be avoided
                #print('vx,vy',self.vx,',',self.vy)
                    # self.max_y = np.max(abs(self.__front_sensor.y))
                    # self.max_x = np.max(abs(self.__front_sensor.x))
                    # print("max y: ", self.max_y)
                    
                    if(self.__currentMode is CopterMode.AUTO and not self.__currentMode is CopterMode.GUIDED):
                        #self.log_data()
                        #self.Custom_break_mode()
                        #GALogging.info('Braked at px', self.__px)
                        print('triggered custom brake mode')
                        print('sent brake mode request')
                        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,4))
                        self.__change_mode_to_guided()
                        
                        #time.sleep(0.01)
                        self.__mode_changed_to_guided_from_auto = True
                        print('vx,vy',self.__vx,',',self.__vy)
                        print("Braked with Guided Mode Logic")
                        #GALogging.info("Braked after px: ", self.__px)
                        #self.roll_right = True
                        #self.__avoidance_mode == 1  
        else:
            self.accelerated = False
            self.acceleratedt1 = 0   

        # # # Avoidance Logic
        
        if self.__sense_and_avoid and self.__mode_changed_to_guided_from_auto :
            self.guided_navigation()
        

