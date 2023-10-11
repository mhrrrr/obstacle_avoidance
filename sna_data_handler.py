'''
@Description:
Data handling for the obstacle avoidance sensor data. 

@TODO: 
Handle multiple sensor 
Pack all the data in one stream
Handle the estimation errors and filtering portion 
Account for offsets (Least priority)
Map Making
'''

import math
import numpy as np
import socket
import struct

from src.vehicle.vehicle_ga3a.obstacle_avoidance.vector_math import AngularTransformation
from src.vehicle.vehicle_ga3a.obstacle_avoidance.vector_math import LateralTransformation
from src.vehicle.vehicle_ga3a.obstacle_avoidance.vector_math import Filter
from src.vehicle.vehicle_ga3a.obstacle_avoidance.vector_math import vector

from src.globals.ga_logging import GALogging
from src.globals.globals import Globals

class SnADataHandler:
    def __init__(self):
        #Assign to class params
        self.id = id
        self.delta_angle = 0
        self.max_range = 0
        self.min_range = 0
        self.init_angle = 0
        self.simulation=0
        self.init_index = 0

        self.simulation = 0

        self.master_array_length = 0

        #x and y are the mags
        self.x = np.array([])
        self.y = np.array([])
        self.z=np.array([])
        
        # For SITL
        self.X = np.array([])
        self.Y = np.array([])

        self.rad_x = 0
        self.rad_y = 0

        self.data = [0]
        self.count = 0

        self.transformations = AngularTransformation()
        
        #Vehicle states
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.px = 0
        self.py = 0
        self.pz = 0
        self.terrainAlt=0
        
        #Initialise the body to inertial matrix as 3x3 identity
        self.b2i_matrix = np.eye(3)
        self.obstacle_vector_inertial = np.zeros([2,2])

        # Calculate drone's heading vector (x,y,z)
        self.heading = np.zeros(3)

        # Socket related parameters
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = ('127.0.0.1', 12345)  # Set the desired server IP address and port
        self.sock.bind(self.server_address)
        self.vector1_format = 'I'  # Unsigned integer representing the size of vector1
        self.vector2_format = 'I'  # Unsigned integer representing the size of vector2
        self.float_format = 'f'  # Float format
    
    def receive_radar_data(self):
        data, address = self.sock.recvfrom(1024)

        # Deserialize the data
        buffer_ptr = 0

        # Read the size of vector1
        vector1_size = struct.unpack(self.vector1_format, data[buffer_ptr:buffer_ptr + struct.calcsize(self.vector1_format)])[0]
        buffer_ptr += struct.calcsize(self.vector1_format)

        # Read vector1 data
        vector1_data = struct.unpack(f'{vector1_size}{self.float_format}', data[buffer_ptr:buffer_ptr + vector1_size * struct.calcsize(self.float_format)])
        buffer_ptr += vector1_size * struct.calcsize(self.float_format)

        # Read the size of vector2
        vector2_size = struct.unpack(self.vector2_format, data[buffer_ptr:buffer_ptr + struct.calcsize(self.vector2_format)])[0]
        buffer_ptr += struct.calcsize(self.vector2_format)

        # Read vector2 data
        vector2_data = struct.unpack(f'{vector2_size}{self.float_format}', data[buffer_ptr:buffer_ptr + vector2_size * struct.calcsize(self.float_format)])

        # Process the received data
        obs_x = list(vector1_data)
        obs_y = list(vector2_data)

        # print("obstacle x: ", (obs_x))
        # print("obstacle y: ", (obs_y))

        self.update_obstacle_variables(obs_x, obs_y)
        

    def handle_sitl_sensor_data(self, id,delta_angle,max_range,min_range,init_angle):
        self.id = id
        self.delta_angle = delta_angle
        self.max_range = max_range
        self.min_range = min_range
        self.init_angle = init_angle

        data = self.data
        
        self.X = np.array([])
        self.Y = np.array([])
        self.Z = np.array([])

        if len(data)<=10:
            GALogging.info("Data not populated in other thread!!")

        else:

            #Create a bit mask for taking trusty readings of 90% 
            #@TODO: This should be based on CC code params because we can't afford to loose data in low range sensors
            sensor_binary_mask = [1.1*self.min_range<=abs(i)<=0.95*self.max_range for i in data]

            #Recreating the X and Y vectors 
            
            #If any reading falls in range
            #if any(sensor_binary_mask) == True:
            for i in range(len(data)):
                
                #Populate only those readings where the range is true else don't bother calculating
                if sensor_binary_mask[i] == True:
                    #print("Sensory Bitmask True")
                    #Piecing out magnitude to X-Y coordinates
                    self.count+=1
                    if(self.simulation==1):
                        if(self.X.size==0):
                            self.X=np.array([float(1*data[i]*math.cos(self.delta_angle*i + self.init_angle))])
                            self.Y=-1*np.array([float(1*data[i]*math.sin(self.delta_angle*i + self.init_angle))])
                            
                            
                        else:
                            self.X=np.concatenate((self.X,np.array([float(data[i]*math.cos(self.delta_angle*i + self.init_angle))])),axis=0)
                            self.Y=np.concatenate((self.Y,-1*np.array([float(1*data[i]*math.sin(self.delta_angle*i + self.init_angle))])),axis=0)
                    else:
                        if(self.X.size==0):
                            self.X=np.array([float(1*data[i]*math.sin(self.delta_angle*i + self.init_angle))])
                            self.Y=1*np.array([float(1*data[i]*math.cos(self.delta_angle*i + self.init_angle))])
                            
                            
                        else:
                            self.X=np.concatenate((self.X,np.array([float(data[i]*math.sin(self.delta_angle*i + self.init_angle))])),axis=0)
                            self.Y=np.concatenate((self.Y,1*np.array([float(1*data[i]*math.cos(self.delta_angle*i + self.init_angle))])),axis=0)


                    if(len(self.X)!=len(self.Y)):
                        GALogging.warn("Issue Here")
                        exit()
                        # self.X[i + self.init_index] = float(data[i]*math.cos(self.delta_angle*i + self.init_angle))
                        # self.Y[i + self.init_index] = float(data[i]*math.sin(self.delta_angle*i + self.init_angle))

    def filter_ground(self):
        pass

    def handle_radar_raw_data(self):
        pass

    def combine_multiple_readings(self):
        pass

    def update_obstacle_variables(self, obs_x, obs_y):
        x=obs_x
        y=obs_y
        temp_x = []
        temp_y = []
        # Consider readings only if x is less than 40m and y is less than 10m (40x10)
        for i in range(0,min(len(x),len(y))):
            if ((x[i] < (self.terrainAlt -4.0) or x[i] > (self.terrainAlt + 4.0)) and x[i] >1):
                temp_x.append(round(x[i],2))
                temp_y.append(round(y[i],2)) 
                
        x = temp_x
        y = temp_y
        
        
        if(len(x) != 0 and len(y) != 0):
            self.x = np.array(x)
            self.y = np.array(y)
        else:
            self.x = np.array([])
            self.y = np.array([])
            #print('length of one of the arrays is zero')

        

    def update_vehicle_states(self):
        #0.00023102760314941406 seconds

        #Calculate trigs and matrices before hand to reduce computational load
        self.transformations.calc_trig_values(self.roll,self.pitch,self.yaw)
        self.b2i_matrix = self.transformations.euler_zyx()
        #self.b2i_matrix = np.linalg.inv(self.transformations.euler_zyx1()) 
        #self.b2i_matrix = (self.transformations.euler_zyx())
        # Update heading of drone
        self.heading = np.dot(self.b2i_matrix, np.array([1,0,0]))

    def rotate_body_to_inertial_frame(self):
        #5.7697296142578125e-05 seconds
        self.obstacle_vector_inertial = np.array([])
        #These x and y are updated in the mainloop
        x = self.x
        y = self.y 
        z = np.zeros(np.size(x))

        try:
            obstacle_vector_body =  np.array([x,y,z])#np.array([x,y,z])# *ignore_obstacle_flag#3*n,axis 1 means uniqueness along columns
            #print("rot mat and obst vec body", self.b2i_matrix, obstacle_vector_body)
            
            #Convert from body to inertial frame in angles
            self.obstacle_vector_inertial = np.dot(self.b2i_matrix,obstacle_vector_body)#3xn,check for uniqueness of columns
        except Exception as e:
            GALogging.warn(f"Exception while rotating Obstacle Vector into Inertial frame: {e}")
        
