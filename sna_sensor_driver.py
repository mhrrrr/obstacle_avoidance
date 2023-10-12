'''
@Description:
Radar data and SITL sensor data are available in this code.

@TODO:
Add code to handle two radar's data together
'''


import os
import math
import can
import time
import numpy as np
import struct
import threading
from cmath import e
import socket
import numpy as np
import sys
import subprocess

from src.util.task_scheduler import SchedulerTask
from src.globals.globals import Globals
from src.globals.ga_logging import GALogging
from src.globals.event_bus1 import EventNotifier

class SnASensorDriver:
    def __init__(self, driverType):

        self.__driver_type = driverType
        self.HOST, self.PORT = "localhost", 8080
        self.raw_data = [0]*10
        self.lid = []
        self.ranges_=np.array([])
        self.__notifier = EventNotifier()
        self.can_bus = None
        self.can_data_received = None
        self.can_node_base_id = None
        self.can_channel = 'can0'
        self.b=[]
        self.numObj = 0
        self.final_x = []
        self.final_y = []
        self.obj_x_dis = []
        self.obj_y_dis = []
        self.obj_radial_speed = []
        self.obj_lateral_speed = []
        self.obj_rcs = []
        self.final_radial_speed = []
        self.final_lateral_speed = []
        self.final_rcs = []
        self.obstacle_data = dict()
        self.__del_time = 0.0001

        self.output = None
        self.data = None
        self.b = None
        self.candump_process = None


        #Lidar parameters
        self.range_min = 0
        self.range_max = 0
        self.angle_min = 0
        self.angle_max = 0
        self.step = 0

        # self.__process_radar_data_thread = SchedulerTask(self.process_radar_data, name='Radar Data processing', delay=0, time_period=self.__del_time)
        #self.__process_radar_data_thread = threading.Thread(target=self.process_radar_data)

    def initialize(self):
        self.can_bus = can.interface.Bus(channel=self.can_channel, bustype='socketcan',
                                            bitrate=250000)
                                   
        #self.__process_radar_data_thread.start()
   
    
    def connect_sitl_sensor(self):
        """Connects to the sensor and starts the scan request
        """
        #SITl
        if self.HOST != None:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.HOST, self.PORT))
            GALogging.info("Bridge initialised")


    def update_sitl_sensor(self):
        """This is for Gazebo SITL. It just decodes the sockets 
        """
        #Empty the array for new fetch
        lid = []
        #empty the data
        data = None
        
        #print("In Update SITL Sensor")
        #print(threading.active_count())
        #Keep fetching until the data is filled with 64 indices
        while len(lid)<=64:
            data = self.s.recv(4).decode("utf-8") 
            #If we recieve a new packet of data
            if data == 'new ':
                if len(lid) == 64:
                    self.raw_data = lid
                    #print('printing dat',self.raw_data)
                    lid = []             
            elif data != None:
                lid.append(float(data))

    
    
    def attach(self, observer):
        self.__notifier.attach(observer)

    def detach(self, observer):
        self.__notifier.detach(observer)

    def handle_notified_object(self, key, val):
        if key is 'CAN_DATA':
            pass
            #self.process_radar_data(val)
            #print("radar handler: ", val)
            #self.can_data_received = val
            #self.process_radar_data(val)
            # if self.can_data_received is not None:
            #     self.obstacle_dictionary = self.obstacle_data
                #print("obst dict", self.obstacle_dictionary)
            

    def process_radar_data(self):
        while True:
        #print("inside loop")
        
            if self.can_data_received is not None:
                #print("Data received in sensor driver ", self.can_data_received)
                self.can_node_base_id = self.can_data_received.arbitration_id
                if self.can_node_base_id == 1546:
                    data=self.can_data_received.data
                    #print("data 60A", data)
                    self.c = struct.unpack('>BBBBBBBB', data)
                    self.numObj = self.c[0]
                    #print("num obj: ", self.numObj)

                if self.can_node_base_id == 1547:
                    #self.c = self.can_data_received
                    data = self.can_data_received.data
                    #print("data 60B", data)
                    # for i in range(len(self.c)):
                    #     self.c[i] = int(self.c[i], 16)
                    self.c = struct.unpack('>BBBBBBBB', data)                    
                    self.obj_id = self.c[0]
                    self.obj_radial_distance = (self.c[1]*32+(self.c[2]>>3))*0.2 - 500        
                    self.obj_lateral_range = ((self.c[2]&7)*256 + self.c[3])*0.2-204.6
                    self.obj_radial_temp_speed = (self.c[4]*4+(self.c[5]>>6))*0.25-128
                    self.obj_lateral_temp_speed = ((self.c[5]&63)*8+(self.c[6]>>5))*0.25-64
                    self.obj_temp_rcs = self.c[7]*0.5-64
                    #print(self.obj_rcs)
                    self.obj_x_temp_dis = (self.c[1]*32+math.floor(self.c[2])/8)*0.2-500
                    self.obj_y_temp_dis = ((self.c[2]%8)*256+self.c[3])*0.2-204.6
                    #print(self.obj_x_temp_dis, self.obj_y_temp_dis)
                    #set range here
                    
                    self.obj_x_dis.append(self.obj_x_temp_dis)
                    self.obj_y_dis.append(self.obj_y_temp_dis)
                    self.obj_radial_speed.append(self.obj_radial_temp_speed)
                    self.obj_lateral_speed.append(self.obj_lateral_temp_speed)
                    self.obj_rcs.append(self.obj_temp_rcs)
                    
                    if len(self.obj_x_dis) >= self.numObj:
                        self.final_x = self.obj_x_dis
                        self.final_y = self.obj_y_dis
                        
                        #print("final_x, final_y", self.final_x)
                        self.final_radial_speed = self.obj_radial_speed
                        self.final_lateral_speed = self.obj_lateral_speed
                        self.final_rcs = self.final_rcs
                        self.obstacle_data = dict(x=self.final_x, y=self.final_y, radial_speed=self.final_radial_speed, lateral_speed=self.final_lateral_speed, rcs=self.final_rcs)
                        #print("obstacle data: ", self.obstacle_data)
                        self.obj_x_dis = []
                        self.obj_y_dis = []
                        self.obj_radial_speed = []
                        self.obj_lateral_speed = []
                        self.obj_rcs = []

                if self.numObj == 0:
                    self.final_x = []
                    self.final_y = []
                    self.final_radial_speed = []
                    self.final_lateral_speed = []
                    self.final_rcs = []
                    self.obstacle_data = dict(x=[], y=[], radial_speed=[], lateral_speed=[], rcs=[])
                self.__notifier.notify('OBSTACLE_DATA', self.obstacle_data)

                #return self.obstacle_data
        #     self.output = self.candump_process.stdout.readline()
        #     # if not self.output:
        #     #     break
        #     self.output = self.output.decode().strip()
        #     # #print("Output", self.output)
        #     self.data = self.output.split(' ')

        #     if self.data[2] == '60A':
        #         self.numObj = self.data[7]
        #         self.numObj = int(self.numObj, 16)
        #         self.obj = True
        #         # print(self.numObj)
        #         # print(type(self.numObj))

        #     if self.data[2]=='60B':
        #         self.b = self.data[7:]
        # #        b = ['30','54','6C','11','80','65','80','A2']update_vars
        #         for i in range(len(self.b)):
        #             self.b[i] = int(self.b[i],16)
        #         self.obj_id = self.b[0]
        #         self.obj_radial_distance = (self.b[1]*32+(self.b[2]>>3))*0.2 - 500        
        #         self.obj_lateral_range = ((self.b[2]&7)*256 + self.b[3])*0.2-204.6
        #         self.obj_radial_temp_speed = (self.b[4]*4+(self.b[5]>>6))*0.25-128
        #         self.obj_lateral_temp_speed = ((self.b[5]&63)*8+(self.b[6]>>5))*0.25-64
        #         self.obj_temp_rcs = self.b[7]*0.5-64
        #         #print(self.obj_rcs)
        #         self.obj_x_temp_dis = (self.b[1]*32+math.floor(self.b[2])/8)*0.2-500
        #         self.obj_y_temp_dis = ((self.b[2]%8)*256+self.b[3])*0.2-204.6
        #         #print(self.obj_x_temp_dis, self.obj_y_temp_dis)
        #         #set range here
        #         #if(self.obj_x_temp_dis*self.obj_x_temp_dis+self.obj_y_temp_dis*self.obj_y_temp_dis<1600):
        #         self.obj_x_dis.append(self.obj_x_temp_dis)
        #         self.obj_y_dis.append(self.obj_y_temp_dis)
        #         self.obj_radial_speed.append(self.obj_radial_temp_speed)
        #         self.obj_lateral_speed.append(self.obj_lateral_temp_speed)
        #         self.obj_rcs.append(self.obj_temp_rcs)
                
        #         if len(self.obj_x_dis) >= self.numObj:
        #             self.final_x = self.obj_x_dis
        #             self.final_y = self.obj_y_dis
        #             #print(self.final_x, self.final_y)
        #             self.final_rad_speed = self.obj_radial_speed
        #             self.final_lat_speed = self.obj_lateral_speed
        #             self.final_rcs = self.obj_rcs
        #             self.obj_x_dis = []
        #             self.obj_y_dis = []
        #             # self.final_x =[]
        #             # self.final_y = []
        #             self.obj_radial_speed = []
        #             self.obj_lateral_speed = []
        #             self.obj_rcs = []
        #             # print("length", len(self.obj_x_dis))
        #             self.obstacle_data = dict(x=self.final_x, y=self.final_y, radial_speed=self.final_radial_speed, lateral_speed=self.final_lateral_speed, rcs=self.final_rcs)
        #             print("obstacle data: ", self.obstacle_data)

        #             #self.obj_y_dis.insert(i, self.obj_y_temp_dis)
        #         #time.sleep(0.03)
        #             # self.obj_x_dis[i] = self.obj_x_temp_dis
        #             # self.obj_y_dis[i] = self.obprint((self.obj_x_dis),(self.obj_y_dis))
        #         # print("length: ", len(self.obj_x_dis))j_y_temp_dis
        #             #time.sleep(0.003)
        #         #print("radial speed", self.obj_radial_speed)
        #         #print("lateral speed", self.obj_lateral_speed)
        #         #csv_line = str(time.ctime())+','+str(round(obj_x_dis,1))+','+str(round(obj_y_dis,1))+','+str(round(obj_lateral_range,1))+','+str(round(obj_radial_distance,1))
        #         #writer.writerow(csv_line)
        #         #f.write(csv_line+'\n')
        #         # if self.numObj == len(self.obj_x_dis):
                
        #         #     print((self.obj_x_dis),(self.obj_y_dis))
        #         #     print("length: ", len(self.obj_x_dis))
        #         #print("1 while completed")
        #     if self.numObj == 0:
        #         self.final_x = []
        #         self.final_y = []
        #         self.final_rad_speed = []
        #         self.final_lat_speed = []
        #         self.final_rcs = []       
        #         self.obstacle_data = dict(x=[], y=[], radial_speed=[], lateral_speed=[], rcs=[])
            
        #     self.__notifier.notify('OBSTACLE_DATA', self.obstacle_data)        