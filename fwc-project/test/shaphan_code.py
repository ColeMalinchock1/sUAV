#!/usr/bin/env python
# -*- coding: utf-8 -*-


#tasks to do
#add code to send to waypoint
#fps to 3
#battery voltage
#decrease wait time for initialization
#add carriage returns

"""
© Copyright 2015-2016, 3D Robotics.
vehicle_state.py: 

Demonstrates how to get and set vehicle state and parameter information, 
and how to observe vehicle attribute (state) changes.

Full documentation is provided at http://python.dronekit.io/examples/vehicle_state.html
"""

#for vehicle state
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions

import drone

#list of latitudes and longitudes through which to fly


#Dix big field
#latitudes=[35.76819560, 35.76786050, 35.76771690, 35.76809550]
#longitudes=[-78.66196450, -78.66168020, -78.66230250, -78.66250630]

#Dix 2
#latitudes=[35.76843500 , 35.76878320, 35.76870490, 35.76835670]
#longitudes=[-78.66287110, -78.66318230, -78.66347190, -78.66324660] 


def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

def grab_waypoints():
    missionlist = download_mission()
    latitudes=[]
    longitudes=[]
    for cmd in missionlist:
        if ( (cmd.x > 10) & (cmd.x < 45) ): #check that latitude is valid before adding
            latitudes.append(cmd.x)
            longitudes.append(cmd.y)
    return latitudes, longitudes



altitude=3.5

waypoint=0 #initialize waypoint to 0

'''for i in range(len(latitudes)):
  a_location= LocationGlobalRelative(latitudes[i], longitudes[i], altitude)

a_location = LocationGlobalRelative(-34.364114, 149.166022, 30)'''


#camera initialization
import jetson.utils #maybe unnecessary
import cv2
import time
import keyboard

cams = []
camera_id=0

def create_camera(csi_port): 
	cams.append( jetson.utils.videoSource( "csi://" + str(csi_port),argv=["--input-flip=none"] ))  #added flip code	
    

#cams.append( jetson.utils.videoSource( "csi://" + str(csi_port),argv=['--input-flip=none'] ))  #added flip code

#options={'width': 1280, 'height': 720, 'framerate': 30, 'flipMethod': 'rotate-180'}

#camera = jetson.utils.videoSource("csi://0", argv=['--input-flip=none'])

#jetson.utils.videoSource("csi://0", argv=['--input-width=1024', '--input-height=768'])

def get_image_size(camera_id):
	return cams[camera_id].GetWidth(), cams[camera_id].GetHeight()

def get_video(camera_id):
    return jetson.utils.cudaToNumpy(cams[camera_id].Capture()) #pycv2.cvtColor(jetson.utils.cudaToNumpy(cams[camera_id].Capture()),cv2.COLOR_RGB2BGR)

def get_video2(camera_id):
    return cv2.cvtColor(jetson.utils.cudaToNumpy(cams[camera_id].Capture()),cv2.COLOR_RGB2BGR)

def get_video3(img):
    return cv2.cvtColor([img],cv2.COLOR_RGB2BGR)

def close_cameras():
    for cam in cams:
        cam.Close()  
  
create_camera(0)

get_image_size(0)

image_width = cams[camera_id].GetWidth()
image_height = cams[camera_id].GetHeight()

timestr = time.strftime("%Y%m%d-%H%M%S")
image_writer = cv2.VideoWriter("test_video"+timestr+".avi",cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 5,(image_width,image_height))


#was image_writer = cv2.VideoWriter("test_video4.avi",cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 20,(image_width,image_height))

#frame rate was 25

#lidar5b initialization*****************************
import serial
import matplotlib.pyplot as plt
#matplotlib.use('Agg')
import numpy as np
import lidar5b

RUN_2D = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x01, 0x00, 0x03]
RUN_3D = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x08, 0x00, 0x0A]
RUN_DUAL = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x07, 0x00, 0x05]
COMMAND_STOP = [0x5A, 0x77, 0xFF, 0x02,0x00,0x02,0x00,0x00]
#SENSITIVITY = [0x5A, 0x77, 0xFF, 0x02,0x00,0x11,0x00,0x00]
#SENSITIVITY = [0x5A, 0x77, 0xFF, 0x02,0x00,0x11,0x32,0x21] #sensitivity of 50
SENSITIVITY = [0x5A, 0x77, 0xFF, 0x02,0x00,0x11,0x64,0x77] #sensitivity of 100
#SENSITIVITY = [0x5A, 0x77, 0xFF, 0x02,0x00,0x11,0x96,0x85]  #sensitivity of 150

d1=50 #critical distance in cm below which to stop vehicle
d2=250 #distance in cm below which to steer vehicle


'''checksum0=0
for i in range(3,7):
	print(SENSITIVITY[i])
	checksum0=checksum0 ^ SENSITIVITY[i]

print(checksum0)'''



HEADER1, HEADER2, HEADER3, LENGTH_LSB, LENGTH_MSB, PAYLOAD_HEADER, PAYLOAD_DATA, CHECKSUM, LEFT_EDGE, RIGHT_EDGE = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
POS_CYGBOT_HEADER, POS_DEVICE, POS_ID, POS_LENGTH_1, POS_LENGTH_2, POS_PAYLOAD_HEADER = 0, 1, 2, 3, 4, 5
PAYLOAD_POS_HEADER, PAYLOAD_POS_DATA = 0, 1
NORMAL_MODE = 0x5A
PRODUCT_CODE = 0x77
DEFAULT_ID = 0xFF
HEADER_LENGTH_SIZE = 5

buffercounter, CPC, lengthLSB, lengthMSB, data_length = 0, 0, 0, 0, 0
step = HEADER1
receivedData = []

skip_bytes=20*2  #a value of 20 skips 15 degs (15/.75) on each side. A factor of 2 is added to account for two bytes per datapoint (LSB and MSB)


def Parser(data):
    global step, CPC, lengthLSB, lengthMSB, data_length, buffercounter, receivedData
    if step != CHECKSUM:  # CPC is a variable for storing checksum. If it is not a checksum part, XOR operation is performed on each data and then stored.
        CPC = CPC ^ data

    if step == HEADER1 and data == NORMAL_MODE:
        step = HEADER2
        
    elif step == HEADER2 and data == PRODUCT_CODE:
        step = HEADER3
        
    elif step == HEADER3 and data == DEFAULT_ID:
        step = LENGTH_LSB
        CPC = 0
        
    elif step == LENGTH_LSB:
        step = LENGTH_MSB
        lengthLSB = data
        
    elif step == LENGTH_MSB:
        step = PAYLOAD_HEADER
        lengthMSB = data
        data_length = ((lengthMSB << 8) & 0xff00) | (lengthLSB & 0x00ff)
        
    elif step == PAYLOAD_HEADER:
        step = PAYLOAD_DATA
        if data_length == 1:
            step = CHECKSUM
        buffercounter = 0
        receivedData = []
        
    elif step == PAYLOAD_DATA:
        receivedData.append(data)
        buffercounter = buffercounter+1
        if buffercounter >= data_length - 1:
            step = CHECKSUM
            
    elif step == CHECKSUM:
        step = HEADER1
        
        if CPC == data:
            return True
    else:
        step = HEADER1
        return False


def Parser2(data):
    global step, CPC, lengthLSB, lengthMSB, data_length, buffercounter, receivedData
    if step != CHECKSUM:  # CPC is a variable for storing checksum. If it is not a checksum part, XOR operation is performed on each data and then stored.
        CPC = CPC ^ data

    if step == HEADER1 and data == NORMAL_MODE:
        step = HEADER2
        
    elif step == HEADER2 and data == PRODUCT_CODE:
        step = HEADER3
        
    elif step == HEADER3 and data == DEFAULT_ID:
        step = LENGTH_LSB
        CPC = 0
        
    elif step == LENGTH_LSB:
        step = LENGTH_MSB
        lengthLSB = data
        
    elif step == LENGTH_MSB:
        step = PAYLOAD_HEADER
        lengthMSB = data
        data_length = ((lengthMSB << 8) & 0xff00) | (lengthLSB & 0x00ff)


    elif step == PAYLOAD_HEADER:
        step = LEFT_EDGE  # PAYLOAD_DATA
        if data_length == 1:
            step = CHECKSUM
        buffercounter = 0
        
    elif step == LEFT_EDGE:
        #receivedData.append(data)
        buffercounter = buffercounter+1
        if buffercounter >= skip_bytes:
            step = PAYLOAD_DATA
        receivedData = []
        
    elif step == PAYLOAD_DATA:
        receivedData.append(data)
        buffercounter = buffercounter+1
        if buffercounter >= data_length - 1 - skip_bytes:
            step = RIGHT_EDGE

    elif step == RIGHT_EDGE:
        #receivedData.append(data)
        buffercounter = buffercounter+1
        if buffercounter >= data_length - 1:
            step = CHECKSUM
            
    elif step == CHECKSUM:
        step = HEADER1
        
        if CPC == data:
            return True
    else:
        step = HEADER1
        return False
    

x2 = np.linspace(-60,60,num=161)

#x = np.linspace(1,322,num=322)
#x2 = np.linspace(1,161,num=161)
y = np.linspace(1,600,num=161) #was 6 * x2

# enable interactive mode
##plt.ion()

# creating subplot and figure
##fig = plt.figure()
##ax = fig.add_subplot(111)
##line1, = ax.plot(x2, y,ls='None',markersize=5,marker='o',)

# setting labels
##plt.xlabel("X-axis")
##plt.ylabel("Y-axis")
##plt.title("Updating plot...")

ser = serial.Serial(  # Port settings
    port= '/dev/ttyUSB0', 
    baudrate=115200, # recommend 250,000
    #baudrate=250000,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.5 #was 0.1
)

print("connected to: " + ser.portstr)
time.sleep(1)
print ("Is port open ? ",ser.isOpen() )   #returns true?




#define navigation commands

def goto(latitudes,longitudes,altitude,waypoint,guided2):
    #print("Guided2: ",guided2)   
    if guided2<1: #if outside WP_radius, check if guided was sent previously and send if needed
        airspeed=1
        targetLocation = LocationGlobalRelative(latitudes[waypoint], longitudes[waypoint], altitude)
        vehicle.simple_goto(targetLocation,airspeed) #send to waypoint
        print("Sending command to waypoint ",waypoint+1)
    return waypoint

def waypoint_check(latitudes,longitudes,altitude,waypoint,guided2):
    WP_radius=2.5  #waypoint radius
    airspeed=1
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = LocationGlobalRelative(latitudes[waypoint], longitudes[waypoint], altitude)
    #targetDistance = get_distance_metres(currentLocation, targetLocation)
    remainingDistance=drone.get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
    print("Distance to waypoint %0.0f: %0.1f m" %(waypoint+1, remainingDistance))
    if remainingDistance<=WP_radius: #see if within tolerance
        print("Reached waypoint ",waypoint+1)
        waypoint+=1 #increment waypoint
        if waypoint< (len(latitudes)-1): #if still have waypoints to go, update target and send to next
            targetLocation = LocationGlobalRelative(latitudes[waypoint], longitudes[waypoint], altitude) 
            vehicle.simple_goto(targetLocation,airspeed) #send to waypoint
    #elif guided2<1: #if outside WP_radius, check if guided was sent previously and send if needed
        #vehicle.simple_goto(targetLocation,airspeed) #send to waypoint
        #print("Sending command to waypoint ",waypoint+1)
    return waypoint


def send_movement_command_YAW(heading): #from AI drones
    #global vehicle  #NEED TO DOUBLE CHECK, MAYBE NEED ONLY WITH IMPORT
    speed = 0 
    #direction = 1 #direction -1 ccw, 1 cw
    
    #heading 0 to 360 degree. if negative then ccw 
    if heading < 0:
        heading = heading*-1
        direction = -1
    else: direction=1
    
    #print("Sending YAW movement command with heading: %0.2f; direction: "% heading,direction )


    #point drone into correct heading 
    msg = vehicle.message_factory.command_long_encode(
        0, 0,       
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0,          
        heading,    
        speed,      #speed deg/s
        direction,  
        1,          #relative offset 1
        0, 0, 0)    

    # send command to vehicle
    vehicle.send_mavlink(msg)
    #Vehicle.commands.flush()


def send_movement_command_XYA(velocity_x,altitude):
    #global vehicle
    velocity_y=0

    #velocity_x positive = forward. negative = backwards
    #velocity_y positive = right. negative = left
    #velocity_z positive = down. negative = up (Yes really!)

    #print("Sending XYZ movement command with v_x(forward/backward): %f " %velocity_x)

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,      
        0, 0,    
        mavutil.mavlink.MAV_FRAME_BODY_NED,  #relative to drone heading pos relative to EKF origin
        0b0000111111100011, #ignore velocity z and other pos arguments
        0, 0, altitude,
        velocity_x, velocity_y, 0, 
        0, 0, 0, 
        0, 0)    

    vehicle.send_mavlink(msg)
    #Vehicle.commands.flush()


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
   # global vehicle  #NEED TO DOUBLE CHECK, MAYBE NEED ONLY WITH IMPORT

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def dummy_printer(x):
    pass

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#code to run

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle. 
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=57600, timeout=60, status_printer=dummy_printer)  #SRJ added baud, timeout, and status_printer

#create listener to extract RC channel data
'''@vehicle.on_message('RC_CHANNELS')
def listener(self, name, message):
     global chan9
     global chan10
     #print('Chan1: %s' % message.chan1_raw) 
     #print('Chan9 message: %s' % message.chan9_raw)  
     chan9=message.chan9_raw
     chan10=message.chan10_raw'''


#deleted stuff to print

#give time to read RC channel data
time.sleep(1)
'''chan9_0=chan9 #initialize chan9 variable
chan10_0=chan10 #initialize chan10 variable'''
recording=1 #tunrn recording state off initially


#read waypoints from mission
print("\nDownloading waypoint coordinates from vehicle")
[latitudes, longitudes]= grab_waypoints()
print("Waypoint latitudes: ",latitudes)
print("Waypoint longitudes: \n",longitudes)
time.sleep(2) #give time to look at waypoints



vehicle.mode = VehicleMode("STABILIZE")

start=time.time()  #record start time

z=0

#run loop initially to check parameters
while  (time.time() - start < 45):
	try:
#for i in range(10):
		print(" Global Location (relative altitude): %s"%vehicle.location.global_relative_frame)    
		print(" Attitude: %s" % vehicle.attitude)
		print(" Velocity: %s" % vehicle.velocity)
		print(" GPS: %s" % vehicle.gps_0)
		print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
		print(" Rangefinder: %s" % vehicle.rangefinder)
		print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
		print(" Heading: %s" % vehicle.heading)
		print(" Is Armable?: %s" % vehicle.is_armable)
		print(" Mode: %s" % vehicle.mode.name)    # settable
		print(" Armed: %s" % vehicle.armed)    # settable
		print(" Battery: %s" % vehicle.battery)
		#print(" Ch1 (roll): %s" % vehicle.channels['2'])
		#print(" Ch9 (video): %s" % vehicle.channels['3'])
		#print(" Ch9 (global variable): %s" % chan9)
		#print(" Ch1: %s" % vehicle.chan1_raw)
		#print(" Ch1: %s" % Channel(1))
		#def Channel(self,chNum)
		#print(message.chan1_raw
		time.sleep(1)
	except KeyboardInterrupt:
		pass
		break



#Arm and take of to altitude of 5 meters***********************
#arm_and_takeoff(2.5)



vehicle.mode = VehicleMode("GUIDED")

#wait until vehicle mode is guided
while not(vehicle.mode.name=="GUIDED"):
	print("waiting for guided mode")
	time.sleep(1)

#send command to adjust sensitivity
ser.write(SENSITIVITY) # Mode settings
time.sleep(2)

#send command to grab data from lidar5b
ser.write(RUN_2D) # Mode settings
print("send : ", RUN_2D)
print("Just sent command")
time.sleep(3)
print("Slept for 3 seconds")

z=0
q=0
v=0

control=[0,0,0,0,0,0,0]

start=time.time()  #record start time
T0=start
delta_t=np.array([0.000,0.000,0.000]) #delta_t=np.array([0.000,0.000,0.000, 0.000,0.000,0.000])


if recording==1:
	img = get_video(0)
	cv2.waitKey(15) #need if not showing image, was 10
	#cv2.imshow("camera", img)
	img2 = cv2.cvtColor(img,cv2.COLOR_RGB2BGR) #convert fro$
	#cv2.imshow("camera2", img2)
	img2 = cv2.rotate(img2, cv2.ROTATE_180)
	#cv2.putText(img2, "Min y: "+ str(control[3]), (50, 50)$
	#cv2.waitKey(15) #was 10
	image_writer.write(img2)  #write frame to movie


print("Sending waypoint command")
guided2=0
waypoint=goto(latitudes,longitudes,altitude,waypoint,guided2)

print("Starting while loop")

while ( (vehicle.mode.name=="GUIDED") & (time.time() - start < 500) ): # value here
	try:
		readdata = ser.read(ser.in_waiting) #readline()#readdata = ser.readline()
		#if len(readdata)>2000
			#print("readdata>2000")
		#print("Size readdata: ",len(readdata))
		for i in range(len(readdata)):
			if Parser2(readdata[i]):
				#print("Parser location: ",i)
				if z==5:  #perform every 20th? iteration was at 7
					#if np.average(delta_t)>.9:
					#	break						
					receivedData2 = []  #initialize variable
					for j in range(0,len(receivedData),2):  #combine every two cells
						receivedData2.append(receivedData[j] << 8  | receivedData[j+1] )
					z=0 #reset iterator
					if 'receivedData2' in locals(): #check if variable exists
						control = lidar5b.process_lidar(receivedData2,x2,d1,d2)
					else:
						print('No data from lidar5b; exiting loop')
						break
					speed=control[0]
					yaw=control[1]
					guided=control[2]
					guided2=control[4]
					if guided>0:
						waypoint=goto(latitudes,longitudes,altitude,waypoint,guided2)
						if waypoint>(len(latitudes)-1): #len is 1-based while waypoint is 0-based
							#e.g., 4 latitudes -> len(latitudes) = 4 -> stop if waypoint>3
							break
					else:
						send_movement_command_YAW(yaw) #from AI drones
						send_movement_command_XYA(speed,altitude)
					#time.sleep(1)
					while  (time.time()-T0<0.2):
						pass
					delta_t=np.roll(delta_t,-1)
					delta_t[2]= time.time()-T0 #round( time.time()-T0 , 3)
					T0=time.time()
					if q==2:  #perform every other iteration of z loop
						q=0
						rangedist= vehicle.rangefinder.distance
						if rangedist<1:
							altitude=altitude+0.1
						elif  ((rangedist>3) & (rangedist<4)):
							altitude=altitude-0.1
						print("Min Y: %0.0f  Min Y Raw: %0.0f "%(control[3],control[5]) )
						print("Guided: ",control[2]," Yaw: ",control[1]," Speed: ",control[0])
						if control[2] > 0:
							waypoint=waypoint_check(latitudes,longitudes,altitude,waypoint,guided2)
						#print(" Global Location (relative altitude): %s"%vehicle.location.global_relative_frame)    
						#print(" Attitude: %s" % vehicle.attitude)
						#print(" Velocity: %s" % vehicle.velocity)
						##print(" GPS: %s" % vehicle.gps_0)
						#print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
						#print(" Rangefinder: %s" % vehicle.rangefinder)
						#print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
						#print(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
						#print(" Heading: %s" % vehicle.heading)
						#print(" Is Armable?: %s" % vehicle.is_armable)
						##print(" Mode: %s" % vehicle.mode.name)    # settable
						#print(" Armed: %s" % vehicle.armed)    # settable
						##print(" Battery: %s" % vehicle.battery)
						#print(" Ch9 (global variable): %s" % chan9)
						#print(" Ch10 (global variable): %s" % chan10)
						#print("\n")
						'''if abs(chan9-chan9_0)>100: #check for change in chan9
							if chan9>1800: #start recording
								timestr = time.strftime("%Y%m%d-%H%M%S")
								image_writer = cv2.VideoWriter("test_video"+timestr+".avi",cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 20,(image_width,image_height))
								recording=1
							elif chan9<1200: #stop recording
								recording = 0
						chan9_0=chan9 #maybe not necessary
						if chan10-chan10_0>100: #check for positive change in chan10
							if chan10>1200: #change mode to land
								vehicle.mode = VehicleMode("LAND")
							#elif chan10<1200: #stop recording
							#	pass
						chan10_0=chan10'''
						print("Delta T:" ,delta_t)
					else: q+=1
				else:
					z+=1 
				if recording==1:
					if v==2:
						v=0
						#print("recording frame")
						img = get_video(0)
						cv2.waitKey(15) #need if not showing image, was 10
						#cv2.imshow("camera", img)
						img2 = cv2.cvtColor(img,cv2.COLOR_RGB2BGR) #convert from RGB to BGR for use in Open CV
						#cv2.imshow("camera2", img2)
						img2 = cv2.rotate(img2, cv2.ROTATE_180)
						#cv2.putText(img2, "Min d: "+ str(control[3]), (50, 50), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 255), 3, cv2.LINE_AA)
						cv2.putText(img2, "Min d: "+ str(control[3]) + "cm Guided: " + str(control[2]) + " Yaw: " + str(control[1]) + " Speed: " + str(control[0]) , (50, 50), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 255), 3, cv2.LINE_AA) 
						cv2.circle(img2, (int(control[6]/120*1280), 360), 20, (0, 255, 0), thickness=-1, lineType=8, shift=0)
						#cv2.waitKey(15) #was 10
						image_writer.write(img2)  #write frame to movie
						#cv2.waitKey(5)
						#keyCode = cv2.waitKey(5) & 0xFF
					else:v+=1
				else: time.sleep(.01)
				if len(readdata)>3000:  #break out of for loop, i.e., don't continue reading all readdata
					print("File size large: breaking")					
					break
	except KeyboardInterrupt:
		pass
		break


ser.write(COMMAND_STOP)
ser.close()

if not (vehicle.mode.name=="LAND"):
	print("Setting RTL mode...")
	vehicle.mode = VehicleMode("RTL")



#Close vehicle object before exiting script
print("\nClose vehicle object")
vehicle.close()

print("\nClose cameras")
close_cameras()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")

#python3 'Documents/OWCF Research/Navigation with record & import 4cc v5b.py' --connect '/dev/ttyTHS1'



