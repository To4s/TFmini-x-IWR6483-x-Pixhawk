from smbus2 import SMBus, i2c_msg
import time
import math
from pymavlink import mavutil

DIST_CM = 0 #defining distance in cm
DEV_ADDR = 0x10 #default value, change if modified
TEMP_C = None #Temperature in degree Celcius

start_time = int(round(time.time() * 1000))
current_milli_time = lambda: int(round(time.time()*1000) - start_time)

#function to send DISTANCE_SENSOR message to mavlink
def send_distance_sensor_msg():
	global DIST_CM
	current_time_ms = current_milli_time()
	conn.mav.distance_sensor_send(
		current_time_ms,
		0,
		1200,
		DIST_CM,
		0,
		0,
		12,
		255,
		math.radians(120),
		math.radians(0),
		[1.0,0.0,0.0,0.0]
	)
	print('Updated Distance is sent as DISTANCE_SENSOR')

#setting up MAVLink connection
conn = mavutil.mavlink_connection(
	device='/dev/ttyAMA4',
	baud=57600,
	autoreconnect = True,
	source_system = 1,
	source_component = 93,
	force_connected=True,
)

#checking heartbeat from Pixhawk
print('Waiting for heartbeat from pixhawk')
conn.wait_heartbeat()
print(f'Heartbeat received from system {conn.target_system} component {conn.target_component}')

def obtainData():
	global DIST_CM, TEMP_C
	with SMBus(1) as bus:
		#setting units of 9 byte distance value to be in cm  
		write = i2c_msg.write(DEV_ADDR, [0x5A,0x05,0x00,0x01,0x60])
		bus.i2c_rdwr(write)
		
		#reading 9 byte data response from Lidar
		read = i2c_msg.read(DEV_ADDR, 9)
		bus.i2c_rdwr(read)
		data = list(read)
	
	print(data) #for debugging purpose
	
	#Performingchecksum test
	
	#obtaining Checksum value
	chkSum = data.pop()
	print(f'obtained checksum value: {chkSum}')
	
	#recalculating checksum value
	calcChkSum = sum(data)
	print(f'calculated checksum value: {calcChkSum}')
	calcChkSum = calcChkSum & 0xFF #Taking lower 8 bits
	
	#Verifying checksum value
	if (chkSum == calcChkSum):
		DIST_CM = data[2] | (data[3] << 8)
		print(f'Distance (cm): {DIST_CM}')
		TEMP_C = data[6] + (data[7] << 8)
		TEMP_C = (TEMP_C >> 3) - 256
		print(f'Temperature (°C): {TEMP_C}')
	else:
		DIST_CM = -1
		TEMP_C = -1
		print('Invalid dataframe')

while(True):
	conn.mav.heartbeat_send(
		mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
		mavutil.mavlink.MAV_AUTOPILOT_INVALID,
		0,0,0
	)
	try:
		obtainData()
		send_distance_sensor_msg()
		time.sleep(1)
	except KeyboardInterrupt:
		print('Program interrupted')
		exit()
