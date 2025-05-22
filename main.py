from smbus2 import SMBus, i2c_msg
import time
import math
from pymavlink import mavutil

import os
os.environ["MAVLINK20"] = "1"

import serial
import numpy as np
import threading
import sys

######################################################
##  Variables                                       ##
######################################################

# ---------------------------For OBSTACLE_DISTANCE_3D----------------------------------

# sensor parameters
DEPTH_RANGE_M = [0.1, 50]

obstacle_distance_msg_hz = 60

start_time =  int(round(time.time() * 1000))
current_milli_time = lambda: int(round(time.time() * 1000) - start_time)

mavlink_x = []
mavlink_y = []
mavlink_z = []

configFileName = 'testconfig1.cfg' # Change path to .cfg file accordingly
CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2**15,dtype = 'uint8')
byteBufferLength = 0;

# ---------------------------For DISTANCE_SENSOR----------------------------------

DIST_CM = 0 #defining distance in cm
DEV_ADDR = 0x10 #default value, change if modified
TEMP_C = None #Temperature in degree Celcius

######################################################
##  Functions - MAVLink                             ##
######################################################

#function to send OBSTACLE_DISTANCE_3D message to FCU
def send_obstacle_distance_3D_message():
    global mavlink_x, mavlink_y, mavlink_z

    current_time_ms = current_milli_time()

    for i in range(len(mavlink_x)):
        conn.mav.obstacle_distance_3d_send(
            current_time_ms,    # us Timestamp (UNIX time or time since system boot)
            0,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            65535,
            float(mavlink_x[i]),
            float(mavlink_y[i]),
            float(mavlink_z[i]),
            float(DEPTH_RANGE_M[0]),
            float(DEPTH_RANGE_M[1])
        )
        print('updated parameters sent as OBSTACLE_DISTANCE_3D')
        
# ------------------------------------------------------------------
        
#function to send DISTANCE_SENSOR message to FCU
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
	print('Updated Distance from Lidar sent as DISTANCE_SENSOR')
	
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

######################################################
##  Functions - TFmini Plus Lidar                   ##
######################################################

#function to request distance in cm, verify packets of data received, and update current_distance parameter for DISTANCE_SENSOR
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
	#print(f'obtained checksum value: {chkSum}') for debugging
	
	#recalculating checksum value
	calcChkSum = sum(data)
	#print(f'calculated checksum value: {calcChkSum}') for debugging
	calcChkSum = calcChkSum & 0xFF #Taking lower 8 bits
	
	#Verifying checksum value
	if (chkSum == calcChkSum):
		DIST_CM = data[2] | (data[3] << 8)
		#print(f'Distance (cm): {DIST_CM}') for debugging
		#TEMP_C = data[6] + (data[7] << 8)
		#TEMP_C = (TEMP_C >> 3) - 256
		#print(f'Temperature (°C): {TEMP_C}') for debugging
		print('TFmini Lidar Data updated')
	else:
		DIST_CM = -1
		TEMP_C = -1
		#print('Invalid dataframe') for debugging

######################################################
##  Functions - IWR6483 mmWave Radar                ##
######################################################

# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig(configFileName):
    
    global CLIport
    global Dataport
    # Open the serial ports for the configuration and the data ports
    
    # Raspberry pi
    CLIport = serial.Serial('/dev/ttyUSB0', 115200)
    Dataport = serial.Serial('/dev/ttyUSB1', 921600)
    
    # Windows
    #CLIport = serial.Serial('COM3', 115200)
    #Dataport = serial.Serial('COM4', 921600)

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport.write((i+'\n').encode())
        print(i)
        time.sleep(0.01)
        
    return CLIport, Dataport

# ------------------------------------------------------------------

# Function to parse the data inside the configuration file
def parseConfigFile(configFileName):
    configParameters = {} # Initialize an empty dictionary to store the configuration parameters
    
    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        
        # Split the line
        splitWords = i.split(" ")
        
        # Number of RX and TX Antennas are specified according to the .cfg file
        numRxAnt = 4
        numTxAnt = 2
        
        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1;
            
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2;
                
            digOutSampleRate = int(splitWords[11]);
            
        # Get the information about the frame configuration    
        elif "frameCfg" in splitWords[0]:
            
            chirpStartIdx = int(splitWords[1]);
            chirpEndIdx = int(splitWords[2]);
            numLoops = int(splitWords[3]);
            numFrames = int(splitWords[4]);
            framePeriodicity = int(splitWords[5]);

            
    # Combine the read data to obtain the configuration parameters           
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
    
    return configParameters
   
# ------------------------------------------------------------------

# Function to read and parse the incoming data
def readAndParseData68xx(Dataport, configParameters):
    global byteBuffer, byteBufferLength
    
    # Constants
    OBJ_STRUCT_SIZE_BYTES = 12;
    BYTE_VEC_ACC_MAX_SIZE = 2**15;
    MMWDEMO_UART_MSG_DETECTED_POINTS = 1;
    MMWDEMO_UART_MSG_RANGE_PROFILE   = 2;
    maxBufferSize = 2**15;
    tlvHeaderLengthInBytes = 8;
    pointLengthInBytes = 16;
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
    
    # Initialize variables
    magicOK = 0 # Checks if magic number has been read
    dataOK = 0 # Checks if the data has been read correctly
    frameNumber = 0
    detObj = {}
    
    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)
    
    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength = byteBufferLength + byteCount
        
    # Check that the buffer has some data
    if byteBufferLength > 16:
        
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc+8]
            if np.all(check == magicWord):
                startIdx.append(loc)
               
        # Check that startIdx is not empty
        if startIdx:
            
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength-startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
                
            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]
            
            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer[12:12+4],word)
            
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    
    # If magicOK is equal to 1 then process the message
    if magicOK:
        # word array to convert 4 bytes to a 32 bit number
        word = [1, 2**8, 2**16, 2**24]
        
        # Initialize the pointer index
        idX = 0
        
        # Frame Header
        magicNumber = byteBuffer[idX:idX+8]
        idX += 8
        version = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        idX += 4
        totalPacketLen = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        platform = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        idX += 4
        frameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        timeCpuCycles = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        numDetectedObj = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        numTLVs = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        subFrameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4

        # Read the TLV messages
        for tlvIdx in range(numTLVs):
            
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]

            # TLV Header
            tlv_type = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            tlv_length = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4

            # Read the data depending on the TLV message
            if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:

                # Initialize the arrays
                x = np.zeros(numDetectedObj,dtype=np.float32)
                y = np.zeros(numDetectedObj,dtype=np.float32)
                z = np.zeros(numDetectedObj,dtype=np.float32)
                velocity = np.zeros(numDetectedObj,dtype=np.float32)
                
                for objectNum in range(numDetectedObj):
                    
                    # Read the data for each object
                    x[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    y[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    z[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    velocity[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                
                # Store the data in the detObj dictionary
                detObj = {"numObj": numDetectedObj, "x": x, "y": y, "z": z, "velocity":velocity}
                dataOK = 1
                
 
        # Remove already processed data
        if idX > 0 and byteBufferLength>idX:
            shiftSize = totalPacketLen
            
                
            byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
            byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),dtype = 'uint8')
            byteBufferLength = byteBufferLength - shiftSize
            
            # Check that there are no errors with the buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0         

    return dataOK, frameNumber, detObj
    
# ------------------------------------------------------------------

# Function to update the data and display in the plot
def update():
    global mavlink_x, mavlink_y, mavlink_z
    dataOk = 0
    global detObj
   
      
    # Read and parse the received data
    dataOk, frameNumber, detObj = readAndParseData68xx(Dataport, configParameters)
    
    if dataOk and len(detObj["x"])>0:
        print(detObj)
        mavlink_y = -detObj["x"]
        mavlink_x = detObj["y"]
        mavlink_z = detObj["z"]
        print('mmWave data updated')
        
          
    return dataOk
    
######################################################
##  			MAIN                        ##
######################################################

# Configurate the serial port
CLIport, Dataport = serialConfig(configFileName)

# Get the configuration parameters from the configuration file
configParameters = parseConfigFile(configFileName)
   
# Main loop 
detObj = {}  
frameData = {}    
currentIndex = 0
while True:
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,0,0
    )
    print('Hearbeat sent to Pixhawk')
    try:
    
	#-----------------------DISTANCE_SENSOR------------------------#
        # Update the data from TFmini Lidar and check if the data is okay
        obtainData()
        send_distance_sensor_msg()      
        
        #--------------------OBSTACLE_DISTANCE_3D----------------------#
        # Update the data from mmWave radar and check if the data is okay
        dataOk = update()
        if dataOk:
            # Store the current frame into frameData
            frameData[currentIndex] = detObj
            send_obstacle_distance_3D_message()
            currentIndex += 1
        
        time.sleep(1) # Sampling frequency of 30 Hz
        
    # Stop the program and close everything if Ctrl + c is pressed
    except KeyboardInterrupt:
        CLIport.write(('sensorStop\n').encode())
        CLIport.close()
        Dataport.close()
        break
