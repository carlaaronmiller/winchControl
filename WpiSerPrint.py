#First upload, no queue handling.
from math import inf
import serial
from datetime import datetime
import os
import threading
from pymavlink import mavutil
import time

#Future improvements:
# - Add queue system to write in / out messages so they don't have to block eachother waiting for connection locks.

loitering = False

#Print information about the process.
print("The print process has the following ID: ")
print(os.getpid())
print("Running")

#Open connection to the autopilot.
master = mavutil.mavlink_connection('tcp:127.0.0.1:5777')

print("Pinging master connection.\n")
master.wait_heartbeat()
print("Master Heartbeat received.\n")
#Create a connection to QGC.
qgc = mavutil.mavlink_connection('udpout:192.168.2.1:14550', source_system=1)
qgcZT = mavutil.mavlink_connection('udpout:192.168.192.96:14550',source_system=1)
#You almost always write to both together, so put them in a list for easy accessing when sending message.
connectionList = [qgc,qgcZT]
#Serial configurations.
serSens = serial.Serial('/dev/ttyS0',115200)
serWinch = serial.Serial('/dev/ttyAMA2',19200) # Serial port for the winch. Change this to the correct port.

#Open the backup file for redundancy.
day = datetime.now()
fileName= "/usr/blueos/userdata/GPSLOGS/" + str(datetime.date(day))+".txt"
textBackup = open(fileName, "a")

#Main function variables.
lastGPSMsg = None
lastNavMsg = None
msgInLock = threading.Lock()
msgOutLock = threading.Lock()
winchFault = False
wpThreshold = 7 #Distance from WP in meters.

def getMessage(connection, msgType):
	#Keep polling and throwing out messages until you get to the most recent version, which you return.
	msg = None
	while temp := connection.recv_match(type=msgType):
		msg = temp
	if msg == None:
		msg = connection.recv_match(type=msgType, blocking = True)
	return msg

def serWrite(serCon, msg):
	serCon.write(bytes(msg,'utf-8'))
	serCon.write(bytes(str('\n'),'utf-8'))

def serReadLine(serCon, timeout):
    start_time = time.time()
    line = []
    fullLine = ""

    # Wait for the end of the next line to start reading a line.
    while True:
        if time.time() - start_time > timeout:
           raise TimeoutError("Timeout while waiting for start of line.")
        line = serCon.read_until(b'\n',size=126).decode('utf-8',errors='ignore').strip('\r\n')
        return line

def QGCSend(connectionList,message):
	with msgOutLock:
		print(message)
		for conn in connectionList:
			conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,message.encode())

#HARDCODED with message types and master connection, TODO: generalize params.
def mavlinkReader():
	print("Starting reader MAVLINK reader thread.")
	global lastGPSMsg, lastNavMsg
	while(True):
		with msgInLock:
				lastGPSMsg = getMessage(master,'GPS_RAW_INT')
				lastNavMsg = getMessage(master,'NAV_CONTROLLER_OUTPUT')
		time.sleep(1)


def GPSWrite():
	print("Starting GPS/Serial thread.")
	while(True):
		#Write GPS DATA to logger.
		with msgInLock:
			GPSData = lastGPSMsg
		print("GPS Write has msgIn lock.")
		e = datetime.now()
		GPSString ="$"+str(time.time()) +"/"+ str(GPSData).replace(" ","")+"\n"
		serWrite(serSens,GPSString)
		QGCSend(connectionList,GPSString)
		#Send the sensor string messsage to QGC.
		try:
			sensorString = serReadLine(serSens,30) #Swapped timeout from inf -> 30.
		except:
			sensorString = "Timeout error in sensorReadLine()"
		QGCSend(connectionList,sensorString)
		time.sleep(1)

def safeSend(func, *args, **kwargs):
	with msgOutLock:
		func(*args, **kwargs)

#Start the message reader thread.
messageReader = threading.Thread(target=mavlinkReader, daemon=True)
messageReader.start()
#Give the message reader time to start up before starting the writer thread.
while lastGPSMsg is None:
    time.sleep(1)
    QGCSend(connectionList,"Waiting for GPS fix to start GPS Writer.")

#Start the GPS writer thread.
GPSWriter = threading.Thread(target=GPSWrite, daemon=True)
GPSWriter.start()
while lastNavMsg is None:
   time.sleep(1)
   QGCSend(connectionList,"Waiting for waypoint to start winch script.")

try:
	while(winchFault): #Main function, which is basically the winch loop:
		#Check how far away from waypoint the boat is.
		with msgInLock:
			wpDist = lastNavMsg.wp_dist
		distMsg = f"Distance to waypoint {wpDist}"
		QGCSend(connectionList,distMsg)

		if(wpDist<=wpThreshold): #If boat is inside wp radius.
			#Start profiling.
			#Send the profile command.
			print("On Station. Start Winch")
			try:
				serWrite(serWinch,"") #Clear the buffer.
				serWrite(serWinch,"PROF") # Send Profile Command.
				serReadLine(serWinch, 10) # Wait 10 seconds for response from winch
			except: # An error here indicates that the winch is not connected.
				print("Error reading from winch serial port. Check connection.\n")
				break
			#Command was sent, winch should now be in profiling loop.
			#Three outcomes: 1) end of profile received("dock switch", 2) upcast timeout ("upcast timeout"), 3) no message from winch, 4) boat leaves waypoint.
			
			while(not winchFault):
				#Read a line from the winch.
				try:
					winchMsg = serReadLine(serWinch, 20)
				#Possible outcomes after command sent:
				# 1) Winch doesn't respond.
				except TimeoutError:
					print("Timeout error on receiving serial message from winch, disarming.")
					with msgOutLock:
						master.mav.command_long_send(1,1,400,0, 0, 21196, 0,0,0,0,0) #Disarm vehicle.
					winchMsg = None
					winchFault = True
					break
				QGCSend(connectionList,winchMsg)
				# 2)Winch sends profile completion message.
				if "dock switch" in winchMsg:
					print("DOCKED.")
					QGCSend(connectionList, "Dock switch confirmed.")
					#Wait until the waypoint is updated to break out of loop to avoid second profile.
					while(wpDist<wpThreshold):
						print("Profile confirmed, waiting for next waypoint.")
						with msgInLock:
							wpDist = lastNavMsg.wp_dist
						time.sleep(5)
					break
					
				# 3)Winch records upcast timeout.
				elif "Upcast timeout" in winchMsg: #If no docking acknowledged, disarm and notify user.
					QGCSend(connectionList,"Upcast timeout, disarming. Check payload.\n")
					with msgOutLock:
						master.mav.command_long_send(1,1,400,0, 0, 21196, 0,0,0,0,0) #Disarm vehicle.
					winchFault = True
					break
		time.sleep(2)
except KeyboardInterrupt:
    print("\nKeyboardInterrupt detected. Exiting gracefully.")
    serSens.close()
    serWinch.close()
    textBackup.close()
