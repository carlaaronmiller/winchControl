#V1.1 Added queue implementation for reading in mav link messages and writing out.
from math import inf
import serial
from datetime import datetime
import os
import threading, queue, time
from pymavlink import mavutil


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
serSens = serial.Serial('/dev/ttyS0',115200,timeout=0.2)
serWinch = serial.Serial('/dev/ttyAMA2',19200,timeout=0.2) # Serial port for the winch. Change this to the correct port.

#Open the backup file for redundancy.
day = datetime.now()
fileName= "/usr/blueos/userdata/GPSLOGS/" + str(datetime.date(day))+".txt"
textBackup = open(fileName, "a")

#Main function variables.
msgOutLock = threading.Lock()
winchFault = False
wpThreshold = 3 #Distance from WP in meters.

#Mavlink message queues.
gpsQueue = queue.Queue(maxsize=1)
navQueue = queue.Queue(maxsize=1)

#Serial line thread queues, not used in implementation.
winchQueue = queue.Queue(maxsize=512)
loggerQueue = queue.Queue(maxsize=512)

#Queue for writing messages out to QGC.
outQueue = queue.Queue(maxsize=256)


def putLatest(q,item): #Kicks out old sample in the queue if it exists.
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            _ = q.get_nowait()
        except queue.Empty:
            pass
        q.put_nowait(item)


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
    while True:
        if time.time() - start_time > timeout:
           raise TimeoutError("Timeout while waiting for line.") 
        raw = serCon.read_until(b'\n')   # relies on serCon.timeout
        if not raw:
            continue
        return raw.decode('utf-8', errors='ignore').rstrip('\r\n')


def QGCSend(connectionList):
    while True:
        message = outQueue.get() # Grab a message for printing out. Blocks automatically.
        with msgOutLock:
            print(message)
            for conn in connectionList:
                conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,message.encode())

def mavlinkReader(connection):
    while True:
        msg = connection.recv_match(blocking=True, timeout=1)
        if msg is None: #Keep throwing out msgs until most recent.
            continue
        
        mType = msg.get_type()
        if mType == "GPS_RAW_INT":
            putLatest(gpsQueue,msg)
        elif mType == "NAV_CONTROLLER_OUTPUT":
            putLatest(navQueue,msg)

def GPSWrite():#Write GPS DATA to logger.
    while(True):
        #Get the most recent item in the GPS queue.
        GPSData = gpsQueue.get()
    
        messageString ="$"+str(time.time()) +"/"+ str(GPSData).replace(" ","")+"\n"
        serWrite(serSens,messageString)
        outQueue.put(messageString)

        #Send the sensor string messsage to QGC.
        try:
            sensorString = serReadLine(serSens,30) #Swapped timeout from inf -> 30.
        except:
            sensorString = "Timeout error in sensorReadLine()"
        outQueue.put(sensorString)
        time.sleep(1)

def heartbeat(): #Write heartbeat message to QGC.
    while(True):
        heartbeat = "Alive: " + str(time.time())
        outQueue.put(heartbeat)
        time.sleep(10)

#Start heartbeat thread.
sendHeartbeat = threading.Thread(target= heartbeat, daemon=True)
sendHeartbeat.start()
#Start thread for writing out messages.
outWriter = threading.Thread(target= QGCSend, args=(connectionList,),daemon=True)
outWriter.start()
#Start the message reader thread.
messageReader = threading.Thread(target=mavlinkReader, args=(master,), daemon=True)
messageReader.start()
#Give the message reader time to start up before starting the writer thread.
while gpsQueue.empty() or navQueue.empty(): 
    print("Waiting for GPS fix and NAV WP.")
    time.sleep(1)
#Start the GPS writer thread.
GPSWriter = threading.Thread(target=GPSWrite, daemon=True)
GPSWriter.start()




try: #Main function, which is basically the winch loop:
    while(not winchFault):
        
        #Check how far away from waypoint the boat is.
        wpDist = navQueue.get().wp_dist

        if(wpDist<=wpThreshold): #If boat is inside wp radius.
            #Start profiling.
            #Send the profile command.
            print("On Station. Start Winch")
            try:
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
                    winchMsg = serReadLine(serWinch, 20) # Wait 10 seconds for response from winch. But will other messages spit out here?
                    print(winchMsg)
                #Possible outcomes after command sent:
                # 1) Winch doesn't respond.
                except TimeoutError:
                    print("Timeout error on receiving serial message from winch, disarming.")
                    with msgOutLock:
                        master.mav.command_long_send(1,1,400,0, 0, 21196, 0,0,0,0,0) #Disarm vehicle.
                    winchMsg = None
                    winchFault = True
                    break
                
                # 2)Winch sends profile completion message.
                if "dock switch" in winchMsg:
                    outQueue.put("Dock switch confirmed.")
                    #Wait until the waypoint is updated to break out of loop to avoid second profile.
                    while(wpDist<wpThreshold):
                        print("Profile confirmed, waiting for next waypoint.")
                        #Need to keep checking for distance here.
                        wpDist = navQueue.get().wp_dist
                        time.sleep(5)
                    break
                    
                # 3)Winch records upcast timeout.
                elif "Upcast timeout" in winchMsg: #If no docking acknowledged, disarm and notify user.
                    print("Upcast timeout, disarming. Check payload.\n")
                    with msgOutLock:
                        master.mav.command_long_send(1,1,400,0, 0, 21196, 0,0,0,0,0) #Disarm vehicle.
                    winchFault = True
                    break
                        
except KeyboardInterrupt:
    print("\nKeyboardInterrupt detected. Exiting gracefully.")
    serSens.close()
    serWinch.close()
    textBackup.close()
