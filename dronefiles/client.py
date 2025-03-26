# client.py
import asyncio
import websockets
import time
import socket
import platform
import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
from recognition import *
import numpy as np
from sensor_msgs.msg import BatteryState

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

'''
red 165 100 170 - 180 150 255
yellow 20 120 190 - 35 150 255
orange(div red) 2 120 170 - 20 170 255
'''

c = 2
if c == 1:
    objects = {
        'red': ( [(160, 100, 160), (180, 170, 255)], False, None, (255, 0, 0), 0.125*2, (255,0,0), 'korob',1 ),
        'yellow': ( [(0, 80, 170), (35, 170, 255)], False, None, (255, 0, 0), 0.125*2, (255,0,0), 'korob',1 ),
    }
else:
    objects = {
        'red': ( ((0, 120, 70), (15, 255, 255)), False, None, (255, 0, 0), 0.125*2, (255,0,0), 'korob',1 ),
        'yellow': ( ((20, 120, 70), (30, 255, 255)), False, None, (255, 0, 0), 0.125*2, (255,0,0), 'korob',1 ),
    }



mark = ObjectDetection(objects, transform_method='rectify_point', dev='clover', cnt_method='default', init=False)
mark.STOP_ALL = True
mark.max_cnt = False


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
    return "OK"


def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)
    return "OK"

EMERGENCY_FLAG = asyncio.Event()
CURRENT_COMMAND = None
CURRENT_WEBSOCKET = None

async def listen_commands():
    global EMERGENCY_FLAG, CURRENT_COMMAND, CURRENT_WEBSOCKET
    
    uri = "ws://192.168.31.170:8000/ws"
    async with websockets.connect(uri) as websocket:
        CURRENT_WEBSOCKET = websocket
        print("Client connected!")
        while True:
            try:
                message = await websocket.recv()
                
                # Handle client info request
                if message == "get_client_info":
                    await websocket.send(get_client_info())
                    continue
                    
                # Check for emergency command prefix
                #print(message)

                if "EMERGENCY" in message:
                    # Emergency commands execute immediately
                    command = message.split("|")[2]
                    print(CURRENT_COMMAND)
                    print(command)
                    print(f"EMERGENCY COMMAND RECEIVED: {command}")
                    
                    # Set emergency flag and cancel current command
                    EMERGENCY_FLAG.set()
                    if CURRENT_COMMAND and not CURRENT_COMMAND.done():
                        CURRENT_COMMAND.cancel()
                        try:
                            await CURRENT_COMMAND
                        except asyncio.CancelledError:
                            print("Normal command cancelled due to emergency")
                    
                    # Execute emergency command
                    try:
                        ns = {}
                        exec(command, globals(), ns)
                        result = ns.get('result', 'OK')
                        await websocket.send(f"ACK:{result}")
                    except Exception as e:
                        error_msg = f"ACK:Error: {str(e)}"
                        print(error_msg)
                        await websocket.send(error_msg)
                    finally:
                        EMERGENCY_FLAG.clear()
                    continue
                elif "spec" in message:
                    _, command = message.split("|")
                    c = asyncio.create_task(
                        execute_command_special(websocket, command)
                    )
                    await c
                    continue
                # Normal command processing
                execute_at, command = message.split("|", 1)
                execute_at = float(execute_at)
                
                print(f"Command received. Will execute at UNIX {execute_at}")
                
                # Wait until execution time, checking for emergencies
                while time.time() < execute_at and not EMERGENCY_FLAG.is_set():
                    await asyncio.sleep(0.01)
                
                if EMERGENCY_FLAG.is_set():
                    print("Skipping normal command due to emergency")
                    await websocket.send("ACK:Command skipped due to emergency")
                    continue
                    
                print(f"Executing at UNIX {time.time()}: {command}")
                try:
                    # Create a task we can cancel if needed
                    CURRENT_COMMAND = asyncio.create_task(
                        execute_command(websocket, command)
                    )
                    await CURRENT_COMMAND
                except asyncio.CancelledError:
                    print("Command was cancelled due to emergency")
                    await websocket.send("ACK:Command cancelled due to emergency")
                except Exception as e:
                    error_msg = f"ACK:Error: {str(e)}"
                    print(error_msg)
                    await websocket.send(error_msg)
                finally:
                    CURRENT_COMMAND = None
                    
            except websockets.exceptions.ConnectionClosed:
                print("Connection closed")
                break

async def execute_command(websocket, command):
    """Execute a command with timeout and emergency checking"""
    try:
        # Execute the command
        ns = {}
        command = "result=" + command
        exec(command, globals(), ns)
        result = ns.get('result', 'OK')
        print(result)
        
        # Check if emergency occurred during execution
        if EMERGENCY_FLAG.is_set():
            return "ACK:Command partially executed before emergency"
            
        await websocket.send(f"ACK:{result}")
        return result
    except Exception as e:
        raise e
    
async def execute_command_special(websocket, command):
    """Execute a command with timeout and emergency checking"""
    try:
        # Execute the command
        ns = {}
        command = "result=" + command
        exec(command, globals(), ns)
        result = ns.get('result', 'OK')
        #print(result)
        

            
        await websocket.send(f"SPECIAL: {result}")
        return result
    except Exception as e:
        raise e

class Battery:
    def __init__(self):
        self.b = '0|0'
        battery_sub = rospy.Subscriber('/mavros/battery', BatteryState, self.battery_cb)
    
    def battery_cb(self, msg):
        self.b = f"{100*msg.percentage:.2f}|{msg.voltage:.2f}"

bat = Battery()


def battery_info():
    #return b[0]
    return bat.b

def get_client_info():
    client_name = f"{socket.gethostname()}*{time.time()}"
    return client_name

def hello_world():
    from random import randint
    print("Hello World")
    time.sleep(randint(5,10))
    return "OK"

def stopRecognition():
    mark.STOP_ALL = True
    return "OK"

def startRecogniton():
    mark.STOP_ALL = False
    return "OK"

def getRecognizedMarks():
    print(mark.Points)
    data = mark.Points
    out = ["OK", "points:"]
    for i in data.keys():
        out.append(str(f"({round(data[i][1][0], 3)} {round(data[i][1][1], 3)} s:{round(data[i][-1], 3)})"))
    out = " ".join(out)
    print(out)
    return out

print(1)

while not rospy.is_shutdown():
    try:
        asyncio.get_event_loop().run_until_complete(listen_commands())
    except asyncio.exceptions.TimeoutError:
        print("Waiting for server")