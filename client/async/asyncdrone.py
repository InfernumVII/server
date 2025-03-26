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
from concurrent.futures import ThreadPoolExecutor
from clover.srv import SetLEDEffect

rospy.init_node('flight')

# ROS services initialization
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

c = 1
if c == 1:
    objects = {
        'red': ( [(160, 100, 160), (180, 170, 255)], True, [[(0, 80, 170), (35, 170, 255)]], (255, 0, 0), 0.125*2, (255,0,0), 'korob',1 ),
        #'yellow': ( , False, None, (255, 0, 0), 0.125*2, (255,0,0), 'korob',1 ),
    }
else:
    objects = {
        'red': ( ((0, 120, 70), (15, 255, 255)), True, [((20, 120, 70), (30, 255, 255))], (255, 0, 0), 0.125*2, (255,0,0), 'korob',1 ),
        #'yellow': ( ((20, 120, 70), (30, 255, 255)), False, None, (255, 0, 0), 0.125*2, (255,0,0), 'korob',1 ),
    }



mark = ObjectDetection(objects, transform_method='rectify_point', dev='clover', cnt_method='default', init=False)
mark.STOP_ALL = True
mark.max_cnt = False

def set_led_blue():
    set_effect(effect='blink', r=0, g=0, b=255)
    return "OK"

def set_led_orange():
    set_effect(r=255, g=140, b=0)
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
        out.append(str(f"{round(data[i][1][0], 3)} {round(data[i][1][1], 3)} s:{round(data[i][-1], 3)}"))
    out = "**".join(out)
    print(out)
    return out

class AsyncROS:
    def __init__(self):
        self.executor = ThreadPoolExecutor()
        self.loop = asyncio.get_event_loop()

    async def run_sync(self, func, *args):
        return await self.loop.run_in_executor(self.executor, func, *args)

async def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, 
                      frame_id='', auto_arm=False, tolerance=0.2):
    print(1)
    async_ros = AsyncROS()
    await async_ros.run_sync(navigate, x, y, z, yaw, speed, frame_id, auto_arm)

    while not rospy.is_shutdown():
        telem = await async_ros.run_sync(get_telemetry, 'navigate_target')
        if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
            break
        await asyncio.sleep(0.2)
    return "OK"

async def land_wait():
    async_ros = AsyncROS()
    await async_ros.run_sync(land)
    
    while True:
        armed = (await async_ros.run_sync(get_telemetry)).armed
        if not armed:
            break
        await asyncio.sleep(0.2)
    return "OK"

class BatteryMonitor:
    def __init__(self):
        self.battery_level = '0|0'
        rospy.Subscriber('/mavros/battery', BatteryState, self.battery_cb)
    
    def battery_cb(self, msg):
        self.battery_level = f"{100*msg.percentage:.2f}|{msg.voltage:.2f}"

battery = BatteryMonitor()

EMERGENCY_FLAG = asyncio.Event()
CURRENT_COMMAND = None
CURRENT_WEBSOCKET = None

async def execute_command(websocket, command, special=False, rec=False):
    if special == False:
        print(command)
    try:
        ns = {
            'result': None,
            'navigate_wait': navigate_wait,
            'land_wait': land_wait,
            'mark': mark,
            'websocket': websocket,
            'asyncio': asyncio
        }

        # Автоматическая обработка await
        if 'await ' in command:
            exec_code = f"""
async def __exec():
    try:
        result = {command}
        print(result)
        return result if 'result' in locals() else 'OK'
    except Exception as e:
        return f'Error: {{str(e)}}'
"""
        else:
            exec_code = f"""
def __exec():
    try:
        result = {command}
        return result if 'result' in locals() else 'OK'
    except Exception as e:
        return f'Error: {{str(e)}}'
"""
        # Выполнение кода
        exec(exec_code, globals(), ns)
        
        if 'await ' in command:
            result = await ns['__exec']()
        else:
            result = ns['__exec']()
        if special == True:
            await websocket.send(f"SPECIAL: {result}")
        elif rec == True:
            await websocket.send(f"REC: {result}")
        else:
            print(result)
            await websocket.send(f"ACK: {result}")
    except Exception as e:
        if special == True:
            await websocket.send(f"SPECIAL:Error: {result}")
        else:
            await websocket.send(f"ACK:Error: {result}")

async def execute_command_special(websocket, command):
    task = asyncio.create_task(execute_command(websocket, command, special=True))
    await task
async def execute_command_rec(websocket, command):
    task = asyncio.create_task(execute_command(websocket, command, special=False, rec=True))
    await task

async def listen_commands():
    global EMERGENCY_FLAG, CURRENT_COMMAND, CURRENT_WEBSOCKET
    
    uri = "ws://192.168.31.170:8000/ws"
    async with websockets.connect(uri) as websocket:
        CURRENT_WEBSOCKET = websocket
        print("Client connected!")
        
        
        while True:
            try:
                message = await websocket.recv()
                
                if message == "get_client_info":
                    await websocket.send(f"{socket.gethostname()}*{time.time()}")
                    continue
                
                #print(f"Received: {message}")

                if "EMERGENCY" in message:
                    parts = message.split("|")
                    if len(parts) >= 3:
                        command = parts[2]
                        EMERGENCY_FLAG.set()
                        if CURRENT_COMMAND and not CURRENT_COMMAND.done():
                            CURRENT_COMMAND.cancel()
                        try:
                            ns = {}
                            exec(command, globals(), ns)
                            result = ns.get('result', 'OK')
                            await websocket.send(f"EMERGENCY_ACK:{result}")
                        finally:
                            EMERGENCY_FLAG.clear()
                    continue
                elif message.startswith("rec|"):
                    _, command = message.split("|", 1)
                    CURRENT_COMMAND = asyncio.create_task(
                        execute_command_rec(websocket, command)
                    )
                    continue
                elif message.startswith("spec|"):
                    _, command = message.split("|", 1)
                    CURRENT_COMMAND = asyncio.create_task(
                        execute_command_special(websocket, command)
                    )
                    continue
                
                if "|" in message:
                    execute_at, cmd = message.split("|", 1)
                    execute_at = float(execute_at)
                    
                    async def scheduled():
                        try:
                            print(time.time())
                            while time.time() < execute_at:
                                print(time.time())
                                await asyncio.sleep(0.1)
                            print(time.time())
                            await execute_command(websocket, cmd)
                        except asyncio.CancelledError:
                            await websocket.send("ACK:Command cancelled")
                        
                    CURRENT_COMMAND = asyncio.create_task(scheduled())
                    
            except websockets.exceptions.ConnectionClosed:
                print("Connection closed")
                break
            except Exception as e:
                print(f"Error: {str(e)}")
                await websocket.send(f"ACK:Error: {str(e)}")

async def main():
    while not rospy.is_shutdown():
        try:
            await listen_commands()
        except (websockets.ConnectionClosed, ConnectionRefusedError) as e:
            print(f"Connection error: {str(e)}, retrying in 3s...")
            await asyncio.sleep(3)
        except Exception as e:
            print(f"Critical error: {str(e)}")
            await asyncio.sleep(1)
if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()