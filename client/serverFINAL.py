from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import asyncio
import time
from typing import Dict, Deque, List, Optional
from collections import deque
import threading


hardcoded = {
    "192.168.0.10": "0",
    "192.168.0.20": "1"
}
app = FastAPI()
class Client():
    def __init__(self, websocket, name, init_time):
        self.paused = False
        self.websocket = websocket
        self.name = name
        self.init_time = init_time
        self.ready = True
        self.pause_event = asyncio.Event()  # New event for pause control
        self.pause_event.set()  # Initially not paused
        self.battery = "1"
        self.voltage = "1"
        self.ip = self._get_client_ip()
    def _get_client_ip(self):
        try:
            # First try to get from websocket scope (ASGI standard)
            if hasattr(self.websocket, 'scope'):
                client = self.websocket.scope.get('client')
                if client and isinstance(client, (tuple, list)) and len(client) > 0:
                    return client[0]
            
            # Fallback to websockets library (common case)
            if hasattr(self.websocket, 'remote_address') and self.websocket.remote_address:
                return self.websocket.remote_address[0]
            
            # For some other WebSocket implementations
            if hasattr(self.websocket, 'client') and hasattr(self.websocket.client, 'host'):
                return self.websocket.client.host
            
            # Try to get from transport
            if hasattr(self.websocket, '_transport'):
                transport = self.websocket._transport
                if transport and hasattr(transport, 'get_extra_info'):
                    peername = transport.get_extra_info('peername')
                    if peername and isinstance(peername, (tuple, list)) and len(peername) > 0:
                        return peername[0]
            
            return "unknown"
        except:
            return "unknown"

class ConnectionManager:
    def __init__(self):
        self.clients: Dict[str, Client] = {}  # client_id -> Client
        self.message_queues: Dict[WebSocket, Deque[str]] = {}
        self.lock = asyncio.Lock()
        self.client_last_id = 0
        self.special_message_queues: Dict[WebSocket, Deque[str]] = {}
        
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        async with self.lock:
            self.message_queues[websocket] = deque()
            self.special_message_queues[websocket] = deque()
        
        await self.get_client_info(websocket)
        asyncio.create_task(self.process_incoming_messages(websocket))
        return websocket
    
    async def get_client_info(self, websocket):
        await websocket.send_text("get_client_info")
        try:
            client_info = await asyncio.wait_for(websocket.receive_text(), timeout=5.0)
            client_info = client_info.split('*')[0]
            async with self.lock:
                client = Client(websocket, client_info, time.time())
                client_id = self.get_unique_id(client.ip, "0")
                print(client_id)
                self.clients[client_id] = client
        except asyncio.TimeoutError:
            async with self.lock:
                client_id = self.get_unique_id(client.ip, "0")
                client = Client(websocket, f"Unknown device {len(self.clients)}", time.time())
                self.clients[client_id] = client
    
    def get_unique_id(self, ip, id):
        print(id in self.clients.keys())
        if ip in hardcoded.keys():
            megaid = hardcoded[ip]
            if megaid in self.clients.keys():
                newid = self.get_unique_id(self.clients[megaid].ip, "0")
                self.clients[newid] = self.clients[megaid]
                return megaid
            else:
                return megaid
        if id not in self.clients.keys():
            return id
        else:
            return self.get_unique_id(ip, str(int(id) + 1))
    
    async def disconnect(self, websocket: WebSocket):
        async with self.lock:
            # Find the client with this websocket
            client_to_remove = None
            for client_id, client in self.clients.items():
                if client.websocket == websocket:
                    client_to_remove = client_id
                    break
            
            if client_to_remove:
                del self.clients[client_to_remove]
            
            if websocket in self.message_queues:
                del self.message_queues[websocket]
            if websocket in self.special_message_queues:
                del self.special_message_queues[websocket]
    
    async def receive_message(self, websocket: WebSocket, timeout: float = None):
        if websocket not in self.message_queues:
            return None
            
        try:
            if not self.message_queues[websocket]:
                if timeout:
                    await asyncio.wait_for(
                        self._wait_for_message(websocket),
                        timeout=timeout
                    )
                else:
                    await self._wait_for_message(websocket)
            return self.message_queues[websocket].popleft()
        except asyncio.TimeoutError:
            return None
    
    async def receive_special_message(self, websocket: WebSocket, timeout: float = None):
        if websocket not in self.special_message_queues:
            return None
            
        try:
            if not self.special_message_queues[websocket]:
                if timeout:
                    await asyncio.wait_for(
                        self._wait_for_special_message(websocket),
                        timeout=timeout
                    )
                else:
                    await self._wait_for_special_message(websocket)
            return self.special_message_queues[websocket].popleft()
        except asyncio.TimeoutError:
            return None
    
    async def _wait_for_message(self, websocket: WebSocket):
        while not self.message_queues[websocket]:
            await asyncio.sleep(0.1)
    
    async def _wait_for_special_message(self, websocket: WebSocket):
        while not self.special_message_queues[websocket]:
            await asyncio.sleep(0.1)
    
    async def process_incoming_messages(self, websocket: WebSocket):
        try:
            while True:
                message = await websocket.receive_text()
                print(message)
                if message.startswith("SPECIAL:"):
                    print(f"Special message received: {message}")
                    async with self.lock:
                        self.special_message_queues[websocket].append(message)
                elif message.startswith("ACK:"):
                    print(f"ACK message received: {message}")
                    async with self.lock:
                        self.message_queues[websocket].append(message)
                else:
                    print(f"Unexpected message format: {message}")
        except WebSocketDisconnect:
            await self.disconnect(websocket)
        except Exception as e:
            print(f"Error processing messages: {e}")
            await self.disconnect(websocket)
    
    async def get_websocket_by_id(self, client_id: str) -> Optional[WebSocket]:
        async with self.lock:
            client = self.clients.get(client_id)
            return client.websocket if client else None

    async def get_client_ids(self) -> List[str]:
        async with self.lock:
            return list(self.clients.keys())
    
    async def get_clients(self):
        out = []
        async with self.lock:
            for key, value in self.clients.items():
                out.append({"id": key, "client_name": value.name, "ready": value.ready, "battery": value.battery, "voltage": value.voltage, "ip": value.ip})
        return out
    
    async def set_pause_state(self, client_id: str, paused: bool):
        async with self.lock:
            if client_id in self.clients:
                client = self.clients[client_id]
                client.paused = paused
                if paused:
                    client.pause_event.clear()  # This will block execution
                else:
                    client.pause_event.set()  # This will resume execution
    
manager = ConnectionManager()

@app.get("/")
async def get():
    with open('templates/test.html', 'r', encoding='utf-8') as f:
        return HTMLResponse(f.read())

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await asyncio.sleep(1)
    except WebSocketDisconnect:
        await manager.disconnect(websocket)

@app.get("/set-pause")
async def set_pause(client_ids: str, paused: str):
    client_list = client_ids.split(',')
    is_paused = paused == "1"
    
    tasks = []
    for client_id in client_list:
        tasks.append(manager.set_pause_state(client_id, is_paused))
    
    await asyncio.gather(*tasks)
    
    return {
        "status": "Completed",
        "results": "OK",
        "success": True
    }


@app.get("/send-special-command-same")
async def send_special_command(client_ids: str, command: str):
    client_list = client_ids.split(',')

    tasks = []
    results = []
    for i in client_list:
        tasks.append(asyncio.create_task(send_special_command(i, command)))
    task_results = await asyncio.gather(*tasks, return_exceptions=True)
    print(task_results)
    for result in task_results:
        if not isinstance(result, Exception):
            results.extend(result['results'])
    success = all("OK" in res or "NAVIGATE_OK" in res for res in results)
    
    return {
        "status": "Completed" if success else "Failed",
        "results": results,
        "success": success
    }

@app.get("/send-special-command")
async def send_special_command(client_id: str, command: str):
    client = manager.clients.get(client_id)
    if client:
        await client.pause_event.wait()  # This will block if pause_event is cleared

    results = []
    
    websocket = await manager.get_websocket_by_id(client_id)
    targets = [websocket] if websocket else []
    
    for websocket in targets:
        try:
            await websocket.send_text(f"spec|{command}")
            spc = await manager.receive_special_message(websocket, timeout=1000000000000000)
            print(spc)
            if spc:
                for client in manager.clients.values():
                    if client.websocket == websocket:
                        break
                results.append(f"{spc[8:]}")
            else:
                results.append(f"Timeout")
        except Exception as e:
            results.append(f"Error - {str(e)}")
    
    if not results:
        results.append(f"Client {client_id} not found")
    print(results)
    return {
        "status": f"OK",
        "results": results
    }

@app.get("/get-battery-and-energy")
async def get_battery(client_id : str):
    await send_special_command(client_id, "battery_info()")
    


#LEGACY
#LEGACY
#LEGACY
@app.get("/send-task")
async def send_task_synced():
    client_ids = manager.get_client_ids()

    if len(client_ids) < 2:
        return {
            "status": "Failed",
            "results": ["Need at least 2 connected clients"],
            "success": False
        }

    t1 = client_ids[0]
    t2 = client_ids[1]
    sequences = {
        "1": [
            {"name": "takeoff", "timing": 1},
            {"name": "takeoff", "timing": 1},
            {"name": "move_forward11", "timing": 1},
            {"name": "go_home1", "timing": 1},
            {"name": "land", "timing": 1},
        ],
        "2": [
            {"name": "takeoff", "timing": 1},
            {"name": "move_forward23", "timing": 1},
            {"name": "go_home2", "timing": 1},
            {"name": "land", "timing": 1},
        ]
    }

    max_length = max(len(sequences["1"]), len(sequences["2"]))
    results = []
    
    for i in range(max_length):
        client1_command = sequences["1"][i] if i < len(sequences["1"]) else None
        client2_command = sequences["2"][i] if i < len(sequences["2"]) else None
        
        print(f"Step {i + 1}:")
        print(f"Client 1 command: {client1_command}")
        print(f"Client 2 command: {client2_command}")
        
        tasks = []
        checktime = time.time()
        tasks.append(asyncio.create_task(execute_command(checktime, client1_command, client_id=t1)))
        tasks.append(asyncio.create_task(execute_command(checktime, client2_command, client_id=t2)))
        task_results = await asyncio.gather(*tasks, return_exceptions=True)
        print(task_results)
        for result in task_results:
            if not isinstance(result, Exception):
                results.extend(result['results'])
    
    success = all("OK" in res or "NAVIGATE_OK" in res for res in results)
    
    return {
        "status": "Completed" if success else "Failed",
        "results": results,
        "success": success
    }

COMMAND_LIBRARY = {
    "takeoff": "navigate_wait(z=1, frame_id='body', auto_arm=True)",
    "go_home1": "navigate_wait(x=0, y=0, z=1, frame_id='aruco_map')",
    "move_forward11": "navigate_wait(x=0, y=4.5, z=1, frame_id='aruco_map')",
    "move_forward12": "navigate_wait(x=0.9, y=4.5, z=1, frame_id='aruco_map')",
    "move_forward13": "navigate_wait(x=0.9, y=0, z=1, frame_id='aruco_map')",
    "go_home2": "navigate_wait(x=3.6, y=0, z=1, frame_id='aruco_map')",
    "move_forward21": "navigate_wait(x=3.6, y=1.8, z=1, frame_id='aruco_map')",
    "move_forward22": "navigate_wait(x=2.7, y=1.8, z=1, frame_id='aruco_map')",
    "move_forward23": "navigate_wait(x=2.7, y=0, z=1, frame_id='aruco_map')",
    "land": "land_wait()",
    "hello": "hello_world()"
}

tasks = {
    '0': [
        {"command": "stopRecognition()", "timing": 1},
        {"command": "navigate_wait(z=1, frame_id='body', auto_arm=True)", "timing": 1},
        {"command": "startRecogniton()", "timing": 1},
        {"command": "navigate_wait(x=0, y=0, z=2, yaw=(math.pi/2), frame_id='aruco_map')", "timing": 1},
        {"command": "navigate_wait(x=0, y=5, z=2, yaw=(math.pi/2), frame_id='aruco_map')", "timing": 1},
        {"command": "navigate_wait(x=1.1, y=5, z=2, yaw=(math.pi/2), frame_id='aruco_map')", "timing": 1},
        {"command": "navigate_wait(x=1.1, y=0, z=2, yaw=(math.pi/2), frame_id='aruco_map')", "timing": 1},
        {"command": "navigate_wait(x=1.1, y=0, z=2, yaw=(math.pi/2), frame_id='aruco_map')", "timing": 1},
        {"command": "navigate_wait(x=0, y=0, z=1.5, yaw=(math.pi/2), frame_id='aruco_map')", "timing": 1},
        {"command": "stopRecognition()", "timing": 1},
        {"command": "land_wait()", "timing": 1},
        {"command": "getRecognizedMarks()", "timing": 1},
    ],
    '1': [
        {"command": "stopRecognition()", "timing": 1},
        {"command": "navigate_wait(z=1, frame_id='body', auto_arm=True)", "timing": 1},
        {"command": "startRecogniton()", "timing": 1},
        {"command": "navigate_wait(x=3.6, y=0, z=2, yaw=(math.pi/2), frame_id='aruco_map')", "timing": 1},
        {"command": "navigate_wait(x=3.6, y=5, z=2, yaw=(math.pi/2), frame_id='aruco_map')", "timing": 1},
        {"command": "navigate_wait(x=2.5, y=5, z=2, yaw=(math.pi/2), frame_id='aruco_map')", "timing": 1},
        {"command": "navigate_wait(x=2.5, y=0, z=2, yaw=(math.pi/2), frame_id='aruco_map')", "timing": 1},
        {"command": "navigate_wait(x=2.5, y=0, z=2, yaw=(math.pi/2), frame_id='aruco_map')", "timing": 1},
        {"command": "navigate_wait(x=3.6, y=0, z=1.5, yaw=(math.pi/2), frame_id='aruco_map')", "timing": 1},
        {"command": "stopRecognition()", "timing": 1},
        {"command": "land_wait()", "timing": 1},
        {"command": "getRecognizedMarks()", "timing": 1},
    ],
    '2': [
        {"command": "getRecognizedMarks()", "timing": 1},
        {"command": "hello_world()", "timing": 1},
        {"command": "hello_world()", "timing": 1},
        {"command": "hello_world()", "timing": 1},
        {"command": "hello_world()", "timing": 1},
        {"command": "hello_world()", "timing": 1},
    ],
    '3': [
        {"command": "navigate_wait(z=1, frame_id='body', auto_arm=True)", "timing": 1},
        {"command": "land_wait()", "timing": 1},
    ]
}

@app.get("/get-devices")
async def get_devices():
    for i in manager.clients.keys():
        t = await get_battery(i)
        try:
            percent, voltage = t["results"][0].split("|")
        except:
            percent, voltage = "0", "0"
        manager.clients[i].voltage = voltage
        manager.clients[i].battery = percent
    devices = await manager.get_clients()
    return {"devices": devices}


@app.get("/send-command-e") #EMERGENCY command
async def send_command_e(client_ids: str, command: str, timing: str):
    t = await send_command_same(client_ids, 'EMERGENCY|' + command, timing)
    return t

    

@app.get("/send-command-same")
async def send_command_same(client_ids: str, command: str, timing: str):
    client_list = client_ids.split(',')
    print(client_ids)

    tasks = []
    results = []
    t = time.time() + float(timing)
    for i in client_list:
        tasks.append(asyncio.create_task(execute_command(t, command, client_id=i)))
    task_results = await asyncio.gather(*tasks, return_exceptions=True)
    print(task_results)
    for result in task_results:
        if not isinstance(result, Exception):
            results.extend(result['results'])
    success = all("OK" in res or "NAVIGATE_OK" in res for res in results)
    
    return {
        "status": "Completed" if success else "Failed",
        "results": results,
        "success": success
    }

@app.get("/send-commands")
async def send_commands(client_ids: str, commands: List[Dict]):
    client_list = client_ids.split(',')
    #commands = commands.split(',')
    if len(client_list) != len(commands):
        return {
            "status": "Failed",
            "results": ["Invalid commands format"],
            "success": False
        }
    """
    commands: [
    {"id": 0, "command": "hello", "timing": 1}
    {"id": 1, "command": "hello", "timing": 1}
    ]
    """
    tasks = []
    results = []
    for command in commands:
        t = time.time() + float(command["timing"])
        tasks.append(asyncio.create_task(execute_command(t, command["command"], client_id=command["id"])))
    task_results = await asyncio.gather(*tasks, return_exceptions=True)
    print(task_results)
    for result in task_results:
        if not isinstance(result, Exception):
            results.extend(result['results'])
    success = all("OK" in res or "NAVIGATE_OK" in res for res in results)
    
    return {
        "status": "Completed" if success else "Failed",
        "results": results,
        "success": success
    }

@app.get("/run-task")
async def run_task(client_ids: str): #, tasks: Dict[int: List]
    client_list = client_ids.split(',')
    print(client_list)

    results = []
    number_of_commands = len(tasks[client_list[0]])
    print("number_of_commands", number_of_commands)
    c=0
    while c < number_of_commands:
        commands = []
        for id in client_list:
            commands.append({"id": id, "command": tasks[id][c]["command"], "timing": tasks[id][c]["timing"]})
        result = await send_commands(client_ids, commands)
        
        results.append(result["results"])
        c+=1
    print(c)
    
    success = all("OK" in r for res in results for r in res)
    print(results)
    return {
        "status": "Completed" if success else "Some troubles",
        "results": results,
        "success": success
    }



async def send_c(execute_at, command, client_id):
    client = manager.clients.get(client_id)
    if client:
        await client.pause_event.wait()  # This will block if pause_event is cleared

    results = []
    
    websocket = await manager.get_websocket_by_id(client_id)
    targets = [websocket] if websocket else []
    
    for websocket in targets:
        try:
            await websocket.send_text(f"{execute_at}|{command}")
            ack = await manager.receive_message(websocket, timeout=1000000000000000)
            if ack:
                client_name = "Unknown"
                for client in manager.clients.values():
                    if client.websocket == websocket:
                        client_name = client.name
                        break
                results.append(f"{client_name}: {ack[4:]}")
            else:
                results.append(f"{client_name}: Timeout")
        except Exception as e:
            results.append(f"{client_name}: Error - {str(e)}")
    
    if not results:
        results.append(f"Client {client_id} not found")
    
    return {
        "status": f"Command scheduled at UNIX {execute_at}",
        "results": results
    }

async def execute_command(executed_at, cmd, client_id):
    results = []
    success = True
    
    cmd_name = cmd
    
    command_str = COMMAND_LIBRARY.get(cmd_name)
    command_str = cmd_name
    
    if not command_str:
        results.append(f"Error: Unknown command '{cmd_name}'")
        success = False
        
    result = await send_c(executed_at, command_str, client_id)
    print(result)
    results.extend(result['results'])
    print(results)
    
    if not any("OK" in res or "NAVIGATE_OK" in res for res in result['results']):
        success = False
    return {
        "status": "Completed" if success else "Failed",
        "results": results,
        "success": success
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)