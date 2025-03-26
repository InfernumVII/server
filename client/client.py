# client.py
import asyncio
import websockets
import time
import socket
import platform

CURRENT_COMMAND = None
CURRENT_WEBSOCKET = None

async def listen_commands():
    global CURRENT_COMMAND, CURRENT_WEBSOCKET
    
    uri = "ws://localhost:8000/ws"
    async with websockets.connect(uri) as websocket:
        CURRENT_WEBSOCKET = websocket
        print("Client connected!")
        while True:
            try:
                message = await websocket.recv()
                
                if message == "get_client_info":
                    await websocket.send(get_client_info())
                    continue
                
                print(f"Received: {message}")

                # Обработка специальных команд
                if message.startswith("spec|"):
                    _, command = message.split("|", 1)
                    asyncio.create_task(
                        execute_command_special(websocket, command)
                    )
                    continue
                
                # Обработка обычных команд
                if "|" in message:
                    execute_at, command = message.split("|", 1)
                    execute_at = float(execute_at)
                    
                    print(f"Scheduled command for UNIX {execute_at}")
                    
                    while time.time() < execute_at:
                        await asyncio.sleep(0.01)
                    
                    print(f"Executing: {command}")
                    try:
                        CURRENT_COMMAND = asyncio.create_task(
                            execute_command(websocket, command)
                        )
                        await CURRENT_COMMAND
                        await websocket.send("ACK :OK")
                    except Exception as e:
                        error_msg = f"ACK :Error: {str(e)}"
                        await websocket.send(error_msg)
                    finally:
                        CURRENT_COMMAND = None
                
            except websockets.exceptions.ConnectionClosed:
                print("Connection closed")
                break
            except ValueError:
                await websocket.send("ACK:Invalid message format")
            except Exception as e:
                await websocket.send(f"ACK:Unexpected error: {str(e)}")


async def execute_command(websocket, command):
    """Execute a command with timeout and emergency checking"""
    try:
        # Execute the command
        ns = {}
        command = "result=" + command
        exec(command, globals(), ns)
        result = ns.get('result', 'OK')
        print(result)
        

            
        await websocket.send(f"ACK:{result}")
        return result
    except Exception as e:
        raise e
    
async def execute_command_special(websocket, command):
    """Выполнение специальной команды с обработкой результата"""
    try:
        ns = {}
        exec(command, globals(), ns)
        result = ns.get('result', 'OK')
        response = f"SPECIAL: {result}"
    except Exception as e:
        response = f"SPECIAL: Error: {str(e)}"
    
    try:
        await websocket.send(response)
    except Exception as e:
        print(f"Failed to send spec response: {str(e)}")
    
def battery_info():
    return "12|13"

def get_client_info():
    client_name = f"{socket.gethostname()}*{time.time()}"
    return client_name

def land_wait():
    return "OK"

def hello_world():
    from random import randint
    print("Hello World")
    time.sleep(randint(5,10))
    return "OK"

def getRecognizedMarks():
    result = "OK 1 2 3 4 5"
    return result
        
asyncio.get_event_loop().run_until_complete(listen_commands())
print(1)