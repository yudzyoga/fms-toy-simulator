from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import httpx

app = FastAPI()

# * additional for browser compatibility
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],    # allor origins make request
    allow_methods=["*"],    # allow all http methods
    allow_headers=["*"],    # allow all http headers
)

# * store client info and avoid duplication
# * store latest confirm data to simulate DBMS (Database Management Systems)
clients = set()
stored_confirm = {}

# * robot API endpoint
ROBOT_URL = "http://robot:8080/pick"  # robot API endpoint

# * stream received data from a client to any other clients, avoid self-receive
@app.websocket("/ws/updates")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.add(websocket)  # add new client
    try:
        while True:
            data = await websocket.receive_text()   # wait async to retrieve data
            print(f"Received: {data}")
            
            # Broadcast to other clients (except sender)
            for client in clients:
                if client != websocket:
                    await client.send_text(data)
    except WebSocketDisconnect:
        clients.remove(websocket)
        print("Client disconnected")

# * simply forward the request asynchronously from user UI to robot system
@app.post("/pick")
async def pick_sync(req: Request):
    body = await req.json()
    async with httpx.AsyncClient() as client:
        response = await client.post(ROBOT_URL, json=body)
    return response.json()

# * store the result temporarily to simulate database system
@app.post("/confirmPick")
async def confirm_do(req: Request):
    global stored_confirm   
    stored_confirm = await req.json()   # wait for retrieving the data
    return {"status": "stored"}

@app.get("/getPick")
async def get_pick():
    return stored_confirm

if __name__ == "__main__":
    uvicorn.run("main_fms:app", host="0.0.0.0", port=8081)