import asyncio
import json
import threading
from pathlib import Path
from typing import Dict, Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
import uvicorn

from config import LOCATIONS
from task_queue import DEFAULT_PRIORITY, PRIORITY_LEVELS, TaskQueue

UI_DIR = Path(__file__).resolve().parent / "ui"
INDEX_FILE = UI_DIR / "index.html"


class TaskCreate(BaseModel):
    origin: str
    destination: str
    priority: str = DEFAULT_PRIORITY


class ConnectionManager:
    def __init__(self) -> None:
        self._connections = set()

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self._connections.add(websocket)

    async def disconnect(self, websocket: WebSocket) -> None:
        self._connections.discard(websocket)

    async def broadcast(self, message: Dict[str, object]) -> None:
        if not self._connections:
            return
        payload = json.dumps(message)
        stale = []
        for websocket in self._connections:
            try:
                await websocket.send_text(payload)
            except Exception:
                stale.append(websocket)
        for websocket in stale:
            self._connections.discard(websocket)


class TaskEventBus:
    def __init__(self) -> None:
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._queue: Optional[asyncio.Queue] = None

    def attach_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop
        self._queue = asyncio.Queue()

    def emit(self, event: Dict[str, object]) -> None:
        if self._loop is None or self._queue is None:
            return
        self._loop.call_soon_threadsafe(self._queue.put_nowait, event)

    async def next_event(self) -> Dict[str, object]:
        if self._queue is None:
            await asyncio.sleep(0.2)
            return {"type": "noop", "payload": None}
        return await self._queue.get()


def _load_index_html() -> str:
    try:
        return INDEX_FILE.read_text(encoding="utf-8")
    except OSError:
        return "<html><body><h1>UI no disponible</h1></body></html>"


def create_app(task_queue: TaskQueue, event_bus: TaskEventBus) -> FastAPI:
    app = FastAPI()
    manager = ConnectionManager()
    task_queue.set_event_emitter(event_bus.emit)
    ui_html = _load_index_html()

    async def event_pump() -> None:
        while True:
            event = await event_bus.next_event()
            if event.get("type") == "noop":
                continue
            await manager.broadcast(event)

    @app.on_event("startup")
    async def startup_event() -> None:
        event_bus.attach_loop(asyncio.get_running_loop())
        asyncio.create_task(event_pump())

    @app.get("/")
    def root_ui() -> HTMLResponse:
        return HTMLResponse(ui_html)

    @app.get("/locations")
    def list_locations() -> Dict[str, object]:
        return {"locations": ["CURRENT"] + sorted(LOCATIONS.keys())}

    @app.get("/tasks")
    def list_tasks() -> Dict[str, object]:
        return task_queue.get_snapshot()

    @app.post("/tasks")
    def create_task(payload: TaskCreate) -> Dict[str, object]:
        if payload.origin != "CURRENT" and payload.origin not in LOCATIONS:
            raise HTTPException(status_code=400, detail="Origen no valido")
        if payload.destination not in LOCATIONS:
            raise HTTPException(status_code=400, detail="Destino no valido")
        if payload.priority not in PRIORITY_LEVELS:
            raise HTTPException(status_code=400, detail="Prioridad no valida")
        task = task_queue.add_task(payload.origin, payload.destination, payload.priority)
        return task.to_dict()

    @app.delete("/tasks/{task_id}")
    def delete_task(task_id: int) -> Dict[str, object]:
        if not task_queue.cancel(task_id):
            raise HTTPException(status_code=404, detail="Tarea no encontrada o en ejecucion")
        return {"ok": True}

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket) -> None:
        await manager.connect(websocket)
        await websocket.send_text(
            json.dumps({"type": "snapshot", "payload": task_queue.get_snapshot()})
        )
        try:
            while True:
                await websocket.receive_text()
        except WebSocketDisconnect:
            await manager.disconnect(websocket)

    return app


def start_task_server(
    task_queue: TaskQueue,
    host: str = "127.0.0.1",
    port: int = 8000,
    log_level: str = "info",
) -> uvicorn.Server:
    event_bus = TaskEventBus()
    app = create_app(task_queue, event_bus)

    config = uvicorn.Config(app=app, host=host, port=port, log_level=log_level)
    server = uvicorn.Server(config)

    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()

    return server
