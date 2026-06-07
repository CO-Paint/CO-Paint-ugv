import json
import os
from typing import Any, Dict, Optional

import asyncpg
from fastapi import FastAPI, Query, status
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field


app = FastAPI(title="CO-Paint UGV Server")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

POSTGRES_USER = os.getenv("POSTGRES_USER")
POSTGRES_PASSWORD = os.getenv("POSTGRES_PASSWORD")
POSTGRES_DB = os.getenv("POSTGRES_DB")
POSTGRES_HOST = os.getenv("POSTGRES_HOST", "localhost")
POSTGRES_PORT = int(os.getenv("POSTGRES_PORT", "5432"))

SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS telemetry_logs (
    id BIGSERIAL PRIMARY KEY,
    timestamp BIGINT NOT NULL,
    x DOUBLE PRECISION,
    y DOUBLE PRECISION,
    z DOUBLE PRECISION,
    yaw DOUBLE PRECISION,
    arming_state SMALLINT,
    nav_state SMALLINT,
    created_at TIMESTAMP DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul')
);

CREATE INDEX IF NOT EXISTS idx_telemetry_logs_id_desc
    ON telemetry_logs (id DESC);

CREATE TABLE IF NOT EXISTS missions (
    mission_id BIGSERIAL PRIMARY KEY,
    mission_name VARCHAR(200) NOT NULL DEFAULT 'default',
    started_at TIMESTAMP,
    ended_at TIMESTAMP,
    mission_status VARCHAR(50),
    operator_name VARCHAR(100),
    description TEXT,
    created_at TIMESTAMP DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul')
);

CREATE TABLE IF NOT EXISTS web_ui_drone_command_logs (
    command_log_id BIGSERIAL PRIMARY KEY,
    command_time TIMESTAMP NOT NULL DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul'),
    mission_id BIGINT REFERENCES missions (mission_id),
    command_source VARCHAR(50) DEFAULT 'Web UI',
    command_type VARCHAR(100) NOT NULL,
    input_event VARCHAR(100),
    target_x DOUBLE PRECISION,
    target_y DOUBLE PRECISION,
    target_z DOUBLE PRECISION,
    target_yaw DOUBLE PRECISION,
    current_x DOUBLE PRECISION,
    current_y DOUBLE PRECISION,
    current_z DOUBLE PRECISION,
    current_yaw DOUBLE PRECISION,
    arming_state SMALLINT,
    nav_state SMALLINT,
    ws_connected BOOLEAN DEFAULT FALSE,
    drone_connected BOOLEAN DEFAULT FALSE,
    rosbridge_url VARCHAR(255),
    key_name VARCHAR(50),
    command_code INTEGER,
    param1 DOUBLE PRECISION,
    param2 DOUBLE PRECISION,
    command_result VARCHAR(100),
    client_timestamp BIGINT,
    details JSONB DEFAULT '{}'::jsonb,
    created_at TIMESTAMP DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul')
);

CREATE INDEX IF NOT EXISTS idx_web_ui_drone_command_logs_command_time
    ON web_ui_drone_command_logs (command_time DESC);

CREATE TABLE IF NOT EXISTS topic_communication_test_logs (
    topic_test_log_id BIGSERIAL PRIMARY KEY,
    received_at TIMESTAMP NOT NULL DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul'),
    topic_name VARCHAR(255) NOT NULL,
    message_type VARCHAR(255) NOT NULL,
    message_data TEXT,
    source_name VARCHAR(100),
    rosbridge_host VARCHAR(255),
    rosbridge_port INTEGER,
    raw_message JSONB DEFAULT '{}'::jsonb,
    created_at TIMESTAMP DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul')
);

CREATE INDEX IF NOT EXISTS idx_topic_communication_test_logs_received_at
    ON topic_communication_test_logs (received_at DESC);

CREATE TABLE IF NOT EXISTS px4_vehicle_status_logs (
    px4_vehicle_status_log_id BIGSERIAL PRIMARY KEY,
    received_at TIMESTAMP NOT NULL DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul'),
    topic_name VARCHAR(255) NOT NULL,
    px4_timestamp BIGINT,
    arming_state SMALLINT,
    nav_state SMALLINT,
    failsafe BOOLEAN,
    gcs_connection_lost BOOLEAN,
    vehicle_type SMALLINT,
    system_type SMALLINT,
    system_id SMALLINT,
    component_id SMALLINT,
    pre_flight_checks_pass BOOLEAN,
    raw_message JSONB DEFAULT '{}'::jsonb,
    created_at TIMESTAMP DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul')
);

CREATE INDEX IF NOT EXISTS idx_px4_vehicle_status_logs_received_at
    ON px4_vehicle_status_logs (received_at DESC);
"""


class CommandLogRequest(BaseModel):
    mission_id: Optional[int] = None
    command_source: str = Field(default="Web UI", max_length=50)
    command_type: str = Field(..., min_length=1, max_length=100)
    input_event: Optional[str] = Field(default=None, max_length=100)
    target_x: Optional[float] = None
    target_y: Optional[float] = None
    target_z: Optional[float] = None
    target_yaw: Optional[float] = None
    current_x: Optional[float] = None
    current_y: Optional[float] = None
    current_z: Optional[float] = None
    current_yaw: Optional[float] = None
    arming_state: Optional[int] = None
    nav_state: Optional[int] = None
    ws_connected: bool = False
    drone_connected: bool = False
    rosbridge_url: Optional[str] = Field(default=None, max_length=255)
    key_name: Optional[str] = Field(default=None, max_length=50)
    command_code: Optional[int] = None
    param1: Optional[float] = None
    param2: Optional[float] = None
    command_result: str = Field(default="attempted", max_length=100)
    client_timestamp: Optional[int] = None
    details: Dict[str, Any] = Field(default_factory=dict)


def normalize_json_row(row, *json_fields):
    data = dict(row)
    for field in json_fields:
        value = data.get(field)
        if isinstance(value, str):
            try:
                data[field] = json.loads(value)
            except json.JSONDecodeError:
                pass
    return data


@app.on_event("startup")
async def startup_db_client():
    app.state.db_pool = await asyncpg.create_pool(
        user=POSTGRES_USER,
        password=POSTGRES_PASSWORD,
        database=POSTGRES_DB,
        host=POSTGRES_HOST,
        port=POSTGRES_PORT,
    )
    async with app.state.db_pool.acquire() as conn:
        await conn.execute(SCHEMA_SQL)


@app.on_event("shutdown")
async def shutdown_db_client():
    await app.state.db_pool.close()


@app.get("/api/health")
async def health_check():
    async with app.state.db_pool.acquire() as conn:
        await conn.fetchval("SELECT 1")
    return {"status": "healthy"}


@app.get("/api/telemetry")
async def get_telemetry(limit: int = Query(default=100, ge=1, le=1000)):
    async with app.state.db_pool.acquire() as conn:
        rows = await conn.fetch(
            """
            SELECT id, timestamp, x, y, z, yaw, arming_state, nav_state, created_at
            FROM telemetry_logs
            ORDER BY id DESC
            LIMIT $1
            """,
            limit,
        )
    return [dict(row) for row in rows]


@app.post("/api/command-logs", status_code=status.HTTP_201_CREATED)
async def create_command_log(log: CommandLogRequest):
    async with app.state.db_pool.acquire() as conn:
        row = await conn.fetchrow(
            """
            INSERT INTO web_ui_drone_command_logs (
                mission_id, command_source, command_type, input_event,
                target_x, target_y, target_z, target_yaw,
                current_x, current_y, current_z, current_yaw,
                arming_state, nav_state, ws_connected, drone_connected,
                rosbridge_url, key_name, command_code, param1, param2,
                command_result, client_timestamp, details
            )
            VALUES (
                $1, $2, $3, $4,
                $5, $6, $7, $8,
                $9, $10, $11, $12,
                $13, $14, $15, $16,
                $17, $18, $19, $20, $21,
                $22, $23, $24::jsonb
            )
            RETURNING command_log_id, command_time, command_type, command_result
            """,
            log.mission_id,
            log.command_source,
            log.command_type,
            log.input_event,
            log.target_x,
            log.target_y,
            log.target_z,
            log.target_yaw,
            log.current_x,
            log.current_y,
            log.current_z,
            log.current_yaw,
            log.arming_state,
            log.nav_state,
            log.ws_connected,
            log.drone_connected,
            log.rosbridge_url,
            log.key_name,
            log.command_code,
            log.param1,
            log.param2,
            log.command_result,
            log.client_timestamp,
            json.dumps(log.details),
        )
    return dict(row)


@app.get("/api/command-logs")
async def get_command_logs(limit: int = Query(default=100, ge=1, le=1000)):
    async with app.state.db_pool.acquire() as conn:
        rows = await conn.fetch(
            """
            SELECT
                command_log_id, command_time, command_source, command_type,
                input_event, target_x, target_y, target_z, target_yaw,
                current_x, current_y, current_z, current_yaw,
                arming_state, nav_state, ws_connected, drone_connected,
                rosbridge_url, key_name, command_code, param1, param2,
                command_result, client_timestamp, details, created_at
            FROM web_ui_drone_command_logs
            ORDER BY command_log_id DESC
            LIMIT $1
            """,
            limit,
        )
    return [normalize_json_row(row, "details") for row in rows]


@app.get("/api/topic-test-logs")
async def get_topic_test_logs(limit: int = Query(default=100, ge=1, le=1000)):
    async with app.state.db_pool.acquire() as conn:
        rows = await conn.fetch(
            """
            SELECT
                topic_test_log_id, received_at, topic_name, message_type,
                message_data, source_name, rosbridge_host, rosbridge_port,
                raw_message, created_at
            FROM topic_communication_test_logs
            ORDER BY topic_test_log_id DESC
            LIMIT $1
            """,
            limit,
        )
    return [normalize_json_row(row, "raw_message") for row in rows]


@app.get("/api/px4-vehicle-status-logs")
async def get_px4_vehicle_status_logs(limit: int = Query(default=100, ge=1, le=1000)):
    async with app.state.db_pool.acquire() as conn:
        rows = await conn.fetch(
            """
            SELECT
                px4_vehicle_status_log_id, received_at, topic_name, px4_timestamp,
                arming_state, nav_state, failsafe, gcs_connection_lost,
                vehicle_type, system_type, system_id, component_id,
                pre_flight_checks_pass, raw_message, created_at
            FROM px4_vehicle_status_logs
            ORDER BY px4_vehicle_status_log_id DESC
            LIMIT $1
            """,
            limit,
        )
    return [normalize_json_row(row, "raw_message") for row in rows]
