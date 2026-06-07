import json
import os
import threading
import time

import psycopg2
import roslibpy


POSTGRES_USER = os.getenv("POSTGRES_USER")
POSTGRES_PASSWORD = os.getenv("POSTGRES_PASSWORD")
POSTGRES_DB = os.getenv("POSTGRES_DB")
POSTGRES_HOST = os.getenv("POSTGRES_HOST", "localhost")
POSTGRES_PORT = int(os.getenv("POSTGRES_PORT", "5432"))

ROS_BRIDGE_HOST = os.getenv("ROS_BRIDGE_HOST", "localhost")
ROS_BRIDGE_PORT = int(os.getenv("ROSBRIDGE_PORT", "9090"))
PX4_LOCAL_POSITION_TOPIC = os.getenv(
    "PX4_LOCAL_POSITION_TOPIC",
    "/fmu/out/vehicle_local_position",
)
PX4_VEHICLE_STATUS_TOPIC = os.getenv(
    "PX4_VEHICLE_STATUS_TOPIC",
    "/fmu/out/vehicle_status",
)
TOPIC_TEST_TOPIC = os.getenv("TOPIC_TEST_TOPIC", "/copaint/net_test")
TOPIC_TEST_MESSAGE_TYPE = os.getenv("TOPIC_TEST_MESSAGE_TYPE", "std_msgs/msg/String")
TOPIC_TEST_SOURCE_NAME = os.getenv("TOPIC_TEST_SOURCE_NAME", "uav-edge")

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
"""

INSERT_TELEMETRY_SQL = """
INSERT INTO telemetry_logs (timestamp, x, y, z, yaw, arming_state, nav_state)
VALUES (%s, %s, %s, %s, %s, %s, %s)
"""

INSERT_TOPIC_TEST_SQL = """
INSERT INTO topic_communication_test_logs (
    topic_name, message_type, message_data, source_name,
    rosbridge_host, rosbridge_port, raw_message
)
VALUES (%s, %s, %s, %s, %s, %s, %s::jsonb)
"""

INSERT_PX4_VEHICLE_STATUS_SQL = """
INSERT INTO px4_vehicle_status_logs (
    topic_name, px4_timestamp, arming_state, nav_state, failsafe,
    gcs_connection_lost, vehicle_type, system_type, system_id,
    component_id, pre_flight_checks_pass, raw_message
)
VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s::jsonb)
"""


def get_db_connection():
    while True:
        try:
            conn = psycopg2.connect(
                dbname=POSTGRES_DB,
                user=POSTGRES_USER,
                password=POSTGRES_PASSWORD,
                host=POSTGRES_HOST,
                port=POSTGRES_PORT,
            )
            print("Connected to PostgreSQL.")
            return conn
        except Exception as exc:
            print(f"Waiting for PostgreSQL... {exc}")
            time.sleep(2)


def main():
    conn = get_db_connection()
    cursor = conn.cursor()
    state_lock = threading.Lock()
    db_lock = threading.Lock()
    state = {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "yaw": 0.0,
        "arming_state": 0,
        "nav_state": 0,
        "updated_pos": False,
        "updated_status": False,
    }

    with db_lock:
        cursor.execute(SCHEMA_SQL)
        conn.commit()

    def write_row(sql, row, label):
        try:
            with db_lock:
                cursor.execute(sql, row)
                conn.commit()
        except Exception as exc:
            with db_lock:
                conn.rollback()
            print(f"Failed to write {label}: {exc}")

    def pos_callback(msg):
        with state_lock:
            state["x"] = msg.get("x", 0.0)
            state["y"] = msg.get("y", 0.0)
            state["z"] = msg.get("z", 0.0)
            state["yaw"] = msg.get("heading", 0.0)
            state["updated_pos"] = True

    def status_callback(msg):
        with state_lock:
            state["arming_state"] = msg.get("arming_state", 0)
            state["nav_state"] = msg.get("nav_state", 0)
            state["updated_status"] = True
        write_vehicle_status_log(PX4_VEHICLE_STATUS_TOPIC, msg)

    def write_topic_test_log(topic_name, message_type, source_name, msg):
        message_data = msg.get("data")
        if message_data is None:
            message_data = json.dumps(msg, sort_keys=True)
        write_row(
            INSERT_TOPIC_TEST_SQL,
            (
                topic_name,
                message_type,
                message_data,
                source_name,
                ROS_BRIDGE_HOST,
                ROS_BRIDGE_PORT,
                json.dumps(msg),
            ),
            "topic test log",
        )

    def write_vehicle_status_log(topic_name, msg):
        write_row(
            INSERT_PX4_VEHICLE_STATUS_SQL,
            (
                topic_name,
                msg.get("timestamp"),
                msg.get("arming_state"),
                msg.get("nav_state"),
                msg.get("failsafe"),
                msg.get("gcs_connection_lost"),
                msg.get("vehicle_type"),
                msg.get("system_type"),
                msg.get("system_id"),
                msg.get("component_id"),
                msg.get("pre_flight_checks_pass"),
                json.dumps(msg),
            ),
            "PX4 vehicle status log",
        )

    def topic_test_callback(msg):
        write_topic_test_log(
            TOPIC_TEST_TOPIC,
            TOPIC_TEST_MESSAGE_TYPE,
            TOPIC_TEST_SOURCE_NAME,
            msg,
        )

    ros = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
    roslibpy.Topic(
        ros,
        PX4_LOCAL_POSITION_TOPIC,
        "px4_msgs/msg/VehicleLocalPosition",
    ).subscribe(pos_callback)
    roslibpy.Topic(
        ros,
        PX4_VEHICLE_STATUS_TOPIC,
        "px4_msgs/msg/VehicleStatus",
    ).subscribe(status_callback)
    roslibpy.Topic(
        ros,
        TOPIC_TEST_TOPIC,
        TOPIC_TEST_MESSAGE_TYPE,
    ).subscribe(topic_test_callback)

    def run_rosbridge():
        while True:
            try:
                ros.run()
                return
            except Exception as exc:
                print(f"Waiting for rosbridge... {exc}")
                time.sleep(2)

    threading.Thread(target=run_rosbridge, daemon=True).start()

    while not ros.is_connected:
        print(f"Waiting for rosbridge at {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}...")
        time.sleep(2)

    print("Connected to rosbridge. Logging telemetry.")

    try:
        while True:
            time.sleep(1.0)
            with state_lock:
                if not (state["updated_pos"] or state["updated_status"]):
                    continue
                row = (
                    int(time.time()),
                    state["x"],
                    state["y"],
                    state["z"],
                    state["yaw"],
                    state["arming_state"],
                    state["nav_state"],
                )
                state["updated_pos"] = False
                state["updated_status"] = False
            write_row(INSERT_TELEMETRY_SQL, row, "telemetry")
    finally:
        ros.terminate()
        cursor.close()
        conn.close()


if __name__ == "__main__":
    main()
