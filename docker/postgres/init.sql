CREATE TABLE IF NOT EXISTS missions (
    mission_id BIGSERIAL PRIMARY KEY,
    mission_name VARCHAR(200) NOT NULL,
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

ALTER TABLE web_ui_drone_command_logs
    ALTER COLUMN command_time SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul'),
    ALTER COLUMN created_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul');

ALTER TABLE web_ui_drone_command_logs
    ADD COLUMN IF NOT EXISTS input_event VARCHAR(100),
    ADD COLUMN IF NOT EXISTS current_x DOUBLE PRECISION,
    ADD COLUMN IF NOT EXISTS current_y DOUBLE PRECISION,
    ADD COLUMN IF NOT EXISTS current_z DOUBLE PRECISION,
    ADD COLUMN IF NOT EXISTS current_yaw DOUBLE PRECISION,
    ADD COLUMN IF NOT EXISTS arming_state SMALLINT,
    ADD COLUMN IF NOT EXISTS nav_state SMALLINT,
    ADD COLUMN IF NOT EXISTS ws_connected BOOLEAN DEFAULT FALSE,
    ADD COLUMN IF NOT EXISTS drone_connected BOOLEAN DEFAULT FALSE,
    ADD COLUMN IF NOT EXISTS rosbridge_url VARCHAR(255),
    ADD COLUMN IF NOT EXISTS key_name VARCHAR(50),
    ADD COLUMN IF NOT EXISTS command_code INTEGER,
    ADD COLUMN IF NOT EXISTS param1 DOUBLE PRECISION,
    ADD COLUMN IF NOT EXISTS param2 DOUBLE PRECISION,
    ADD COLUMN IF NOT EXISTS client_timestamp BIGINT,
    ADD COLUMN IF NOT EXISTS details JSONB DEFAULT '{}'::jsonb;

CREATE TABLE IF NOT EXISTS drone_state_logs (
    drone_state_id BIGSERIAL PRIMARY KEY,
    mission_id BIGINT REFERENCES missions (mission_id),
    recorded_at TIMESTAMP NOT NULL,
    position_x DOUBLE PRECISION,
    position_y DOUBLE PRECISION,
    position_z DOUBLE PRECISION,
    roll DOUBLE PRECISION,
    pitch DOUBLE PRECISION,
    yaw DOUBLE PRECISION,
    velocity_vx DOUBLE PRECISION,
    velocity_vy DOUBLE PRECISION,
    velocity_vz DOUBLE PRECISION,
    battery_status VARCHAR(100),
    arm_state VARCHAR(50),
    flight_mode VARCHAR(50),
    px4_status VARCHAR(100),
    created_at TIMESTAMP DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul')
);

CREATE TABLE IF NOT EXISTS drone_lidar_logs (
    drone_lidar_id BIGSERIAL PRIMARY KEY,
    mission_id BIGINT REFERENCES missions (mission_id),
    recorded_at TIMESTAMP NOT NULL,
    sensor_name VARCHAR(100) DEFAULT 'Drone Livox360 LiDAR',
    topic_name VARCHAR(255),
    slam_processed BOOLEAN DEFAULT FALSE,
    pointcloud_file_path TEXT,
    coordinate_frame VARCHAR(100),
    created_at TIMESTAMP DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul')
);

CREATE TABLE IF NOT EXISTS ugv_lidar_logs (
    ugv_lidar_id BIGSERIAL PRIMARY KEY,
    mission_id BIGINT REFERENCES missions (mission_id),
    recorded_at TIMESTAMP NOT NULL,
    sensor_name VARCHAR(100) DEFAULT 'UGV LiDAR',
    topic_name VARCHAR(255),
    slam_processed BOOLEAN DEFAULT FALSE,
    pointcloud_file_path TEXT,
    coordinate_frame VARCHAR(100),
    created_at TIMESTAMP DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul')
);

CREATE TABLE IF NOT EXISTS drone_slam_results (
    drone_slam_result_id BIGSERIAL PRIMARY KEY,
    mission_id BIGINT REFERENCES missions (mission_id),
    processed_at TIMESTAMP NOT NULL,
    input_sensor VARCHAR(100) DEFAULT 'Drone Livox360 LiDAR',
    input_topic VARCHAR(255),
    output_topic VARCHAR(255),
    slam_position_x DOUBLE PRECISION,
    slam_position_y DOUBLE PRECISION,
    slam_position_z DOUBLE PRECISION,
    slam_roll DOUBLE PRECISION,
    slam_pitch DOUBLE PRECISION,
    slam_yaw DOUBLE PRECISION,
    coordinate_frame VARCHAR(100),
    processing_device VARCHAR(100) DEFAULT 'Main Desktop',
    created_at TIMESTAMP DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul')
);

CREATE TABLE IF NOT EXISTS ugv_slam_results (
    ugv_slam_result_id BIGSERIAL PRIMARY KEY,
    mission_id BIGINT REFERENCES missions (mission_id),
    processed_at TIMESTAMP NOT NULL,
    input_sensor VARCHAR(100) DEFAULT 'UGV LiDAR',
    input_topic VARCHAR(255),
    output_topic VARCHAR(255),
    ugv_position_x DOUBLE PRECISION,
    ugv_position_y DOUBLE PRECISION,
    ugv_yaw DOUBLE PRECISION,
    coordinate_frame VARCHAR(100),
    processing_device VARCHAR(100) DEFAULT 'UGV mini PC',
    created_at TIMESTAMP DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul')
);

CREATE TABLE IF NOT EXISTS drone_camera_detections (
    detection_id BIGSERIAL PRIMARY KEY,
    mission_id BIGINT REFERENCES missions (mission_id),
    detected_at TIMESTAMP NOT NULL,
    camera_name VARCHAR(100) DEFAULT 'Drone Intel Camera',
    image_file_path TEXT,
    model_name VARCHAR(100) DEFAULT 'ResNet50',
    detected_class VARCHAR(100),
    confidence DOUBLE PRECISION,
    bbox_x_min DOUBLE PRECISION,
    bbox_y_min DOUBLE PRECISION,
    bbox_x_max DOUBLE PRECISION,
    bbox_y_max DOUBLE PRECISION,
    processing_result VARCHAR(100),
    created_at TIMESTAMP DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul')
);

CREATE TABLE IF NOT EXISTS equipment_network_status_logs (
    equipment_status_id BIGSERIAL PRIMARY KEY,
    mission_id BIGINT REFERENCES missions (mission_id),
    recorded_at TIMESTAMP NOT NULL,
    equipment_name VARCHAR(100),
    equipment_type VARCHAR(100),
    ip_address INET,
    connection_status VARCHAR(50),
    last_received_at TIMESTAMP,
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

CREATE TABLE IF NOT EXISTS emergency_error_events (
    event_id BIGSERIAL PRIMARY KEY,
    mission_id BIGINT REFERENCES missions (mission_id),
    event_time TIMESTAMP NOT NULL,
    event_type VARCHAR(100),
    device_name VARCHAR(100),
    cause TEXT,
    action_taken TEXT,
    severity VARCHAR(50),
    created_at TIMESTAMP DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul')
);

CREATE TABLE IF NOT EXISTS ugv_motor_logs (
    ugv_motor_log_id BIGSERIAL PRIMARY KEY,
    mission_id BIGINT REFERENCES missions (mission_id),
    recorded_at TIMESTAMP NOT NULL,
    left_motor_speed DOUBLE PRECISION,
    right_motor_speed DOUBLE PRECISION,
    target_speed DOUBLE PRECISION,
    actual_speed DOUBLE PRECISION,
    motor_current DOUBLE PRECISION,
    motor_temperature DOUBLE PRECISION,
    drive_status VARCHAR(50),
    created_at TIMESTAMP DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul')
);

ALTER TABLE missions
    ALTER COLUMN created_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul');

ALTER TABLE drone_state_logs
    ALTER COLUMN created_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul');

ALTER TABLE drone_lidar_logs
    ALTER COLUMN created_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul');

ALTER TABLE ugv_lidar_logs
    ALTER COLUMN created_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul');

ALTER TABLE drone_slam_results
    ALTER COLUMN created_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul');

ALTER TABLE ugv_slam_results
    ALTER COLUMN created_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul');

ALTER TABLE drone_camera_detections
    ALTER COLUMN created_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul');

ALTER TABLE equipment_network_status_logs
    ALTER COLUMN created_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul');

ALTER TABLE topic_communication_test_logs
    ALTER COLUMN received_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul'),
    ALTER COLUMN created_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul');

ALTER TABLE px4_vehicle_status_logs
    ALTER COLUMN received_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul'),
    ALTER COLUMN created_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul');

ALTER TABLE emergency_error_events
    ALTER COLUMN created_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul');

ALTER TABLE ugv_motor_logs
    ALTER COLUMN created_at SET DEFAULT (CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Seoul');

CREATE INDEX IF NOT EXISTS idx_web_ui_drone_command_logs_mission_time
    ON web_ui_drone_command_logs (mission_id, command_time);

CREATE INDEX IF NOT EXISTS idx_web_ui_drone_command_logs_command_time
    ON web_ui_drone_command_logs (command_time DESC);

CREATE INDEX IF NOT EXISTS idx_web_ui_drone_command_logs_event_time
    ON web_ui_drone_command_logs (input_event, command_time DESC);

CREATE INDEX IF NOT EXISTS idx_web_ui_drone_command_logs_result_time
    ON web_ui_drone_command_logs (command_result, command_time DESC);

CREATE INDEX IF NOT EXISTS idx_drone_state_logs_mission_time
    ON drone_state_logs (mission_id, recorded_at);

CREATE INDEX IF NOT EXISTS idx_drone_lidar_logs_mission_time
    ON drone_lidar_logs (mission_id, recorded_at);

CREATE INDEX IF NOT EXISTS idx_ugv_lidar_logs_mission_time
    ON ugv_lidar_logs (mission_id, recorded_at);

CREATE INDEX IF NOT EXISTS idx_drone_slam_results_mission_time
    ON drone_slam_results (mission_id, processed_at);

CREATE INDEX IF NOT EXISTS idx_ugv_slam_results_mission_time
    ON ugv_slam_results (mission_id, processed_at);

CREATE INDEX IF NOT EXISTS idx_drone_camera_detections_mission_time
    ON drone_camera_detections (mission_id, detected_at);

CREATE INDEX IF NOT EXISTS idx_equipment_network_status_logs_mission_time
    ON equipment_network_status_logs (mission_id, recorded_at);

CREATE INDEX IF NOT EXISTS idx_topic_communication_test_logs_received_at
    ON topic_communication_test_logs (received_at DESC);

CREATE INDEX IF NOT EXISTS idx_topic_communication_test_logs_topic_time
    ON topic_communication_test_logs (topic_name, received_at DESC);

CREATE INDEX IF NOT EXISTS idx_px4_vehicle_status_logs_received_at
    ON px4_vehicle_status_logs (received_at DESC);

CREATE INDEX IF NOT EXISTS idx_px4_vehicle_status_logs_topic_time
    ON px4_vehicle_status_logs (topic_name, received_at DESC);

CREATE INDEX IF NOT EXISTS idx_emergency_error_events_mission_time
    ON emergency_error_events (mission_id, event_time);

CREATE INDEX IF NOT EXISTS idx_ugv_motor_logs_mission_time
    ON ugv_motor_logs (mission_id, recorded_at);

-- Legacy table used by the current server telemetry logger/API.
CREATE TABLE IF NOT EXISTS telemetry_logs (
    id SERIAL PRIMARY KEY,
    timestamp BIGINT NOT NULL,
    x DOUBLE PRECISION NOT NULL,
    y DOUBLE PRECISION NOT NULL,
    z DOUBLE PRECISION NOT NULL,
    yaw DOUBLE PRECISION NOT NULL,
    arming_state SMALLINT,
    nav_state SMALLINT,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX IF NOT EXISTS idx_telemetry_timestamp ON telemetry_logs (timestamp);
