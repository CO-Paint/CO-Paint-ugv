// ROS Constants and State
const WS_PROTOCOL = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
const WS_URL = `${WS_PROTOCOL}//${window.location.host}/rosbridge/`;
const DRONE_TIMEOUT_MS = 3000;
const RECONNECT_DELAY_MS = 3000;
const MANUAL_RECONNECT_DELAY_MS = 250;
const TARGET_Z_MAX = 0.0;
const DISPLAY_EPSILON = 0.000001;
const COMMAND_LOG_ENDPOINT = '/api/command-logs';
const KEYBOARD_LOG_THROTTLE_MS = 300;
const WEB_COMMAND_TOPIC = '/web_ui/flight_command';
const WEB_COMMAND_STATUS_TOPIC = '/web_ui/flight_command/status';
const WEB_COMMAND_MESSAGE_TYPE = 'std_msgs/msg/String';
const SUPPORTED_FLIGHT_COMMANDS = new Set(['ARM', 'DISARM', 'TAKEOFF', 'LAND', 'EMERGENCY']);

let ros = new ROSLIB.Ros();
let toastTimer = null;
let reconnectTimer = null;
let manualReconnectRequested = false;
let lastKeyboardLogAt = 0;

const state = {
    curr_x: 0, curr_y: 0, curr_z: 0, curr_yaw: 0,
    velocity_x: null, velocity_y: null, velocity_z: null, speed: null,
    accel_x: null, accel_y: null, accel_z: null, acceleration: null,
    last_accel_sample: null,
    arming_state: 0, nav_state: 0,
    target_x: 0, target_y: 0, target_z: -2.0, target_yaw: 0,
    ws_connected: false,
    ws_connecting: false,
    drone_connected: false,
    last_drone_message_at: 0,
    battery_connected: false,
    battery_remaining: null,
    battery_voltage: null,
    battery_current: null,
    battery_warning: 0
};

// UI Elements
const els = {
    droneInd: document.getElementById('drone-indicator'),
    droneTxt: document.getElementById('drone-text'),
    wsInd: document.getElementById('ws-indicator'),
    wsTxt: document.getElementById('ws-text'),
    valX: document.getElementById('val-x'),
    valY: document.getElementById('val-y'),
    valZ: document.getElementById('val-z'),
    valYaw: document.getElementById('val-yaw'),
    valSpeed: document.getElementById('val-speed'),
    valSpeedDetail: document.getElementById('val-speed-detail'),
    valAccel: document.getElementById('val-accel'),
    valAccelDetail: document.getElementById('val-accel-detail'),
    batteryItem: document.getElementById('battery-item'),
    valBattery: document.getElementById('val-battery'),
    valBatteryDetail: document.getElementById('val-battery-detail'),
    batteryGaugeFill: document.getElementById('battery-gauge-fill'),
    badgeArm: document.getElementById('arm-badge'),
    curTarget: document.getElementById('cur-target'),
    inX: document.getElementById('in-x'),
    inY: document.getElementById('in-y'),
    inZ: document.getElementById('in-z'),
    inYaw: document.getElementById('in-yaw'),
    targetForm: document.getElementById('target-form'),
    armBtn: document.getElementById('btn-arm'),
    offboardBtn: document.getElementById('btn-offboard'),
    disarmBtn: document.getElementById('btn-disarm'),
    landBtn: document.getElementById('btn-land'),
    killBtn: document.getElementById('btn-kill'),
    reconnectBtn: document.getElementById('btn-reconnect'),
    helpPanel: document.getElementById('help-panel'),
    toastMessage: document.getElementById('toast-message')
};

// High-level command publisher and telemetry subscribers.
let flightCommandPub = new ROSLIB.Topic({
    ros: ros,
    name: WEB_COMMAND_TOPIC,
    messageType: WEB_COMMAND_MESSAGE_TYPE
});

let flightCommandStatusSub = new ROSLIB.Topic({
    ros: ros,
    name: WEB_COMMAND_STATUS_TOPIC,
    messageType: WEB_COMMAND_MESSAGE_TYPE
});

let posSub = new ROSLIB.Topic({
    ros: ros,
    name: '/fmu/out/vehicle_local_position',
    messageType: 'px4_msgs/msg/VehicleLocalPosition'
});

let statusSub = new ROSLIB.Topic({
    ros: ros,
    name: '/fmu/out/vehicle_status',
    messageType: 'px4_msgs/msg/VehicleStatus'
});

let statusV1Sub = new ROSLIB.Topic({
    ros: ros,
    name: '/fmu/out/vehicle_status_v1',
    messageType: 'px4_msgs/msg/VehicleStatus'
});

let batterySub = new ROSLIB.Topic({
    ros: ros,
    name: '/fmu/out/battery_status',
    messageType: 'px4_msgs/msg/BatteryStatus'
});

// Event Listeners for ROS Connection
ros.on('connection', () => {
    state.ws_connected = true;
    state.ws_connecting = false;
    window.clearTimeout(reconnectTimer);
    els.wsInd.classList.remove('disconnected');
    els.wsInd.classList.add('connected');
    els.wsTxt.textContent = 'Connected';
    updateReconnectButton();
});

ros.on('error', (err) => {
    state.ws_connected = false;
    state.ws_connecting = false;
    console.error('Error connecting to websocket server: ', err);
    els.wsInd.classList.remove('connected');
    els.wsInd.classList.add('disconnected');
    els.wsTxt.textContent = 'Disconnected';
    updateReconnectButton();
    scheduleReconnect(RECONNECT_DELAY_MS);
});

ros.on('close', () => {
    state.ws_connected = false;
    state.ws_connecting = false;
    els.wsInd.classList.remove('connected');
    els.wsInd.classList.add('disconnected');
    els.wsTxt.textContent = 'Disconnected';
    state.last_drone_message_at = 0;
    setDroneConnected(false);
    updateReconnectButton();

    const delay = manualReconnectRequested ? MANUAL_RECONNECT_DELAY_MS : RECONNECT_DELAY_MS;
    manualReconnectRequested = false;
    scheduleReconnect(delay);
});

function connectRosbridge() {
    if (state.ws_connected || state.ws_connecting) return;
    window.clearTimeout(reconnectTimer);
    state.ws_connecting = true;
    els.wsTxt.textContent = 'Connecting';
    updateReconnectButton();
    console.info(`Connecting to rosbridge at ${WS_URL}`);
    try {
        ros.connect(WS_URL);
    } catch (err) {
        console.error('Failed to start rosbridge connection:', err);
        state.ws_connecting = false;
        els.wsTxt.textContent = 'Disconnected';
        updateReconnectButton();
        scheduleReconnect(RECONNECT_DELAY_MS);
    }
}

function scheduleReconnect(delayMs = RECONNECT_DELAY_MS) {
    window.clearTimeout(reconnectTimer);
    reconnectTimer = setTimeout(connectRosbridge, delayMs);
}

function updateReconnectButton() {
    if (!els.reconnectBtn) return;
    els.reconnectBtn.textContent = 'RECONNECT';
}

function requestRosbridgeReconnect() {
    window.clearTimeout(reconnectTimer);
    state.last_drone_message_at = 0;
    setDroneConnected(false);
    showToast('Reconnecting to rosbridge');

    if (els.reconnectBtn) {
        flashCommandButton(els.reconnectBtn);
    }

    logCommandAttempt({
        commandType: 'RECONNECT',
        inputEvent: 'button_click',
        commandResult: 'attempted'
    });

    if (state.ws_connected || state.ws_connecting) {
        manualReconnectRequested = true;
        try {
            ros.close();
        } catch (err) {
            console.warn('Failed to close rosbridge socket before reconnect:', err);
            state.ws_connected = false;
            state.ws_connecting = false;
            manualReconnectRequested = false;
            connectRosbridge();
        }
        return;
    }

    connectRosbridge();
}

// Callbacks
posSub.subscribe((msg) => {
    markDroneSeen();
    state.curr_x = msg.x || 0;
    state.curr_y = msg.y || 0;
    state.curr_z = msg.z || 0;
    state.curr_yaw = msg.heading || 0;

    els.valX.innerText = state.curr_x.toFixed(2);
    els.valY.innerText = state.curr_y.toFixed(2);
    els.valZ.innerText = state.curr_z.toFixed(2);
    els.valYaw.innerText = (state.curr_yaw * (180 / Math.PI)).toFixed(1);

    updateMotionState(msg);
    updateMotionDisplay();
});

function handleStatusMessage(msg) {
    markDroneSeen();
    state.arming_state = msg.arming_state || 0;
    state.nav_state = msg.nav_state || 0;

    if (state.arming_state === 2) {
        if (!els.badgeArm.classList.contains('armed')) {
            els.badgeArm.classList.add('armed');
            els.badgeArm.innerText = 'ARMED (DANGER)';
        }
    } else {
        els.badgeArm.classList.remove('armed');
        els.badgeArm.innerText = 'DISARMED (SAFE)';
    }

    if (state.arming_state === 1) { // Transitioning?
        setTarget(0, 0, -2.0, 0);
    }
}

statusSub.subscribe(handleStatusMessage);
statusV1Sub.subscribe(handleStatusMessage);

batterySub.subscribe((msg) => {
    markDroneSeen();
    const remaining = Number.isFinite(msg.remaining) ? msg.remaining : null;
    const voltage = Number.isFinite(msg.voltage_v) ? msg.voltage_v : null;
    const current = Number.isFinite(msg.current_a) ? msg.current_a : null;

    state.battery_connected = msg.connected === true;
    state.battery_remaining = remaining !== null && remaining >= 0 ? remaining : null;
    state.battery_voltage = voltage !== null && voltage > 0 ? voltage : null;
    state.battery_current = current !== null && current >= 0 ? current : null;
    state.battery_warning = Number(msg.warning) || 0;

    updateBatteryDisplay();
});

flightCommandStatusSub.subscribe((msg) => {
    const status = String(msg.data || '');
    if (status.startsWith('REJECTED:')) {
        showToast(`Relay rejected ${status.slice('REJECTED:'.length)}`);
    } else if (status.startsWith('ACCEPTED:')) {
        showToast(`Relay accepted ${status.slice('ACCEPTED:'.length)}`);
    }
});

function updateBatteryDisplay() {
    els.batteryItem.classList.remove('battery-warning', 'battery-danger', 'battery-disconnected');

    if (!state.battery_connected) {
        els.valBattery.innerText = '--%';
        els.valBatteryDetail.innerText = 'Battery not connected';
        els.batteryGaugeFill.style.width = '0%';
        els.batteryItem.classList.add('battery-disconnected');
        return;
    }

    const hasRemaining = state.battery_remaining !== null;
    const percent = hasRemaining ? Math.round(Math.max(0, Math.min(1, state.battery_remaining)) * 100) : null;
    const detailParts = [];

    if (state.battery_voltage !== null) detailParts.push(`${state.battery_voltage.toFixed(1)} V`);
    if (state.battery_current !== null) detailParts.push(`${state.battery_current.toFixed(1)} A`);

    els.valBattery.innerText = hasRemaining ? `${percent}%` : '--%';
    els.valBatteryDetail.innerText = detailParts.length ? detailParts.join(' | ') : 'Connected';
    els.batteryGaugeFill.style.width = hasRemaining ? `${percent}%` : '0%';

    if (state.battery_warning >= 2 || (hasRemaining && percent <= 15)) {
        els.batteryItem.classList.add('battery-danger');
    } else if (state.battery_warning === 1 || (hasRemaining && percent <= 30)) {
        els.batteryItem.classList.add('battery-warning');
    }
}

function markDroneSeen() {
    state.last_drone_message_at = Date.now();
    setDroneConnected(true);
}

function setDroneConnected(connected) {
    state.drone_connected = connected;
    els.droneInd.classList.toggle('connected', connected);
    els.droneInd.classList.toggle('disconnected', !connected);
    els.droneTxt.innerText = connected ? 'Connected' : 'Disconnected';
}

function finiteNumber(value) {
    const numberValue = Number(value);
    return Number.isFinite(numberValue) ? numberValue : null;
}

function validFlaggedNumber(value, valid = true) {
    if (valid === false) return null;
    return finiteNumber(value);
}

function vectorMagnitude(values) {
    if (values.some((value) => value === null)) return null;
    return Math.sqrt(values.reduce((sum, value) => sum + value * value, 0));
}

function getLocalPositionTimestampSeconds(msg) {
    const timestampSample = finiteNumber(msg.timestamp_sample);
    const timestamp = finiteNumber(msg.timestamp);
    const timestampUs = timestampSample !== null && timestampSample > 0 ? timestampSample : timestamp;
    return timestampUs !== null && timestampUs > 0 ? timestampUs / 1000000 : Date.now() / 1000;
}

function updateMotionState(msg) {
    const vx = validFlaggedNumber(msg.vx, msg.v_xy_valid);
    const vy = validFlaggedNumber(msg.vy, msg.v_xy_valid);
    const vz = validFlaggedNumber(msg.vz, msg.v_z_valid);
    const ax = finiteNumber(msg.ax);
    const ay = finiteNumber(msg.ay);
    const az = finiteNumber(msg.az);
    const accelTime = getLocalPositionTimestampSeconds(msg);

    state.velocity_x = vx;
    state.velocity_y = vy;
    state.velocity_z = vz;
    state.speed = vectorMagnitude([vx, vy, vz]);

    state.accel_x = ax;
    state.accel_y = ay;
    state.accel_z = az;
    state.acceleration = vectorMagnitude([ax, ay, az]);

    if (ax !== null && ay !== null && az !== null) {
        state.last_accel_sample = { x: ax, y: ay, z: az, time: accelTime };
    }
}

function formatTelemetryValue(value, digits = 2) {
    return value === null ? '--' : value.toFixed(digits);
}

function formatNedDetail(north, east, down, digits = 2) {
    return `N ${formatTelemetryValue(north, digits)} | E ${formatTelemetryValue(east, digits)} | D ${formatTelemetryValue(down, digits)}`;
}

function updateMotionDisplay() {
    els.valSpeed.innerText = formatTelemetryValue(state.speed, 2);
    els.valSpeedDetail.innerText = formatNedDetail(state.velocity_x, state.velocity_y, state.velocity_z, 2);
    els.valAccel.innerText = formatTelemetryValue(state.acceleration, 2);
    els.valAccelDetail.innerText = formatNedDetail(state.accel_x, state.accel_y, state.accel_z, 2);
}

function updateDroneConnectionStatus() {
    const hasRecentMessage = state.last_drone_message_at > 0 &&
        Date.now() - state.last_drone_message_at <= DRONE_TIMEOUT_MS;
    if (hasRecentMessage !== state.drone_connected) {
        setDroneConnected(hasRecentMessage);
    }
}

function showToast(message) {
    window.clearTimeout(toastTimer);
    els.toastMessage.innerText = message;
    els.toastMessage.classList.add('visible');
    toastTimer = window.setTimeout(() => {
        els.toastMessage.classList.remove('visible');
    }, 1600);
}

function nullableNumber(value) {
    return Number.isFinite(value) ? value : null;
}

function getTargetChangeResult() {
    if (state.drone_connected) return 'target_updated_local_drone_connected';
    if (state.ws_connected) return 'target_updated_local_ws_only';
    return 'target_updated_local_offline';
}

function logCommandAttempt(fields = {}) {
    const payload = {
        command_source: 'Web UI',
        command_type: fields.commandType || 'UNKNOWN',
        input_event: fields.inputEvent || null,
        target_x: nullableNumber(state.target_x),
        target_y: nullableNumber(state.target_y),
        target_z: nullableNumber(state.target_z),
        target_yaw: nullableNumber(state.target_yaw),
        current_x: nullableNumber(state.curr_x),
        current_y: nullableNumber(state.curr_y),
        current_z: nullableNumber(state.curr_z),
        current_yaw: nullableNumber(state.curr_yaw),
        arming_state: Number.isFinite(state.arming_state) ? state.arming_state : null,
        nav_state: Number.isFinite(state.nav_state) ? state.nav_state : null,
        ws_connected: state.ws_connected,
        drone_connected: state.drone_connected,
        rosbridge_url: WS_URL,
        key_name: fields.keyName || null,
        command_code: Number.isFinite(fields.commandCode) ? fields.commandCode : null,
        param1: nullableNumber(fields.param1),
        param2: nullableNumber(fields.param2),
        command_result: fields.commandResult || 'attempted',
        client_timestamp: Date.now(),
        details: fields.details || {}
    };

    fetch(COMMAND_LOG_ENDPOINT, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
    }).then((response) => {
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }
    }).catch((err) => {
        console.warn('Failed to write command log:', err);
    });
}

// Update Target display
function normalizeDisplayZero(value) {
    return Math.abs(value) < DISPLAY_EPSILON ? 0 : value;
}

function clampTargetZ(value) {
    return Math.min(TARGET_Z_MAX, normalizeDisplayZero(value));
}

function formatFixed(value, digits = 1) {
    const factor = 10 ** digits;
    const rounded = Math.round(normalizeDisplayZero(value) * factor) / factor;
    return (Object.is(rounded, -0) ? 0 : rounded).toFixed(digits);
}

function updateTargetDisplay() {
    els.curTarget.innerText = `X: ${formatFixed(state.target_x)} | Y: ${formatFixed(state.target_y)} | Z: ${formatFixed(state.target_z)} | Yaw: ${formatFixed(state.target_yaw)}°`;
}

function setTarget(x, y, z, yaw) {
    state.target_x = normalizeDisplayZero(x);
    state.target_y = normalizeDisplayZero(y);
    state.target_z = clampTargetZ(z);
    state.target_yaw = normalizeDisplayZero(yaw);
    updateTargetDisplay();
}

function incrementTarget(dx = 0, dy = 0, dz = 0, dyaw = 0) {
    setTarget(state.target_x + dx, state.target_y + dy, state.target_z + dz, state.target_yaw + dyaw);
}

// Commands
function isSupportedFlightCommand(commandName) {
    return SUPPORTED_FLIGHT_COMMANDS.has(commandName) || commandName.startsWith('ALIGN_FOR_LAND:');
}

function makeAlignCommand() {
    return `ALIGN_FOR_LAND:${state.target_x},${state.target_y},${state.target_z},${state.target_yaw}`;
}

function sendFlightCommand(commandName) {
    const highLevelCommand = String(commandName || '').trim().toUpperCase();
    if (!highLevelCommand) {
        return {
            published: false,
            result: 'blocked_empty_command'
        };
    }
    if (!isSupportedFlightCommand(highLevelCommand)) {
        return {
            published: false,
            result: 'blocked_unsupported_by_flight_controller'
        };
    }

    if (!state.ws_connected) {
        return {
            published: false,
            result: 'blocked_ws_disconnected'
        };
    }

    let msg = new ROSLIB.Message({
        data: highLevelCommand
    });

    try {
        flightCommandPub.publish(msg);
        return {
            published: true,
            result: state.drone_connected ? 'published_to_relay_drone_connected' : 'published_to_relay_ws_only'
        };
    } catch (err) {
        console.error('Failed to publish flight command:', err);
        return {
            published: false,
            result: 'publish_failed',
            error: err && err.message ? err.message : String(err)
        };
    }
}

function logFlightCommandPublish(commandType, inputEvent, publishResult, details = {}) {
    logCommandAttempt({
        commandType,
        inputEvent,
        commandResult: publishResult.result,
        details: {
            published: publishResult.published,
            error: publishResult.error || null,
            command_topic: WEB_COMMAND_TOPIC,
            relay_status_topic: WEB_COMMAND_STATUS_TOPIC,
            route: 'web_ui -> ugv_web_command_relay -> flight_control_node',
            ...details
        }
    });
}

function flashCommandButton(button) {
    button.classList.remove('command-clicked');
    void button.offsetWidth;
    button.classList.add('command-clicked');
    window.setTimeout(() => {
        button.classList.remove('command-clicked');
    }, 180);
}

function sendFlightCommandWithFeedback(button, message, commandType, flightCommand) {
    flashCommandButton(button);
    const publishResult = sendFlightCommand(flightCommand);
    logFlightCommandPublish(commandType, 'button_click', publishResult, {
        high_level_command: flightCommand
    });
    if (publishResult.result === 'blocked_ws_disconnected') {
        showToast('WebSocket disconnected; command logged');
    } else if (publishResult.result === 'blocked_unsupported_by_flight_controller') {
        showToast(`${commandType} is not supported by current flight controller API`);
    } else {
        showToast(message);
    }
}

// Button Bindings
els.armBtn.addEventListener('click', () => {
    sendFlightCommandWithFeedback(els.armBtn, 'ARM command sent to relay', 'ARM', 'ARM');
});
els.offboardBtn.addEventListener('click', () => {
    sendFlightCommandWithFeedback(els.offboardBtn, 'TAKEOFF command sent to relay', 'TAKEOFF', 'TAKEOFF');
});
els.disarmBtn.addEventListener('click', () => {
    sendFlightCommandWithFeedback(els.disarmBtn, 'DISARM command sent to relay', 'DISARM', 'DISARM');
});
els.landBtn.addEventListener('click', () => {
    sendFlightCommandWithFeedback(els.landBtn, 'LAND command sent to relay', 'LAND', 'LAND');
});
els.killBtn.addEventListener('click', () => {
    sendFlightCommandWithFeedback(els.killBtn, 'EMERGENCY command sent to relay', 'EMERGENCY', 'EMERGENCY');
});

els.reconnectBtn.addEventListener('click', requestRosbridgeReconnect);

// Form Binding
els.targetForm.addEventListener('submit', (e) => {
    e.preventDefault();
    const parseInput = (input, fallback) => {
        const value = parseFloat(input.value);
        return Number.isFinite(value) ? value : fallback;
    };

    const nextTarget = {
        x: parseInput(els.inX, 0),
        y: parseInput(els.inY, 0),
        z: parseInput(els.inZ, -2.0),
        yaw: parseInput(els.inYaw, 0)
    };

    setTarget(nextTarget.x, nextTarget.y, nextTarget.z, nextTarget.yaw);
    if (e.submitter) {
        flashCommandButton(e.submitter);
    }
    const alignCommand = makeAlignCommand();
    const publishResult = sendFlightCommand(alignCommand);
    logFlightCommandPublish('SUBMIT_COORDINATES', 'form_submit', publishResult, {
        high_level_command: alignCommand,
        submitted_target: nextTarget
    });
    showToast(publishResult.published ? 'ALIGN command sent to relay' : 'Coordinates updated locally');
});

function logLocalTargetChange(action, publishResult, keyName) {
    logCommandAttempt({
        commandType: action.commandType,
        inputEvent: 'keyboard',
        keyName,
        commandResult: publishResult.result,
        details: {
            published: publishResult.published,
            error: publishResult.error || null,
            high_level_command: makeAlignCommand(),
            command_topic: WEB_COMMAND_TOPIC,
            relay_status_topic: WEB_COMMAND_STATUS_TOPIC,
            repeat: action.repeat,
            dx: action.dx,
            dy: action.dy,
            dz: action.dz,
            dyaw: action.dyaw
        }
    });
}

// Keyboard Teleop
window.addEventListener('keydown', (e) => {
    if (document.activeElement.tagName === 'INPUT') return; // Don't trigger if typing

    const step = 0.2;
    let action = null;

    switch (e.key.toLowerCase()) {
        case 'w': action = { commandType: 'KEYBOARD_FORWARD_X', dx: step, dy: 0, dz: 0, dyaw: 0 }; break;
        case 's': action = { commandType: 'KEYBOARD_BACKWARD_X', dx: -step, dy: 0, dz: 0, dyaw: 0 }; break;
        case 'a': action = { commandType: 'KEYBOARD_LEFT_Y', dx: 0, dy: -step, dz: 0, dyaw: 0 }; break;
        case 'd': action = { commandType: 'KEYBOARD_RIGHT_Y', dx: 0, dy: step, dz: 0, dyaw: 0 }; break;
        case 'arrowup': action = { commandType: 'KEYBOARD_UP_Z', dx: 0, dy: 0, dz: -step, dyaw: 0, preventDefault: true }; break;
        case 'arrowdown': action = { commandType: 'KEYBOARD_DOWN_Z', dx: 0, dy: 0, dz: step, dyaw: 0, preventDefault: true }; break;
        case 'q': action = { commandType: 'KEYBOARD_YAW_LEFT', dx: 0, dy: 0, dz: 0, dyaw: -10.0 }; break;
        case 'e': action = { commandType: 'KEYBOARD_YAW_RIGHT', dx: 0, dy: 0, dz: 0, dyaw: 10.0 }; break;
        default: return;
    }

    incrementTarget(action.dx, action.dy, action.dz, action.dyaw);
    if (action.preventDefault) {
        e.preventDefault();
    }

    const publishResult = sendFlightCommand(makeAlignCommand());
    const now = Date.now();
    if (now - lastKeyboardLogAt >= KEYBOARD_LOG_THROTTLE_MS) {
        lastKeyboardLogAt = now;
        action.repeat = e.repeat;
        logLocalTargetChange(action, publishResult, e.key);
    }
});

setInterval(updateDroneConnectionStatus, 500);

connectRosbridge();
