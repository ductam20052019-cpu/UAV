// Copyright (c) 2024 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Fail-safe functions

#define RC_LOSS_TIMEOUT 1
#define DESCEND_TIME 10
#define MAVLINK_TIMEOUT 1   // giây
extern float controlTime;
extern float controlRoll, controlPitch, controlThrottle, controlYaw;
extern float lastMavlinkTime;
void failsafe() {
	rcLossFailsafe();
	autoFailsafe();
}
/*void wifiFailsafe() {
    if (armed && (t - lastMavlinkTime > MAVLINK_TIMEOUT)) {
        // UAV mất MAVLink → disarm ngay
        armed = false;
        thrustTarget = 0;
        memset(motors, 0, sizeof(motors));
        resetAllPIDs();
        // Optional log
        // Serial.println("Failsafe: MAVLink lost, UAV d
}*/
// RC loss failsafe
void rcLossFailsafe() {
	if (controlTime == 0) return; // no RC at all
	if (!armed) return;
	if (t - controlTime > RC_LOSS_TIMEOUT) {
		descend();
	}
}

// Smooth descend on RC lost
void descend() {

	attitudeTarget = Quaternion();
	thrustTarget -= dt / DESCEND_TIME;
	if (thrustTarget < 0) {
		thrustTarget = 0;
		armed = false;
	}
}

// Allow pilot to interrupt automatic flight
void autoFailsafe() {
	static float roll, pitch, yaw, throttle;
	if (roll != controlRoll || pitch != controlPitch || yaw != controlYaw || abs(throttle - controlThrottle) > 0.05) {
		// controls changed
	}
	roll = controlRoll;
	pitch = controlPitch;
	yaw = controlYaw;
	throttle = controlThrottle;
}
