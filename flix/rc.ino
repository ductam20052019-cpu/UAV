// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Work with the RC receiver

#include <IBusBM.h>
#include "util.h"

IBusBM ibus;

uint16_t channels[16]; // raw rc channels
int channelZero[16]; // calibration zero values
int channelMax[16]; // calibration max values

float controlRoll, controlPitch, controlYaw, controlThrottle; // pilot's inputs, range [-1, 1]
float controlMode = NAN;
float controlTime = NAN; // time of the last controls update

int rollChannel = -1, pitchChannel = -1, throttleChannel = -1, yawChannel = -1, modeChannel = -1; // channel mapping

void setupRC() {
	print("Setup RC (iBUS on GPIO16)\n");
	ibus.begin(Serial2, IBUSBM_NOTIMER, 16, 17);
}

bool readRC() {
	ibus.loop();

	bool gotSignal = false;
	for (int i = 0; i < 14; i++) {
		uint16_t value = ibus.readChannel(i);
		channels[i] = value;
		if (value != 0) gotSignal = true;
	}

	if (!gotSignal) return false;
	normalizeRC();
	controlTime = t;
	return true;
}

void normalizeRC() {
	float controls[16];
	for (int i = 0; i < 16; i++) {
		controls[i] = mapf(channels[i], channelZero[i], channelMax[i], 0, 1);
	}
	// Roll/pitch/yaw: 0..1 (RC) -> -1..1 (center 0). Tăng đều 0→1, giảm đều 0→-1.
	float r = rollChannel < 0 ? 0.5f : controls[rollChannel];
	float p = pitchChannel < 0 ? 0.5f : controls[pitchChannel];
	float y = yawChannel < 0 ? 0.5f : controls[yawChannel];
	controlRoll   = constrain((r - 0.5f) * 2.0f, -1.0f, 1.0f);
	controlPitch  = constrain((p - 0.5f) * 2.0f, -1.0f, 1.0f);
	controlYaw    = constrain((y - 0.5f) * 2.0f, -1.0f, 1.0f);
	controlThrottle = throttleChannel < 0 ? 0 : controls[throttleChannel];
	controlMode = modeChannel < 0 ? NAN : controls[modeChannel];
}

void calibrateRC() {
	uint16_t zero[16];
	uint16_t center[16];
	uint16_t max[16];
	print("1/8 Calibrating RC: put all switches to default positions [3 sec]\n");
	pause(3);
	calibrateRCChannel(NULL, zero, zero, "2/8 Move sticks [3 sec]\n...     ...\n...     .o.\n.o.     ...\n");
	calibrateRCChannel(NULL, center, center, "3/8 Move sticks [3 sec]\n...     ...\n.o.     .o.\n...     ...\n");
	calibrateRCChannel(&throttleChannel, zero, max, "4/8 Move sticks [3 sec]\n.o.     ...\n...     .o.\n...     ...\n");
	calibrateRCChannel(&yawChannel, center, max, "5/8 Move sticks [3 sec]\n...     ...\n..o     .o.\n...     ...\n");
	calibrateRCChannel(&pitchChannel, zero, max, "6/8 Move sticks [3 sec]\n...     .o.\n...     ...\n.o.     ...\n");
	calibrateRCChannel(&rollChannel, zero, max, "7/8 Move sticks [3 sec]\n...     ...\n...     ..o\n.o.     ...\n");
	calibrateRCChannel(&modeChannel, zero, max, "8/8 Put mode switch to max [3 sec]\n");
	printRCCalibration();
}

void calibrateRCChannel(int *channel, uint16_t in[16], uint16_t out[16], const char *str) {
	print("%s", str);
	pause(3);
	for (int i = 0; i < 30; i++) readRC(); // try update 30 times max
	memcpy(out, channels, sizeof(channels));

	if (channel == NULL) return; // no channel to calibrate

	// Find channel that changed the most between in and out
	int ch = -1, diff = 0;
	for (int i = 0; i < 16; i++) {
		if (abs(out[i] - in[i]) > diff) {
			ch = i;
			diff = abs(out[i] - in[i]);
		}
	}
	if (ch >= 0 && diff > 10) { // difference threshold is 10
		*channel = ch;
		channelZero[ch] = in[ch];
		channelMax[ch] = out[ch];
	} else {
		*channel = -1;
	}
}

void printRCCalibration() {
	print("Control   Ch     Zero   Max\n");
	print("Roll      %-7d%-7d%-7d\n", rollChannel, rollChannel < 0 ? 0 : channelZero[rollChannel], rollChannel < 0 ? 0 : channelMax[rollChannel]);
	print("Pitch     %-7d%-7d%-7d\n", pitchChannel, pitchChannel < 0 ? 0 : channelZero[pitchChannel], pitchChannel < 0 ? 0 : channelMax[pitchChannel]);
	print("Yaw       %-7d%-7d%-7d\n", yawChannel, yawChannel < 0 ? 0 : channelZero[yawChannel], yawChannel < 0 ? 0 : channelMax[yawChannel]);
	print("Throttle  %-7d%-7d%-7d\n", throttleChannel, throttleChannel < 0 ? 0 : channelZero[throttleChannel], throttleChannel < 0 ? 0 : channelMax[throttleChannel]);
	print("Mode      %-7d%-7d%-7d\n", modeChannel, modeChannel < 0 ? 0 : channelZero[modeChannel], modeChannel < 0 ? 0 : channelMax[modeChannel]);
}