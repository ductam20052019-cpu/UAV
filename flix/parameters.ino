// Copyright (c) 2024 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Parameters storage in flash memory

#include <Preferences.h>
#include "util.h"

extern int channelZero[16];
extern int channelMax[16];
extern int rollChannel, pitchChannel, throttleChannel, yawChannel, armedChannel, modeChannel;
Preferences storage;

struct Parameter {
	const char *name; // max length is 15 (Preferences key limit)
	float *variable;
	float value; // cache
};

Parameter parameters[] = {
	// PID control parameters
	{"ROLLRATE_P", &rollRatePID.p},  // Pitch/Roll rate PID P
	{"ROLLRATE_I", &rollRatePID.i},  // Pitch/Roll rate PID I
	{"ROLLRATE_D", &rollRatePID.d},  // Pitch/Roll rate PID D

	{"PITCHRATE_P", &pitchRatePID.p},  // Pitch/Roll rate PID P
	{"PITCHRATE_I", &pitchRatePID.i},  // Pitch/Roll rate PID I
	{"PITCHRATE_D", &pitchRatePID.d},  // Pitch/Roll rate PID D

	{"ROLL_P", &rollPID.p},        // Roll/Pitch angle PID P
	{"PITCH_P", &pitchPID.p},        // Pitch/Pitch angle PID P
	{"YAW_P", &yawRatePID.p},        // Yaw rate PID P

	{"VEL_P", &vel_zPID.p},        // Altitude PID P
	{"VEL_I", &vel_zPID.i},        // Altitude PID I
	{"VEL_D", &vel_zPID.d},
	        // Altitude PID D
	{"VEL_D", &vel_zPID.d},


};

void setupParameters() {
	storage.begin("flix", false);
	// Read parameters from storage
	for (auto &parameter : parameters) {
		if (!storage.isKey(parameter.name)) {
			storage.putFloat(parameter.name, *parameter.variable);
		}
		*parameter.variable = storage.getFloat(parameter.name, *parameter.variable);
		parameter.value = *parameter.variable;
	}
}

int parametersCount() {
	return sizeof(parameters) / sizeof(parameters[0]);
}

const char *getParameterName(int index) {
	if (index < 0 || index >= parametersCount()) return "";
	return parameters[index].name;
}

float getParameter(int index) {
	if (index < 0 || index >= parametersCount()) return NAN;
	return *parameters[index].variable;
}

float getParameter(const char *name) {
	for (auto &parameter : parameters) {
		if (strcmp(parameter.name, name) == 0) {
			return *parameter.variable;
		}
	}
	return NAN;
}

bool setParameter(const char *name, const float value) {
	for (auto &parameter : parameters) {
		if (strcmp(parameter.name, name) == 0) {
			*parameter.variable = value;
			return true;
		}
	}
	return false;
}

void syncParameters() {
	static Rate rate(1);
	if (!rate) return; // sync once per second
	if (motorsActive()) return; // don't use flash while flying, it may cause a delay

	for (auto &parameter : parameters) {
		if (parameter.value == *parameter.variable) continue;
		if (isnan(parameter.value) && isnan(*parameter.variable)) continue; // handle NAN != NAN
		storage.putFloat(parameter.name, *parameter.variable);
		parameter.value = *parameter.variable;
	}
}

void printParameters() {
	for (auto &parameter : parameters) {
		print("%s = %g\n", parameter.name, *parameter.variable);
	}
}

void resetParameters() {
	storage.clear();
	ESP.restart();
}