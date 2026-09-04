// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// In-RAM logging

#include "vector.h"
#include "util.h"

#define LOG_RATE 100
#define LOG_DURATION 10
#define LOG_SIZE LOG_DURATION * LOG_RATE
extern Vector rate;
Vector attitudeEuler;
Vector attitudeTargetEuler;
extern float pitch_H;
extern float pitch_;
extern float roll_H;
extern float altitude;
extern float kf_vel;
extern float kf_alt;
extern float roll_;
extern float err_vz;
extern float vz_set;
extern Vector rate;
struct LogEntry {
	const char *name;
	float *value;
};

LogEntry logEntries[] = {
	{"t", &t},
	{"gyro.x", &gyro.x},
	{"gyro.y", &gyro.y},
	{"roll_H", &roll_H},
	{"pitch_H", &pitch_H},
};

const int logColumns = sizeof(logEntries) / sizeof(logEntries[0]);
float logBuffer[LOG_SIZE][logColumns];

void prepareLogData() {
	attitudeEuler = attitude.toEuler();
	attitudeTargetEuler = attitudeTarget.toEuler();
}
int glog = 0; 
int lcntr = 0; // Khởi tạo lcntr = 0 để gửi ngay lần đầu
void logData() {
	if(glog == 1){
		if(!(lcntr > 0)){
			print("%f %f %f\n", err_vz, vz_set, kf_vel);		
			lcntr = 10;
		}
		lcntr--;
		return;
	}
	if(glog == 2){
		if(!(lcntr > 0)){
			print("%f %f %f %f %f %f\n", pitch_, pitch_H, attitude.getPitch(),  roll_, roll_H, attitude.getRoll());		
			lcntr = 10;
		}
		lcntr--;
		return;
	}
	if(glog == 3){
		if(!(lcntr > 0)){
			print("%f %f %f\n", altitude, kf_alt, kf_vel);		
			lcntr = 10;
		}
		lcntr--;
		return;
	}
	if(glog == 4){
		// Plot pitch control data: pitchError_rad, pitchRateError, torqueTarget.y, rates.y, pitchRateSp
		// Các biến đã được khai báo global trong control.ino
		extern float pitchError_rad, pitchRateSp, pitchRateError;
		extern Vector rates, torqueTarget;
		extern float dt; // Control loop period
		
		// Accumulate time and send data every 10ms
		static float accumulatedTime = 0.0f;
		const float updateInterval = 0.01f; // 10ms target interval
		
		accumulatedTime += dt;
		
		if(accumulatedTime >= updateInterval){
			// Format: pitchError pitchRateError torqueTarget.y rates.y pitchRateSp
			print("%.3f %.3f %.3f %.3f %.3f\n", 
			      pitchError_rad, 
			      pitchRateError, 
			      torqueTarget.y, 
			      rate.y, 
			      pitchRateSp);
			accumulatedTime -= updateInterval; // Keep remainder for next cycle
		}
	return;
}
	if (!armed) return;
	static int logPointer = 0;
	static Rate period(LOG_RATE);
	if (!period) return;
	prepareLogData();

	for (int i = 0; i < logColumns; i++) {
		logBuffer[logPointer][i] = *logEntries[i].value;
	}

	logPointer++;
	if (logPointer >= LOG_SIZE) {
		logPointer = 0;
	}
}

void dumpLog() {
	// Print header
	for (int i = 0; i < logColumns; i++) {
		print("%s%s", logEntries[i].name, i < logColumns - 1 ? "," : "\n");
	}
	// Print data
	for (int i = 0; i < LOG_SIZE; i++) {
		if (logBuffer[i][0] == 0) continue; // skip empty records
		for (int j = 0; j < logColumns; j++) {
			print("%g%s", logBuffer[i][j], j < logColumns - 1 ? "," : "\n");
		}
	}
}
