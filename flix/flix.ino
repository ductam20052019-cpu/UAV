// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Main firmware file

#include "vector.h"
#include "quaternion.h"
#include "util.h"

#define WIFI_ENABLED 1

extern float t, dt;
extern float controlRoll, controlPitch, controlYaw, controlThrottle, controlMode;
extern Vector gyro, acc;
extern Vector rates;
extern Quaternion attitude;
extern bool landed;
extern float motors[4];

void setup() {
	Serial.begin(115200);
	print("Initializing flix\n");
	disableBrownOut();
	setupParameters();
	setupLED();
	setupMotors();
	setLED(true);
#if WIFI_ENABLED
	setupWiFi();
#endif
	setupIMU();
	setupBarometer();
	setupRC();
	setLED(false);
	print("Initializing complete\n");
}

void loop() {
	readIMU();
	update_altitude();
	step();
	readRC();
	estimate();
	control();
	sendMotors();
	handleInput();
	// === CODE CHÈN THÊM ĐỂ ĐẨY DATA SANG MATLAB ===
// === CODE TỰ TÍNH EULER TỪ QUATERNION ===
  // 1. Trích xuất 4 thành phần trục của Quaternion
  float w = attitude.w;
  float x = attitude.x;
  float y = attitude.y;
  float z = attitude.z;

  /* 2. Ép vào công thức toán học chuẩn để ra góc (Radian)
  float roll  = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  float pitch = asin(2.0 * (w * y - z * x));
  float yaw   = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

  // 3. Đổi sang độ (Degree) và đẩy ra cổng Serial cho MATLAB
  Serial.print(roll * (180.0 / PI));
  Serial.print(",");
  Serial.print(pitch * (180.0 / PI));
  Serial.print(",");
  Serial.println(yaw * (180.0 / PI)); */
#if WIFI_ENABLED
	processMavlink();
#endif
	logData();
	syncParameters();
}