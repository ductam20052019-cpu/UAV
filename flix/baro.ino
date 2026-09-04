#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "kalman_z.h"
 
extern float dt;
extern float t;
extern bool armed;
float kf_vel = 0.0f;
float kf_alt = 0.0f;
// Barometer variables
Adafruit_BMP280 bmp;
float pressure0 = 0.0f;      // Reference pressure at startup
float altitude = 0.0f;        // Raw altitude from barometer (m)

const float BARO_ALPHA = 0.1f; // Low-pass filter coefficient (0.05–0.2)
float altitudeCalibration = 0.0f; // Calibration offset from startup
extern bool isLanded;
extern float a_dyn_z;    // vertical dynamic acceleration (up positive), used by Kalman
 
// Baro: read at lower rate so I2C doesn't slow the 1 kHz loop (30 Hz is enough for altitude)
#define BARO_READ_RATE_HZ  30.0f
static Rate baroRate(BARO_READ_RATE_HZ);
static bool baroReadThisFrame = false;  // Track if baro was read this frame

// Smoother altitude: average several pressure reads per tick + rotating buffer (from reference Altitude.h)
#define PRESSURE_SAMPLES_PER_READ  3   // average 3 reads to reduce single-sample noise
#define PRESSURE_BUFFER_SIZE       10  // rotating buffer for smoothed pressure (fewer spikes)
static float pressureBuffer[PRESSURE_BUFFER_SIZE];
static uint8_t pressureBufferIdx = 0;
static bool pressureBufferFilled = false;

void setupBarometer() {
	print("Setup Barometer (BMP280)\n");
	
	if (!bmp.begin(0x76)) {  // try 0x77 if 0x76 doesn't work
		print("ERROR: BMP280 not found! Trying 0x77...\n");
		if (!bmp.begin(0x77)) {
			print("ERROR: BMP280 initialization failed!\n");
			return;
		}
	}
	// Configure oversampling & filter in sensor for better pressure reading
	bmp.setSampling(
		Adafruit_BMP280::MODE_NORMAL,
		Adafruit_BMP280::SAMPLING_X2,   // temperature oversampling
		Adafruit_BMP280::SAMPLING_X16,  // pressure oversampling (important!)
		Adafruit_BMP280::FILTER_X16,
		Adafruit_BMP280::STANDBY_MS_1
	);
	
	delay(100);
	// Set reference pressure at startup
	pressure0 = bmp.readPressure();
	print("Barometer initialized, reference pressure: %.0f Pa\n", pressure0);
	// Calibrate altitude offset by averaging 500 samples
	print("Calibrating barometer altitude offset (500 samples)...\n");
	float altitudeSum = 0.0f;
	for (int i = 0; i < 500; i++) {
		float pressure = bmp.readPressure();
		float alt = 44330.0f * (1.0f - powf(pressure / pressure0, 0.1903f));
		altitudeSum += alt;
		delay(1);
		if (i % 100 == 0) {
			print("  %d/500 samples\n", i);
		}
	}
	altitudeCalibration = altitudeSum / 500.0f;
	print("Barometer calibration complete. Offset: %.2f m\n", altitudeCalibration);
}
static KalmanAltitude kf;      // persistent Kalman state
static bool prevArmed = false;

void update_altitude() {
	baroReadThisFrame = false;
	if (baroRate) {
		readBarometer();
		baroReadThisFrame = true;
	}
	if (armed && !prevArmed) {
		kf.reset();
	}
	prevArmed = armed;
	kf.predict(a_dyn_z, dt);
	// Correction only when we actually have a baro sample
	if (baroReadThisFrame) {
		kf.update(altitude);
	}
	kf_alt = kf.z;
	kf_vel = kf.v;
	if (kf_alt<0.0f) kf_alt = 0.0f;
	if (kf_vel<0.0f) kf_vel = 0.0f;
}

void readBarometer() {
	if (pressure0 == 0) return; // Not initialized yet
	// 1) Average several pressure reads to reduce single-sample noise (like samplePressureReadings)
	float pSum = 0.0f;
	for (int i = 0; i < PRESSURE_SAMPLES_PER_READ; i++) {
		pSum += bmp.readPressure();
	}
	float pressureAvg = pSum / (float)PRESSURE_SAMPLES_PER_READ;
	// 2) Rotating buffer for smoothed pressure (like smoothPressureReadings)
	pressureBuffer[pressureBufferIdx] = pressureAvg;
	pressureBufferIdx = (pressureBufferIdx + 1) % PRESSURE_BUFFER_SIZE;
	if (pressureBufferIdx == 0) pressureBufferFilled = true;
	float pMean = 0.0f;
	int n = pressureBufferFilled ? PRESSURE_BUFFER_SIZE : (int)pressureBufferIdx;
	if (n == 0) n = 1;
	for (int i = 0; i < n; i++) pMean += pressureBuffer[i];
	pMean /= (float)n;
	// 3) Convert smoothed pressure to altitude (barometric formula, 0.1903 ≈ 1/5.255)
	altitude = 44330.0f * (1.0f - powf(pMean / pressure0, 0.1903f));
	altitude -= altitudeCalibration;
}

