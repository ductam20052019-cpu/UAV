#include <Wire.h>
#include <FlixPeriph.h>
#include "vector.h"
#include "lpf.h"
#include "util.h"


static int IMU_INT  = 34;

extern float dt;
extern float t;
extern bool armed;
float roll_H = 0.0f;
float pitch_H = 0.0f;
float roll_ = 0.0f;
float pitch_ = 0.0f;

// Vertical acceleration in world frame (Z up), computed each readIMU()
Vector calibratedAcc;
float AccZInertial = 0.0f;
float a_dyn_z = 0.0f;     // dynamic vertical acceleration (up positive)
bool isLanded = true;     // updated each readIMU()
// Complementary filter: alpha = weight for gyro integration, (1-alpha) for acc correction.
// Higher alpha = less acc noise but more gyro drift. Typical 0.96–0.99.
#define COMPLEMENTARY_ALPHA 0.99f  
Vector gyro; // gyroscope data
Vector acc; // accelerometer data, m/s/s
MPU6050 imu(Wire, IMU_INT);
Vector gyroBias;
Vector accBias;
Vector accScale(1, 1, 1);
LowPassFilter<Vector> accFilter(0.1); // Bộ lọc thông thấp cho accelerometer

void setupIMU() {
	print("Setup IMU\n");
	Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
	if (!imu.begin()) {
		print("ERROR: IMU initialization failed!\n");
	} else {
		print("IMU initialized successfully\n");
	}
	configureIMU();
}
void configureIMU() {
	imu.setAccelRange(imu.ACCEL_RANGE_4G);
	imu.setGyroRange(imu.GYRO_RANGE_2000DPS);
	imu.setDLPF(imu.DLPF_MAX);
	imu.setRate(imu.RATE_1KHZ_APPROX);
	imu.setupInterrupt();
}

float approx_atan2_quadrant(float y, float z) {
    const float kEPS = 1e-6f;
    float ay = fabsf(y);
    float az = fabsf(z);
    if (ay < kEPS && az < kEPS) { return 0.0f; }
    float beta;
    if (ay <= az)  beta = M_PI_4 * (ay / az);
    else  beta = M_PI_4 * (2.0f - az / ay);

    float th;
    if (z >= 0.0f) th = (y >= 0.0f) ? beta : -beta;
    else th = (y >= 0.0f) ? (M_PI - beta) : (-(M_PI) + beta);
    return th;
}
void updateAttitude(const Vector& acc) {
	  roll_H += gyro.x*dt;
	  pitch_H += gyro.y*dt;
    // Tilt from accelerometer (noisy, no drift)
    roll_ = atan2(acc.y, acc.z);
    pitch_ = atan2(acc.x, sqrt(acc.y*acc.y + acc.z*acc.z));

    float a = COMPLEMENTARY_ALPHA;
    roll_H = a * roll_H + (1.0f - a) * roll_;
    pitch_H = a * pitch_H + (1.0f - a) * pitch_;
}

void readIMU() {
	imu.waitForData();
	imu.getGyro(gyro.x, gyro.y, gyro.z);
	imu.getAccel(acc.x, acc.y, acc.z);
	calibrateGyroOnce();
	// Hiệu chuẩn accelerometer trước
	acc = (acc - accBias) / accScale;
	// Sau đó mới lọc bằng low-pass filter
	calibratedAcc = accFilter.update(acc);
	gyro = gyro - gyroBias;
	gyro.y = -gyro.y;
	updateAttitude(calibratedAcc);

	// Vertical acceleration in world frame using rotation matrix from roll_H/pitch_H
	AccZInertial =	-sinf(pitch_H) * calibratedAcc.x +
		 cosf(pitch_H) * sinf(roll_H) * calibratedAcc.y +
		 cosf(pitch_H) * cosf(roll_H) * calibratedAcc.z;
	// Detect landed condition: motors off AND accel ≈ 1G (stationary)
	float accNorm = acc.norm();
	isLanded = !motorsActive() && fabsf(accNorm - ONE_G) < ONE_G * 0.1f;

	// Gravity reference and raw dynamic vertical acceleration (for baro/Kalman)
	static float G_ref = ONE_G;
	if (isLanded) {
		const float alphaG = 0.01f;
		G_ref = (1.0f - alphaG) * G_ref + alphaG * AccZInertial;
		a_dyn_z = 0.0f;
	} else {
		a_dyn_z = AccZInertial - G_ref;
	}
}
void calibrateGyroOnce() {
	
	static int sampleCount = 0;
	static Vector gyroSum(0, 0, 0);
	static bool initialCalibrationDone = false;
	static LowPassFilter<Vector> gyroBiasFilter(0.001);
	static Delay notLandedDelay(0.5); // Chỉ reset nếu không landed liên tục 0.5 giây
	const int requiredSamples = 2000; // Thu thập 2000 mẫu để ước tính bias chính xác
	
	// Nếu không landed, đợi một chút trước khi reset (tránh reset do nhiễu tạm thời)
	if (!isLanded) {
		if (notLandedDelay.update(true)) {
			// Không landed liên tục 0.5 giây -> reset calibration
			if (!initialCalibrationDone || sampleCount == 0) {
				sampleCount = 0;
				gyroSum = Vector(0, 0, 0);
			}
		}
		return;
	}
	
	// Reset delay khi landed
	notLandedDelay.update(false);
	
	// Debug: in ra khi bắt đầu calibration lần đầu
	static bool debugPrinted = false;
	if (!initialCalibrationDone && sampleCount == 0 && !debugPrinted) {
		debugPrinted = true;
	}
	if (initialCalibrationDone) debugPrinted = false;
	
	// Thu thập mẫu cho hiệu chuẩn ban đầu
	if (!initialCalibrationDone && sampleCount < requiredSamples) {
		// Tích lũy các mẫu gyro thô
		gyroSum = gyroSum + gyro;
		sampleCount++;
		
		// In debug mỗi 200 mẫu để theo dõi tiến trình
		if (sampleCount % 200 == 0) {
			print("Gyro calibration: %d/%d samples\n", sampleCount, requiredSamples);
		}
		
		// Tính giá trị bias trung bình sau khi thu thập đủ mẫu
		if (sampleCount >= requiredSamples) {
			gyroBias = gyroSum / sampleCount;
			print("Gyro calibration done! Bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
			// Khởi tạo bộ lọc với bias đã tính toán
			gyroBiasFilter.reset();
			gyroBiasFilter.update(gyroBias);
			initialCalibrationDone = true;
			sampleCount = 0;
			gyroSum = Vector(0, 0, 0);
		}
	} else if (initialCalibrationDone) {
		// Hiệu chuẩn liên tục sử dụng trung bình động hàm mũ
		// Sử dụng bộ lọc chậm (alpha = 0.0001) để điều chỉnh độ lệch bias liên tục
		gyroBias = gyroBiasFilter.update(gyro);
	}
}

void calibrateAccel() {
	print("Calibrating accelerometer\n");
	imu.setAccelRange(imu.ACCEL_RANGE_2G); // the most sensitive mode

	print("1/6 Place level [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("2/6 Place nose up [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("3/6 Place nose down [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("4/6 Place on right side [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("5/6 Place on left side [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("6/6 Place upside down [8 sec]\n");
	pause(8);
	calibrateAccelOnce();

	printIMUCalibration();
	print("✓ Calibration done!\n");
	configureIMU();
}

void calibrateAccelOnce() {
	const int samples = 1000;
	static Vector accMax(-INFINITY, -INFINITY, -INFINITY);
	static Vector accMin(INFINITY, INFINITY, INFINITY);

	// Compute the average of the accelerometer readings
	acc = Vector(0, 0, 0);
	for (int i = 0; i < samples; i++) {
		imu.waitForData();
		Vector sample;
		imu.getAccel(sample.x, sample.y, sample.z);
		acc = acc + sample;
	}
	acc = acc / samples;

	// Update the maximum and minimum values
	if (acc.x > accMax.x) accMax.x = acc.x;
	if (acc.y > accMax.y) accMax.y = acc.y;
	if (acc.z > accMax.z) accMax.z = acc.z;
	if (acc.x < accMin.x) accMin.x = acc.x;
	if (acc.y < accMin.y) accMin.y = acc.y;
	if (acc.z < accMin.z) accMin.z = acc.z;
	// Compute scale and bias
	accScale = (accMax - accMin) / 2 / ONE_G;
	accBias = (accMax + accMin) / 2;
}

void printIMUCalibration() {
	print("gyro bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
	print("accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
	print("accel scale: %f %f %f\n", accScale.x, accScale.y, accScale.z);
}

void printIMUInfo() {
	imu.status() ? print("status: ERROR %d\n", imu.status()) : print("status: OK\n");
	print("model: %s\n", imu.getModel());
	print("who am I: 0x%02X\n", imu.whoAmI());
	print("rate: %.0f\n", loopRate);
	print("gyro: %f %f %f\n", rates.x, rates.y, rates.z);
	print("acc: %f %f %f\n", acc.x, acc.y, acc.z);
	print("roll_H: %f, pitch_H: %f\n", roll_H, pitch_H);
	print("roll: %f, pitch: %f\n", attitude.getRoll(), attitude.getPitch());

	imu.waitForData();
	Vector rawGyro, rawAcc;
	imu.getGyro(rawGyro.x, rawGyro.y, rawGyro.z);
	imu.getAccel(rawAcc.x, rawAcc.y, rawAcc.z);
	print("raw gyro: %f %f %f\n", rawGyro.x, rawGyro.y, rawGyro.z);
	print("raw acc: %f %f %f\n", rawAcc.x, rawAcc.y, rawAcc.z);
}
