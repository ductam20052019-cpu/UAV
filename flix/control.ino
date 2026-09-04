#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "lpf.h"
#include "util.h"

// PID parameters (có thể thay đổi từ Qt/parameters)
#define PITCHRATE_P 0.12f
#define PITCHRATE_I 0.2f
#define PITCHRATE_D 0.0008f
#define PITCHRATE_I_LIM 0.3f

// Giới hạn cho iTerm = constrain(i * integral, -windup, windup)
#define ROLLRATE_P PITCHRATE_P
#define ROLLRATE_I PITCHRATE_I
#define ROLLRATE_D PITCHRATE_D
#define ROLLRATE_I_LIM PITCHRATE_I_LIM

#define YAWRATE_P 0.5f
#define YAWRATE_I 0.0
#define YAWRATE_D 0.0
#define YAWRATE_I_LIM 0.3f

#define ROLL_P 3.0f
#define ROLL_I 0
#define ROLL_D 0

#define PITCH_P ROLL_P
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D

#define YAW_P 0.0f

#define PITCHRATE_MAX radians(180)
#define ROLLRATE_MAX radians(180)
#define YAWRATE_MAX radians(90)
#define TILT_MAX radians(30)

#define RATES_D_LPF_ALPHA 0.2

const int RAW = 0, ACRO = 1, STAB = 2, AUTO = 3;

extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;
extern float roll_H, pitch_H, dt;

int mode = STAB;
bool armed = false;

extern float kf_vel;
extern float kf_alt;

Quaternion attitudeTarget;
Vector ratesTarget;
Vector ratesExtra;
Vector torqueTarget;
float thrustTarget;
Vector rate;

PID rollRatePID(ROLLRATE_P, ROLLRATE_I, ROLLRATE_D, ROLLRATE_I_LIM, RATES_D_LPF_ALPHA);
PID pitchRatePID(PITCHRATE_P, PITCHRATE_I, PITCHRATE_D, PITCHRATE_I_LIM, RATES_D_LPF_ALPHA);
PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);

PID rollPID(ROLL_P, ROLL_I, ROLL_D);
PID pitchPID(PITCH_P, PITCH_I, PITCH_D);

#define VEL_KP 0.4f
#define VEL_KI 0.1f
#define VEL_KD 0.0f

#define ALT_KP 0.4f
#define ALT_KI 0.4f
#define ALT_KD 0.4f
PID Alt_zPID(VEL_KP, ALT_KI, ALT_KD);
PID vel_zPID(VEL_KP, VEL_KI, VEL_KD);
float z_hold = 0.0f;
Vector maxRate(ROLLRATE_MAX, PITCHRATE_MAX, YAWRATE_MAX);
float tiltMax = TILT_MAX;

float rollSp_mrad = 0.0f;
float pitchSp_mrad = 0.0f;

float pitchError_rad = 0.0f;
float pitchRateSp = 0.0f;
float pitchRateError = 0.0f;

float HOVER_THROTTLE = 0.4f;

extern bool testingMotor;
extern float motors[4];

float lambda_phi = 1.0f;
float lambda_theta = 1.0f;
float lambda_psi = 0.4f;

float I_p = 12.65625e-8;
float ohm = 1.0f;

float K_phi = 150.0;
float K_theta = 150.0;
float K_psi = 100.0;

float I_xx = 7.09e-4;
float I_yy = 7.09e-4;
float I_zz = 8.23e-4;

float err_vz = 0.0f;
float vz_set = 0.0f;

void control()
{
    interpretControls();
    failsafe();
    controlAttitude();
    controlTorque();
}

void resetAllPIDs()
{
    rollPID.reset();
    pitchPID.reset();
    rollRatePID.reset();
    pitchRatePID.reset();
    yawRatePID.reset();
    vel_zPID.reset();
    Alt_zPID.reset();
}

// Reset only altitude-related controller state (used on DISARM)
void resetAltitudeControl()
{
    z_hold = 0.0f;
    vel_zPID.reset();
    Alt_zPID.reset();
}

#define VZ_MAX      1.0f    // max vertical speed command (m/s)
#define DEADZONE    0.06f   // throttle deadzone around center
   // att_ctrl.h (hoặc ngay trong control.ino)   

// void Altitude()
// {
//     if (!armed) {
//         thrustTarget = controlThrottle;
//         return;
//     }

//     // Map throttle stick 0..1 -> -1..1
//      float stick = (controlThrottle - 0.5f) * 2.0f;
//     // Desired vertical speed from shaped stick
//     vz_set = stick;
//     // Velocity PID on (vz_set - kf_vel)
//     err_vz = vz_set - kf_vel;
//     float correction = vel_zPID.update(err_vz);

//     // Thrust around hover throttle
//     thrustTarget = HOVER_THROTTLE + correction;
//     thrustTarget = constrain(thrustTarget, 0.0f, 1.0f);
// }

void interpretControls()
{
    if (abs(controlYaw) < 0.4)
        controlYaw = 0;
    thrustTarget = controlThrottle;
    rollSp_mrad = controlRoll * tiltMax;
    pitchSp_mrad = controlPitch * tiltMax;

//    Altitude();
}

float saturation(float s)
{
    const float tol = 1.0f;
    if (s > tol) return 1.0f;
    if (s < -tol) return -1.0f;
    return s / tol;
}

void controlAttitude()
{
    static LowPassFilter<Vector> ratesFilter(0.2);
    rate = ratesFilter.update(gyro);

    float rollError_rad = rollSp_mrad - roll_H;
    float rollRateSp = rollPID.update(rollError_rad);
    rollRateSp = constrain(rollRateSp, -ROLLRATE_MAX, ROLLRATE_MAX);
    torqueTarget.x = rollRatePID.update(rollRateSp - rate.x);

    pitchError_rad = pitchSp_mrad - pitch_H;
    pitchRateSp = pitchPID.update(pitchError_rad);
    pitchRateSp = constrain(pitchRateSp, -PITCHRATE_MAX, PITCHRATE_MAX);
    torqueTarget.y = pitchRatePID.update(pitchRateSp - rate.y);

    float yawRateSp = controlYaw * YAWRATE_MAX;
    torqueTarget.z = yawRatePID.update(yawRateSp - rate.z);

			  // float s2_error_dot = rate.x;
    // float s2_error = wrapAngle(roll_H - rollSp_mrad);
    // float s2 = s2_error_dot + lambda_phi * s2_error;
    // torqueTarget.x = -((rate.y * rate.z * (I_yy - I_zz)) - (I_p * ohm * rate.y) + (lambda_phi * I_xx * s2_error_dot) + (I_xx * K_phi * saturation(s2)));

    // // Pitch: s3_error_dot = dtheta (q), s3_error = theta - theta_des
    // float s3_error_dot = rate.y;
    // float s3_error = wrapAngle(pitch_H - pitchSp_mrad);
    // float s3 = s3_error_dot + lambda_theta * s3_error;
    // torqueTarget.y = -((rate.x * rate.z * (I_zz - I_xx)) + (I_p * ohm * rate.x) + (lambda_theta * I_yy * s3_error_dot) + (I_yy * K_theta * saturation(s3)));

    // // Yaw: s4_error_dot = dpsi (r), s4_error = wrap(psi); reference psi_des=0. Không có góc yaw => dùng proxy s4_error từ controlYaw
    // float s4_error_dot = rate.z;
    // float s4_error = wrapAngle(-controlYaw * (float)M_PI);
    // float s4 = s4_error_dot + lambda_psi * s4_error;
    // torqueTarget.z = -((rate.x * rate.y * (I_xx - I_yy)) + (lambda_psi * I_zz * s4_error_dot) + (I_zz * K_psi * saturation(s4))); 
}

void controlTorque()
{
    if (!torqueTarget.valid()) return;

    if (!armed) {
        memset(motors, 0, sizeof(motors));
        return;
    }

    motors[MOTOR_FRONT_LEFT]  = thrustTarget + torqueTarget.x + torqueTarget.y + torqueTarget.z;
    motors[MOTOR_FRONT_RIGHT] = thrustTarget - torqueTarget.x + torqueTarget.y - torqueTarget.z;
    motors[MOTOR_REAR_LEFT]   = thrustTarget + torqueTarget.x - torqueTarget.y - torqueTarget.z;
    motors[MOTOR_REAR_RIGHT]  = thrustTarget - torqueTarget.x - torqueTarget.y + torqueTarget.z;

    // áp min sau cùng
    const float IDLE = 0.06f;
    for (int i = 0; i < 4; i++) {
        motors[i] = constrain(motors[i], IDLE, 1.0f);
    }
}

const char* getModeName()
{
    switch (mode) {
        case RAW:  return "RAW";
        case ACRO: return "ACRO";
        case STAB: return "STAB";
        case AUTO: return "AUTO";
        default:   return "UNKNOWN";
    }
}
