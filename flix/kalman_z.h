#include <iostream>
#include <cmath>

class KalmanAltitude
{
public:

    float z;   // altitude
    float v;   // velocity

    float P00, P01, P10, P11;

    float Qz = 0.005f;
    float Qv = 0.005f;

    float R  = 0.1f;

    KalmanAltitude()
    {
        reset();
    }

    // Reset filter state (used on arm / startup)
    void reset()
    {
        z = 0.0f;
        v = 0.0f;

        P00 = 1.0f;
        P01 = 0.0f;
        P10 = 0.0f;
        P11 = 1.0f;
    }

    void predict(float a, float dt)
    {
        float z_pred = z + v * dt + 0.5f * a * dt * dt;
        float v_pred = v + a * dt;

        float F00 = 1;
        float F01 = dt;
        float F10 = 0;
        float F11 = 1;

        float P00_new = F00*P00 + F01*P10;
        float P01_new = F00*P01 + F01*P11;
        float P10_new = F10*P00 + F11*P10;
        float P11_new = F10*P01 + F11*P11;

        P00 = P00_new*F00 + P01_new*F01 + Qz;
        P01 = P00_new*F10 + P01_new*F11;
        P10 = P10_new*F00 + P11_new*F01;
        P11 = P10_new*F10 + P11_new*F11 + Qv;

        z = z_pred;
        v = v_pred;
    }

    void update(float z_meas)
    {
        float y = z_meas - z;

        float S = P00 + R;

        float K0 = P00 / S;
        float K1 = P10 / S;

        z = z + K0 * y;
        v = v + K1 * y;

        float P00_new = (1 - K0) * P00;
        float P01_new = (1 - K0) * P01;
        float P10_new = -K1 * P00 + P10;
        float P11_new = -K1 * P01 + P11;

        P00 = P00_new;
        P01 = P01_new;
        P10 = P10_new;
        P11 = P11_new;
    }
};