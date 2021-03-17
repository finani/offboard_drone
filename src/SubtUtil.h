
#ifndef SubtUtil_H
#define SubtUtil_H

#define R2D	      180.0/3.141592653
#define D2R	      3.141592653/180.0
#define eps           0.00000001
#define PI            3.1415926535

#define Kpx           0.5
#define Kpz           0.7
#define Kdz           0.14
#define Kr            1.0

#define Kpx_rel       1.0 // 0.5
#define Kpz_rel       1.0 // 0.7
#define Kdz_rel       0.0 // 0.14
#define Kr_rel        1.0

// for tracking mission
#define VX_MAX        0.6
#define VZ_MAX        0.5
#define R_MAX         1.5

// for additional control
#define PWM_ROL       1514
#define PWM_PIT       1514
#define PWM_THR       1514
#define PWM_YAW       1514
#define PWM_LEN       420.0

#define VELX_MAX      2.0
#define VELZ_MAX      1.0
#define VELR_MAX      2.0

float   q[4];

static void QuaterniontoEuler(float& roll, float& pitch, float& yaw)
{

    // roll (x-axis rotation)
    float t0 = +2.0 * (q[3] * q[0] + q[1] * q[2]);
    float t1 = +1.0 - 2.0 * (q[0] * q[0] + q[1]*q[1]);
    roll = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    float t2 = +2.0 * (q[3] * q[1] - q[2] * q[0]);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = -std::asin(t2);

    // yaw (z-axis rotation)
    float t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
    float t4 = +1.0 - 2.0 * (q[1]*q[1] + q[2] * q[2]);
    yaw = std::atan2(t3, t4);
}

float satmax(float data, float max)
{
    float res;

    if(fabs(data) > max)
        res = (data + eps)/fabs(data + eps)*max;
    else
        res = data;

    return res;
}

float wrap(float data)
{
    float res;
    data = fmod(data+180.0*D2R, 360.0*D2R);
    if(data < 0.0)
        data = data + 360.0*D2R;

    res = data-180.0*D2R;
    return res;
}

float GetNED_angle_err(float cmd, float cur)
{
    float res;

    res = wrap(cmd - cur);

    return res;
}

float LPF(float data, float data_pre, float freq)
{
    float res;
    float K_LPF;
    float delT = 0.05;

    K_LPF = freq*2*PI*delT / (1 + freq*2*PI*delT);

    res = (data - data_pre) * K_LPF + data_pre;

    return res;
}

#endif
