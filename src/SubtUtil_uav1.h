
#ifndef SubtUtil_UAV1_H
#define SubtUtil_UAV1_H

#define R2D	      180.0/3.141592653
#define D2R	      3.141592653/180.0
#define eps           0.00000001
#define PI            3.1415926535

//#define  CAMERA_PARAM_U0   453.186
//#define  CAMERA_PARAM_a    216.507935

//#define  CAMERA_PARAM_V0   377.357
//#define  CAMERA_PARAM_b    200.712952

#define  CAMERA_PARAM_U0   480.0
#define  CAMERA_PARAM_a    215.0

#define  CAMERA_PARAM_V0   360.357
#define  CAMERA_PARAM_b    215.0

#define  a0               -375.4402138
#define  a1                0.0
#define  a2                0.001130137
#define  a3               -1.1578e-6
#define  a4                2.3398e-9

#define  refangle          -90*D2R
#define  alt_offset        0.2        // [m]
#define  Vx_track          2.5

#define Kpx           2.0
#define Kpz           1.5
#define Kdz           0.3
#define Kr            1.0

#define VX_MAX        0.6
#define VZ_MAX        0.5
#define R_MAX         1.5

#define VEL_MIN       0.2
#define VEL_TRACKING  0.3
#define VELZ          0.6

#define takeoff_alt   0.6
#define hovering_alt  0.6
#define takeoff_time  1.0

#define flag_highlevel 1

float   q[4];

float   u_img = 0.0;
float   v_img = 0.0;
float   psi_img = 0.0;
float   theta_img = 0.0;

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

float satmin(float data, float min)
{
    float res;

    if(fabs(data) < min)
        res = (data + eps)/fabs(data + eps)*min;
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

struct Trajectory
{
    float x;
    float y;
    float psi;
    float wp_x;
    float wp_y;
}path;

struct struct_Tar_data
{
    float   pos[3];
    float   posfil[2];
    float   vel[3];
    float   impos[2];
    float   imposfil[2];
    float   size[2];
    float   sizefil[2];
    float   dist;

    float   psi_ref;
    float   a_n;
    float   del_sigma;
    float   beta;

    uint8_t flag_detect;
}tar_data;

#endif

