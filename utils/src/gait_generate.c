#include "gait_generate.h"
#include <math.h>

// ==================== 前进 trot 步态表 ====================
void generateGaitTable20(int gaitTable[20][2], double step_length, double step_height) {
    double l1 = 0.055, l2 = 0.045;
    const int TOTAL_POINTS = 100;
    double x_foot[TOTAL_POINTS], z_foot[TOTAL_POINTS];

    // 上摆腿轨迹
    for(int i = 0; i < 50; ++i) {
        double t = (double)i / 49;
        x_foot[i] = -step_length/2 + step_length * t;
        z_foot[i] = -0.08 + step_height * sin(M_PI * t);
    }
    // 下摆腿轨迹
    for(int i = 0; i < 50; ++i) {
        double t = (double)i / 49;
        x_foot[50+i] = step_length/2 - step_length * t;
        z_foot[50+i] = -0.08;
    }

    // 采样生成 20 个步态点
    for(int k = 0; k < 20; ++k) {
        int idx = k * 99 / 19;
        double x = x_foot[idx], z = z_foot[idx];
        double D = (x*x + z*z - l1*l1 - l2*l2) / (2*l1*l2);
        if(D > 1) D = 1; if(D < -1) D = -1;
        double t2 = atan2(sqrt(1 - D*D), D);
        double t1 = atan2(z, x) - atan2(l2*sin(t2), l1 + l2*cos(t2));
        int thigh = (int)((-t1 * 180 / M_PI - 90) + 0.5);
        int shank = (int)(t2 * 180 / M_PI + 0.5);
        if(shank > 90) shank = 90;
        gaitTable[k][0] = thigh;
        gaitTable[k][1] = shank;
    }
}

// ==================== 后退 trot 步态表 ====================
void generateGaitTable20_reversed(int gaitTable[20][2], double step_length, double step_height) {
    int temp[20][2];
    generateGaitTable20(temp, step_length, step_height);
    for (int i = 0; i < 20; i++) {
        gaitTable[i][0] = temp[19 - i][0];
        gaitTable[i][1] = temp[19 - i][1];
    }
}