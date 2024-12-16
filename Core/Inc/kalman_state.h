#ifndef KALMAN_STATE
#define KALMAN_STATE

#include <stdint.h>

typedef struct kalman_state_s {
    float imu1_results[3];
    float imu2_results[3];
    float imu_diff_results[3];

    float imu1_mag;
    float imu2_mag;

    float imu1_mag_sqrt;
    float imu2_mag_sqrt;

    float current_accel_orthogonal;
    float current_vel;

    uint16_t tof_distance;
    uint16_t tof_error;
    float estimated_distance;
} kalman_state;

kalman_state kalman_state_init(void);

void kalman_update_accel(kalman_state *state);

void kalman_predict_next(kalman_state *state);

#endif
