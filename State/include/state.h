#ifndef KALMAN_STATE
#define KALMAN_STATE

#include <stddef.h>
#include <stdint.h>
#define IMU_DELTA_TIME (1 / 833.f)
#define MAX_IMU_AVG_COUNT 200

typedef struct state_s {
    float vel_past_results[MAX_IMU_AVG_COUNT];
    float imu1_results[3];
    float imu2_results[3];
    float imu_diff_results[3];

    float imu1_mag;
    float imu2_mag;

    float imu1_mag_sqrt;
    float imu2_mag_sqrt;

    float current_accel_orthogonal;
    float current_vel_imu;
    float current_vel_imu_avg;

    size_t imu_vel_ind;
    uint16_t tof_distance;
    uint16_t tof_error;
    float estimated_distance;
    float estimated_delta;
} state;

state state_init(void);

void state_update_accel(state *state);

void state_predict_next(state *state);

void state_update_displacement(state *state, uint16_t velocity, uint16_t error);

#endif
