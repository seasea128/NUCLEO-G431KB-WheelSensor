#ifndef STATE_H__
#define STATE_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define DELTA_TIME (1 / 416.f)
#define TOF_DELTA_TIME (1 / 100.f)
#define MAX_TOF_VEL_EST 100

typedef struct state_s {
    float past_vel_tof[MAX_TOF_VEL_EST];
    float imu1_results[3];
    float imu2_results[3];
    float imu_diff_results[3];

    float imu1_mag;
    float imu2_mag;

    float imu1_mag_sqrt;
    float imu2_mag_sqrt;

    float current_accel_orthogonal;
    float current_vel;
    float current_vel_tof;
    float vel_calibrate;
    float estimated_distance;
    float estimated_delta;

    uint16_t tof_distance;
    uint16_t tof_error;

    size_t vel_tof_ind;
    bool calibrated;
} state;

state state_init(void);

void state_update_accel(state *state);

void state_predict_next(state *state);

void state_update_vel(state *state, uint16_t velocity, uint16_t error);

#endif
