#include "kalman_state.h"
#include <math.h>

#define DELTA_TIME 1 / 416.f

kalman_state kalman_state_init(void) {
    return (kalman_state){
        .tof_error = 0,
        .tof_distance = 0,
        .current_accel_orthogonal = 0.f,
        .current_vel = 0.f,
        .estimated_distance = 0.f,
        .imu1_results = {0, 0, 0},
        .imu2_results = {0, 0, 0},
        .imu_diff_results = {0, 0, 0},
    };
}

void kalman_update_accel(kalman_state *state) {
    // float diff_squared = 0;
    // for (int i = 0; i < 3; i++) {
    //     state->imu_diff_results[i] =
    //         state->imu1_results[i] - state->imu2_results[i];
    //     diff_squared += state->imu_diff_results[i] *
    //     state->imu_diff_results[i];
    // }

    state->imu1_mag = 0;
    state->imu2_mag = 0;
    for (int i = 0; i < 3; i++) {
        state->imu1_mag += state->imu1_results[i] * state->imu1_results[i];
        state->imu2_mag += state->imu2_results[i] * state->imu2_results[i];
    }
    state->imu1_mag_sqrt = sqrt(state->imu1_mag);
    state->imu2_mag_sqrt = sqrt(state->imu2_mag);

    state->current_accel_orthogonal =
        state->imu2_mag_sqrt - state->imu1_mag_sqrt - 1000;
    state->current_vel += state->current_accel_orthogonal * DELTA_TIME;
}

void kalman_predict_next(kalman_state *state) {
    if (state->current_vel > 10)
        state->estimated_distance =
            state->tof_distance + DELTA_TIME * state->current_vel;
    else
        state->estimated_distance = state->tof_distance;
}
