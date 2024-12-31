#include "kalman_state.h"
#include <math.h>

kalman_state kalman_state_init(void) {
    return (kalman_state){
        .imu1_results = {0, 0, 0},
        .imu2_results = {0, 0, 0},
        .imu_diff_results = {0, 0, 0},

        .imu1_mag = 0.f,
        .imu2_mag = 0.f,

        .current_accel_orthogonal = 0.f,
        .current_vel = 0.f,

        .tof_error = 0,
        .tof_distance = 0,
        .estimated_distance = 0.f,
        .estimated_delta = 0.f,
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
        state->imu2_mag_sqrt - state->imu1_mag_sqrt;
    state->current_vel += state->current_accel_orthogonal * DELTA_TIME;
}

void kalman_predict_next(kalman_state *state) {
    if (state->current_vel > 10) {
        state->estimated_delta =
            state->estimated_delta + DELTA_TIME * state->current_vel;
        state->estimated_distance =
            state->tof_distance + state->estimated_delta;
    } else {
        state->estimated_distance = state->tof_distance;
        state->estimated_delta = 0;
    }
}
