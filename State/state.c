#include "state.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

state state_init(void) {
    state new_state;
    memset(&new_state, 0, sizeof(state));
    return new_state;
    // return (state){
    //     .imu1_results = {0, 0, 0},
    //     .imu2_results = {0, 0, 0},
    //     .imu_diff_results = {0, 0, 0},

    //    .imu1_mag = 0.f,
    //    .imu2_mag = 0.f,

    //    .current_accel_orthogonal = 0.f,
    //    .current_vel_imu = 0.f,
    //    .current_vel_imu_avg = 0.f,

    //    .imu_vel_ind = 0,
    //    .tof_error = 0,
    //    .tof_distance = 0,
    //    .estimated_distance = 0.f,
    //    .estimated_delta = 0.f,
    //};
}

void state_update_accel(state *state) {
    // float diff_squared = 0;
    // for (int i = 0; i < 3; i++) {
    //     state->imu_diff_results[i] =
    //         state->imu1_results[i] - state->imu2_results[i];
    //     diff_squared += state->imu_diff_results[i] *
    //     state->imu_diff_results[i];
    // }

    // float accel = sqrt(diff_squared);
    // if (is_neg) {

    //     state->current_accel_orthogonal = -accel;

    // } else {

    //     state->current_accel_orthogonal = accel;
    // }
    float squared_accel = 0;
    for (int i = 0; i < 3; i++) {
        squared_accel += state->imu1_results[i] * state->imu1_results[i];
    }

    // Unit of accel here is mG, convert to mms^-2 by multiplying with 9.81
    state->current_accel_orthogonal = (sqrt(squared_accel) - 1000) * 9.81;

    if (state->current_accel_orthogonal <= 500 &&
        state->current_accel_orthogonal >= -500) {
        state->current_vel_imu = 0;
        state->current_vel_imu_avg = 0;
    } else {
        state->current_vel_imu +=
            state->current_accel_orthogonal * IMU_DELTA_TIME;
    }

    state->vel_past_results[state->imu_vel_ind++] = state->current_vel_imu;
    if (state->imu_vel_ind >= MAX_IMU_AVG_COUNT) {
        state->imu_vel_ind = 0;
    }

    float avg_speed = 0;
    for (int i = 0; i < MAX_IMU_AVG_COUNT; ++i) {
        avg_speed += state->vel_past_results[i];
    }

    avg_speed /= MAX_IMU_AVG_COUNT;

    state->current_vel_imu_avg += avg_speed;
    // printf("Current IMU vel: %f, %f\r\n", state->current_vel_imu,
    //        state->current_vel_imu_avg);

    // state->imu1_mag = 0;
    // state->imu2_mag = 0;
    // for (int i = 0; i < 3; i++) {
    //     state->imu1_mag += state->imu1_results[i] * state->imu1_results[i];
    //     state->imu2_mag += state->imu2_results[i] * state->imu2_results[i];
    // }
    // state->imu1_mag_sqrt = sqrt(state->imu1_mag);
    // state->imu2_mag_sqrt = sqrt(state->imu2_mag);

    // state->current_accel_orthogonal =
    //     state->imu2_mag_sqrt - state->imu1_mag_sqrt;
    // state->current_vel += state->current_accel_orthogonal *
    // IMU_IMU_DELTA_TIME;
}

void state_predict_next(state *state) {
    if (state->current_vel_imu > 10) {
        state->estimated_delta =
            state->estimated_delta + IMU_DELTA_TIME * state->current_vel_imu;
        state->estimated_distance =
            state->tof_distance + state->estimated_delta;
    } else {
        state->estimated_distance = state->tof_distance;
        state->estimated_delta = 0;
    }
}

void state_update_displacement(state *state, uint16_t displacement,
                               uint16_t error) {
    state->tof_distance = displacement;
    state->tof_error = error;
    state->estimated_distance = displacement;
    // state->current_vel_imu = 0.f;
    // state->current_vel_imu_avg = 0.f;
}
