#include "state.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

state state_init(void) {
    return (state){
        .imu1_results = {0, 0, 0},
        .imu2_results = {0, 0, 0},
        .imu_diff_results = {0, 0, 0},

        .imu1_mag = 0.f,
        .imu2_mag = 0.f,

        .current_accel_orthogonal = 0.f,
        .current_vel = 0.f,
        .current_vel_tof = 0.f,
        .vel_tof_ind = 0,
        .vel_calibrate = 0,
        .calibrated = false,

        .tof_error = 0,
        .tof_distance = 0,
        .estimated_distance = 0.f,
        .estimated_delta = 0.f,
    };
}

void state_update_accel(state *state) {
    float diff_squared = 0;
    bool is_neg = false;
    for (int i = 0; i < 3; i++) {
        state->imu_diff_results[i] =
            state->imu1_results[i] - state->imu2_results[i];
        diff_squared += state->imu_diff_results[i] * state->imu_diff_results[i];
        if (diff_squared < 0)
            is_neg = true;
    }

    float accel = sqrt(diff_squared);
    if (is_neg) {
        state->current_accel_orthogonal = -accel;
    } else {
        state->current_accel_orthogonal = accel;
    }
    state->current_vel += state->current_accel_orthogonal * DELTA_TIME;

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
    // state->current_vel += state->current_accel_orthogonal * DELTA_TIME;
}

void state_predict_next(state *state) {
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

void state_update_vel(state *state, uint16_t distance, uint16_t error) {
    // Calculate speed from ToF
    state->past_vel_tof[state->vel_tof_ind++] =
        // state->current_vel_tof =
        ((abs(state->tof_distance - distance) / TOF_DELTA_TIME) -
         state->vel_calibrate) < 0
            ? 0
            : ((abs(state->tof_distance - distance) / TOF_DELTA_TIME) -
               state->vel_calibrate);

    float vel_sum = 0;
    for (int i = 0; i < MAX_TOF_VEL_EST; ++i) {
        vel_sum += state->past_vel_tof[i];
    }
    state->current_vel_tof = vel_sum / MAX_TOF_VEL_EST;
    printf("ToF Vel: %f", state->current_vel_tof);

    if (state->calibrated == false && state->vel_tof_ind >= MAX_TOF_VEL_EST) {
        state->vel_calibrate = state->current_vel_tof + 200;
        state->calibrated = true;
    }

    printf(", vel_sum: %f", vel_sum);

    if (state->vel_tof_ind >= MAX_TOF_VEL_EST) {
        state->vel_tof_ind = 0;
    }

    state->tof_distance = distance;
    state->tof_error = error;
    state->estimated_distance = distance;
    state->current_vel = 0.f;
    printf("\r\n");
}
