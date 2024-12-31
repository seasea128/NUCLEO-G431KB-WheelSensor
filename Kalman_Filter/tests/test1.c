#include "unity_internals.h"
#include <string.h>
#include <unity.h>

#include <kalman_state.h>

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_kalman_set_accel() {
    printf("\ntest_kalman_set_accel\n");
    kalman_state state = kalman_state_init();
    float new_imu_val[3] = {1., 1., 1.};
    memcpy(state.imu1_results, new_imu_val, sizeof(new_imu_val));
    kalman_update_accel(&state);

    printf("Current imu1 magnitude: %f, Expected: %f\n", state.imu1_mag, 3.);
    TEST_ASSERT(state.imu1_mag == 3.);
}

void test_kalman_update_estimated_delta() {
    printf("\ntest_kalman_update_estimated_delta\n");
    kalman_state state = kalman_state_init();
    float new_imu_val[3] = {1000000., 1000000., 1000000.};
    memcpy(state.imu2_results, new_imu_val, sizeof(new_imu_val));
    kalman_update_accel(&state);
    printf("Current imu1 accel: %f, imu2 accel: %f\n", state.imu1_mag_sqrt,
           state.imu2_mag_sqrt);
    printf("Current vel: %f\n", state.current_vel);
    kalman_predict_next(&state);

    printf("Current prediction: %f, Expected: %f\n", state.estimated_delta,
           state.current_vel * DELTA_TIME);
    TEST_ASSERT(state.estimated_delta == state.current_vel * DELTA_TIME);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_kalman_set_accel);
    RUN_TEST(test_kalman_update_estimated_delta);

    return UNITY_END();
}
