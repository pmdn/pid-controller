/**
 * \file            test_pid_controller.c
 * \brief           Unit tests for PID controller library
 *
 * Author:          pmdn <pmdn@mailbox.org>
 * Version:         0.1
 */

#include "unity.h"
#include "pid_controller.h"
#include "unity_internals.h"
#include <stdint.h>

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

void test_pid_update_execution(void)
{
    pid_controller_t my_test_pid;
    float kp = 1.0f;
    float ki = 1.0f;
    float kd = 1.0f;
    float limit_high = 1.0f;
    float limit_low = 1.0f;
    float time_delta = 200e-6;
    int32_t filter_n = 100;
    float ki_escaled_expected = ki * 0.5f * time_delta;
    float kd1_escaled_expected = 1.0f / (1.0f + (float) filter_n * time_delta);
    float kd2_escaled_expected = kd * ((float) filter_n / (1.0f + (float) filter_n * time_delta));
    
    pid_update(&my_test_pid, kp, ki, kd, limit_high, limit_low, time_delta, filter_n);
    TEST_ASSERT_EQUAL_FLOAT(kp, my_test_pid.kp);
    TEST_ASSERT_EQUAL_FLOAT(ki, my_test_pid.ki);
    TEST_ASSERT_EQUAL_FLOAT(kd, my_test_pid.kd);
    TEST_ASSERT_EQUAL_FLOAT(limit_high, my_test_pid.limit_max);
    TEST_ASSERT_EQUAL_FLOAT(limit_low, my_test_pid.limit_min);
    TEST_ASSERT_EQUAL_FLOAT(time_delta, my_test_pid.time_diff);
    TEST_ASSERT_EQUAL_FLOAT(ki_escaled_expected, my_test_pid.ki_escaled);
    TEST_ASSERT_EQUAL_FLOAT(kd1_escaled_expected, my_test_pid.kd1_escaled);
    TEST_ASSERT_EQUAL_FLOAT(kd2_escaled_expected, my_test_pid.kd2_escaled);
}

void test_pid_reset_execution(void)
{
    pid_controller_t my_test_pid;
    float kp = 1.0f;
    float ki = 1.0f;
    float kd = 1.0f;
    float limit_high = 1.0f;
    float limit_low = 1.0f;
    float time_delta = 200e-6;
    int nFilter = 100;
    pid_update(&my_test_pid, kp, ki, kd, limit_high, limit_low, time_delta, nFilter);
    my_test_pid.error_previous = 1.0f;
    my_test_pid.integral_term = 1.0f;
    my_test_pid.derivative_term = 1.0f;
    my_test_pid.out = 1.0f;

    pid_reset(&my_test_pid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, my_test_pid.error_previous);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, my_test_pid.integral_term);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, my_test_pid.derivative_term);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, my_test_pid.out);
}

void test_pid_limit_value_is_higher(void)
{
    float input = 300.0f;
    float limit_high = 100.0f;
    float limit_low = 0.0f;
    float expected = 100.0f;
    
    float actual = pid_limit(input, limit_high, limit_low);
    TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

void test_pid_limit_value_is_lower(void)
{
    float input = -300.0f;
    float limit_high = 100.0f;
    float limit_low = 0.0f;
    float expected = 0.0f;
    
    float actual = pid_limit(input, limit_high, limit_low);
    TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

void test_pid_limit_value_is_between(void)
{
    float input = 50.0f;
    float limit_high = 100.0f;
    float limit_low = 0.0f;
    float expected = 50.0f;
    float actual = pid_limit(input, limit_high, limit_low);

    TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

void test_pid_calculate_only_Kp_reference_one_measurement_zero(void)
{
    pid_controller_t my_test_pid;
    float kp = 1.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float limit_high = 1.0f;
    float limit_low = 1.0f;
    float time_delta = 200e-6;
    int32_t filter_n = 100;
    pid_update(&my_test_pid, kp, ki, kd, limit_high, limit_low, time_delta, filter_n);
    my_test_pid.error_previous = 1.0f;
    my_test_pid.integral_term = 1.0f;
    float reference = 1.0f;
    float measurement = 0.0f;
    float feedforward = 0.0f;
    float expected = (reference - measurement) * kp;
    
    pid_calculate(reference, measurement, feedforward, &my_test_pid);
    TEST_ASSERT_EQUAL_FLOAT(expected, my_test_pid.out);
}

// tests are called here
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_pid_update_execution);
    RUN_TEST(test_pid_reset_execution);
    RUN_TEST(test_pid_limit_value_is_higher);
    RUN_TEST(test_pid_limit_value_is_lower);
    RUN_TEST(test_pid_limit_value_is_between);
    RUN_TEST(test_pid_calculate_only_Kp_reference_one_measurement_zero);
    return UNITY_END();
}
