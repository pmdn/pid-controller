#include "unity.h"
#include "pid_controller.h"

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

void test_initPID_execution(void)
{
    //test stuff
    pidController myTestPID = {
        .Kp = 1.0f,
        .Ki = 1.0f,
        .Kd = 1.0f,
        .limMax = 1.0f,
        .limMin = -1.0f,
        .prevError = 1.0f,
        .intTerm = 1.0f
    };
    initPID(&myTestPID);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, myTestPID.prevError);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, myTestPID.intTerm);
}

void test_calculatePID_only_Kp_reference_one_measurement_zero(void)
{
    //test stuff
    pidController myTestPID = {
        .Kp = 1.0f,
        .Ki = 1.0f,
        .Kd = 1.0f,
        .limMax = 1.0f,
        .limMin = -1.0f,
        .prevError = 1.0f,
        .intTerm = 1.0f
    };
    calculatePID(1.0f, 0.0f, &myTestPID);
    float expected = (1.0f - 0.0f) * myTestPID.Kp;
    TEST_ASSERT_EQUAL_FLOAT(expected, myTestPID.out);
}

void test_limitValue_value_is_higher(void)
{
    float expected = 100.0f;
    float actual = limitValue(300.0f, 100.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

void test_limitValue_value_is_lower(void)
{
    float expected = 0.0f;
    float actual = limitValue(-300.0f, 100.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

void test_limitValue_value_is_between(void)
{
    float expected = 50.0f;
    float actual = limitValue(50.0f, 100.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

// tests are called here
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initPID_execution);
    RUN_TEST(test_calculatePID_only_Kp_reference_one_measurement_zero);
    RUN_TEST(test_limitValue_value_is_higher);
    RUN_TEST(test_limitValue_value_is_lower);
    RUN_TEST(test_limitValue_value_is_between);
    return UNITY_END();
}
