#include "unity.h"
#include "pid_controller.h"
#include "unity_internals.h"

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

void test_updatePID_execution(void)
{
    pidController myTestPID;
    float kp = 1.0f;
    float ki = 1.0f;
    float kd = 1.0f;
    float highLimit = 1.0f;
    float lowLimit = 1.0f;
    float deltaTime = 200e-6;
    float escaledKiExpected = ki * 0.5f * deltaTime;
    float escaledKdExpected = kd / deltaTime;
    
    updatePID(&myTestPID, kp, ki, kd, highLimit, lowLimit, deltaTime);
    TEST_ASSERT_EQUAL_FLOAT(kp, myTestPID.Kp);
    TEST_ASSERT_EQUAL_FLOAT(ki, myTestPID.Ki);
    TEST_ASSERT_EQUAL_FLOAT(kd, myTestPID.Kd);
    TEST_ASSERT_EQUAL_FLOAT(highLimit, myTestPID.limMax);
    TEST_ASSERT_EQUAL_FLOAT(lowLimit, myTestPID.limMin);
    TEST_ASSERT_EQUAL_FLOAT(deltaTime, myTestPID.deltaT);
    TEST_ASSERT_EQUAL_FLOAT(escaledKiExpected, myTestPID.escaledKi);
    TEST_ASSERT_EQUAL_FLOAT(escaledKdExpected, myTestPID.escaledKd);
}

void test_initPID_execution(void)
{
    pidController myTestPID;
    float kp = 1.0f;
    float ki = 1.0f;
    float kd = 1.0f;
    float highLimit = 1.0f;
    float lowLimit = 1.0f;
    float deltaTime = 200e-6;
    updatePID(&myTestPID, kp, ki, kd, highLimit, lowLimit, deltaTime);
    myTestPID.prevError = 1.0f;
    myTestPID.intTerm = 1.0f;
    myTestPID.out = 1.0f;

    resetPID(&myTestPID);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, myTestPID.prevError);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, myTestPID.intTerm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, myTestPID.out);
}

void test_limitValue_value_is_higher(void)
{
    float inputValue = 300.0f;
    float highLimit = 100.0f;
    float lowLimit = 0.0f;
    float expected = 100.0f;
    
    float actual = limitValue(inputValue, highLimit, lowLimit);
    TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

void test_limitValue_value_is_lower(void)
{
    float inputValue = -300.0f;
    float highLimit = 100.0f;
    float lowLimit = 0.0f;
    float expected = 0.0f;
    
    float actual = limitValue(inputValue, highLimit, lowLimit);
    TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

void test_limitValue_value_is_between(void)
{
    float inputValue = 50.0f;
    float highLimit = 100.0f;
    float lowLimit = 0.0f;
    float expected = 50.0f;
    float actual = limitValue(inputValue, highLimit, lowLimit);

    TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

void test_calculatePID_only_Kp_reference_one_measurement_zero(void)
{
    pidController myTestPID;
    float kp = 1.0f;
    float ki = 1.0f;
    float kd = 1.0f;
    float highLimit = 1.0f;
    float lowLimit = 1.0f;
    float deltaTime = 200e-6;
    updatePID(&myTestPID, kp, ki, kd, highLimit, lowLimit, deltaTime);
    myTestPID.prevError = 1.0f;
    myTestPID.intTerm = 1.0f;
    float reference = 1.0f;
    float measurement = 0.0f;
    float feedforward = 0.0f;
    float expected = (reference - measurement) * myTestPID.Kp;
    
    calculatePID(reference, measurement, feedforward, &myTestPID);
    TEST_ASSERT_EQUAL_FLOAT(expected, myTestPID.out);
}

// tests are called here
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_updatePID_execution);
    RUN_TEST(test_initPID_execution);
    RUN_TEST(test_limitValue_value_is_higher);
    RUN_TEST(test_limitValue_value_is_lower);
    RUN_TEST(test_limitValue_value_is_between);
    RUN_TEST(test_calculatePID_only_Kp_reference_one_measurement_zero);
    return UNITY_END();
}
