#include "filter.h"
#include "FreeRTOSConfig.h"
#include "GyroData.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"

#define SWAP_ENDIAN(x) ((int16_t)(((uint16_t)x & 0xff00) >> 8) | ((uint16_t)x & 0xff))

uint32_t lastTick;
float deltaT;

void PopulateRealValues(struct MPU6050Data *data) {
    data->accel_x_raw = SWAP_ENDIAN(data->accel_x_raw);
    data->accel_y_raw = SWAP_ENDIAN(data->accel_y_raw);
    data->accel_z_raw = SWAP_ENDIAN(data->accel_z_raw);
    data->gyro_x_raw = SWAP_ENDIAN(data->gyro_x_raw);
    data->gyro_y_raw = SWAP_ENDIAN(data->gyro_y_raw);
    data->gyro_z_raw = SWAP_ENDIAN(data->gyro_z_raw);
    data->temp_raw = SWAP_ENDIAN(data->temp_raw);

    // In degress / s
    data->gyro_x = ((float)data->gyro_x_raw / 131.0f) * M_PI / 180.f;
    data->gyro_y = ((float)data->gyro_y_raw / 131.0f) * M_PI / 180.f;
    data->gyro_z = ((float)data->gyro_z_raw / 131.0f) * M_PI / 180.f;

    // In celsius
    data->temp = ((float)data->temp_raw/340.0f) + 36.53f;

    // In G forces
    data->accel_x = (float)data->accel_x_raw / 16384.0f;
    data->accel_y = (float)data->accel_y_raw / 16384.0f;
    data->accel_z = (float)data->accel_z_raw / 16384.0f;
}

void ApplyMadgwickFilter(struct MPU6050Data* data) {
    MadgwickAHRSupdateIMU(data);
    MadgwickGetEuler(data);
}

// Initialize quaternion variables representing home orientation
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// 6-DoF Madgwick implementation optimized for IMUs without a magnetometer
void MadgwickAHRSupdateIMU(struct MPU6050Data* data) {
    float recipNorm;
    float s0, s1, s2, s3;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    float gx = data->gyro_x;
    float gy = data->gyro_y;
    float gz = data->gyro_z;

    float ax = data->accel_x;
    float ay = data->accel_y;
    float az = data->accel_z;

    deltaT = (float)(HAL_GetTick() - lastTick) / configTICK_RATE_HZ;

    // Rate of change of quaternion from gyroscope
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        // Normalise step magnitude
        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change to convert to quaternion
    // q0 += qDot1 * (1.0f / sampleFreq);
    // q1 += qDot2 * (1.0f / sampleFreq);
    // q2 += qDot3 * (1.0f / sampleFreq);
    // q3 += qDot4 * (1.0f / sampleFreq);

    q0 += qDot1 * deltaT;
    q1 += qDot2 * deltaT;
    q2 += qDot3 * deltaT;
    q3 += qDot4 * deltaT;

    // Normalise quaternion
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    lastTick = HAL_GetTick();
}

// Convert output Quaternions to standard Euler Angles (Degrees)
void MadgwickGetEuler(struct MPU6050Data* data) {
    data->roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * (180.0f / M_PI);
    data->pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * (180.0f / M_PI);
    data->yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * (180.0f / M_PI);
}