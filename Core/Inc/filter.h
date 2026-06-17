#pragma once

#include "GyroData.h"

void ApplyMadgwickFilter(struct MPU6050Data* data);
void PopulateRealValues(struct MPU6050Data* data);

#include <math.h>

// Global filter parameters
#define beta        0.1f    // Filter gain (higher = trusts accelerometer more, lower = smoother/gyro trust)
#define sampleFreq  200.0f  // Frequency of your execution loop in Hz (e.g., 200 Hz = 5ms interval)

void MadgwickAHRSupdateIMU(struct MPU6050Data* data);
void MadgwickGetEuler(struct MPU6050Data* data);