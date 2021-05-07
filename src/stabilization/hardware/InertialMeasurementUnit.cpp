#include <avr/wdt.h>
#include "InertialMeasurementUnit.h"
void InertialMeasurementUnit::Init() {
    // MPU6050: join I2C bus
    Wire.begin();
    Wire.setClock(400000L); // Communication with MPU-6050 at 400KHz

    // Initialize MPU 6050
    if (!accelgyro.begin()) {
        CustomSerialPrint::println(F("InertialMeasurementUnit: Function testConnection failed"));
    } else {
        SetGyroRange(MPU6050_RANGE_1000_DEG);
        SetAccRange(MPU6050_RANGE_8_G);

        mpu_accel = accelgyro.getAccelerometerSensor();
        mpu_gyro = accelgyro.getGyroSensor();
    }
}

void InertialMeasurementUnit::GetCorrectedAccelGyro(float _accMeasures[], float _gyroMeasures[]) {
    int16_t accel[AXIS_NB] = {0, 0, 0};
    int16_t gyro[AXIS_NB] = {0, 0, 0};

    sensors_event_t accelevt;
    sensors_event_t gyroevt;
    sensors_event_t tempevt;
    accelgyro.getEvent(&accelevt, &gyroevt, &tempevt);

    accel[0] = accelevt.acceleration.x;
    accel[1] = accelevt.acceleration.y;
    accel[2] = accelevt.acceleration.z;

    gyro[0] = gyroevt.gyro.x;
    gyro[1] = gyroevt.gyro.y;
    gyro[2] = gyroevt.gyro.z;

    // Correct raw data with offset
    for (int axis = 0; axis < AXIS_NB; axis++) {
        _accMeasures[axis] =
                static_cast<float>((accel[axis] - accOffsets[axis]) / AcceleroSensitivity);
        _gyroMeasures[axis] =
                static_cast<float>((gyro[axis] - gyroOffsets[axis]) / GyroSensitivity);
    }
}

void InertialMeasurementUnit::SetAccRange(mpu6050_accel_range_t _range) {
    switch (_range) {
    case MPU6050_RANGE_2_G:
        AcceleroSensitivity = 16384;
        break;
    case MPU6050_RANGE_4_G:
        AcceleroSensitivity = 8192;
        break;
    case MPU6050_RANGE_8_G:
        AcceleroSensitivity = 4096;
        break;
    case MPU6050_RANGE_16_G:
        AcceleroSensitivity = 2048;
        break;
    }
    accelgyro.setAccelerometerRange(_range);
}

void InertialMeasurementUnit::SetGyroRange(mpu6050_gyro_range_t _range) {
    switch (_range) {
    case MPU6050_RANGE_250_DEG:
        GyroSensitivity = 131;
        break;
    case MPU6050_RANGE_500_DEG:
        GyroSensitivity = 65.5;
        break;
    case MPU6050_RANGE_1000_DEG:
        GyroSensitivity = 32.8;
        break;
    case MPU6050_RANGE_2000_DEG:
        GyroSensitivity = 16.4;
        break;
    }
    accelgyro.setGyroRange(_range);
}

// Compute accelerometer and gyroscope offsets
void InertialMeasurementUnit::ComputeOffsets() {
    if (ComputeGyroOffsets() && ComputeAccelOffsets())
        offsetComputed = true;
    else
        offsetComputed = false;
}

bool InertialMeasurementUnit::ComputeGyroOffsets() {
    int16_t gyroRaw[AXIS_NB][SAMPLES_NB];
    float mean = 0;

    for (int axis = 0; axis < AXIS_NB; axis++)
        for (int sample = 0; sample < 10; sample++)
            gyroRaw[axis][sample] = 0;

    // Get 10 samples during 2 sec
    for (int sample = 0; sample < 10; sample++) {
        sensors_event_t gyro;
        mpu_gyro->getEvent(&gyro);
        gyroRaw[0][sample] = gyro.gyro.x;
        gyroRaw[1][sample] = gyro.gyro.y;
        gyroRaw[2][sample] = gyro.gyro.z;
        CustomSerialPrint::print(gyroRaw[0][sample]);
        CustomSerialPrint::print("\t");
        CustomSerialPrint::print(gyroRaw[1][sample]);
        CustomSerialPrint::print("\t");
        CustomSerialPrint::println(gyroRaw[2][sample]);
        delay(200);
    }

    // Compute mean
    for (int axis = 0; axis < AXIS_NB; axis++) {
        if (!CustomMath::ComputeMean(gyroRaw[axis], SAMPLES_NB, (10 * GyroSensitivity), &mean)) {
            CustomSerialPrint::println(F("ERROR DURING SPEED OFFSETS COMPUTATION !!"));
            return false;
        }
        gyroOffsets[axis] = static_cast<int16_t>(mean);
    }

    CustomSerialPrint::print(F("Gyroscope offsets Computed :"));
    for (int axis = 0; axis < AXIS_NB; axis++) {
        CustomSerialPrint::print(gyroOffsets[axis] / GyroSensitivity);
        CustomSerialPrint::print(" ");
    }
    CustomSerialPrint::println("(deg.s-1) ");
    return true;
}

bool InertialMeasurementUnit::ComputeAccelOffsets() {
    int16_t accRaw[AXIS_NB][SAMPLES_NB];
    float mean = 0.0;

    for (int axis = 0; axis < AXIS_NB; axis++)
        for (int sample = 0; sample < 10; sample++)
            accRaw[axis][sample] = 0;

    // Get 10 samples during 2 sec
    for (int sample = 0; sample < 10; sample++) {
        sensors_event_t accel;
        mpu_accel->getEvent(&accel);
        accRaw[0][sample] = accel.acceleration.x;
        accRaw[1][sample] = accel.acceleration.y;
        accRaw[2][sample] = accel.acceleration.z;        
        CustomSerialPrint::print(accRaw[0][sample]);
        CustomSerialPrint::print("\t");
        CustomSerialPrint::print(accRaw[1][sample]);
        CustomSerialPrint::print("\t");
        CustomSerialPrint::println(accRaw[2][sample]);
        delay(200);
    }

    // Mean computation
    for (int axis = 0; axis < AXIS_NB; axis++) {
        if (!CustomMath::ComputeMean(accRaw[axis], SAMPLES_NB, (0.2 * AcceleroSensitivity),
                                     &mean)) {
            CustomSerialPrint::println(F("ERROR DURING ACCELERATION OFFSETS COMPUTATION !!"));
            return false;
        }
        accOffsets[axis] = static_cast<int16_t>(mean);
    }

    // Zacc is gravity, it should be 1G ie 4096 LSB/g at -+8g sensitivity
    accOffsets[2] = accOffsets[2] - AcceleroSensitivity;

    CustomSerialPrint::print(F("Acceleration offsets Computed :"));
    for (int axis = 0; axis < AXIS_NB; axis++) {
        CustomSerialPrint::print(accOffsets[axis] / AcceleroSensitivity);
        CustomSerialPrint::print(" ");
    }
    CustomSerialPrint::print("(m.s-2) ");
    return true;
}