
#include "InertialMeasurementUnit.h"
void InertialMeasurementUnit::Init() {
    // MPU6050: join I2C bus
    Wire.begin();
    Wire.setClock(400000L); // Communication with MPU-6050 at 400KHz

    // Initialize MPU 6050
    if (!accelgyro.begin()) {
        CustomSerialPrint::println(F("InertialMeasurementUnit: Function testConnection failed"));
    } else {
        accelgyro.setAccelerometerRange(MPU6050_RANGE_8_G);
        accelgyro.setGyroRange(MPU6050_RANGE_1000_DEG);
        accelgyro.setFilterBandwidth(MPU6050_BAND_21_HZ);

        mpu_accel = accelgyro.getAccelerometerSensor();
        mpu_gyro = accelgyro.getGyroSensor();
    }
}

void InertialMeasurementUnit::GetCorrectedAccelGyro(float _accMeasures[], float _gyroMeasures[]) {
    float accel[AXIS_NB] = {0, 0, 0};
    float gyro[AXIS_NB] = {0, 0, 0};

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
        _accMeasures[axis] = (accel[axis] - accOffsets[axis]);
        _gyroMeasures[axis] = (gyro[axis] - gyroOffsets[axis]);
    }
}

// Compute accelerometer and gyroscope offsets
void InertialMeasurementUnit::ComputeOffsets() {
    if (ComputeGyroOffsets() && ComputeAccelOffsets())
        offsetComputed = true;
    else
        offsetComputed = false;
}

bool InertialMeasurementUnit::ComputeGyroOffsets() {
    float gyroRaw[AXIS_NB][SAMPLES_NB];

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
    float gyroDeltas[AXIS_NB] = {0, 0, 0};  
    for (int axis = 0; axis < AXIS_NB; axis++) {
        if (!CustomMath::ComputeMean(gyroRaw[axis], SAMPLES_NB, 0.2 , &gyroOffsets[axis], &gyroDeltas[axis])) {
            CustomSerialPrint::println(F("ERROR DURING SPEED OFFSETS COMPUTATION !!"));
            return false;
        }     
    }

    CustomSerialPrint::print(F("Gyroscope offsets Computed :"));
    for (int axis = 0; axis < AXIS_NB; axis++) {
        CustomSerialPrint::print(gyroOffsets[axis]);
        CustomSerialPrint::print("(");
        CustomSerialPrint::print(gyroDeltas[axis]);
        CustomSerialPrint::print(") ");
    }
    CustomSerialPrint::println("(rad.s-1) ");
    return true;
}

bool InertialMeasurementUnit::ComputeAccelOffsets() {
    float accRaw[AXIS_NB][SAMPLES_NB];

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
    float accDeltas[AXIS_NB] = {0, 0, 0};
    for (int axis = 0; axis < AXIS_NB; axis++) {
        if (!CustomMath::ComputeMean(accRaw[axis], SAMPLES_NB, 0.2, &accOffsets[axis], &accDeltas[axis])) {
            CustomSerialPrint::println(F("ERROR DURING ACCELERATION OFFSETS COMPUTATION !!"));
            return false;
        }    
    }

    CustomSerialPrint::print(F("Acceleration offsets Computed :"));
    for (int axis = 0; axis < AXIS_NB; axis++) {
        CustomSerialPrint::print(accOffsets[axis]);
        CustomSerialPrint::print("(");
        CustomSerialPrint::print(accDeltas[axis]);
        CustomSerialPrint::print(") ");
    }
    CustomSerialPrint::print("(m.s-2) ");
    return true;
}