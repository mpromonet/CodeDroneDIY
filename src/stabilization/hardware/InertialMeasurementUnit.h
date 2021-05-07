#ifndef INERTIALMEASUREMENTUNIT_H_
#define INERTIALMEASUREMENTUNIT_H_

#include "Wire.h"
#include "Adafruit_MPU6050.h"
#include "../../customLibs/CustomMath.h"
#include "../../customLibs/CustomSerialPrint.h"

class InertialMeasurementUnit{
  private:
    static const int AXIS_NB = 3;
    static const int SAMPLES_NB = 10;
    float AcceleroSensitivity = -1;
    float GyroSensitivity = -1;
    int16_t gyroOffsets[AXIS_NB] = {0, 0, 0};
    int16_t accOffsets[AXIS_NB] = {0, 0, 0};
    bool initialized = false;
    bool offsetComputed = false;
    Adafruit_MPU6050 accelgyro; // IMU
    Adafruit_Sensor *mpu_accel;
    Adafruit_Sensor *mpu_gyro;

  private:
    bool ComputeGyroOffsets();
    bool ComputeAccelOffsets();
    void SetAccRange(mpu6050_accel_range_t _range);
    void SetGyroRange(mpu6050_gyro_range_t _range);

  public:
    void Init();
    bool AreOffsetComputed(void) {
        return offsetComputed;
    }
    void ComputeOffsets();
    void GetCorrectedAccelGyro(float _accMeasures[], float _gyroMeasures[]);
};

#endif // INERTIALMEASUREMENTUNIT_H_