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
    float gyroOffsets[AXIS_NB] = {0, 0, 0};
    float accOffsets[AXIS_NB] = {0, 0, 0};
    bool initialized = false;
    bool offsetComputed = false;
    Adafruit_MPU6050 accelgyro; // IMU
    Adafruit_Sensor *mpu_accel;
    Adafruit_Sensor *mpu_gyro;

  private:
    bool ComputeGyroOffsets();
    bool ComputeAccelOffsets();

  public:
    void Init();
    bool AreOffsetComputed(void) {
        return offsetComputed;
    }
    void ComputeOffsets();
    void GetCorrectedAccelGyro(float _accMeasures[], float _gyroMeasures[]);
};

#endif // INERTIALMEASUREMENTUNIT_H_