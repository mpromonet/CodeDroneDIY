#ifndef CUSTOMMATH_H_
#define CUSTOMMATH_H_

#include <avr/wdt.h>
#include <limits.h>
#include <math.h>

#include "CustomSerialPrint.h"

#define RAD2DEG(angle) angle * 180 / PI

class CustomMath {
  public:
    static bool ComputeDelta(float _list[], int _size, float *_delta);
    static bool ComputeMean(float _list[], int _size, float _deltaThreshold, float *_mean);
    static void VectorNormalize(float _vectorIn[], const int vectorSize);
};

#endif // CUSTOMMATH_H_
