#ifndef UNIT_TEST
#include <avr/wdt.h>
#include <math.h>

#include "customLibs/CustomTime.h"
#include "stabilization/hardware/RadioReception.h"
#include "stabilization/Stabilization.h"
#include "stateMachine/StateMachine.h"

CustomTime time;
Stabilization stabilization;
StateMachine stateMachine;
void PrintConfig();

// Initialiaze all sensors and communication pipes
void setup() {
    wdt_disable();

    CustomSerialPrint::begin(230400); // Console print: initialize serial communication
    delay(1000);
    CustomSerialPrint::println( F("!!!!!!!!!!!!!!!!!!!! BOOT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "));
    stabilization.Init();
    time.Init();
    stateMachine.Init();

    for (int i = 0; i < 5; i++) {
        pinMode(13, OUTPUT);
        digitalWrite(13, LOW);
        delay(250);

        digitalWrite(13, HIGH);    
        delay(250);
    }

    PrintConfig();

    wdt_enable(WDTO_8S); // Set watchdog reset
}

// Main loop
void loop() {
    float loopTimeSec = 0.0;
    uint16_t loopNb = 0;
    float meanLoopTime = 0.0;

    loopTimeSec = time.GetloopTimeMilliseconds();

    // State Machine Initializing -> Ready -> AngleMode/AccroMode -> Safety -> Disarmed -> AngleMode/AccroMode
    stateMachine.Run(loopTimeSec);

    // Compute mean loop time and complementary filter time constant
    int flyingMode = stabilization.GetFlyingMode();
    if ((flyingMode == angleMode) || (flyingMode == accroMode)) {
        if (!stabilization.IsThrottleIdle()) {
            time.ComputeMeanLoopTime(loopTimeSec, meanLoopTime, loopNb);
        }
    }
    wdt_reset();
}

void PrintConfig() {
    if ((stabilization.GetMotorsMaxPower() == 1860)
        && (stabilization.GetMotorsMaxThrottle() >= (1860 * 0.8)))
        CustomSerialPrint::println(
                F("!!!!!!!!!!!!!!!!!!!!FLYING MODE "
                  "POWER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "));
    else if ((stabilization.GetMotorsMaxPower() <= 1300))
        CustomSerialPrint::println(F("DEBUG MODE POWER!!! "));
    else
        CustomSerialPrint::println(F("UNEXPECTED POWER "));

    CustomSerialPrint::print(F("MAX_POWER: "));
    CustomSerialPrint::print(stabilization.GetMotorsMaxPower());
    CustomSerialPrint::print(F(" MAX_THROTTLE_PERCENT: "));
    // CustomSerialPrint::println(stabilization.GetMotorsMaxThrottlePercent());
    CustomSerialPrint::println(F("Setup Finished"));
}
#endif