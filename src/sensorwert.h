#ifndef SENSORWERT_H
#define SENSORWERT_H
#include <Arduino.h>


//Structe for storing a SensorWert Datapoint
struct SensorWert {
        float temperatur;
        float energie;
        uint64_t time;
};


#endif // SENSORWERT_H
