#ifndef HTTPSENSORPROTO_H
#define HTTPSENSORPROTO_H

#include "sensorwert.h"
#include <Arduino.h>


//Send a Wakeup message to the EEC server
// return 0 == OK -1 == Connection error
int sendHTTP_Wakeup();

//Send a encyted & authenticated data package to the EEC server
//-999 One Time Auth Request faild
//-1XXX One Time Auth Request faild
//-5 Memory allocation for Hashing faild
//-6 Memory allocation for Encryption faild
//-101 Packet not accepted from the Server
int sendHTTP_Package(SensorWert * werte, size_t count);

//Send a Value to the Thinkspeak Server
int sendHTTP_Api(float temp, float energie);

#endif //HTTPSENSORPROTO_H
