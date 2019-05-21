#include <Arduino.h>
#include <ESP8266WiFi.h>

#define DEBUG_HTTPCLIENT(x)  Serial.println(x);

#include <ESP8266HTTPClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <SPI.h>
#include <Wire.h>
#include <BME280I2C.h>

#include "sensorwert.h"
#include "httpSensorProto.h"

#define SEALEVELPRESSURE_HPA (1013.25)


//WLAN CONFIG
const char* ssid = "SSID"; // zum Testen im Makerspace, muss für den Betrieb zu Hause geändert werden
const char* password = "123456";


WiFiUDP ntpUDP = WiFiUDP();
NTPClient timeClient = NTPClient(ntpUDP);


//SETUP BME SENSOR
BME280I2C::Settings settings(
        BME280::OSR_X1,
        BME280::OSR_X1,
        BME280::OSR_X1,
        BME280::Mode_Forced,
        BME280::StandbyTime_1000ms,
        BME280::Filter_Off,
        BME280::SpiEnable_False,
        BME280I2C::I2CAddr_0x76 // I2C address. I2C specific.
        );

BME280I2C bme(settings);

//### BUFFER MEM ####
SensorWert speicher[1000]; //Buffer memory for sensor values
                            //is maxed to 1000;
int currentIdx; //Currrent Index in the Buffer
SensorWert * next; //Pointer to the next value  in the Buffer

// Variables for calculating used Energey
volatile bool energie_on;
volatile float currentEnergie;
uint64_t lastE_millis;


bool blink;

//Calculate used Energey
float calculateEnergie(uint64_t currentTime,uint64_t lastTime) {
  int delta = (currentTime - lastTime);

  if( lastTime > currentTime ) {
    return 0; //Overflow abfangen
  }

  //Watt * Hour
  return ((((delta) * 0.1) / 60) / 60);

}

//The Light is tured on
void stgInterrupt() {
  if(energie_on) return;
  Serial.println("ON");
  lastE_millis = millis();
  energie_on = true;
}

//The Light is tured off
void falInterrupt() {
  if(!energie_on) return;
  uint64_t currentT = millis();

  //Watt * Seconds
  currentEnergie += calculateEnergie(currentT ,lastE_millis);
  Serial.println("OFF");
  lastE_millis = currentT;
  energie_on = false;
}

//Blink at some pin
void blinki(int pin,int delay_t, int whd) {

  for(int i = 0; i < whd; i++) {
      digitalWrite(pin,HIGH);
      delay(delay_t);
      digitalWrite(pin,LOW);
      delay(delay_t);
  }

}

void setup() {
        //#########   DEBUG Connection
        Serial.begin(9600);

        //#########  Setup Wifi
        WiFi.begin(ssid, password);

        // Wait for connection AND IP address from DHCP
        Serial.println();
        Serial.println("Waiting for connection and IP Address from DHCP");
        while (WiFi.status() != WL_CONNECTED) {
                blinki(2,500,5);
                Serial.print(".");
        }
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("MAC: ");
        Serial.println(WiFi.macAddress());
        //WiFi Connection READY

        //######### Setup NTP getting the curretn time
        timeClient.begin();
        timeClient.setTimeOffset(3600);
        timeClient.update();
        timeClient.setUpdateInterval(600000);


        //######## Setup PINS
        pinMode(2,OUTPUT); //Blinki output information LED
        pinMode(12, INPUT); //LIGHT Sensor Interrupt pin
        pinMode(13, INPUT); //LIGHT Sensor Interrupt pin
        pinMode(16,OUTPUT);//Blinki output information LED
        attachInterrupt(digitalPinToInterrupt(12),stgInterrupt, RISING);
        attachInterrupt(digitalPinToInterrupt(13),falInterrupt, FALLING);
        //######## Send Wakeup massage to EEC Server
        sendHTTP_Wakeup();

        // Init Memory pointer
        currentIdx = 0;
        next = speicher;

        //Init ernergie counter
        lastE_millis =0;
        currentEnergie = 0;

        blink = false;

        //########### Setup I2C BME Connection
        Wire.begin();
        while(!bme.begin())
        {
                Serial.println("Could not find BME280I2C sensor!");
                blinki(16,100,10); // ERROR MSG
                blinki(2,100,10);
        }

        switch(bme.chipModel())
        {
        case BME280::ChipModel_BME280:
                Serial.println("Found BME280 sensor! Success.");
                break;
        case BME280::ChipModel_BMP280:
                Serial.println("Found BMP280 sensor! No Humidity available.");
                break;
        default:
                Serial.println("Found UNKNOWN sensor! Error!");
        }

        // Change some settings before using.
        settings.tempOSR = BME280::OSR_X4;

        bme.setSettings(settings);

}

bool reconnect;

void loop() {
        //Blinky
        unsigned long runtime = millis();

        //Connecting to WIFI faild
        while (WiFi.status() != WL_CONNECTED) {
                blinki(2,500,5); //Connecting ino
                Serial.print(WiFi.status());
        }

        //Blink While running on
        if(millis() % 2000 == 0 && blink==false) {
                digitalWrite(2,HIGH);
                blink = true;
        }
        //Blik While running off
        if(millis() % 2000 == 1000 && blink==true) {
                digitalWrite(2,LOW);
                blink = false;
        }

        timeClient.update();

        blinki(16,100,3);

        digitalWrite(16,HIGH);
        Serial.print("RUN---SENSOR---");
        Serial.println(timeClient.getFormattedTime());


//Sensorwerte Lesen!
//Energie Berechen wenn an;
//Speichen und counter erhöhen


float temp(NAN), hum(NAN), pres(NAN);

BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_Pa);

bme.read(pres, temp, hum, tempUnit, presUnit);

Serial.print("Temp: ");
Serial.print(temp);
Serial.print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
Serial.print("\t\tHumidity: ");
Serial.print(hum);
Serial.print("% RH");
Serial.print("\t\tPressure: ");
Serial.print(pres);
Serial.print("Pa----");


// Energie verbrauch erfassen
int energie = digitalRead(12);
Serial.print(energie);
Serial.print("-energie-");
currentEnergie += calculateEnergie(millis() ,lastE_millis) * energie;
lastE_millis = millis();
Serial.println(currentEnergie);

//Werte In Buffer Speichen

next->temperatur = temp;
next->energie = currentEnergie;
currentEnergie = 0.0;
next->time = timeClient.getEpochTime();
next++;
currentIdx++;



Serial.println(currentIdx);

// Sobald 10 Werte im buffer sind Hochladen
if (currentIdx >= 10) {
  Serial.println("SEND DATA");
      int ret = sendHTTP_Package(speicher,currentIdx );

      if (ret > 0) {
          Serial.println("SUCCSESS");
        currentIdx = 0;
        next = speicher;
        blinki(16,500,2);
      } else {
        Serial.print("FAIL-");
        Serial.println(ret);
        blinki(16,50,10);
      }
}

   digitalWrite(16,LOW);
}
