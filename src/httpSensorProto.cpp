#include "httpSensorProto.h"

#include <Arduino.h>
#include <ESP8266WiFi.h>

#define DEBUG_HTTPCLIENT(x)  Serial.println(x);

#include <ESP8266HTTPClient.h>
#include <Crypto.h>
#include <AES.h>
#include <CTR.h>
#include <BLAKE2s.h>

#define SEN_NUM 0
#define IDENT_SIZE 15
#define NUM_SIZE 1
#define IV_SIZE 16
#define HASH_SIZE 16


//Thingspeak connection
const char* server = "api.thingspeak.com";
String apiKey = "1234567890"; //4


//EEC Server Connection
const char* ip = "123.123.123.123";
const uint16_t port = 8000;

const byte key[32] PROGMEM  = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
	                             0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};


//Encryption Variables
byte indent[IDENT_SIZE];
byte sum[HASH_SIZE];
byte iv[IV_SIZE];
CTR<AES256> aes;


//send values to server via HTTP
int sendHTTP_Package(SensorWert * werte, size_t count) {

	// dont send more than 50 values due to overloading
	// recursive sending in multible parts
	if(count > 50) {
				size_t half1 = count /2;
				size_t half2 = count - half1;
				int ret1 = sendHTTP_Package(werte,half1);
				if(ret1 < 0) {
					return ret1;
				}


				int ret2 = sendHTTP_Package(werte+half1,half2);
				if(ret2 < 0) {
					return ret2;
				}

				return ret1 + ret2;
	}



	HTTPClient http;

	//Authentication
	//Request a one Use Authentication from the EEC Server
  bool ret = http.begin(ip,port, "/key");
	if (!ret) {
		return -999;
	}


  int httpCode = http.GET(); //get the one use Authentication

  String payload;

  if (httpCode != 200) { //Check the returning code
      http.end();
      return (httpCode*(-1))-1000;
  }

  payload = http.getString(); // The Returned payload ist a One Use Authentication
  http.end();

  for(int i = 0; i < IDENT_SIZE; i++) { //Copy and print KEY
      indent[i] = payload[i];
      Serial.print((int)indent[i],HEX);
  }
  Serial.println("");



  size_t data_size = count * sizeof(SensorWert);
  size_t payload_size = IDENT_SIZE + NUM_SIZE + data_size;
  size_t plain_size = payload_size +  HASH_SIZE;
  size_t package_size = IV_SIZE + plain_size;

	//Dynamicly allocate memory ! can go Wrong
  //PREPARE MEM######################
  // 15 INDENT, 1 SEN_NUM, n*16 SensorWert, 16 Prüfhash
  byte * plain = (byte*) malloc( plain_size );
  byte * cipher = (byte*) malloc( plain_size );


  if(plain == NULL || cipher == NULL) {
    return -5;
  }



  //time for IV ########################
  unsigned long times = micros();
	unsigned long timem = millis() * micros();
  memcpy(iv, &times, sizeof(unsigned long) );
  memcpy(iv+8, &timem, sizeof(unsigned long) );
  //Sensor Number#########################
  byte sensornum = SEN_NUM;

  //COPY INTO PLAINTEXT BUFFER
  memcpy(plain,indent, IDENT_SIZE);
  memcpy(plain+IDENT_SIZE,&sensornum, NUM_SIZE);
  memcpy(plain+IDENT_SIZE+NUM_SIZE,werte, data_size );

  //CALCULATE HASH
  BLAKE2s blake;
  blake.reset(key,16,HASH_SIZE);
  blake.update(plain, payload_size);
  blake.finalize(sum, HASH_SIZE);

  //COPY HASH INTO PLAINTEXT BUFFER
  memcpy(plain+payload_size,sum, HASH_SIZE);
  //HMAC Authentication with one use Auth


  //encrypt!!
  crypto_feed_watchdog();
  aes.setKey(key, 32);
  aes.setIV(iv, IV_SIZE);
  aes.encrypt(cipher, plain, plain_size );

  //Not longer used
  free(plain);
  plain = NULL;
  //16( IV,  15 INDENT, 1 SEN_NUM, n*16 SensorWert, 16 Prüfhash
  byte * output = (byte*) malloc ( package_size);
  if(output == NULL) {
    return -6;
  }

  //WRTIE to OUTPUT
  memcpy(output, iv, IV_SIZE);
  memcpy(output+IV_SIZE, cipher, plain_size);
  // Not longer Used
  free(cipher);
  cipher = NULL;


  // Upload the encyted data to the server
  //PUT CONNECTION
  http.begin(ip, port, "/put");
  httpCode = http.PUT( (uint8_t*) (output), package_size);
	free(output);
  if (httpCode > 0) { //Check the returning code

  payload = http.getString();   //Get the request response payload
  Serial.println(payload);                     //Print the response payload

	if(!(payload == String("OK"))) {
		http.end();   //Close connectiona
		return -101;
	}

	}

  Serial.println(httpCode);
  http.end();   //Close connectiona

  return httpCode;
}


//Wakeup massage send if the Sensor is Rebooted
int sendHTTP_Wakeup() {
HTTPClient http;
http.begin(ip,port,"/wake");
for(int i = 0; i < 5; i++) {

  byte sensornum = SEN_NUM;
  int httpCode = http.PUT( (uint8_t*) &sensornum, 1);
  if (httpCode == 200) {
		http.end();
    return 0;
  }
}
http.end();
return -1;
}


//Thinkspeak API send values;
int sendHTTP_Api(float temp, float energie) {
	WiFiClient client;

	if (client.connect(server,80)) {
		String postStr = apiKey;
		postStr +="&field1=";
		postStr += String(temp);
		postStr +="&field2=";
		postStr += String(energie);
		postStr += "\r\n\r\n";

		//HTTP protocoll
		client.print("POST /update HTTP/1.1\n");
		client.print("Host: api.thingspeak.com\n");
		client.print("Connection: close\n");
		client.print("X-THINGSPEAKAPIKEY: "+apiKey+"\n");
		client.print("Content-Type: application/x-www-form-urlencoded\n");
		client.print("Content-Length: ");
		client.print(postStr.length());
		client.print("\n\n");
		client.print(postStr);

	}

client.stop();

return 0;
}
