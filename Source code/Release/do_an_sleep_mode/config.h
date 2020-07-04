#include <Arduino.h>
#include <ArduinoJson.h>
#include <CRC32.h>
#include "Update.h"
#include "FS.h"
#include <PubSubClient.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

const String version = "1.1.0";

#define DEBUG   1
//#define DEBUG_CAMERA    1   

#define MQTT    1   

#define SerialAT Serial

#define SHT_READ_CYCLE  5000   //milli seconds
#define SDS_READ_CYCLE  5000   //milli seconds
#define GPS_READ_CYCLE  10   //milli seconds
#define TASK_HANDLER    1000   //milli seconds
#define POST_DATA_HANDLER    1000   //milli seconds


//#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
//#define TIME_TO_SLEEP  120        /* Time ESP32 will go to sleep (in seconds) */


#define LED_BUILD_IN 33


// Specify data and clock connections and instantiate SHT1x object
#define SHT_SDA 2
#define SHT_SCL 14


#define SDS_TX 16
#define SDS_RX 17


#define GPS_RXPin 13
#define GPS_TXPin 15
static const uint32_t GPSBaud = 9600;


// Set serial for DEBUG_GPS console (to the Serial Monitor)
#ifdef DEBUG
SoftwareSerial SerialMon(GPS_TXPin, GPS_RXPin, false, 256);
#endif // DEBUG


// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[] = "m-wap";
const char user[] = "mms";
const char pass[] = "mms";

// Server details
const char* serverName = "diedd212.000webhostapp.com";
const int  port = 80;
const char* resource = "/post-data.php";
const char *post_image_url = "/post-image.php"; // Location where images are POSTED
String apiKeyValue = "diendd212";
#define FILE_PHOTO "/image.txt"


//MQTT
String mqtt_clientID = "DDD_01";
const char mqtt_topic_pub[] = "diendd212/data/DDD_01";
const char mqtt_topic_sub[] = "diendd212/cmd/DDD_01";
const char mqtt_server[] = "broker.hivemq.com";
long mqtt_port = 1883;


uint8_t msg[500];


#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


/*
command:

/firmware/version/

/firmware/update/diedd212.000webhostapp.com/do_an_sleep_mode.bin

*/