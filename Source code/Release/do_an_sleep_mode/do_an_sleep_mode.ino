#include "config.h"


//uint64_t WDT_TIMEOUT = 120;  //time in ss to trigger the watchdog    3p
//hw_timer_t *timer = NULL;
//
//void IRAM_ATTR resetModule() {
//#ifdef DEBUG
//
//    ets_printf("reboot\n");
//    Serial.println("-------------------------------------");
//    Serial.println("------->Watch dog reboot<------------");
//    Serial.println("-------------------------------------");
//#endif // DEBUG
//    esp_restart();
//}



/////////////////////////////////////////
#include "esp_camera.h"



#include <SHT1x.h>
SHT1x sht1x(SHT_SDA, SHT_SCL);

float G_temp_c = -1;
float G_temp_f = -1;
float G_humidity = -1;
bool locked = false;

void sht_read();


#include <SDS011.h>
SDS011 my_sds;

float G_p10, G_p25;
int err;

void sds_read();
void sds_init();



// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial GPS_Serial(GPS_TXPin, GPS_RXPin, false, 2048);
double G_gps_lat = -1;
double G_gps_lng = -1;

void gps_init();
int gps_pareInfor();
void gps_read();




#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <ArduinoJson.h>

#include "SPIFFS.h"
#define FORMAT_SPIFFS_IF_FAILED true

void writeFile(fs::FS &fs, const char * path, const char * message);

void deleteFile(fs::FS &fs, const char * path);

void appendFile(fs::FS &fs, const char * path, const char * message);




// Select your modem:
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>

// Increase RX buffer if needed
#define TINY_GSM_RX_BUFFER 1024
#define TINY_GSM_USE_GPRS true


//#define DUMP_AT_COMMANDS

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
TinyGsmClient client(modem, 1);



#ifdef MQTT
TinyGsmClient client_2(modem, 2);
PubSubClient mqtt(client_2);
#endif // MQTT


enum Commands {
    UNKNOW,
    MQTT_READ_FIRMWARE,
    MQTT_UPDATE_FIRMWARE,
    HTTP_POST_DATA,
    TAKE_SEND_PICTURE
};


enum ErrorCode {
    Success,
    SHT_INIT_FAILD,
    GPS_INIT_FAILD,
    SDS_INIT_FAILD,
    CAMERA_INIT_FAILD,
    GSM_INIT_FAILD
};

char error_code = 0;;

void gsm_connect();
void gsm_setup();
void gsm_postData();



int downloadFirmWare(String server, String resource);
int updateFirmWare();
void printPercent(uint32_t readLength, uint32_t contentLength);


void mqtt_init();
int mqtt_connect();
void mqtt_Callback(char* topic, unsigned char* payload, unsigned int len);
void mqtt_handle(String _cmd[]);

void mqtt_post();

void camera_setup();
esp_err_t take_send_photo();



void enter_sleep() {
    gsm_sleep();

    /*
    Next we decide what all peripherals to shut down/keep on
    By default, ESP32 will automatically power down the peripherals
    not needed by the wakeup source, but if you want to be a poweruser
    this is for you. Read in detail at the API docs
    http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
    Left the line commented as an example of how to configure peripherals.
    The line below turns off all RTC peripherals in deep sleep.
    */
    //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

    /*
    Now that we have setup a wake cause and if needed setup the
    peripherals state in deep sleep, we can now start going to
    deep sleep.
    In the case that no wake up sources were provided but deep
    sleep was started, it will sleep forever unless hardware
    reset occurs.
    */
#ifdef DEBUG
    SerialMon.println("Going to sleep now");

    SerialMon.flush();
#endif // DEBUG

    esp_deep_sleep_start();

}

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  120        /* Time ESP32 will go to sleep (in seconds) */


const int wdtTimeout = 120000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

void IRAM_ATTR resetModule() {
#ifdef DEBUG
    ets_printf("reboot\n");
    SerialMon.println("-------------------------------------");
    SerialMon.println("------->Watch dog reboot<------------");
    SerialMon.println("-------------------------------------");
    esp_restart();
#endif // DEBUG

}

void watchdog_feed() {
    timerWrite(timer, 0); //reset timer (feed watchdog)
}


void setup() {

    timer = timerBegin(0, 80, true);                  //timer 0, div 80
    timerAttachInterrupt(timer, &resetModule, true);  //attach callback
    timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
    timerAlarmEnable(timer);                          //enable interrupt

    /*
    First we configure the wake up source
    We set our ESP32 to wake up every 5 seconds
    */
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

#ifdef DEBUG
    SerialMon.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
#endif // DEBUG


#ifdef DEBUG
    SerialMon.begin(9600);
    delay(1000);
    SerialMon.println("Start debug");

#endif // DEBUG

    pinMode(LED_BUILD_IN, OUTPUT);
    digitalWrite(LED_BUILD_IN, LOW);
    delay(500);
    digitalWrite(LED_BUILD_IN, HIGH);
    delay(500);
    digitalWrite(LED_BUILD_IN, LOW);
    delay(500);
    digitalWrite(LED_BUILD_IN, HIGH);

    if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
#ifdef DEBUG
        SerialMon.println("SPIFFS Mount Failed");
#endif // DEBUG

        return;
    }

    //sensor init
    sds_init();
    //gps_init();
    //gps_read();

    camera_setup();

    //gsm init
    gsm_setup();
    gsm_connect();

#ifdef MQTT
    mqtt_init();
    mqtt_connect();
#endif // MQTT

    watchdog_feed();
}


void loop() {
    static unsigned long counterTimmer = millis();


    sds_read();

    sht_read();


    if (!modem.isGprsConnected()) {
        gsm_setup();
        gsm_connect();
    }

    if (!mqtt.connected()) {
        mqtt_connect();
    }
    mqtt.loop();

    if (millis() - counterTimmer > 60000) {
        counterTimmer = millis();

#ifdef MQTT
        mqtt_post();
#endif // MQTT

        gsm_postData();

        //take_send_photo();

        enter_sleep();
        
    }

    watchdog_feed();
    delay(2000);
}




//////////////////////////

void sht_read() {

    G_temp_c = sht1x.readTemperatureC();
    G_temp_f = sht1x.readTemperatureF();
    G_humidity = sht1x.readHumidity();

    if (G_temp_c < -10 || G_temp_c > 70) {
        G_temp_c = -1;
    }
    if (G_humidity < 0 || G_humidity > 100) {
        G_humidity = -1;
    }

#ifdef DEBUG
    SerialMon.print("Humi:  ");
    SerialMon.println(G_humidity);

    SerialMon.print("Temp:  ");
    SerialMon.println(G_temp_c);
#endif // DEBUG

}


void  sds_read() {
    int error;

    error = my_sds.read(&G_p25, &G_p10);
    if (!error) {
#ifdef DEBUG
        SerialMon.println("P2.5: " + String(G_p25));
        SerialMon.println("P10:  " + String(G_p10));
#endif // DEBUG

    }

}

void sds_init()
{
#ifdef DEBUG
    SerialMon.println("my_sds.begin");
#endif // DEBUG
    my_sds.begin(&Serial2, SDS_TX, SDS_RX);
}



void gps_init()
{
#ifdef DEBUG
    SerialMon.println("gps.begin");
#else
    GPS_Serial.begin(GPSBaud);
#endif // DEBUG
}


int gps_pareInfor() {
    int ret = 0;


    if (gps.location.isValid())
    {
#ifdef DEBUG
        SerialMon.print(F("Location: "));
#endif // DEBUG

        ret = 1;

        G_gps_lng = gps.location.lng();
        G_gps_lat = gps.location.lat();

#ifdef DEBUG
        SerialMon.print(G_gps_lat, 6);
        SerialMon.print(F(","));
        SerialMon.print(G_gps_lng, 6);
#endif // DEBUG
    }
    else
    {
#ifdef DEBUG
        SerialMon.print(F("INVALID"));
#endif // DEBUG
    }

    /*SerialMon.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
        SerialMon.print(gps.date.month());
        SerialMon.print(F("/"));
        SerialMon.print(gps.date.day());
        SerialMon.print(F("/"));
        SerialMon.print(gps.date.year());
    }
    else
    {
        SerialMon.print(F("INVALID"));
    }

    SerialMon.print(F(" "));
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10) SerialMon.print(F("0"));
        SerialMon.print(gps.time.hour());
        SerialMon.print(F(":"));
        if (gps.time.minute() < 10) SerialMon.print(F("0"));
        SerialMon.print(gps.time.minute());
        SerialMon.print(F(":"));
        if (gps.time.second() < 10) SerialMon.print(F("0"));
        SerialMon.print(gps.time.second());
        SerialMon.print(F("."));
        if (gps.time.centisecond() < 10) SerialMon.print(F("0"));
        SerialMon.print(gps.time.centisecond());
    }
    else
    {
        SerialMon.print(F("INVALID"));
    }*/
    //#ifdef DEBUG
    //    SerialMon.println();
    //#endif // DEBUG
    return ret;
}



void writeFile(fs::FS &fs, const char * path, const char * message) {
    //Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file) {
        //Serial.println("- failed to open file for writing!");
        return;
    }
    if (file.print(message)) {
        //Serial.println("- file written");
    }
    else {
        //Serial.println("- frite failed");
    }
    file.close();
}

void deleteFile(fs::FS &fs, const char * path) {
    //Serial.printf("Deleting file: %s\r\n", path);
    if (fs.remove(path)) {
        //Serial.println("- file deleted");
    }
    else {
        //Serial.println("- delete failed");
    }
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
    //Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        //Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(message)) {
        //Serial.println("Message appended");
    }
    else {
        //Serial.println("Append failed");
    }
    file.close();
}



void gsm_connect() {

    if (!modem.restart()) {
        return;
    }

#ifdef DEBUG
    SerialMon.print(F("Waiting for network..."));
#endif // DEBUG
    if (!modem.waitForNetwork()) {
#ifdef DEBUG
        SerialMon.println(" fail");
#endif // DEBUG
        delay(10000);
        return;
    }
#ifdef DEBUG
    SerialMon.println(" OK");
#endif // DEBUG

#ifdef DEBUG
    SerialMon.print(F("Connecting to APN"));
#endif // DEBUG
    if (!modem.gprsConnect(apn, user, pass)) {
#ifdef DEBUG
        SerialMon.println(" fail");
#endif // DEBUG
        delay(10000);
        return;
    }
#ifdef DEBUG
    SerialMon.println(" OK");
#endif // DEBUG
}

void gsm_setup() {
    // Set GSM module baud rate
    SerialAT.begin(9600);
    delay(3000);
    gsm_wakeUp();

    // Restart takes quite some time
    // To skip it, call init() instead of restart()
#ifdef DEBUG
    SerialMon.println(F("Initializing modem..."));
#endif // DEBUG
    modem.restart();

#ifdef DEBUG
    String modemInfo = modem.getModemInfo();
    SerialMon.print(F("Modem: "));
    SerialMon.println(modemInfo);
#endif // DEBUG


    // Unlock your SIM card with a PIN
    //modem.simUnlock("1234");
}

bool gsm_sleep()
{
    modem.sendAT(GF("+CSCLK=2"));
    if (modem.waitResponse(10000L) != 1) {
        //Serial.println("sleep Error");
        return false;
    }
    return true;
}

bool gsm_wakeUp()
{
    modem.sendAT(GF("+CSCLK=0"));
    if (modem.waitResponse(10000L) != 1) {
        //Serial.println("wakeUp Error");
        return false;
    }
    return true;
}


int updateFirmWare() {
    if (!SPIFFS.begin(true)) {
#ifdef DEBUG
        SerialMon.println("An Error has occurred while mounting SPIFFS");
#endif // DEBUG
        return 0;
    }

    File file = SPIFFS.open("/firmware.bin");

    if (!file) {
#ifdef DEBUG
        SerialMon.println("Failed to open file for reading");
#endif // DEBUG
        return 0;
    }

#ifdef DEBUG

    SerialMon.println("Starting update..");
#endif // DEBUG


    size_t fileSize = file.size();

    if (!Update.begin(fileSize)) {
#ifdef DEBUG
        SerialMon.println("Cannot do the update");
#endif // DEBUG
        return 0;
    };

    Update.writeStream(file);

    if (Update.end()) {
#ifdef DEBUG
        SerialMon.println("Successful update");
        //Serial.println("Reset in 4 seconds...");
        //ESP.restart();
#endif // DEBUG
        return 1;
    }
    else {
#ifdef DEBUG
        SerialMon.println("Error Occurred: " + String(Update.getError()));
#endif // DEBUG
        return 0;
    }

    file.close();
    delay(2000);
    return 1;
}

void printPercent(uint32_t readLength, uint32_t contentLength) {
    // If we know the total length
#ifdef DEBUG
    if (contentLength != -1) {
        SerialMon.print("\r ");
        SerialMon.print((100.0 * readLength) / contentLength);
        SerialMon.print('%');
    }
    else {
        SerialMon.println(readLength);
    }
#endif // DEBUG

}

/*
Content-Length:
Calc. CRC32:
*/
int downloadFirmWare(String server, String resource)
{
    String fileName = "/" + resource;
    deleteFile(SPIFFS, fileName.c_str());
    writeFile(SPIFFS, fileName.c_str(), "");

#ifdef DEBUG
    SerialMon.print(F("Connecting to "));
    SerialMon.print(server);
#endif // DEBUG


    if (!client.connect(server.c_str(), port, 10000L)) {
#ifdef DEBUG
        SerialMon.println(" fail");
#endif // DEBUG

        return 0;
    }
#ifdef DEBUG
    SerialMon.println(" OK");
#endif // DEBUG
    // Make a HTTP GET request:
    client.print(String("GET ") + "/" + resource + " HTTP/1.0\r\n");
    client.print(String("Host: ") + server + "\r\n");
    client.print("Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    while (client.available() == 0) {
        if (millis() - timeout > 5000L) {
#ifdef DEBUG
            SerialMon.println(F(">>> Client Timeout !"));
#endif // DEBUG
            client.stop();
            return 0;
        }
    }

#ifdef DEBUG
    SerialMon.println(F("Reading response header"));
#endif // DEBUG

    uint32_t contentLength;

    while (client.available()) {
        String line = client.readStringUntil('\n');
        line.trim();
#ifdef DEBUG
        SerialMon.println(line);    // Uncomment this to show response header
#endif // DEBUG

        line.toLowerCase();
        if (line.startsWith("content-length:")) {
            contentLength = line.substring(line.lastIndexOf(':') + 1).toInt();
        }
        else if (line.length() == 0) {
            break;
        }
    }

#ifdef DEBUG
    SerialMon.println(F("Reading response data"));
#endif // DEBUG

    timeout = millis();
    uint32_t readLength = 0;
    CRC32 crc;

    String msg = "";
    File file = SPIFFS.open(fileName, FILE_APPEND);

    unsigned long timeElapsed = millis();
    printPercent(readLength, contentLength);
    while (readLength < contentLength && client.connected() && millis() - timeout < 10000L) {
        while (client.available()) {
            int c = client.read();
            file.write(c);
            //msg += (char)c;
            //Serial.print((char)c);       // Uncomment this to show data
            crc.update(c);
            readLength++;

            if (readLength % (contentLength / 100) == 0) {
                printPercent(readLength, contentLength);
            }
            watchdog_feed();
            timeout = millis();
        }
        ets_delay_us(1);
    }

    if (msg != "") {
        file.print(msg);
    }
    file.close();
    printPercent(readLength, contentLength);

    /*if (crc.finalize() == checksum.toInt()) {
        return 1;
    }*/
    //#ifdef DEBUG
    //    SerialMon.print("Content-Length: ");   SerialMon.println(contentLength);
    //    SerialMon.print("Actually read:  ");   SerialMon.println(readLength);
    //    SerialMon.print("Calc. CRC32:    0x"); SerialMon.println(crc.finalize(), HEX);
    //#endif // DEBUG

        //read file size
    File root = SPIFFS.open(fileName);
#ifdef DEBUG
    SerialMon.print("  FILE: ");
    SerialMon.print(root.name());
    SerialMon.print("\tSIZE: ");
    SerialMon.println(root.size());
#endif // DEBUG
    if (readLength != contentLength)
        return 0;

    return 1;
}


void gsm_postData() {

    if (!client.connect(serverName, port)) {
#ifdef DEBUG
        SerialMon.println("connect fail");
#endif // DEBUG
        gsm_connect();
    }
    else {
#ifdef DEBUG
        SerialMon.println("connect OK");
#endif // DEBUG

        // Making an HTTP POST request
#ifdef DEBUG
        SerialMon.println("Performing HTTP POST request...");
#endif // DEBUG
        // Prepare your HTTP POST request data (Temperature in Celsius degrees)
        String httpRequestData = "api_key=" + apiKeyValue +
            "&value1=" + String(G_temp_c) + "&value2=" + String(G_humidity) +
            "&value3=" + String(G_p25) + "&value4=" + String(G_p10) +
            "&value5=" + String(G_gps_lat, 6) + "&value6=" + String(G_gps_lng, 6) +
            //"&value7=" + String(analogBatteryValue) +
            "&value7=0" +
            "";

        client.print(String("POST ") + resource + " HTTP/1.1\r\n");
        client.print(String("Host: ") + serverName + "\r\n");
        client.println("Connection: close");
        client.println("Content-Type: application/x-www-form-urlencoded");
        client.print(F("Content-Length: "));
        client.println(httpRequestData.length());
        client.println();
        client.println(httpRequestData);
#ifdef DEBUG
        SerialMon.println("send httpRequestData");
#endif // DEBUG

        unsigned long timeout = millis();
        while (client.connected() && millis() - timeout < 10000L) {
            // Print available data (HTTP response from server)
            while (client.available()) {
                char c = client.read();
                //#ifdef DEBUG
                //                SerialMon.print(c);
                //#endif // DEBUG
                timeout = millis();
            }
        }
    }
}


#ifdef MQTT


void mqtt_init()
{
    mqtt.setServer(mqtt_server, mqtt_port);
    mqtt.setCallback(mqtt_Callback);
}

int mqtt_connect() {
#ifdef DEBUG
    SerialMon.println("connect mqtt");
#endif // DEBUG

    if (mqtt.connect(mqtt_clientID.c_str())) {
#ifdef DEBUG
        SerialMon.println("connect mqtt success");
#endif // DEBUG
        mqtt.subscribe(mqtt_topic_sub, 0);
        error_code += '0';
        String msg = "Error code: " + error_code;
        mqtt.publish(mqtt_topic_pub, msg.c_str());
        return mqtt.connected();
    }
    else {
        return 0;
    }
}


void mqtt_Callback(char* topic, unsigned char* payload, unsigned int len) {
    uint8_t j = -1;
    String strCommand[8];
    String cmd = "";
    int count = 0;
    char c;
    for (int i = 0; i < len; i++) {
        c = ((char)payload[i]);
        //		Serial.print(c);
        if (c == '/')
        {
            j++;
        }
        if (c != '/') {
            if (j == -1) {
                //Serial.println("Ban tin khong hop le");
                //Loi ban tin
                return;
            }
            strCommand[j] += c;
        }
        if (j > 7) break;
    }
#ifdef DEBUG
    SerialMon.println((char *)payload);
#endif // DEBUG

    mqtt_handle(strCommand);
}

void mqtt_handle(String _cmd[]) {
    int cmdType;
    float _pm25[4];
    float _pm10[4];
    float _sht[2];
    float a, b;
    float _dust[2];
    bool _b = true;
    if (_cmd[0] == "firmware") {
        if (_cmd[1] == "update")
            cmdType = MQTT_UPDATE_FIRMWARE;
        else if (_cmd[1] == "version")
            cmdType = MQTT_READ_FIRMWARE;
    }
    else {
        cmdType = UNKNOW;
    }


    switch (cmdType)
    {
    case MQTT_READ_FIRMWARE:

        mqtt.publish(mqtt_topic_pub, version.c_str(), 0);
        delay(10);
        break;
    case MQTT_UPDATE_FIRMWARE:
        //  update_firmware/version/crc
        int ret;

        ret = downloadFirmWare(_cmd[2], _cmd[3]);
        if (ret == 0) {
#ifdef DEBUG
            SerialMon.println("download error");
            esp_restart();
#endif // DEBUG
            mqtt.publish(mqtt_topic_pub, version.c_str(), 0);

            delay(1000);
            return;
        }

        ret = updateFirmWare();

        mqtt.loop();

        if (ret == 1) {
            mqtt.publish(mqtt_topic_pub, version.c_str(), 0);
            mqtt.loop();
            delay(1000);
            esp_restart();
        }
        else {
            mqtt.publish(mqtt_topic_pub, version.c_str(), 0);
            mqtt.loop();
        }
        break;
    default:
        mqtt.publish(mqtt_topic_pub, "invalid!");
        break;
    }
}

void mqtt_post()
{
    StaticJsonBuffer<500> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    JsonArray& dust = root.createNestedArray("dust");
    JsonArray& sht = root.createNestedArray("sht");
    JsonArray& gps = root.createNestedArray("gps");
    dust.add(G_p25);
    dust.add(G_p10);
    sht.add(G_temp_c);
    sht.add(G_humidity);
    gps.add(G_gps_lat);
    gps.add(G_gps_lng);

    //root["battery"] = analogBatteryValue;

    String msg;
    root.printTo(msg);

    mqtt.publish(mqtt_topic_pub, msg.c_str(), 0);
}
#endif // MQTT



void camera_setup() {
#ifdef DEBUG
    SerialMon.println("camera setup");
#endif // DEBUG
    camera_config_t config;

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    //init with high specs to pre-allocate larger buffers
    /*config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;*/
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;


    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
#ifdef DEBUG
        SerialMon.printf("Camera init failed with error 0x%x", err);
#endif // DEBUG
        error_code |= 0x0f << CAMERA_INIT_FAILD;
        return;
    }
#ifdef DEBUG
    SerialMon.println("done");
#endif // DEBUG
}

esp_err_t take_send_photo()
{
    camera_fb_t * fb = NULL;
#ifdef DEBUG
    SerialMon.println("Taking picture...");
#endif // DEBUG
    fb = esp_camera_fb_get();
    if (!fb) {
#ifdef DEBUG
        SerialMon.println("Camera capture failed");
#endif // DEBUG
        return ESP_FAIL;
    }
#ifdef DEBUG
    SerialMon.println("Camera capture success");
#endif // DEBUG

    File file = SPIFFS.open(FILE_PHOTO, FILE_WRITE);
    file.print("");
    file.close();
    file = SPIFFS.open(FILE_PHOTO, FILE_APPEND);
    if (!file) {
#ifdef DEBUG
        SerialMon.println("There was an error opening the file");
#endif // DEBUG
        return -1;
    }

    uint8_t *p = fb->buf;
    int len = fb->len;
    int cnt = 0;

    while (cnt < len)
    {
        if (file.write(*p)) {
            //#ifdef DEBUG
            //            SerialMon.println("File was written");
            //#endif // DEBUG
        }
        else {
#ifdef DEBUG
            SerialMon.println("File write failed");
#endif // DEBUG
        }
        *p++;
        cnt++;
    }
    file.close();
    esp_camera_fb_return(fb);


    if (!client.connect(serverName, port)) {
#ifdef DEBUG
        SerialMon.println("connect server fail");
#endif // DEBUG
        gsm_connect();
    }
    else {
#ifdef DEBUG
        SerialMon.println("connect server OK");

        // Making an HTTP POST request
        SerialMon.println("sending picture...");
#endif // DEBUG

        client.print(String("POST ") + post_image_url + " HTTP/1.1\r\n");
        client.print(String("Host: ") + serverName + "\r\n");
        client.println("Connection: close");
        client.println("Content-Type: image/jpg");
        client.print("Content-Length: ");
        client.println(len);
        client.println();
        //client.write(fb->buf, fb->len);

        file = SPIFFS.open(FILE_PHOTO);
        cnt = 0;
        while (cnt < len)
        {
            int ret = file.readBytes((char *)msg, 500);
            client.write(msg, ret);
            cnt += ret;
        }
        client.println();
        file.close();

#ifdef DEBUG
        SerialMon.print("transfer success:  ");
        SerialMon.print(cnt);
        SerialMon.println("  bytes");
#endif // DEBUG

        unsigned long timeout = millis();
        while (client.connected() && millis() - timeout < 10000L) {
            // Print available data (HTTP response from server)
            while (client.available()) {
                //#ifdef DEBUG
                //                char c = client.read();
                //                SerialMon.print(c);
                //#endif // DEBUG

                timeout = millis();
            }
        }

#ifdef DEBUG
        SerialMon.println();
#endif // DEBUG
    }
}


void gps_read() {
    static unsigned long lastRead = millis();

    while (GPS_Serial.available()) {
        gps.encode(GPS_Serial.read());
        if (gps.location.isUpdated()) {
#ifdef DEBUG
            SerialMon.print("Latitude= ");
            SerialMon.print(gps.location.lat(), 6);
            SerialMon.print(" Longitude= ");
            SerialMon.println(gps.location.lng(), 6);
#endif // DEBUG

            G_gps_lng = gps.location.lng();
            G_gps_lat = gps.location.lat();
            return;
        }

        if (millis() - lastRead > 15000) {
            return;
        }
    }
}


