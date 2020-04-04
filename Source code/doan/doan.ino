

#define DEBUG



#include "FS.h"
#include "SPIFFS.h"
#define FORMAT_SPIFFS_IF_FAILED true


#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;


// Select your modem:
#define TINY_GSM_MODEM_SIM800

// Increase RX buffer if needed
#define TINY_GSM_RX_BUFFER 1024
#define TINY_GSM_USE_GPRS true

#include <TinyGsmClient.h>
//#include <ArduinoHttpClient.h>

// Uncomment this if you want to see all AT commands
//#define DUMP_AT_COMMANDS

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon SerialBT

// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial

// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[] = "m-wap";
const char user[] = "mms";
const char pass[] = "mms";

// Server details
const char* serverName = "diedd212.000webhostapp.com";
const int  port = 80;
const char* resource = "/post-data.php";
String apiKeyValue = "diendd212";
const char *post_url = "/post-image.php"; // Location where images are POSTED
#define FILE_PHOTO "/image.txt"


#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

//TinyGsmClient client(modem, 1);
//HttpClient http(client, post_data, port);
TinyGsmClient client(modem);



#include "esp_system.h"
const int wdtTimeout = 600000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

void IRAM_ATTR resetModule() {
#ifdef DEBUG

    ets_printf("reboot\n");
    Serial.println("-------------------------------------");
    Serial.println("------->Watch dog reboot<------------");
    Serial.println("-------------------------------------");
#endif // DEBUG
    esp_restart();
}



#include <SDS011.h>
float _dustSensor[2];//0=pm25, 1=pm10
float _shtSensor[3];//0=hum, 1=tempC, 2=tempF

SDS011 my_sds;

#include <SHT1x.h>
#define SHT_dataPin  2
#define SHT_clockPin 14
SHT1x _sht(SHT_dataPin, SHT_clockPin);


#define CAMERA_MODEL_AI_THINKER
#include "esp_camera.h"
#include "Camerapins.h"

void cameraSetup() {
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
    //if (psramFound()) {
    //    //config.frame_size = FRAMESIZE_UXGA;
    //    config.frame_size = FRAMESIZE_QQVGA;
    //    config.jpeg_quality = 10;
    //    config.fb_count = 2;
    //}
    //else {
    //    config.frame_size = FRAMESIZE_SVGA;
    //    config.jpeg_quality = 12;
    //    config.fb_count = 1;
    //}
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 20;
    config.fb_count = 1;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        SerialMon.printf("Camera init failed with error 0x%x", err);
        return;
    }
}




void setup()
{
    timer = timerBegin(0, 80, true);                  //timer 0, div 80
    timerAttachInterrupt(timer, &resetModule, true);  //attach callback
    timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
    timerAlarmEnable(timer);                          //enable interrupt


    if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
        SerialMon.println("SPIFFS Mount Failed");
        return;
    }

#ifdef DEBUG
    SerialBT.begin("ESP32test"); //Bluetooth device name
#endif // DEBUG
    delay(5000);

    // Set GSM module baud rate
    SerialAT.begin(115200);
    delay(3000);

    cameraSetup();


    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    SerialMon.println(F("Initializing modem..."));
    modem.restart();
    String modemInfo = modem.getModemInfo();
    SerialMon.print(F("Modem: "));
    SerialMon.println(modemInfo);

    connect_gsm();


    my_sds.begin(&Serial2, 16, 17); //RX, TX


    timerWrite(timer, 0); //reset timer (feed watchdog)
}

void loop()
{
    timerWrite(timer, 0); //reset timer (feed watchdog)
    SerialMon.print(F("New cycle"));

    readSensor();
    take_send_sensor_value();
    
    //take_send_photo();
    
    delay(600000);
}


void readSensor() {

    _shtSensor[0] = _sht.readHumidity();
    _shtSensor[1] = _sht.readTemperatureC();
    _shtSensor[2] = _sht.readTemperatureF();
    if (_shtSensor[0] < 0 || _shtSensor[0] > 100) {
        _shtSensor[0] = -1;
    }
    if (_shtSensor[1] < 0 || _shtSensor[1] > 55) {
        _shtSensor[1] = -1;
    }

    float p10, p25;
    int err;
    err = my_sds.read(&p25, &p10);
    if (!err) {
        _dustSensor[0] = p25;
        _dustSensor[1] = p10;
    }
    else {
        _dustSensor[0] = -1;
        _dustSensor[1] = -1;
    }

    SerialMon.print("Temp:  ");
    SerialMon.println(_shtSensor[1]);
    SerialMon.print("Humi:  ");
    SerialMon.println(_shtSensor[0]);
    SerialMon.print("pm25:  ");
    SerialMon.println(_dustSensor[0]);
    SerialMon.print("pm10:  ");
    SerialMon.println(_dustSensor[1]);

}


void connect_gsm() {

    SerialMon.print(F("Waiting for network..."));
    if (!modem.waitForNetwork()) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" OK");

    SerialMon.print(F("Connecting to APN"));
    if (!modem.gprsConnect(apn, user, pass)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" OK");
}


int take_send_photo()
{
    SerialMon.println("Taking picture...");
    camera_fb_t * fb = NULL;
    delay(2000);
    esp_err_t res = ESP_OK;

    SerialMon.println("Waiting camera to be ready...");
    delay(2000);
    fb = esp_camera_fb_get();
    if (!fb) {
        SerialMon.println("Camera capture failed");
        return ESP_FAIL;
    }
    SerialMon.println("Camera capture success");


    File file = SPIFFS.open(FILE_PHOTO, FILE_WRITE);
    file.print("");
    file.close();
    file = SPIFFS.open(FILE_PHOTO, FILE_APPEND);
    if (!file) {
        SerialMon.println("There was an error opening the file");
        return -1;
    }

    uint8_t *p = fb->buf;
    int len = fb->len;
    int cnt = 0;

    while (cnt < len)
    {
        if (file.write(*p)) {
            //SerialMon.println("File was written");;
        }
        else {
            SerialMon.println("File write failed");
        }
        *p++;
        cnt++;
    }
    file.close();
    esp_camera_fb_return(fb);


    if (!client.connect(serverName, port)) {
        SerialMon.println("connect server fail");
        connect_gsm();
    }
    else {
        SerialMon.println("connect server OK");

        // Making an HTTP POST request
        SerialMon.println("Performing HTTP POST request...");

        String msg = "12345";

        client.print(String("POST ") + post_url + " HTTP/1.1\r\n");
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
            uint8_t msg[500];
            int ret = file.readBytes((char *)msg, 500);
            client.write(msg, ret);
            cnt += ret;
            delay(500);
        }
        client.println();
        file.close();

        SerialMon.print("transfer success:  ");
        SerialMon.print(cnt);
        SerialMon.println("  bytes");


        unsigned long timeout = millis();
        while (client.connected() && millis() - timeout < 10000L) {
            // Print available data (HTTP response from server)
            while (client.available()) {
                char c = client.read();
                SerialMon.print(c);
                timeout = millis();
            }
        }
        SerialMon.println();
    }

}

int take_send_sensor_value() {
    if (!client.connect(serverName, port)) {
        SerialMon.println("connect server  fail");
    }
    else {
        SerialMon.println("connect server  OK");

        // Making an HTTP POST request
        SerialMon.println("Performing HTTP POST take_send_sensor_value() request...");
        // Prepare your HTTP POST request data (Temperature in Celsius degrees)
        String httpRequestData = "api_key=" + apiKeyValue + "&value1=" + _shtSensor[1]
            + "&value2=" + _shtSensor[0] + "&value3=" + _dustSensor[0] + "&value4=" + _dustSensor[1] + "";

        client.print(String("POST ") + resource + " HTTP/1.1\r\n");
        client.print(String("Host: ") + serverName + "\r\n");
        client.println("Connection: close");
        client.println("Content-Type: application/x-www-form-urlencoded");
        client.print("Content-Length: ");
        client.println(httpRequestData.length());
        client.println();
        client.println(httpRequestData);

        unsigned long timeout = millis();
        while (client.connected() && millis() - timeout < 10000L) {
            // Print available data (HTTP response from server)
            while (client.available()) {
                char c = client.read();
                SerialMon.print(c);
                timeout = millis();
            }
        }
        SerialMon.println();
    }

}

