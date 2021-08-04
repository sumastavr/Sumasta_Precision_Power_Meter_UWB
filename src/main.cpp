#include <sumastaConfig.h>

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <esp_now.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "esp_adc_cal.h"
#include <HardwareSerial.h>

#define ADC_EN              14  //ADC_EN is the ADC detection enable port
#define ADC_PIN             34
#define BUTTON_1            35
#define BUTTON_2            0

HardwareSerial UWBSerial(2);

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library

char buff[512];
int vref = 1100;
int btnCick = false;

uint8_t broadcastAddress[] = { 0xFF, 0xFF,0xFF,0xFF,0xFF,0xFF };

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use thi for the 12-bit version */

// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
const int ADC_DC = 27;

// variable for storing the potentiometer value
int potValue = 0;

int ADCVal = 0;
int ADCCounter = 0;
int ADCAverage = 0;
int remoteADCAverage = 0;

long streamtimer = 0;
long running = millis();

String UWBDistanceFromTag = "";
boolean receivedUWBDistanceFromTag = false;

long uptime = millis();

void setup() {
    Serial.begin(115200);
    UWBSerial.begin(115200, SERIAL_8N1, 2, 17);

    /*
    ADC_EN is the ADC detection enable port
    If the USB port is used for power supply, it is turned on by default.
    If it is powered by battery, it needs to be set to high level
    */

    pinMode(ADC_EN, OUTPUT);
    digitalWrite(ADC_EN, HIGH);

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_GREEN);
    tft.setCursor(0, 0);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);

    tft.setSwapBytes(true);
    //tft.pushImage(0, 0, 240, 135, ttgo);
    //espDelay(5000);


    tft.setRotation(0);
    tft.fillScreen(TFT_RED);
    espDelay(500);
    tft.fillScreen(TFT_BLUE);
    espDelay(500);
    tft.fillScreen(TFT_GREEN);
    espDelay(500);

    tft.fillScreen(TFT_BLUE);

    tft.setCursor(0, 0);
    tft.setTextColor(TFT_GREEN);
    tft.setTextFont(1);
    
    tft.println("Init TFT Success...");

    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
    }
    else {
        Serial.println("Default Vref: 1100mV");
    }

    tft.println("Init On-board ADC Success...");
    
    initUWB();

    tft.println("Init UWB Success...");

    /*
    tft.drawString("[WiFi Scan]", tft.width() / 2, tft.height() / 2);
    tft.drawString("RightButton:", tft.width() / 2, tft.height() / 2 + 16);
    tft.drawString("[Voltage Monitor]", tft.width() / 2, tft.height() / 2 + 32);
    tft.drawString("RightButtonLongPress:", tft.width() / 2, tft.height() / 2 + 48);
    tft.drawString("[Deep Sleep]", tft.width() / 2, tft.height() / 2 + 64);
    */

    //tft.setTextDatum(TL_DATUM);


    WiFi.mode(WIFI_STA);

    Serial.println();
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // register peer
    esp_now_peer_info_t peerInfo;

    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
    else {
        tft.println("Init ESP NOW Wifi Success...");
    }

    esp_now_register_recv_cb(onReceiveData);

    //sendData(ADCAverage);

    delay(1000);
    sendData(ADCAverage);

    // The ADC input range (or gain) can be changed via the following
 // functions, but be careful never to exceed VDD +0.3V max, or to
 // exceed the upper and lower limits if you adjust the input range!
 // Setting these values incorrectly may destroy your ADC!
 //                                                                ADS1015  ADS1115
 //                                                                -------  -------
 // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    //ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
   // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
   // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
   // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

    ads.begin();

    tft.println("Init external ADC Success...");

   // int r = digitalRead(TFT_BL);
   // digitalWrite(TFT_BL, !r);
    tft.print("Starting in 3 seconds");

    for (int i = 0; i < 3; i++) {
        tft.print(".");
        delay(1000);
    }

    initFrame();

}

void loop() {
    
    if (millis() - uptime > 5000) {
        Serial.println("");
        Serial.print("Uptime: ");
        Serial.println(millis());
        Serial.println("");
        uptime = millis();
        showVoltage();
    }

    ADCVal += ads.readADC_SingleEnded(0);
    ADCCounter++;

    if (millis() - streamtimer > 100) {
        ADCAverage = ADCVal / ADCCounter;

        //if (normalizeADC) ADCAverage -= normalizedADCFlorr;
        if (ADCAverage < 0) ADCAverage = 0;

        //showRSSILED(scaleRSSIBar(ADCAverage));

        //float multiplier = 3300 / 4095;
        //float PDCurrent = (ADCAverage * 0.125) / 10.1;

       // Serial.print(PDCurrent);
       // Serial.println("uA");

        //initFrame();
        //updateRemoteDevice(ADCAverage);

        streamtimer = millis();
        ADCVal = 0;
        ADCCounter = 0;

        updateThisDevice(ADCAverage);
        updateRemoteDevice(remoteADCAverage);

        if (receivedUWBDistanceFromTag) {
            updateDistanceTagNoSending(UWBDistanceFromTag);
        }
        //sendData(ADCAverage);

    }
    
    if (millis() - running > 1000) {
        Serial.println("Idle Sent");
        sendData(ADCAverage);
    }

    String dist = "";
    boolean count = false;
    while (UWBSerial.available()>0) {
        int inByte = UWBSerial.read();
        //Serial.write(inByte);

        if (count && inByte == 13) {
            count = false;
            Serial.print("Dist: ");
            Serial.println(dist);
            updateDistanceTag(dist);
        }

        if (count) {
            dist += (char)inByte;
        }

        if (inByte == 61) { // ascii 61= =
            count = true;
        }
    }

    if (digitalRead(BUTTON_2) == LOW) {
        delay(300);
        deepSleep();

    }

}

void initUWB() {
    
    delay(100);
    int counter = 0;
    for (int i = 0; i < 5; i++) {

        UWBSerial.write(13);
        UWBSerial.write(13);

        Serial.print("Trying: ");
        Serial.println(i);

        tft.print("Try Connect UWB Anchor: ");
        tft.println(i);

        delay(500);
        while (UWBSerial.available() > 0) {
            Serial.write(UWBSerial.read());
            counter++;
        }

        if (counter > 20) {
            Serial.println("Enter Mode");
        }

        delay(500);
        UWBSerial.print("les");
        UWBSerial.write(13);
        delay(500);

        counter = 0;
        while (UWBSerial.available() > 0) {
            Serial.write(UWBSerial.read());
            counter++;
        }

        if (counter > 20) {
            Serial.println("Success");
            i = 4;
            tft.println("Success, this is UWB tag...");
            break;
        }

    }
    int timeout = 0;

    /*
    while (timeout < 1000) {
        if (Serial.available() > 0) {

        }
        delay(1);
    }
    */
}

void onReceiveData(const uint8_t* mac, const uint8_t* data, int len) {

    /*
    Serial.print("Received from MAC: ");

    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", mac[i]);
        if (i < 5)Serial.print(":");
    }
    */

    int* messagePointer = (int*)data;

    int received = *messagePointer;

    if (received < 100000) {
        Serial.print("Remote_ADC: ");
        Serial.println(received);

        remoteADCAverage = received;
        delay(200);
        sendData(ADCAverage);
    }
    else if (received >100000){
        float distance = received - 100000.00;
        distance /= 100.0;
        Serial.print("GotDist: ");
        Serial.println(distance);
        UWBDistanceFromTag = String(distance, 2);
        receivedUWBDistanceFromTag = true;
        //updateDistanceTagNoSending(String(distance,2));
    }

    delay(10);

}


void sendData(int dataOut) {
    
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&dataOut, sizeof(int));
    if (result != ESP_OK) {
        Serial.println("Error sending the data");
    }
    running = millis();
}

void showVoltage()
{

        uint16_t v = analogRead(ADC_PIN);
        float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
        String voltage = String(battery_voltage,2) + "V";
        Serial.println(voltage);
        tft.setTextSize(1);
        tft.fillRect(tft.width() - 35, tft.height() / 2 + 110, 27, 7, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.setCursor(tft.width() - 35, tft.height() / 2 + 15 + 75 + 20);
        tft.setTextColor(TFT_WHITE);
        tft.print(voltage);

}

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms)
{
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}

void initFrame() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);

    tft.setTextSize(2);

    tft.drawRect(0, 0, tft.width(), tft.height() / 2 - 15, TFT_GREEN);
    tft.drawRect(0, tft.height() / 2 + 15, tft.width(), tft.height() / 2 - 15, TFT_GREEN);
    tft.drawRect(2, 2, 25, tft.height() / 2 - 19, TFT_WHITE);
    tft.drawRect(2, tft.height() / 2 + 17, 25, tft.height() / 2 - 19, TFT_WHITE);

    tft.drawString("20 Meter", tft.width() / 2, tft.height() / 2);

    tft.setTextSize(2);
    tft.setCursor(45, 3);
    tft.setTextColor(TFT_CYAN);
    //tft.drawString("Remote Device", 20, 3);
    tft.print("Remote");

    tft.setTextColor(TFT_CYAN);
    tft.setCursor(55, tft.height() / 2 + 18);
    tft.println("This");


    tft.setTextSize(1);
    tft.setCursor(65, 18);
    tft.print("device");
    tft.setCursor(65, tft.height() / 2 + 33);
    tft.print("device");

    tft.setCursor(45, 95);
    tft.print("Ave Detect");

    tft.setCursor(30, tft.height() / 2 + 15 + 75 + 20);
    tft.print("Ave Detect");

    tft.setCursor(tft.width() - 35, tft.height() / 2 + 15 + 75 + 20);
    tft.setTextColor(TFT_WHITE);
    tft.print("3.47V");

    // remote device
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(2);
    tft.setCursor(35, 35);
    tft.print("225.3 uA");

    tft.setTextColor(TFT_ORANGE);
    tft.setCursor(35, 55);
    tft.print("425.3 mW");

    tft.setTextColor(TFT_RED);
    tft.setCursor(35, 75);
    tft.print("4.3 dBm");

    // This/Nearby device
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(2);
    tft.setCursor(35, tft.height() / 2 + 15 + 35);
    tft.print("225.3 uA");

    tft.setTextColor(TFT_ORANGE);
    tft.setCursor(35, tft.height() / 2 + 15 + 55);
    tft.print("425.3 mW");

    tft.setTextColor(TFT_RED);
    tft.setCursor(35, tft.height() / 2 + 15 + 75);
    tft.print("4.3 dBm");

    //drawBarGraphRemote();
    //drawBarGraphThis();
}

void updateThisDevice(int ADCin) {

    float PDCurrent = (ADCin * 0.125/4.3); // 4.3 is gain of the op amp

    tft.fillRect(35, tft.height() / 2 + 15 + 35, tft.width() - 36, tft.height() / 2 - 17 - 48, TFT_BLACK);

    // This/Nearby device
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(2);
    tft.setCursor(35, tft.height() / 2 + 15 + 35);
    tft.print(PDCurrent,1);
    tft.print(" uA");

    tft.setTextColor(TFT_ORANGE);
    tft.setCursor(35, tft.height() / 2 + 15 + 55);
    float mw = PDCurrent * 2 / 1000;
    tft.print(mw,2);
    tft.print(" mW");

    tft.setTextColor(TFT_RED);
    tft.setCursor(35, tft.height() / 2 + 15 + 75);
    float dbm = 10 * log10(mw);
    tft.print(dbm, 1);
    tft.print("dBm");

    drawBarGraphThis(GetScaleBar(PDCurrent));

}

void updateRemoteDevice(int ADCin) {

    float PDCurrent = (ADCin * 0.125/4.3); // 4.3 is gain of the op amp

    tft.fillRect(35, 35, tft.width() - 36, tft.height() / 2 - 17 - 48, TFT_BLACK);

    //String sPDCurrent = String(PDCurrent, 1);

    // remote device
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(2);
    tft.setCursor(35, 35);
    tft.print(PDCurrent,1);
    tft.print(" uA");

    tft.setTextColor(TFT_ORANGE);
    tft.setCursor(35, 55);
    float mw = PDCurrent * 2 / 1000;
    tft.print(mw, 2);
    tft.print(" mW");

    tft.setTextColor(TFT_RED);
    tft.setCursor(35, 75);
    float dbm = 10 * log10(mw);
    tft.print(dbm, 1);
    tft.print("dBm");

    drawBarGraphRemote(GetScaleBar(PDCurrent));
}

void updateDistanceTag(String Distance) {
    tft.setTextDatum(MC_DATUM);
    int intDistance = Distance.toFloat() * 100; // round it off remove the coma
    sendDistanceInfotoTag(intDistance);
    Distance += "Meter";

    tft.fillRect(0, tft.height() / 2-14, tft.width(), 28, TFT_BLACK);

    tft.setTextColor(TFT_GREEN);
    tft.drawString(Distance, tft.width() / 2, tft.height() / 2);
    
}

void updateDistanceTagNoSending(String Distance) {
    tft.setTextDatum(MC_DATUM);
    Distance += " Meter";
    //Serial.println("A");

    tft.setTextColor(TFT_GREEN);
    //Serial.println("D");
    tft.fillRect(0, 106, 135, 28, TFT_BLACK);

    //Serial.println("B");
    tft.drawString(Distance, tft.width() / 2, tft.height() / 2);
    //Serial.println("C");
}

void sendDistanceInfotoTag(int distance) {

    distance += 100000;

    Serial.print("SerDist");
    Serial.println(distance);
    sendData(distance);
}

const int barsize = 8;
int barcolor[10] = { TFT_RED,TFT_RED,TFT_ORANGE,TFT_ORANGE,TFT_ORANGE,TFT_GREEN,TFT_GREEN,TFT_GREEN,TFT_GREEN,TFT_RED };

int RSSIBarLimit[10] = {1,5,10,20,35,55,100,200,400,700};

void drawBarGraphRemote(int scale) {
    tft.fillRect(2, 2, 25, tft.height() / 2 - 19, TFT_BLACK);
    tft.drawRect(2, 2, 25, tft.height() / 2 - 19, TFT_WHITE);
    for (int i = 0; i < scale; i++) {
        tft.fillRect(4, 93-((i*barsize)+(i*2)), 21, barsize,barcolor[i]);
    }
}

void drawBarGraphThis(int scale) {
    tft.fillRect(2, tft.height() / 2 + 17, 25, tft.height() / 2 - 19, TFT_BLACK);
    tft.drawRect(2, tft.height() / 2 + 17, 25, tft.height() / 2 - 19, TFT_WHITE);
    for (int i = 0; i < scale; i++) {
        tft.fillRect(4, 228 - ((i * barsize) + (i * 2)), 21, barsize, barcolor[i]);
    }
}

int GetScaleBar(float PDCurrent) {
    int scaleBar = -1;

    if (PDCurrent > RSSIBarLimit[0] && PDCurrent <= RSSIBarLimit[1]) {
        scaleBar = 1;
    }
    else if (PDCurrent > RSSIBarLimit[1] && PDCurrent <= RSSIBarLimit[2]) {
        scaleBar = 2;
    }
    else if (PDCurrent > RSSIBarLimit[2] && PDCurrent <= RSSIBarLimit[3]) {
        scaleBar = 3;
    }
    else if (PDCurrent > RSSIBarLimit[3] && PDCurrent <= RSSIBarLimit[4]) {
        scaleBar = 4;
    }
    else if (PDCurrent > RSSIBarLimit[4] && PDCurrent <= RSSIBarLimit[5]) {
        scaleBar = 5;
    }
    else if (PDCurrent > RSSIBarLimit[5] && PDCurrent <= RSSIBarLimit[6]) {
        scaleBar = 6;
    }
    else if (PDCurrent > RSSIBarLimit[6] && PDCurrent <= RSSIBarLimit[7]) {
        scaleBar = 7;
    }
    else if (PDCurrent > RSSIBarLimit[7] && PDCurrent <= RSSIBarLimit[8]) {
        scaleBar = 8;
    }
    else if (PDCurrent > RSSIBarLimit[8] && PDCurrent <= RSSIBarLimit[9]) {
        scaleBar = 9;
    }
    else if (PDCurrent > RSSIBarLimit[9]) {
        scaleBar = 10;
    }

    return scaleBar;
}

void deepSleep() {
    #define TFT_BL        4
    #define TFT_DISPOFF  0x28
    #define TFT_SLPIN  0x10
    
    int r = digitalRead(TFT_BL);
    digitalWrite(TFT_BL, !r);

    tft.writecommand(TFT_DISPOFF);
    tft.writecommand(TFT_SLPIN);
    //After using light sleep, you need to disable timer wake, because here use external IO port to wake up
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    // esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
    delay(200);
    esp_deep_sleep_start();
}