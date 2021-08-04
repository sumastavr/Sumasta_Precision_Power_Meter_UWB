#include <Arduino.h>

// SW Version
const String SWVersion = "V1.00 (28-07-21) P.Sumasta";

// Functions Prototypes
void espDelay(int ms);
void onReceiveData(const uint8_t* mac, const uint8_t* data, int len);
void initUWB();
void initFrame();
void sendData(int dataOut);
void showVoltage();
void updateThisDevice(int ADCin);
void sendDistanceInfotoTag(int distance);
void drawBarGraphRemote(int scale);
void updateDistanceTag(String Distance);
void updateDistanceTagNoSending(String Distance);
void updateRemoteDevice(int ADCin);
void deepSleep();
void drawBarGraphThis(int scale);
int GetScaleBar(float PDCurrent);