#include <Stepper.h>
//wifi
#include <WebServer.h>
//OTA
#include "ArduinoJson.h"
#include <WiFiMulti.h>
#include "esp32fota.h"
#include <WiFiClientSecure.h>
//Wifimanager
#include <AutoConnect.h>
//Time
#include <ESP32Time.h>



//Wifimanager variables
WebServer Server;
AutoConnect Portal(Server);
AutoConnectConfig Config;

//update variables
char* test_root_ca= \
     "-----BEGIN CERTIFICATE-----\n"  
  "MIIDxTCCAq2gAwIBAgIQAqxcJmoLQJuPC3nyrkYldzANBgkqhkiG9w0BAQUFADBsMQswCQYDVQQG\n"  
  "EwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3d3cuZGlnaWNlcnQuY29tMSsw\n"  
  "KQYDVQQDEyJEaWdpQ2VydCBIaWdoIEFzc3VyYW5jZSBFViBSb290IENBMB4XDTA2MTExMDAwMDAw\n"  
  "MFoXDTMxMTExMDAwMDAwMFowbDELMAkGA1UEBhMCVVMxFTATBgNVBAoTDERpZ2lDZXJ0IEluYzEZ\n"  
  "MBcGA1UECxMQd3d3LmRpZ2ljZXJ0LmNvbTErMCkGA1UEAxMiRGlnaUNlcnQgSGlnaCBBc3N1cmFu\n"  
  "Y2UgRVYgUm9vdCBDQTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMbM5XPm+9S75S0t\n"  
  "Mqbf5YE/yc0lSbZxKsPVlDRnogocsF9ppkCxxLeyj9CYpKlBWTrT3JTWPNt0OKRKzE0lgvdKpVMS\n"  
  "OO7zSW1xkX5jtqumX8OkhPhPYlG++MXs2ziS4wblCJEMxChBVfvLWokVfnHoNb9Ncgk9vjo4UFt3\n"  
  "MRuNs8ckRZqnrG0AFFoEt7oT61EKmEFBIk5lYYeBQVCmeVyJ3hlKV9Uu5l0cUyx+mM0aBhakaHPQ\n"  
  "NAQTXKFx01p8VdteZOE3hzBWBOURtCmAEvF5OYiiAhF8J2a3iLd48soKqDirCmTCv2ZdlYTBoSUe\n"  
  "h10aUAsgEsxBu24LUTi4S8sCAwEAAaNjMGEwDgYDVR0PAQH/BAQDAgGGMA8GA1UdEwEB/wQFMAMB\n"  
  "Af8wHQYDVR0OBBYEFLE+w2kD+L9HAdSYJhoIAu9jZCvDMB8GA1UdIwQYMBaAFLE+w2kD+L9HAdSY\n"  
  "JhoIAu9jZCvDMA0GCSqGSIb3DQEBBQUAA4IBAQAcGgaX3NecnzyIZgYIVyHbIUf4KmeqvxgydkAQ\n"  
  "V8GK83rZEWWONfqe/EW1ntlMMUu4kehDLI6zeM7b41N5cdblIZQB2lWHmiRk9opmzN6cN82oNLFp\n"  
  "myPInngiK3BD41VHMWEZ71jFhS9OMPagMRYjyOfiZRYzy78aG6A9+MpeizGLYAiJLQwGXFK3xPkK\n"  
  "mNEVX58Svnw2Yzi9RKR/5CYrCsSXaQ3pjOLAEFe4yHYSkVXySGnYvCoCWw9E1CAx2/S6cCZdkGCe\n"  
  "vEsXCS+0yx5DaMkHJ8HSXPfqIbloEpw8nL+e/IBcm2PN7EeqJSdnoDfzAIJ9VNep+OkuE6N36B9K\n" 
  "-----END CERTIFICATE-----\n" ;

WiFiClientSecure clientForOta;
secureEsp32FOTA secureEsp32FOTA("CalendarV1", 1);



//Deepsleep variables
RTC_DATA_ATTR long currentTime;
#define uS_TO_S_FACTOR 1000000ULL //to multiply with seconds for deep sleep wakeup so it is easyer readable
int TIME_TO_SLEEP;         //amount of seconds to go to deep sleep
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;
const char * defaultTimezone = "CET-1CEST,M3.5.0/02,M10.5.0/03";
ESP32Time rtc;

//Hardware variables
//stepper variables
const int stepsPerRevolution = 2048;
Stepper Stepper4 = Stepper(stepsPerRevolution, 33, 35, 34, 36);
Stepper Stepper3 = Stepper(stepsPerRevolution, 37, 39, 38, 40);
Stepper Stepper2 = Stepper(stepsPerRevolution, 41, 43, 42, 44);
Stepper Stepper1 = Stepper(stepsPerRevolution, 1, 3, 2, 4);
int sStepsnumber = 714; //step number for small gear (inner ones) per number
int bStepsnumber = 583; //step number for Big gear (outer ones) per number

RTC_DATA_ATTR int StepperPosition[4] = {1, 1, 1, 1}; //These dont stand for the Numbers displayed instead for the Number position counted from the Bottom up
// pices with numbers on them
int ZeroToNine[] = {0, 9, 8, 7, 6, 5, 4, 3, 2, 1};
int ZeroToThree[] = {3, 2, 1, 0};
int ZeroToOne[] = {3, 2, 1, 0}; //three stands for the Connection symbol and 2 for the Battery symbole

//Endstop variables
int Endstop1 = 9;
int Endstop2 = 10;
int Endstop3 = 11;
int Endstop4 = 12;

//other stuff
int Boostconverter = 45; //Boostconverter enable(5v for motor output)
int adc = 5;             //Read voltage from pin
int adcEnable = 6;       //mosfet for voltage divider as it would always cusome power if enabled(required for adc reading)

void setup()
{
  Serial.begin(115200);
  delay(5000);
  //set as the Enable Pin for Boostconverter as output
  pinMode(Boostconverter, OUTPUT);
  //update

  
  //if speed is not set the motors wont work
  Stepper1.setSpeed(12);
  Stepper2.setSpeed(12);
  Stepper3.setSpeed(12);
  Stepper4.setSpeed(12);

  checkVoltage();


  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER)
  {
    Serial.println("Wakeup by timer! Time:");
    Serial.println(currentTime);

    rtc.setTime(currentTime);
    delay(5000);


    if (rtc.getDayofWeek() == 4) //To save power only resync time every 4th weekday
    {
      WifiConnectAndTime();
      Serial.println("the 4th weekday! Clock will be synced");
    }
  }


  else
  {
    Serial.println("Calendar was manually reset ");
    home1();
    home2();
    home3();
    home4();

    //Connect to Wifi
    WifiConnectAndTime();
  }
  Serial.println(rtc.getDateTime());
  int firstNumber =  String(rtc.getDay()).substring(0, 1).toInt();
  int secondNumber = String(rtc.getDay()).substring(1).toInt();
  int thirdNumber = String(rtc.getMonth()+1).substring(0, 1).toInt();
  int fourthNumber = String(rtc.getMonth()+1).substring(1).toInt();
  Serial.println(firstNumber);
  Serial.println(secondNumber);
  Serial.println("will set number now");
  setNumber(firstNumber, secondNumber, thirdNumber, fourthNumber);


  TIME_TO_SLEEP = (60-rtc.getMinute())*60 + (24-rtc.getHour(true))*3600 + 60; //Calculte seconds till midnight

  currentTime  = rtc.getEpoch() + TIME_TO_SLEEP;
  Serial.print(currentTime);
  esp_deep_sleep(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void loop(){

}
void home1()
{
  digitalWrite(Boostconverter, HIGH);
  while (digitalRead(Endstop1) == LOW)
  {
    Stepper1.step(-1);
  }
  Stepper1.step(200);

  while (digitalRead(Endstop1) == LOW)
  {
    Stepper1.step(-1);
  }
  Stepper1.step(270);
  digitalWrite(Boostconverter, LOW);

  StepperPosition[0] = 0;
}
void home2()
{
  digitalWrite(Boostconverter, HIGH);
  while (digitalRead(Endstop2) == LOW)
  {
    Stepper2.step(-1);
  }
  Stepper2.step(200);

  while (digitalRead(Endstop2) == LOW)
  {
    Stepper2.step(-1);
  }
  Stepper2.step(180);
  digitalWrite(Boostconverter, LOW);
  StepperPosition[1] = 0;
}
void home3()
{
  digitalWrite(Boostconverter, HIGH);

  while (digitalRead(Endstop3) == LOW)
  {
    Stepper3.step(1);
  }
  Stepper3.step(-200);

  while (digitalRead(Endstop3) == LOW)
  {
    Stepper3.step(1);
  }
  Stepper3.step(-280);
  digitalWrite(Boostconverter, LOW);

  StepperPosition[2] = 0;
}
void home4()
{
  digitalWrite(Boostconverter, HIGH);

  while (digitalRead(Endstop4) == LOW)
  {
    Stepper4.step(1);
  }
  Stepper4.step(-200);
  while (digitalRead(Endstop4) == LOW)
  {
    Stepper4.step(1);
  }
  Stepper4.step(-140);
  digitalWrite(Boostconverter, LOW);

  StepperPosition[3] = 0;
}

void setNumber(int Number1, int Number2, int Number3, int Number4)
{

  if ((ZeroToThree[Number1] == 0) && (StepperPosition[0] != 0))
  {
    home1();
  } //home itself if it goes to the lowest position
  if ((ZeroToNine[Number2] == 0) && (StepperPosition[1] != 0))
  {
    home2();
  }
  if ((ZeroToOne[Number3] == 0) && (StepperPosition[2] != 0))
  {
    home3();
  }
  if ((ZeroToThree[Number4] == 0) && (StepperPosition[3] != 0))
  {
    home4();
  }

  digitalWrite(Boostconverter, HIGH);
  delay(200);

  //stepper1

  int difference = ZeroToThree[Number1] - StepperPosition[0]; //Calculate amount of Numbers between is and want to move this amount times steps for one Number

  Stepper1.step(difference * bStepsnumber);

  StepperPosition[0] = ZeroToThree[Number1];

  if (difference < 0)
  {
    Stepper1.step(-100);
    Stepper1.step(100);
  }
  //stepper 2

  difference = ZeroToNine[Number2] - StepperPosition[1]; //Calculate amount of Numbers between is and want to move this amount times steps for one Number

  Stepper2.step(difference * sStepsnumber);

  StepperPosition[1] = ZeroToNine[Number2];

  if (difference < 0)
  {
    Stepper2.step(-100);
    Stepper2.step(100);
  }

  //stepper 3

  difference = ZeroToOne[Number3] - StepperPosition[2]; //Calculate amount of Numbers between is and want to move this amount times steps for one Number

  Stepper3.step(difference * -sStepsnumber);

  StepperPosition[2] = ZeroToOne[Number3];

  if (difference < 0)
  {
    Stepper3.step(100);
    Stepper3.step(-100);
  }

  //stepper 4

  difference = ZeroToNine[Number4] - StepperPosition[3]; //Calculate amount of Numbers between is and want to move this amount times steps for one Number

  Stepper4.step(difference * -bStepsnumber);

  StepperPosition[3] = ZeroToNine[Number4];

  if (difference < 0)
  {
    Stepper4.step(100);
    Stepper4.step(-100);
  }

  digitalWrite(Boostconverter, LOW);
}

void WifiConnectAndTime()
{
  Serial.println("will connect to wifi and refresh time");
  //Connect to Wifi with Autoconnect libary
  Config.title = "Kalender";
  Config.homeUri = "AUTOCONNECT_URI_CONNECT"; //sets the path which
  Config.apid = "Kalender";                   //Acces Point Name
  Config.psk = "";                            //Acces Point Password here it will have no password
  Config.menuItems = AC_MENUITEM_CONFIGNEW | AC_MENUITEM_OPENSSIDS;
  Config.autoReconnect = true;
  Config.portalTimeout = 300000;  // It will time out in 60 seconds
  Portal.config(Config);
  Portal.begin();
  delay(100);
  if (WiFi.status() != WL_CONNECTED) {
      setNumber(0, 0, 3, 0);                             
      esp_deep_sleep_start();
  }
  Serial.println(WiFi.RSSI());
  //check for new updates and do them if necassary
  checkUpdate();
  //Get Time over WIFI
  configTzTime( defaultTimezone, ntpServer);
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)){
    rtc.setTimeStruct(timeinfo); 
  }
  delay(5000);
}

void checkVoltage(){
pinMode(adcEnable,OUTPUT);
digitalWrite(adcEnable, HIGH);
if(analogRead(5)*2.55/8192*2 < 3.3){
  setNumber(3, 0, 2, 0);                                                    //only home if not done before(just for safety and could probaply be removed )
  esp_deep_sleep_start();

}
}

void checkUpdate(){
  Serial.println("Checking for Updates");
  secureEsp32FOTA._host="raw.githubusercontent.com";
  secureEsp32FOTA._descriptionOfFirmwareURL="/1JonaB1/Calendar/main/ota%20update/firmware.json";
  secureEsp32FOTA._certificate=test_root_ca;
  secureEsp32FOTA.clientForOta=clientForOta;

  bool shouldExecuteFirmwareUpdate=secureEsp32FOTA.execHTTPSCheck();
  if(shouldExecuteFirmwareUpdate)
  {
    Serial.println("Firmware update available!");
    secureEsp32FOTA.executeOTA();
  }
}