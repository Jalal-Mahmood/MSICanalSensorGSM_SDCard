
/*
  Bridge Sensor with GSM, SDCard, thermistor, US and RTC.

  Sketch used by MSI Sensors platform.

  Created 04JUL16
*/
#include <ARTF_SDCard.h>
#include <SdFat.h>
#include <LowPower.h>
#include <math.h>

#include <Time.h>  /////////////Jalal Changes////////////////Beside RTCLib there must also by Time library. It includes time_t type where we can store the unix datetime type

#include <Adafruit_FONA.h> /////////////////new Changes/////////////// include fona

//#include "Sim800.h"
//#include <SD.h>
#include <SPI.h>


// RTC Dependency
#include <Wire.h>
#include <RTClib.h>

// Analog Pins
#define THERMISTOR_PIN A1
#define ULTRASONIC_PIN A2

//Analog pins for HC-SR04. Block these if using Maxbotix US sensor.
//int echoPin = 13;
//int trigPin = 12;
//int duration;
//int cm;

//GSM pins
#define FONA_RX 9
#define FONA_TX 8
#define FONA_RST 4
#define FONA_RI 7
#define FONA_PWR 11
// this is a large buffer for replies
char replybuffer[255];

// Digital Pins
// Digital Pin for Maxbotix US sensor. Block if using HC-SR04.
#define US_PIN    12
//GSM DTR power pin. Adafruit documentation says to cut the KEY to use this. Otherwise GMS always remains on.
#define GSM_PIN   5
//Thermistor power pin.
#define THERM_PIN        6
//GMS Power Key. Verify if this is needed.
//#define GSM_PWRKEY       9
//SD CS pin. Seemed to work on pin 10.
#define SD_CS_PIN        10


// Settings
// -------------------------------
//Distance from sensor to river bed in centimeters.
#define SENSOR_TO_RIVER_BED        200
//Sensor number. Jalal's unique identifier.
#define SENSOR_NUM                "1"
//Number of readings per text message.
#define SEND_DATA_AFTER_X_READINGS 1
//Each sleep cycle is approximately 8 seconds.
#define SLEEP_CYCLES               480
//Number of thermistor readings to be averaged.
#define NUM_THERM_READINGS         5
//Delay in miliseconds between temperature readings.
#define THERM_READING_DELAY        20
//Number of distance readings to be averaged.
#define NUM_DISTANCE_READINGS      3
//Delay in milliseconds between distance readings.
#define DISTANCE_READING_DELAY     200
//Character type between data points.
#define DATA_DELIM                 ';'
#define BACKUP_FILENAME            "backup.csv"
#define UNSENT_FILENAME            "unsent.csv"
#define ERROR_FILENAME             "error.csv"
// Google Voice phone number for msi.artf2@gmail.com
//#define PHONE_NUMBER               "+13027152285"
//char PHONE_NUMBER[21] = "+19072236094";
//char PHONE_NUMBER[21] = "+93771515622"; // GoogleVoice for msi.artf2@gmail.com
char PHONE_NUMBER[21] = "282"; // GoogleVoice for msi.artf2@gmail.com
char EMAIL_ID[121] = "COM msi.artf2@gmail.com ";

#define ERROR_GSM                  "GSM Failed"
#define ERROR_SMS                  "SMS Failed"

#define SD_CS_PIN                   10
ARTF_SDCard sd(SD_CS_PIN);
//Fona library requirment
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);


// Custom Datatypes
typedef struct {
  int distance;
  int temperature;
  time_t timestamp;
} SensorReading;


// Global Variables
int numCachedReadings = 0;
int totalReadings = 0;
SensorReading sensorReadings[SEND_DATA_AFTER_X_READINGS];
RTC_PCF8523 rtc;


void setup()
{
  // SC card CS pin defined.
  pinMode(SD_CS_PIN, OUTPUT);
  //Thermistor pin defined.
  pinMode(THERM_PIN, OUTPUT);
  // PinMode settings for HC-SR04 ultrasonic sensor. Block these if using Maxbotix US sensor.
  //pinMode(trigPin, OUTPUT);
  //pinMode(echoPin, INPUT);
  //PinMode settings for Maxbotix ultrasonic sensor.
  pinMode(US_PIN, OUTPUT);
  pinMode(FONA_RST, OUTPUT);
  pinMode(FONA_PWR, OUTPUT);

  //Turn on GSM.
  digitalWrite(FONA_PWR, HIGH);
  delay(2000);
  digitalWrite(FONA_PWR, LOW);
  delay(15000);

  while (!Serial);

  Serial.begin(115200);
  Serial.println(F("FONA SMS caller ID test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  Serial.println(F("FONA is OK"));

  delay(30000);
  char message[121] = "COM msi.artf2@gmail.com GSM Testing!";

  if (!fona.sendSMS(PHONE_NUMBER, message)) {
    Serial.println(F("Not sent!"));
  } else {
    Serial.println(F("Sent"));
  }

}


void loop()
{
  //Serial.println(F("Wake up loop."));
  // 1. Wake up.
  // -----------

  // Enter idle state for 8 s increments with the rest of peripherals turned off.
  for (int i = 0; i < SLEEP_CYCLES; ++i)
  {
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF,
                  TIMER0_OFF, SPI_OFF, USART1_OFF, TWI_OFF, USB_OFF);
  }

  // 2. Turn on thermistor.
  // ----------------------
  digitalWrite(THERM_PIN, HIGH);

  // 3. Take 5 thermistor readings. (one every 20ms)
  // -----------------------------------------------
  int thermReadings[NUM_THERM_READINGS];
  for (int i = 0; i < NUM_THERM_READINGS; ++i)
  {
    thermReadings[i] = analogRead(THERMISTOR_PIN);
    delay(THERM_READING_DELAY);
  }

  // 4. Turn off thermistor.
  // -----------------------
  digitalWrite(THERM_PIN, LOW);
  delay(500);

  // 5. Average 5 thermistor readings.
  // ---------------------------------
  double sumTherm = 0;
  for (int i = 0; i < NUM_THERM_READINGS; ++i)
  {
    sumTherm += thermReadings[i];
  }
  double avgTherm = sumTherm / NUM_THERM_READINGS;
  avgTherm = 1023 / avgTherm - 1;
  double R = 10000 / avgTherm;


  // 6. Convert average thermistor reading into temperature.
  // -------------------------------------------------------

  // Steinhart-Hart, modified:
  // Measure the actual value of the 10K resistor for a more accurate reading.
  double avgTemperature = ( 3950.0 / (log( R / (10000.0 * exp( -3950.0 / 298.13 ) ) ) ) ) - 273.13;


  //At room temperature, sound velocity is approximately 29 cm/ms. Divide by 2 to account for return time.
  //Doing this to simplify sketch and do temperature augmentation for either US sensor with same code.
  //Only one reading with method.
  delay(1000);

  //Maxbotix US sensor. Block if using HC-SR04.
  //Using Maxbotix pin 4 to trigger power to US. LOW = off, HIGH = on.
  digitalWrite(US_PIN, HIGH);
  // Calibration time
  delay(2000);


  // 8. Take 3 distance readings. (One every 200ms)
  // ----------------------------------------------
  int distanceReadings[NUM_DISTANCE_READINGS];
  for (int i = 0; i < NUM_DISTANCE_READINGS; ++i)
  {
    distanceReadings[i] = analogRead(ULTRASONIC_PIN);
    delay(DISTANCE_READING_DELAY);
  }


  // 9. Turn off Maxbotix ultrasonic sensor.
  // ---------------------------------------
  digitalWrite(US_PIN, LOW);
  delay(500);


  // 10. Average 3 distance measurements.
  // ------------------------------------
  double sumDistance = 0.0;
  for (int i = 0; i < NUM_DISTANCE_READINGS; ++i)
  {
    sumDistance += distanceReadings[i];
  }
  double avgDistance = sumDistance / NUM_DISTANCE_READINGS;


  // 11. Use average temperature to calculate actual distance.
  // ---------------------------------------------------------
  double adjustedDistance = SENSOR_TO_RIVER_BED - ( ( 331.1 + .6 * avgTemperature ) / 344.5 ) * avgDistance;

  int roundedDistance = round(adjustedDistance);
  int roundedTemperature = round(avgTemperature);


  // 12. Get time from RTC Shield.
  // -----------------------------
  Wire.begin();
  rtc.begin();

  //DateTime unixTime = now.unixTime();
  DateTime   unixTime = rtc.now(); ///////////////Jalal Changes//////////////// it is rtc.now(); not now.unixTime();

  // 12b. Get battery voltage and percentage
  uint16_t vbat;
  fona.getBattPercent(&vbat);



  // 18. Prepare text message
  // ---------------------
  String textMessage = String(EMAIL_ID) + " " +
                       String(SENSOR_NUM) + " " +
                       String(vbat) + " " +
                       //                      String(sensorReadings[0].timestamp) + " " +
                       String(unixTime.unixtime()) + " " +
                       //                      String(sensorReadings[0].distance) + " " +
                       String(roundedDistance) + " " +
                       //                      String(sensorReadings[0].temperature);
                       String(roundedTemperature);


  char message[121];
  strncpy(message, textMessage.c_str(), sizeof(message));
  message[sizeof(message) - 1] = 0;

  //Serial.println(message);
  // fona.sendSMS(PHONE_NUMBER, message);

  sd.begin();

  if (!fona.sendSMS(PHONE_NUMBER, message)) {
    Serial.println(F("Not sent!"));
    if (!sd.writeFile(UNSENT_FILENAME, message) ) {
      Serial.print("SD Card not available\n");
    } else {
      Serial.print("Sd card available\n");
    }
  } else {
    Serial.println(F("Sent"));
    if (!sd.writeFile(BACKUP_FILENAME, message)) {
      Serial.print("SD Card not available\n");
    } else {
      Serial.print("Sd card available\n");
    }
  }

  delay(10000);

}

