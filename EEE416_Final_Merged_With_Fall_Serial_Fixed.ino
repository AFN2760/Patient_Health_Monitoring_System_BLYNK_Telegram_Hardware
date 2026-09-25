// ============================================================
// ESP32 - COMBINED HEALTH MONITOR
// WITH BLYNK + TELEGRAM ALERT SYSTEM
// + EXHAUST FAN
// + RGB STATUS LED
// + MPU6050 FALL DETECTION
// + GPIO32 BUZZER PWM
//
// HEART RATE:
//   checkForBeat() from heartRate.h
//
// SpO2:
//   maxim_heart_rate_and_oxygen_saturation()
//
// I2C CONNECTION:
//   MAX30102 + OLED + MPU6050
//   SDA -> GPIO21
//   SCL -> GPIO22
//
// RELAY:
//   GPIO27
//   3.3V ACTIVE-HIGH
//
// RGB:
//   RED   -> GPIO25
//   GREEN -> GPIO26
//   BLUE  -> GPIO33
//
// Serial Monitor Baud Rate: 115200
// ============================================================


// ============================================================
// BLYNK CREDENTIALS
// ============================================================

#define BLYNK_TEMPLATE_ID "TMPL6mXPf7Qcs"
#define BLYNK_TEMPLATE_NAME "ESP32 Health Monitor"
#define BLYNK_AUTH_TOKEN "z_ZNLrX0_nK3q5rhDEYPGbdx5YcKth5S"

#define BLYNK_PRINT Serial


// ============================================================
// LIBRARIES
// ============================================================

#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>

#include <BlynkSimpleEsp32.h>

#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

#include <time.h>

#include <DHT.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// ============================================================
// WI-FI CREDENTIALS
// ============================================================

const char WIFI_SSID[] = "TP-Link_4BA8";
const char WIFI_PASSWORD[] = "15644237";


// ============================================================
// TELEGRAM CREDENTIALS
// ============================================================

#define TELEGRAM_BOT_TOKEN "8846879103:AAHpF85pLJjEildBfafTFk-3m1xWYwOfX9k"
#define TELEGRAM_CHAT_ID   "1906197123"


WiFiClientSecure telegramClient;

UniversalTelegramBot telegramBot(
  TELEGRAM_BOT_TOKEN,
  telegramClient
);


// ============================================================
// SIX ALERT THRESHOLDS
// ============================================================

// 1. MQ2

int MQ2_HIGH_THRESHOLD = 800;


// 2. Room temperature

float ROOM_TEMP_HIGH_THRESHOLD = 35.0;


// 3. Room humidity

float ROOM_HUMIDITY_HIGH_THRESHOLD = 95.0;


// 4. Body temperature

float BODY_TEMP_HIGH_THRESHOLD = 33.0;


// 5. Heart rate

int HEART_RATE_HIGH_THRESHOLD = 200;


// 6. SpO2

float SPO2_LOW_THRESHOLD = 98.0;


// ============================================================
// TELEGRAM ALERT STATES
// ============================================================

bool mq2AlertActive = false;

bool roomTempAlertActive = false;

bool roomHumidityAlertActive = false;

bool bodyTempAlertActive = false;

bool heartRateAlertActive = false;

bool spo2AlertActive = false;


// ============================================================
// BLYNK VIRTUAL PIN MAP
// ============================================================

#define VPIN_MQ2              V0
#define VPIN_ROOM_TEMPERATURE V1
#define VPIN_ROOM_HUMIDITY    V2
#define VPIN_BODY_TEMPERATURE V3
#define VPIN_HEART_RATE       V4
#define VPIN_SPO2             V5


// ============================================================
// BLYNK / NETWORK / TELEGRAM TIMING
// ============================================================

BlynkTimer blynkTimer;

const unsigned long BLYNK_UPLOAD_INTERVAL = 1000;

const unsigned long TELEGRAM_CHECK_INTERVAL = 2000;

const unsigned long WIFI_CONNECT_TIMEOUT = 15000;

const unsigned long RECONNECT_INTERVAL = 10000;


unsigned long lastWiFiReconnectAttempt = 0;

unsigned long lastBlynkReconnectAttempt = 0;


// ============================================================
// OLED
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define OLED_FIRST_LINE_Y 16
#define OLED_LINE_SPACING 10


Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);


// ============================================================
// MQ2
// ============================================================

#define MQ2_PIN 35

int mq2ADC = 0;


// ============================================================
// DHT22
// ============================================================

#define DHT_PIN 13
#define DHT_TYPE DHT22


DHT dht(
  DHT_PIN,
  DHT_TYPE
);


float roomTemperature = NAN;

float roomHumidity = NAN;


// ============================================================
// DS18B20
// ============================================================

#define ONE_WIRE_BUS 4


OneWire oneWire(
  ONE_WIRE_BUS
);


DallasTemperature ds18b20(
  &oneWire
);


float bodyTemperature = NAN;


unsigned long dsRequestTime = 0;

const unsigned long DS_CONVERSION_TIME = 750;


// ============================================================
// I2C PINS
//
// MAX30102 + OLED + MPU6050 SHARE THIS BUS
// ============================================================

#define SDA_PIN 21
#define SCL_PIN 22


// ============================================================
// EXHAUST FAN RELAY
//
// 3.3V ACTIVE-HIGH RELAY
//
// LOW  = Relay OFF = Fan OFF
// HIGH = Relay ON  = Fan ON
// ============================================================

#define RELAY_PIN 27


// ============================================================
// RGB LED
//
// Assumes COMMON-CATHODE RGB LED
//
// HIGH = LED color ON
// LOW  = LED color OFF
// ============================================================

#define RGB_RED_PIN   25
#define RGB_GREEN_PIN 26
#define RGB_BLUE_PIN  33


// ============================================================
// MPU6050 FALL DETECTION + BUZZER
//
// MPU6050 shares the same I2C bus:
// SDA -> GPIO21
// SCL -> GPIO22
//
// Buzzer I/O -> GPIO32
// RGB red     -> GPIO25
// ============================================================

#define BUZZER_PWM_PIN 32
#define BUZZER_PWM_FREQ 1000
#define BUZZER_PWM_RESOLUTION 8
#define BUZZER_PWM_DUTY 179       // 70% of 255

MPU6050 mpu;

const float ACCEL_SCALE = 16384.0;       // ±2 g
const float GYRO_SCALE = 131.0;          // ±250 deg/s
const float DEG_TO_RAD_VALUE = 0.01745329252;

const float FREE_FALL_THRESHOLD = 0.60;  // g
const float IMPACT_THRESHOLD = 2.0;      // g
const float GYRO_THRESHOLD = 2.5;        // rad/s

const unsigned long FALL_WINDOW = 1500;       // ms
const unsigned long FALL_ALARM_DURATION = 5000; // ms
const unsigned long MPU_READ_INTERVAL = 50;   // 20 readings/s

bool freeFallDetected = false;
unsigned long freeFallTime = 0;

bool fallAlarmActive = false;
unsigned long fallAlarmStartTime = 0;

bool fallTelegramPending = false;

float fallAccelerationAtDetection = 0.0;
float fallGyroAtDetection = 0.0;

// Latest MPU6050 values for the 1-second Serial Monitor summary
float latestAcceleration = 0.0;
float latestGyro = 0.0;

unsigned long lastMPURead = 0;


// ============================================================
// MAX30102
// ============================================================

MAX30105 particleSensor;


// ============================================================
// FINGER DETECTION
//
// No finger ~1000
// Finger    ~200000
// ============================================================

const uint32_t FINGER_IR_THRESHOLD = 50000;


uint32_t latestIR = 0;


// ============================================================
// SpO2 BUFFER
//
// Maxim algorithm expects 100 samples at 25 samples/sec.
// ============================================================

uint32_t irBuffer[100];

uint32_t redBuffer[100];


int32_t bufferLength = 100;


int32_t spo2 = 0;

int8_t validSPO2 = 0;


// Maxim function requires HR output variables.
// We deliberately DO NOT use them.
// BPM comes from checkForBeat().
// ============================================================

int32_t maximHeartRate = 0;

int8_t maximValidHeartRate = 0;


// ============================================================
// HEART RATE - checkForBeat()
// ============================================================

const byte RATE_SIZE = 4;

byte rates[RATE_SIZE];

byte rateSpot = 0;

byte rateCount = 0;


unsigned long lastBeat = 0;


float instantBPM = 0;


int32_t heartRate = 0;

bool validHeartRate = false;


// ============================================================
// MAX30102 SAMPLE SETTINGS
//
// HR detector = 100 raw samples/sec.
//
// SpO2:
// Every 4 raw samples averaged.
//
// 100 / 4 = 25 samples/sec
// ============================================================

const byte SPO2_AVERAGE_FACTOR = 4;


// ============================================================
// SENSOR / DISPLAY TIMING
// ============================================================

unsigned long lastDHTRead = 0;

unsigned long lastDisplayUpdate = 0;


const unsigned long DHT_INTERVAL = 2000;

const unsigned long DISPLAY_INTERVAL = 1000;


// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void initializeNetworkAndBlynk();

void maintainNetworkAndBlynk();

void runBlynkIfConnected();

void sendSensorDataToBlynk();


void initializeTelegram();

void checkTelegramAlerts();

bool telegramNetworkReady();


void resetHeartRate();

void processHeartRateSample(uint32_t irValue);

void collectOneSpO2Sample(
  uint32_t &redValue,
  uint32_t &irValue
);


void updateFanAndRGB();

void initializeMPU6050();
void serviceFallDetection();
void triggerFallAlert(
  float totalAcceleration,
  float totalGyro
);


// ============================================================
// RESET HEART RATE
// ============================================================

void resetHeartRate()
{

  lastBeat = 0;

  instantBPM = 0;

  heartRate = 0;

  validHeartRate = false;

  rateSpot = 0;

  rateCount = 0;


  for (
    byte i = 0;
    i < RATE_SIZE;
    i++
  )
  {

    rates[i] = 0;

  }

}


// ============================================================
// PROCESS ONE RAW IR SAMPLE FOR BPM
// ============================================================

void processHeartRateSample(
  uint32_t irValue
)
{

  latestIR =
    irValue;


  // ==========================================================
  // NO FINGER
  // ==========================================================

  if (
    irValue <
    FINGER_IR_THRESHOLD
  )
  {

    resetHeartRate();

    return;

  }


  // ==========================================================
  // CHECK FOR HEARTBEAT
  // ==========================================================

  if (
    checkForBeat(
      (int32_t)irValue
    )
  )
  {

    unsigned long currentBeatTime =
      millis();


    if (
      lastBeat != 0
    )
    {

      unsigned long delta =
        currentBeatTime -
        lastBeat;


      if (
        delta > 0
      )
      {

        instantBPM =
          60000.0 /
          delta;


        if (
          instantBPM > 20 &&
          instantBPM < 220
        )
        {

          rates[rateSpot] =
            (byte)(
              instantBPM +
              0.5
            );


          rateSpot++;


          if (
            rateSpot >=
            RATE_SIZE
          )
          {

            rateSpot = 0;

          }


          if (
            rateCount <
            RATE_SIZE
          )
          {

            rateCount++;

          }


          int total = 0;


          for (
            byte i = 0;
            i < rateCount;
            i++
          )
          {

            total +=
              rates[i];

          }


          heartRate =
            total /
            rateCount;


          validHeartRate =
            true;

        }

      }

    }


    lastBeat =
      currentBeatTime;

  }

}


// ============================================================
// COLLECT ONE 25-Hz SAMPLE FOR SpO2
//
// While MAX30102 data is being collected, serviceFallDetection()
// is also called. This is important because the SpO2 routine
// otherwise occupies about one second and could miss a fall.
// ============================================================

void collectOneSpO2Sample(
  uint32_t &redValue,
  uint32_t &irValue
)
{
  uint64_t redSum = 0;
  uint64_t irSum = 0;

  for (
    byte sample = 0;
    sample < SPO2_AVERAGE_FACTOR;
    sample++
  )
  {
    while (
      particleSensor.available() ==
      false
    )
    {
      particleSensor.check();

      runBlynkIfConnected();

      serviceFallDetection();
    }

    uint32_t currentRed =
      particleSensor.getRed();

    uint32_t currentIR =
      particleSensor.getIR();

    processHeartRateSample(
      currentIR
    );

    redSum +=
      currentRed;

    irSum +=
      currentIR;

    particleSensor.nextSample();

    // Keep MPU6050 sampling close to 20 Hz
    serviceFallDetection();
  }

  redValue =
    redSum /
    SPO2_AVERAGE_FACTOR;

  irValue =
    irSum /
    SPO2_AVERAGE_FACTOR;
}


// ============================================================
// MPU6050 INITIALIZATION
// ============================================================

void initializeMPU6050()
{
  mpu.initialize();

  if (
    !mpu.testConnection()
  )
  {
    Serial.println(
      "MPU6050 connection failed."
    );

    while (true)
    {
      delay(10);
    }
  }

  // Accelerometer ±2 g
  mpu.setFullScaleAccelRange(
    MPU6050_ACCEL_FS_2
  );

  // Gyroscope ±250 deg/s
  mpu.setFullScaleGyroRange(
    MPU6050_GYRO_FS_250
  );

  // Configure GPIO32 for 1 kHz PWM
  ledcAttach(
    BUZZER_PWM_PIN,
    BUZZER_PWM_FREQ,
    BUZZER_PWM_RESOLUTION
  );

  // Buzzer initially OFF
  ledcWrite(
    BUZZER_PWM_PIN,
    0
  );

  Serial.println(
    "MPU6050 connected. Fall detection started."
  );
}


// ============================================================
// TRIGGER FALL RESPONSE
// ============================================================

void triggerFallAlert(
  float totalAcceleration,
  float totalGyro
)
{
  fallAccelerationAtDetection =
    totalAcceleration;

  fallGyroAtDetection =
    totalGyro;

  fallAlarmActive =
    true;

  fallAlarmStartTime =
    millis();

  fallTelegramPending =
    true;

  // Buzzer ON: 1 kHz, 70% duty
  ledcWrite(
    BUZZER_PWM_PIN,
    BUZZER_PWM_DUTY
  );

  // Force RGB RED immediately
  digitalWrite(
    RGB_RED_PIN,
    HIGH
  );

  digitalWrite(
    RGB_GREEN_PIN,
    LOW
  );

  digitalWrite(
    RGB_BLUE_PIN,
    LOW
  );

  Serial.println();
  Serial.println(
    "=========================="
  );
  Serial.println(
    "FALL DETECTED!"
  );
  Serial.print(
    "Impact Acceleration: "
  );
  Serial.print(
    totalAcceleration,
    2
  );
  Serial.println(
    " g"
  );
  Serial.print(
    "Gyroscope: "
  );
  Serial.print(
    totalGyro,
    2
  );
  Serial.println(
    " rad/s"
  );
  Serial.println(
    "Red LED + buzzer active for 5 seconds."
  );
  Serial.println(
    "Telegram alert queued."
  );
  Serial.println(
    "=========================="
  );
  Serial.println();

  // Reset fall sequence so a future fall can be detected
  freeFallDetected =
    false;
}


// ============================================================
// MPU6050 FALL DETECTION SERVICE
//
// Runs the MPU6050 fall logic at approximately 20 Hz.
// Recurring MPU values are shown in the complete 1-second
// Serial Monitor summary together with the other sensors.
// Immediate fall-detection messages are still printed here.
// ============================================================

void serviceFallDetection()
{
  unsigned long now =
    millis();

  // ----------------------------------------------------------
  // End the 5-second fall alarm
  // ----------------------------------------------------------

  if (
    fallAlarmActive &&
    now - fallAlarmStartTime >=
    FALL_ALARM_DURATION
  )
  {
    ledcWrite(
      BUZZER_PWM_PIN,
      0
    );

    fallAlarmActive =
      false;

    // Restore RGB according to the other six thresholds.
    updateFanAndRGB();

    Serial.println(
      "Fall 5-second alarm finished."
    );
  }

  // ----------------------------------------------------------
  // Read MPU6050 at approximately 20 Hz
  // ----------------------------------------------------------

  if (
    now - lastMPURead <
    MPU_READ_INTERVAL
  )
  {
    return;
  }

  lastMPURead =
    now;

  int16_t rawAx, rawAy, rawAz;
  int16_t rawGx, rawGy, rawGz;

  mpu.getMotion6(
    &rawAx,
    &rawAy,
    &rawAz,
    &rawGx,
    &rawGy,
    &rawGz
  );

  float ax =
    rawAx /
    ACCEL_SCALE;

  float ay =
    rawAy /
    ACCEL_SCALE;

  float az =
    rawAz /
    ACCEL_SCALE;

  float gxDegrees =
    rawGx /
    GYRO_SCALE;

  float gyDegrees =
    rawGy /
    GYRO_SCALE;

  float gzDegrees =
    rawGz /
    GYRO_SCALE;

  float gx =
    gxDegrees *
    DEG_TO_RAD_VALUE;

  float gy =
    gyDegrees *
    DEG_TO_RAD_VALUE;

  float gz =
    gzDegrees *
    DEG_TO_RAD_VALUE;

  float totalAcceleration =
    sqrt(
      ax * ax +
      ay * ay +
      az * az
    );

  float totalGyro =
    sqrt(
      gx * gx +
      gy * gy +
      gz * gz
    );

  // ----------------------------------------------------------
  // STORE LATEST MPU6050 VALUES
  //
  // Fall detection still runs at approximately 20 Hz, but the
  // values are printed with the complete Serial Monitor summary
  // once per second so the monitor remains readable.
  // ----------------------------------------------------------

  latestAcceleration =
    totalAcceleration;

  latestGyro =
    totalGyro;

  // ----------------------------------------------------------
  // STAGE 1: FREE FALL
  // ----------------------------------------------------------

  if (
    totalAcceleration <
    FREE_FALL_THRESHOLD &&
    !freeFallDetected
  )
  {
    freeFallDetected =
      true;

    freeFallTime =
      now;

    Serial.println(
      "Possible free fall detected..."
    );
  }

  // ----------------------------------------------------------
  // STAGE 2: IMPACT + ROTATION
  // ----------------------------------------------------------

  if (
    freeFallDetected
  )
  {
    if (
      totalAcceleration >
      IMPACT_THRESHOLD &&
      totalGyro >
      GYRO_THRESHOLD
    )
    {
      triggerFallAlert(
        totalAcceleration,
        totalGyro
      );

      return;
    }

    // Cancel if impact does not arrive within 1.5 s
    if (
      now - freeFallTime >
      FALL_WINDOW
    )
    {
      freeFallDetected =
        false;
    }
  }
}


// ============================================================
// EXHAUST FAN + RGB LED CONTROL
// ============================================================

void updateFanAndRGB()
{

  bool fingerDetected =
    latestIR >=
    FINGER_IR_THRESHOLD;


  // ==========================================================
  // CHECK THE SIX THRESHOLDS
  // ==========================================================

  bool mq2Abnormal =
    mq2ADC >
    MQ2_HIGH_THRESHOLD;


  bool roomTempAbnormal =
    !isnan(roomTemperature) &&
    roomTemperature >
    ROOM_TEMP_HIGH_THRESHOLD;


  bool roomHumidityAbnormal =
    !isnan(roomHumidity) &&
    roomHumidity >
    ROOM_HUMIDITY_HIGH_THRESHOLD;


  bool bodyTempAbnormal =
    !isnan(bodyTemperature) &&
    bodyTemperature >
    BODY_TEMP_HIGH_THRESHOLD;


  bool heartRateAbnormal =
    fingerDetected &&
    validHeartRate &&
    heartRate >
    HEART_RATE_HIGH_THRESHOLD;


  bool spo2Abnormal =
    fingerDetected &&
    validSPO2 &&
    spo2 <
    SPO2_LOW_THRESHOLD;


  // ==========================================================
  // ANY THRESHOLD CROSSED?
  // ==========================================================

  bool anyAbnormal =
    mq2Abnormal ||
    roomTempAbnormal ||
    roomHumidityAbnormal ||
    bodyTempAbnormal ||
    heartRateAbnormal ||
    spo2Abnormal ||
    fallAlarmActive;


  // ==========================================================
  // EXHAUST FAN CONTROL
  //
  // ONLY MQ2 controls the fan.
  //
  // MQ2 normal:
  // Relay LOW -> Fan OFF
  //
  // MQ2 threshold crossed:
  // Relay HIGH -> Fan ON
  // ==========================================================

  if (
    mq2Abnormal
  )
  {

    digitalWrite(
      RELAY_PIN,
      HIGH
    );

  }

  else
  {

    digitalWrite(
      RELAY_PIN,
      LOW
    );

  }


  // ==========================================================
  // RGB LED CONTROL
  //
  // Everything normal:
  // GREEN
  //
  // Any threshold crossed:
  // RED
  //
  // BLUE currently unused.
  // ==========================================================

  if (
    anyAbnormal
  )
  {

    // RED ON

    digitalWrite(
      RGB_RED_PIN,
      HIGH
    );


    digitalWrite(
      RGB_GREEN_PIN,
      LOW
    );


    digitalWrite(
      RGB_BLUE_PIN,
      LOW
    );

  }

  else
  {

    // GREEN ON

    digitalWrite(
      RGB_RED_PIN,
      LOW
    );


    digitalWrite(
      RGB_GREEN_PIN,
      HIGH
    );


    digitalWrite(
      RGB_BLUE_PIN,
      LOW
    );

  }

}


// ============================================================
// BLYNK - SEND SENSOR VALUES
// ============================================================

void sendSensorDataToBlynk()
{

  if (
    !Blynk.connected()
  )
  {

    return;

  }


  Blynk.virtualWrite(
    VPIN_MQ2,
    mq2ADC
  );


  if (
    !isnan(
      roomTemperature
    )
  )
  {

    Blynk.virtualWrite(
      VPIN_ROOM_TEMPERATURE,
      roomTemperature
    );

  }


  if (
    !isnan(
      roomHumidity
    )
  )
  {

    Blynk.virtualWrite(
      VPIN_ROOM_HUMIDITY,
      roomHumidity
    );

  }


  if (
    !isnan(
      bodyTemperature
    )
  )
  {

    Blynk.virtualWrite(
      VPIN_BODY_TEMPERATURE,
      bodyTemperature
    );

  }


  bool fingerDetected =
    latestIR >=
    FINGER_IR_THRESHOLD;


  if (
    fingerDetected &&
    validHeartRate
  )
  {

    Blynk.virtualWrite(
      VPIN_HEART_RATE,
      heartRate
    );

  }

  else
  {

    Blynk.virtualWrite(
      VPIN_HEART_RATE,
      0
    );

  }


  if (
    fingerDetected &&
    validSPO2
  )
  {

    Blynk.virtualWrite(
      VPIN_SPO2,
      spo2
    );

  }

  else
  {

    Blynk.virtualWrite(
      VPIN_SPO2,
      0
    );

  }

}


// ============================================================
// WI-FI + BLYNK INITIAL CONNECTION
// ============================================================

void initializeNetworkAndBlynk()
{

  Serial.println();


  Serial.print(
    "Connecting to Wi-Fi: "
  );


  Serial.println(
    WIFI_SSID
  );


  WiFi.mode(
    WIFI_STA
  );


  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );


  Blynk.config(
    BLYNK_AUTH_TOKEN
  );


  unsigned long connectionStart =
    millis();


  while (
    WiFi.status() !=
    WL_CONNECTED
    &&
    millis() -
    connectionStart <
    WIFI_CONNECT_TIMEOUT
  )
  {

    delay(250);

    Serial.print(".");

  }


  Serial.println();


  if (
    WiFi.status() ==
    WL_CONNECTED
  )
  {

    Serial.println(
      "Wi-Fi connected."
    );


    Serial.print(
      "ESP32 IP address: "
    );


    Serial.println(
      WiFi.localIP()
    );


    Serial.println(
      "Connecting to Blynk Cloud..."
    );


    if (
      Blynk.connect(
        5000
      )
    )
    {

      Serial.println(
        "Blynk Cloud connected."
      );

    }

    else
    {

      Serial.println(
        "Blynk connection failed for now."
      );


      Serial.println(
        "The ESP32 will retry automatically."
      );

    }

  }

  else
  {

    Serial.println(
      "Wi-Fi connection timed out."
    );


    Serial.println(
      "Serial and OLED monitoring will continue."
    );


    Serial.println(
      "The ESP32 will retry automatically."
    );

  }


  lastWiFiReconnectAttempt =
    millis();


  lastBlynkReconnectAttempt =
    millis();

}


// ============================================================
// AUTOMATIC WI-FI + BLYNK RECONNECTION
// ============================================================

void maintainNetworkAndBlynk()
{

  unsigned long currentTime =
    millis();


  if (
    WiFi.status() !=
    WL_CONNECTED
  )
  {

    if (
      currentTime -
      lastWiFiReconnectAttempt >=
      RECONNECT_INTERVAL
    )
    {

      lastWiFiReconnectAttempt =
        currentTime;


      Serial.println(
        "Wi-Fi offline. Reconnecting..."
      );


      WiFi.reconnect();

    }


    return;

  }


  if (
    !Blynk.connected()
  )
  {

    if (
      currentTime -
      lastBlynkReconnectAttempt >=
      RECONNECT_INTERVAL
    )
    {

      lastBlynkReconnectAttempt =
        currentTime;


      Serial.println(
        "Blynk offline. Reconnecting..."
      );


      Blynk.connect(
        1000
      );

    }

  }


  runBlynkIfConnected();

}


// ============================================================
// SERVICE BLYNK
// ============================================================

void runBlynkIfConnected()
{

  if (
    Blynk.connected()
  )
  {

    Blynk.run();

  }

}


// ============================================================
// TELEGRAM INITIALIZATION
// ============================================================

void initializeTelegram()
{

  telegramClient.setCACert(
    TELEGRAM_CERTIFICATE_ROOT
  );


  configTime(
    0,
    0,
    "pool.ntp.org",
    "time.nist.gov"
  );


  Serial.println(
    "Telegram HTTPS configured."
  );


  Serial.println(
    "Telegram will become ready after internet and time synchronization."
  );

}


// ============================================================
// TELEGRAM NETWORK READY?
// ============================================================

bool telegramNetworkReady()
{

  if (
    WiFi.status() !=
    WL_CONNECTED
  )
  {

    return false;

  }


  time_t now =
    time(nullptr);


  if (
    now <
    24 * 3600
  )
  {

    return false;

  }


  return true;

}


// ============================================================
// TELEGRAM ALERT SYSTEM
// ============================================================

void checkTelegramAlerts()
{

  bool fingerDetected =
    latestIR >=
    FINGER_IR_THRESHOLD;


  bool newMQ2Alert = false;

  bool newRoomTempAlert = false;

  bool newRoomHumidityAlert = false;

  bool newBodyTempAlert = false;

  bool newHeartRateAlert = false;

  bool newSpO2Alert = false;

  bool newFallAlert = false;


  bool newAlertExists = false;


  String message =
    "ALERT - ESP32 Health Monitor\n\n";


  // ==========================================================
  // 1. MQ2
  // ==========================================================

  bool mq2Abnormal =
    mq2ADC >
    MQ2_HIGH_THRESHOLD;


  if (
    mq2Abnormal &&
    !mq2AlertActive
  )
  {

    message +=
      "Gas/Smoke level HIGH\n";


    message +=
      "MQ2 Value: ";


    message +=
      String(
        mq2ADC
      );


    message +=
      "\nThreshold: ";


    message +=
      String(
        MQ2_HIGH_THRESHOLD
      );


    message +=
      "\n\n";


    newMQ2Alert =
      true;


    newAlertExists =
      true;

  }


  if (
    !mq2Abnormal
  )
  {

    mq2AlertActive =
      false;

  }


  // ==========================================================
  // 2. ROOM TEMPERATURE
  // ==========================================================

  if (
    !isnan(
      roomTemperature
    )
  )
  {

    bool roomTempAbnormal =
      roomTemperature >
      ROOM_TEMP_HIGH_THRESHOLD;


    if (
      roomTempAbnormal &&
      !roomTempAlertActive
    )
    {

      message +=
        "Room Temperature HIGH\n";


      message +=
        "Value: ";


      message +=
        String(
          roomTemperature,
          1
        );


      message +=
        " C\nThreshold: ";


      message +=
        String(
          ROOM_TEMP_HIGH_THRESHOLD,
          1
        );


      message +=
        " C\n\n";


      newRoomTempAlert =
        true;


      newAlertExists =
        true;

    }


    if (
      !roomTempAbnormal
    )
    {

      roomTempAlertActive =
        false;

    }

  }


  // ==========================================================
  // 3. ROOM HUMIDITY
  // ==========================================================

  if (
    !isnan(
      roomHumidity
    )
  )
  {

    bool roomHumidityAbnormal =
      roomHumidity >
      ROOM_HUMIDITY_HIGH_THRESHOLD;


    if (
      roomHumidityAbnormal &&
      !roomHumidityAlertActive
    )
    {

      message +=
        "Room Humidity HIGH\n";


      message +=
        "Value: ";


      message +=
        String(
          roomHumidity,
          1
        );


      message +=
        " %\nThreshold: ";


      message +=
        String(
          ROOM_HUMIDITY_HIGH_THRESHOLD,
          1
        );


      message +=
        " %\n\n";


      newRoomHumidityAlert =
        true;


      newAlertExists =
        true;

    }


    if (
      !roomHumidityAbnormal
    )
    {

      roomHumidityAlertActive =
        false;

    }

  }


  // ==========================================================
  // 4. BODY TEMPERATURE
  // ==========================================================

  if (
    !isnan(
      bodyTemperature
    )
  )
  {

    bool bodyTempAbnormal =
      bodyTemperature >
      BODY_TEMP_HIGH_THRESHOLD;


    if (
      bodyTempAbnormal &&
      !bodyTempAlertActive
    )
    {

      message +=
        "Body Temperature HIGH\n";


      message +=
        "Value: ";


      message +=
        String(
          bodyTemperature,
          1
        );


      message +=
        " C\nThreshold: ";


      message +=
        String(
          BODY_TEMP_HIGH_THRESHOLD,
          1
        );


      message +=
        " C\n\n";


      newBodyTempAlert =
        true;


      newAlertExists =
        true;

    }


    if (
      !bodyTempAbnormal
    )
    {

      bodyTempAlertActive =
        false;

    }

  }


  // ==========================================================
  // 5. HEART RATE
  // ==========================================================

  if (
    fingerDetected &&
    validHeartRate
  )
  {

    bool heartRateAbnormal =
      heartRate >
      HEART_RATE_HIGH_THRESHOLD;


    if (
      heartRateAbnormal &&
      !heartRateAlertActive
    )
    {

      message +=
        "Heart Rate HIGH\n";


      message +=
        "Value: ";


      message +=
        String(
          heartRate
        );


      message +=
        " BPM\nThreshold: ";


      message +=
        String(
          HEART_RATE_HIGH_THRESHOLD
        );


      message +=
        " BPM\n\n";


      newHeartRateAlert =
        true;


      newAlertExists =
        true;

    }


    if (
      !heartRateAbnormal
    )
    {

      heartRateAlertActive =
        false;

    }

  }


  // ==========================================================
  // 6. SpO2
  // ==========================================================

  if (
    fingerDetected &&
    validSPO2
  )
  {

    bool spo2Abnormal =
      spo2 <
      SPO2_LOW_THRESHOLD;


    if (
      spo2Abnormal &&
      !spo2AlertActive
    )
    {

      message +=
        "SpO2 LOW\n";


      message +=
        "Value: ";


      message +=
        String(
          spo2
        );


      message +=
        " %\nThreshold: ";


      message +=
        String(
          SPO2_LOW_THRESHOLD,
          0
        );


      message +=
        " %\n\n";


      newSpO2Alert =
        true;


      newAlertExists =
        true;

    }


    if (
      !spo2Abnormal
    )
    {

      spo2AlertActive =
        false;

    }

  }


  // ==========================================================
  // 7. FALL DETECTION
  // ==========================================================

  if (
    fallTelegramPending
  )
  {
    message +=
      "FALL DETECTED!\n";

    message +=
      "Impact Acceleration: ";

    message +=
      String(
        fallAccelerationAtDetection,
        2
      );

    message +=
      " g\nGyroscope: ";

    message +=
      String(
        fallGyroAtDetection,
        2
      );

    message +=
      " rad/s\n\n";

    newFallAlert =
      true;

    newAlertExists =
      true;
  }


  // ==========================================================
  // NOTHING NEW
  // ==========================================================

  if (
    !newAlertExists
  )
  {

    return;

  }


  // ==========================================================
  // INTERNET / TIME READY?
  // ==========================================================

  if (
    !telegramNetworkReady()
  )
  {

    Serial.println(
      "Telegram alert waiting: internet/time not ready."
    );


    return;

  }


  // ==========================================================
  // SEND TELEGRAM
  // ==========================================================

  Serial.println(
    "Sending Telegram alert..."
  );


  bool messageSent =
    telegramBot.sendMessage(
      TELEGRAM_CHAT_ID,
      message,
      ""
    );


  if (
    messageSent
  )
  {

    Serial.println(
      "Telegram alert sent successfully."
    );


    if (
      newMQ2Alert
    )
    {

      mq2AlertActive =
        true;

    }


    if (
      newRoomTempAlert
    )
    {

      roomTempAlertActive =
        true;

    }


    if (
      newRoomHumidityAlert
    )
    {

      roomHumidityAlertActive =
        true;

    }


    if (
      newBodyTempAlert
    )
    {

      bodyTempAlertActive =
        true;

    }


    if (
      newHeartRateAlert
    )
    {

      heartRateAlertActive =
        true;

    }


    if (
      newSpO2Alert
    )
    {

      spo2AlertActive =
        true;

    }


    if (
      newFallAlert
    )
    {

      fallTelegramPending =
        false;

    }

  }

  else
  {

    Serial.println(
      "Telegram alert FAILED."
    );


    Serial.println(
      "The ESP32 will retry automatically."
    );

  }

}


// ============================================================
// SETUP
// ============================================================

void setup()
{

  Serial.begin(
    115200
  );


  delay(
    1000
  );


  // ==========================================================
  // EXHAUST FAN RELAY
  //
  // Set LOW before enabling OUTPUT.
  // This prevents the fan from briefly turning on at startup.
  // ==========================================================

  digitalWrite(
    RELAY_PIN,
    LOW
  );


  pinMode(
    RELAY_PIN,
    OUTPUT
  );


  // ==========================================================
  // RGB LED
  //
  // Initial condition:
  //
  // RED   = OFF
  // GREEN = ON
  // BLUE  = OFF
  // ==========================================================

  digitalWrite(
    RGB_RED_PIN,
    LOW
  );


  digitalWrite(
    RGB_GREEN_PIN,
    HIGH
  );


  digitalWrite(
    RGB_BLUE_PIN,
    LOW
  );


  pinMode(
    RGB_RED_PIN,
    OUTPUT
  );


  pinMode(
    RGB_GREEN_PIN,
    OUTPUT
  );


  pinMode(
    RGB_BLUE_PIN,
    OUTPUT
  );


  // ==========================================================
  // MQ2
  // ==========================================================

  analogReadResolution(
    12
  );


  // ==========================================================
  // DHT22
  // ==========================================================

  dht.begin();


  // ==========================================================
  // DS18B20
  // ==========================================================

  ds18b20.begin();


  Serial.print(
    "DS18B20 devices found: "
  );


  Serial.println(
    ds18b20.getDeviceCount()
  );


  ds18b20.setWaitForConversion(
    false
  );


  ds18b20.requestTemperatures();


  dsRequestTime =
    millis();


  // ==========================================================
  // I2C
  // ==========================================================

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );


  // ==========================================================
  // OLED
  // ==========================================================

  if (
    !display.begin(
      SSD1306_SWITCHCAPVCC,
      SCREEN_ADDRESS
    )
  )
  {

    Serial.println(
      "OLED initialization failed!"
    );


    while (1);

  }


  display.clearDisplay();


  display.setTextSize(
    1
  );


  display.setTextColor(
    SSD1306_WHITE
  );


  display.setCursor(
    0,
    OLED_FIRST_LINE_Y
  );


  display.println(
    "Initializing..."
  );


  display.display();


  // ==========================================================
  // MAX30102 INITIALIZATION
  // ==========================================================

  Serial.println(
    "Initializing MAX30102..."
  );


  if (
    !particleSensor.begin(
      Wire,
      I2C_SPEED_FAST
    )
  )
  {

    Serial.println(
      "MAX30102 was not found."
    );


    Serial.println(
      "Check wiring and power."
    );


    display.clearDisplay();


    display.setCursor(
      0,
      OLED_FIRST_LINE_Y
    );


    display.println(
      "MAX30102 ERROR"
    );


    display.display();


    while (1);

  }


  Serial.println(
    "MAX30102 detected."
  );


  // ==========================================================
  // MAX30102 SETTINGS
  // ==========================================================

  byte ledBrightness = 60;

  byte sampleAverage = 1;

  byte ledMode = 2;

  int sampleRate = 100;

  int pulseWidth = 411;

  int adcRange = 4096;


  particleSensor.setup(
    ledBrightness,
    sampleAverage,
    ledMode,
    sampleRate,
    pulseWidth,
    adcRange
  );


  // ==========================================================
  // MPU6050 + BUZZER PWM
  // ==========================================================

  initializeMPU6050();


  // ==========================================================
  // NETWORK
  // ==========================================================

  initializeNetworkAndBlynk();


  initializeTelegram();


  // ==========================================================
  // CLEAR OLD MAX30102 SAMPLES
  // ==========================================================

  particleSensor.clearFIFO();


  resetHeartRate();


  // ==========================================================
  // INITIAL 100 SpO2 SAMPLES
  // ==========================================================

  Serial.println(
    "Collecting initial MAX30102 samples..."
  );


  display.clearDisplay();


  display.setCursor(
    0,
    OLED_FIRST_LINE_Y
  );


  display.println(
    "Place finger..."
  );


  display.setCursor(
    0,
    OLED_FIRST_LINE_Y +
    OLED_LINE_SPACING
  );


  display.println(
    "Collecting data..."
  );


  display.display();


  for (
    int i = 0;
    i < 100;
    i++
  )
  {

    collectOneSpO2Sample(
      redBuffer[i],
      irBuffer[i]
    );

  }


  // ==========================================================
  // INITIAL SpO2 CALCULATION
  // ==========================================================

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer,
    bufferLength,
    redBuffer,
    &spo2,
    &validSPO2,
    &maximHeartRate,
    &maximValidHeartRate
  );


  // ==========================================================
  // BLYNK TIMER
  // ==========================================================

  blynkTimer.setInterval(
    BLYNK_UPLOAD_INTERVAL,
    sendSensorDataToBlynk
  );


  // ==========================================================
  // TELEGRAM TIMER
  // ==========================================================

  blynkTimer.setInterval(
    TELEGRAM_CHECK_INTERVAL,
    checkTelegramAlerts
  );


  display.clearDisplay();

  display.display();


  Serial.println();


  Serial.println(
    "All sensors initialized."
  );


  Serial.println(
    "Heart rate: checkForBeat()"
  );


  Serial.println(
    "SpO2: Maxim algorithm"
  );


  Serial.println(
    "Blynk monitoring started."
  );


  Serial.println(
    "Telegram monitoring started."
  );


  Serial.println(
    "Fan and RGB control started."
  );


  Serial.println();

}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{

  maintainNetworkAndBlynk();

  serviceFallDetection();


  // ==========================================================
  // SHIFT OLD 75 SpO2 SAMPLES
  // ==========================================================

  for (
    byte i = 25;
    i < 100;
    i++
  )
  {

    redBuffer[i - 25] =
      redBuffer[i];


    irBuffer[i - 25] =
      irBuffer[i];

  }


  // ==========================================================
  // COLLECT 25 NEW SpO2 SAMPLES
  // ==========================================================

  for (
    byte i = 75;
    i < 100;
    i++
  )
  {

    collectOneSpO2Sample(
      redBuffer[i],
      irBuffer[i]
    );

  }


  // ==========================================================
  // RECALCULATE SpO2
  // ==========================================================

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer,
    bufferLength,
    redBuffer,
    &spo2,
    &validSPO2,
    &maximHeartRate,
    &maximValidHeartRate
  );


  unsigned long currentTime =
    millis();


  // ==========================================================
  // MQ2
  // ==========================================================

  mq2ADC =
    analogRead(
      MQ2_PIN
    );


  // ==========================================================
  // DHT22
  // ==========================================================

  if (
    currentTime -
    lastDHTRead >=
    DHT_INTERVAL
  )
  {

    lastDHTRead =
      currentTime;


    float newHumidity =
      dht.readHumidity();


    float newTemperature =
      dht.readTemperature();


    if (
      !isnan(
        newHumidity
      )
      &&
      !isnan(
        newTemperature
      )
    )
    {

      roomHumidity =
        newHumidity;


      roomTemperature =
        newTemperature;

    }

  }


  // ==========================================================
  // DS18B20
  // ==========================================================

  if (
    currentTime -
    dsRequestTime >=
    DS_CONVERSION_TIME
  )
  {

    float newBodyTemperature =
      ds18b20.getTempCByIndex(
        0
      );


    if (
      newBodyTemperature !=
      DEVICE_DISCONNECTED_C
    )
    {

      bodyTemperature =
        newBodyTemperature;

    }


    ds18b20.requestTemperatures();


    dsRequestTime =
      currentTime;

  }


  // ==========================================================
  // FAN + RGB CONTROL
  // ==========================================================

  updateFanAndRGB();


  // ==========================================================
  // BLYNK + TELEGRAM TIMERS
  // ==========================================================

  blynkTimer.run();


  // ==========================================================
  // OLED UPDATE
  // ==========================================================

  if (
    currentTime -
    lastDisplayUpdate >=
    DISPLAY_INTERVAL
  )
  {

    lastDisplayUpdate =
      currentTime;


    bool fingerDetected =
      latestIR >=
      FINGER_IR_THRESHOLD;


    // ========================================================
    // SERIAL MONITOR
    //
    // Keep the original health-monitor Serial output and add:
    //   - Acceleration
    //   - Gyroscope
    //   - Fall status
    // ========================================================

    Serial.println(
      "================================"
    );


    // --------------------------------------------------------
    // MQ2
    // --------------------------------------------------------

    Serial.print(
      "MQ2 Raw Value: "
    );

    Serial.println(
      mq2ADC
    );


    // --------------------------------------------------------
    // ROOM TEMPERATURE
    // --------------------------------------------------------

    Serial.print(
      "Room Temperature: "
    );

    if (
      isnan(
        roomTemperature
      )
    )
    {
      Serial.println(
        "Invalid"
      );
    }
    else
    {
      Serial.print(
        roomTemperature,
        1
      );
      Serial.println(
        " C"
      );
    }


    // --------------------------------------------------------
    // ROOM HUMIDITY
    // --------------------------------------------------------

    Serial.print(
      "Room Humidity: "
    );

    if (
      isnan(
        roomHumidity
      )
    )
    {
      Serial.println(
        "Invalid"
      );
    }
    else
    {
      Serial.print(
        roomHumidity,
        1
      );
      Serial.println(
        " %"
      );
    }


    // --------------------------------------------------------
    // BODY TEMPERATURE
    // --------------------------------------------------------

    Serial.print(
      "Body Temperature: "
    );

    if (
      isnan(
        bodyTemperature
      )
    )
    {
      Serial.println(
        "Invalid"
      );
    }
    else
    {
      Serial.print(
        bodyTemperature,
        2
      );
      Serial.println(
        " C"
      );
    }


    // --------------------------------------------------------
    // HEART RATE + SpO2
    // --------------------------------------------------------

    Serial.print(
      "Heart Rate: "
    );

    if (
      fingerDetected &&
      validHeartRate
    )
    {
      Serial.print(
        heartRate
      );
      Serial.print(
        " BPM"
      );
    }
    else
    {
      Serial.print(
        "--"
      );
    }

    Serial.print(
      "   |   SpO2: "
    );

    if (
      fingerDetected &&
      validSPO2
    )
    {
      Serial.print(
        spo2
      );
      Serial.print(
        " %"
      );
    }
    else
    {
      Serial.print(
        "--"
      );
    }

    if (
      !fingerDetected
    )
    {
      Serial.print(
        "   |   No finger"
      );
    }

    Serial.println();


    // --------------------------------------------------------
    // MPU6050
    // --------------------------------------------------------

    Serial.print(
      "Acceleration: "
    );
    Serial.print(
      latestAcceleration,
      2
    );
    Serial.println(
      " g"
    );

    Serial.print(
      "Gyroscope: "
    );
    Serial.print(
      latestGyro,
      2
    );
    Serial.println(
      " rad/s"
    );


    // --------------------------------------------------------
    // FALL STATUS
    // --------------------------------------------------------

    Serial.print(
      "Fall Status: "
    );

    if (
      fallAlarmActive
    )
    {
      Serial.println(
        "FALL DETECTED"
      );
    }
    else if (
      freeFallDetected
    )
    {
      Serial.println(
        "Possible fall - waiting for impact"
      );
    }
    else
    {
      Serial.println(
        "No fall detected"
      );
    }


    // --------------------------------------------------------
    // BLYNK STATUS
    // --------------------------------------------------------

    Serial.print(
      "Blynk Cloud: "
    );

    if (
      Blynk.connected()
    )
    {
      Serial.println(
        "Connected"
      );
    }
    else
    {
      Serial.println(
        "Offline / reconnecting"
      );
    }

    Serial.println(
      "================================"
    );
    Serial.println();


    // ========================================================
    // OLED
    // ========================================================

    display.clearDisplay();


    display.setTextSize(
      1
    );


    display.setTextColor(
      SSD1306_WHITE
    );


    // --------------------------------------------------------
    // OLED LINE 1 - MQ2
    // --------------------------------------------------------

    display.setCursor(
      0,
      OLED_FIRST_LINE_Y
    );


    display.print(
      "MQ2: "
    );


    display.print(
      mq2ADC
    );


    // --------------------------------------------------------
    // OLED LINE 2 - ROOM TEMPERATURE
    // --------------------------------------------------------

    display.setCursor(
      0,
      OLED_FIRST_LINE_Y +
      OLED_LINE_SPACING
    );


    display.print(
      "R.Temp: "
    );


    if (
      isnan(
        roomTemperature
      )
    )
    {

      display.print(
        "--"
      );

    }

    else
    {

      display.print(
        roomTemperature,
        1
      );


      display.print(
        " C"
      );

    }


    // --------------------------------------------------------
    // OLED LINE 3 - ROOM HUMIDITY
    // --------------------------------------------------------

    display.setCursor(
      0,
      OLED_FIRST_LINE_Y +
      (2 * OLED_LINE_SPACING)
    );


    display.print(
      "R.Hum: "
    );


    if (
      isnan(
        roomHumidity
      )
    )
    {

      display.print(
        "--"
      );

    }

    else
    {

      display.print(
        roomHumidity,
        1
      );


      display.print(
        " %"
      );

    }


    // --------------------------------------------------------
    // OLED LINE 4 - BODY TEMPERATURE
    // --------------------------------------------------------

    display.setCursor(
      0,
      OLED_FIRST_LINE_Y +
      (3 * OLED_LINE_SPACING)
    );


    display.print(
      "B.Temp: "
    );


    if (
      isnan(
        bodyTemperature
      )
    )
    {

      display.print(
        "--"
      );

    }

    else
    {

      display.print(
        bodyTemperature,
        1
      );


      display.print(
        " C"
      );

    }


    // --------------------------------------------------------
    // OLED LINE 5 - BPM + SpO2
    // --------------------------------------------------------

    display.setCursor(
      0,
      OLED_FIRST_LINE_Y +
      (4 * OLED_LINE_SPACING)
    );


    display.print(
      "BPM:"
    );


    if (
      fingerDetected &&
      validHeartRate
    )
    {

      display.print(
        heartRate
      );

    }

    else
    {

      display.print(
        "--"
      );

    }


    display.print(
      " SpO2:"
    );


    if (
      fingerDetected &&
      validSPO2
    )
    {

      display.print(
        spo2
      );


      display.print(
        "%"
      );

    }

    else
    {

      display.print(
        "--"
      );

    }


    display.display();

  }


  serviceFallDetection();

  runBlynkIfConnected();

}