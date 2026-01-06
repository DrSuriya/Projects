#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <event_groups.h>

void watchdogSetup(void) {
  uint32_t wdt_value = 512;
  WDT->WDT_MR = WDT_MR_WDRSTEN | WDT_MR_WDV(wdt_value) | WDT_MR_WDD(0xFFF);
}

inline void kickHardwareWDT() {
  WDT->WDT_CR = WDT_CR_KEY(0xA5) | WDT_CR_WDRSTT;
}

typedef struct {
    float filteredS1;
    float filteredS2;
    float distL;
    float distF;
    float distR;
    bool s1Det;
    bool s2Det;
} SensorData_t;

SensorData_t sensorData = {900, 900, 999, 999, 999, false, false};

TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t motorTaskHandle  = NULL;
TaskHandle_t debugTaskHandle  = NULL;

SemaphoreHandle_t sensorDataMutex;
SemaphoreHandle_t sensorDataSemaphore;

EventGroupHandle_t watchdogEventGroup;

#define TASK_SENSOR_BIT  (1 << 0)
#define TASK_MOTOR_BIT   (1 << 1)
#define TASK_DEBUG_BIT   (1 << 2)
#define ALL_TASKS_MASK   (TASK_SENSOR_BIT | TASK_MOTOR_BIT | TASK_DEBUG_BIT)

const TickType_t WDT_CHECK_PERIOD = pdMS_TO_TICKS(300);

const uint8_t DIR_LEFT  = 5;
const uint8_t DIR_RIGHT = 10;
const uint8_t PWM_PIN = 2;

volatile uint32_t g_rc1;

inline void setPWM(float duty) {
    if (duty <= 0.0f) TC0->TC_CHANNEL[0].TC_RA = 0;
    else if (duty >= 1.0f) TC0->TC_CHANNEL[0].TC_RA = g_rc1;
    else TC0->TC_CHANNEL[0].TC_RA = (uint32_t)(g_rc1 * duty);
}

inline void stop_motors() {
    setPWM(0);
}

inline void go_straight(float r) {
    digitalWrite(DIR_LEFT, HIGH);
    digitalWrite(DIR_RIGHT, LOW);
    setPWM(r);
}

inline void go_backward(float r) {
    digitalWrite(DIR_LEFT, LOW);
    digitalWrite(DIR_RIGHT, HIGH);
    setPWM(r);
}  

void safeDelay(uint32_t ms) {
    const uint32_t slice = 150;
    uint32_t remaining = ms;

    while (remaining > 0) {  
        xEventGroupSetBits(watchdogEventGroup, TASK_MOTOR_BIT);

        if (remaining >= slice) {
            vTaskDelay(pdMS_TO_TICKS(slice));
            remaining -= slice;
        } else {
            vTaskDelay(pdMS_TO_TICKS(remaining));
            remaining = 0;
        }  
    }  
}  

void turnRight90_custom() {
    digitalWrite(DIR_LEFT, LOW);
    digitalWrite(DIR_RIGHT, LOW);
    setPWM(0.8);
    int turnTime = random(700, 1101);
    safeDelay(turnTime);
    stop_motors();
}  

const int SENSOR1_PINS[3] = {A0, A1, A2};
const int SENSOR2_PINS[3] = {A4, A5, A6};
const int BLACK_HIGH_THRESHOLD = 600;

float filteredS1 = 900;
float filteredS2 = 900;

int readTriplet(const int pins[3]) {
    return (analogRead(pins[0]) + analogRead(pins[1]) + analogRead(pins[2])) / 3;
}

const uint8_t TRIG_L = 9;
const uint8_t ECHO_L = 8;
const uint8_t TRIG_M = 7;
const uint8_t ECHO_M = 6;
const uint8_t TRIG_R = 12;
const uint8_t ECHO_R = 11;

const float MIN_DISTANCE_CM = 25.0;

float readUS(uint8_t trig, uint8_t echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    long duration = pulseIn(echo, HIGH, 25000);
    if (duration == 0) return 999;
    return duration / 58.0;
}  

void sensorTask(void *pvParameters) {
  (void) pvParameters;
  const float alpha = 0.55;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);

  while (1) {
    float s1 = alpha * readTriplet(SENSOR1_PINS) + (1 - alpha) * filteredS1;
    float s2 = alpha * readTriplet(SENSOR2_PINS) + (1 - alpha) * filteredS2;
    bool s1Det = s1 > BLACK_HIGH_THRESHOLD;
    bool s2Det = s2 > BLACK_HIGH_THRESHOLD;

    float distL = readUS(TRIG_L, ECHO_L);
    float distF = readUS(TRIG_M, ECHO_M);
    float distR = readUS(TRIG_R, ECHO_R);

    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      sensorData.filteredS1 = s1;
      sensorData.filteredS2 = s2;
      sensorData.s1Det = s1Det;
      sensorData.s2Det = s2Det;
      sensorData.distL = distL;
      sensorData.distF = distF;
      sensorData.distR = distR;
      filteredS1 = s1;
      filteredS2 = s2;
      xSemaphoreGive(sensorDataMutex);
    }

    xSemaphoreGive(sensorDataSemaphore);
    xEventGroupSetBits(watchdogEventGroup, TASK_SENSOR_BIT);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void motorTask(void *pvParameters) {
  (void) pvParameters;
  SensorData_t localData;

  while (1) {
    if (xSemaphoreTake(sensorDataSemaphore, portMAX_DELAY) == pdTRUE) {
      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        localData = sensorData;
        xSemaphoreGive(sensorDataMutex);
      } else continue;

      if (localData.s1Det || localData.s2Det || localData.distL <= MIN_DISTANCE_CM || localData.distF <= MIN_DISTANCE_CM || localData.distR <= MIN_DISTANCE_CM) {
        stop_motors();
        vTaskDelay(pdMS_TO_TICKS(50));
        go_backward(0.6);
        safeDelay(900);
        stop_motors();
        vTaskDelay(pdMS_TO_TICKS(30));
        turnRight90_custom();
      } else {
        go_straight(0.6);
      }
    }
    xEventGroupSetBits(watchdogEventGroup, TASK_MOTOR_BIT);
  }
}

void debugTask(void *pvParameters) {
  (void) pvParameters;
  SensorData_t localData;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);

  while (1) {
    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      localData = sensorData;
      xSemaphoreGive(sensorDataMutex);

      Serial.print(" L="); Serial.print(localData.distL);
      Serial.print(" F="); Serial.print(localData.distF);
      Serial.print(" R="); Serial.print(localData.distR);
      Serial.print(" S1="); Serial.print(localData.filteredS1);
      Serial.print(" S2="); Serial.println(localData.filteredS2);
    }
    xEventGroupSetBits(watchdogEventGroup, TASK_DEBUG_BIT);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void watchdogTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    EventBits_t uxBits = xEventGroupWaitBits(watchdogEventGroup, ALL_TASKS_MASK, pdTRUE, pdTRUE, WDT_CHECK_PERIOD);
    if ((uxBits & ALL_TASKS_MASK) == ALL_TASKS_MASK) {
      kickHardwareWDT();
    } else {  
      Serial.println("SYSTEM HANG DETECTED - RESETTING");
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  watchdogEventGroup = xEventGroupCreate();
  watchdogSetup();

  randomSeed(analogRead(A8));

  pinMode(DIR_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);

  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(TRIG_M, OUTPUT); pinMode(ECHO_M, INPUT);

  PMC->PMC_PCER0 |= PMC_PCER0_PID27;
  PIOB->PIO_PDR  |= PIO_PDR_P25;
  PIOB->PIO_ABSR |= PIO_PB25B_TIOA0;

  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET;
  g_rc1 = 42000000 / 100;
  TC0->TC_CHANNEL[0].TC_RC = g_rc1;
  TC0->TC_CHANNEL[0].TC_RA = 0;
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

  stop_motors();

  sensorDataMutex = xSemaphoreCreateMutex();
  sensorDataSemaphore = xSemaphoreCreateBinary();

  xTaskCreate(sensorTask, "SensorTask", 512, NULL, 3, &sensorTaskHandle);
  xTaskCreate(motorTask, "MotorTask", 512, NULL, 2, &motorTaskHandle);
  xTaskCreate(debugTask, "DebugTask", 256, NULL, 1, &debugTaskHandle);
  xTaskCreate(watchdogTask, "WatchdogTask", 256, NULL, 4, NULL);

  vTaskStartScheduler();
}

void loop() {  
}
