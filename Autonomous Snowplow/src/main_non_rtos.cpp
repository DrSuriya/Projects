#include <Arduino.h>
#include <sam.h>
#include <variant.h>

const uint8_t DIR_RIGHT = 10;
const uint8_t DIR_LEFT = 5;

const uint8_t PWM_RIGHT = 2;
volatile uint32_t g_rc1;

const int SENSOR1_PINS[3] = { A0, A1, A2 };
const int SENSOR2_PINS[3] = { A4, A5, A6 };

const int BLACK_HIGH_THRESHOLD = 500;

float filteredS1 = 900;
float filteredS2 = 900;

const uint8_t TRIG_L = 9;
const uint8_t ECHO_L = 8;

const uint8_t TRIG_R = 7;
const uint8_t ECHO_R = 6;

const uint8_t TRIG_M = 12;
const uint8_t ECHO_M = 11;

const float MIN_DISTANCE_CM = 30.0;

inline void setPWMRight(float duty) {
  if (duty <= 0.0f)
    TC0->TC_CHANNEL[0].TC_RA = 0;
  else if (duty >= 1.0f)
    TC0->TC_CHANNEL[0].TC_RA = g_rc1;
  else
    TC0->TC_CHANNEL[0].TC_RA = (uint32_t)(g_rc1 * duty);
}

inline void go_straight(float r) {
  digitalWrite(DIR_LEFT, HIGH);
  digitalWrite(DIR_RIGHT, LOW);
  setPWMRight(r);
}

inline void go_backward(float r) {
  digitalWrite(DIR_LEFT, LOW);
  digitalWrite(DIR_RIGHT, HIGH);
  setPWMRight(r);
}

inline void stop_motors() {
  setPWMRight(0);
}

void turnRight90_custom() {
  digitalWrite(DIR_LEFT, LOW);
  digitalWrite(DIR_RIGHT, LOW);
  setPWMRight(0.8);

  int turnTime = random(700, 1101);
  delay(turnTime);
  stop_motors();
}

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

int readTriplet(const int pins[3]) {
  return (analogRead(pins[0]) + analogRead(pins[1]) + analogRead(pins[2])) / 3;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  randomSeed(analogRead(A8));

  pinMode(DIR_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);

  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);

  pinMode(TRIG_M, OUTPUT);
  pinMode(ECHO_M, INPUT);

  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  PMC->PMC_PCER0 |= PMC_PCER0_PID27;
  PIOB->PIO_PDR |= PIO_PDR_P25;
  PIOB->PIO_ABSR |= PIO_PB25B_TIOA0;

  TC0->TC_CHANNEL[0].TC_CMR =
    TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |
    TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET;

  g_rc1 = 42000000 / 100;
  TC0->TC_CHANNEL[0].TC_RC = g_rc1;
  TC0->TC_CHANNEL[0].TC_RA = 0;
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

  stop_motors();
}

void loop() {
  float alpha = 0.55;
  filteredS1 = alpha * readTriplet(SENSOR1_PINS) + (1 - alpha) * filteredS1;
  filteredS2 = alpha * readTriplet(SENSOR2_PINS) + (1 - alpha) * filteredS2;

  bool s1Det = filteredS1 > BLACK_HIGH_THRESHOLD;
  bool s2Det = filteredS2 > BLACK_HIGH_THRESHOLD;

  float dL = readUS(TRIG_L, ECHO_L);
  float dF = readUS(TRIG_R, ECHO_R);
  float dR = readUS(TRIG_M, ECHO_M);

  if (s1Det || s2Det) {
    stop_motors();
    delay(50);

    go_backward(0.6);
    delay(900);

    stop_motors();
    delay(30);

    turnRight90_custom();
    return;
  }

  if (dL <= MIN_DISTANCE_CM || dF <= MIN_DISTANCE_CM || dR <= MIN_DISTANCE_CM) {
    stop_motors();
    delay(50);

    go_backward(0.6);
    delay(900);

    stop_motors();
    delay(30);
    turnRight90_custom();
  } else {
    go_straight(0.6);
  }

  Serial.print("L=");
  Serial.print(dL);
  Serial.print(" R=");
  Serial.print(dF);
  Serial.print(" S1=");
  Serial.print(dR);
  Serial.print(" S3=");
  Serial.print(filteredS1);
  Serial.print(" S2=");
  Serial.println(filteredS2);

  delay(10);
}
