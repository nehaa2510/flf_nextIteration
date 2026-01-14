#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

/* ===================== CONFIG ===================== */
#define UART_TUNING 1
#define USB_DEBUG   1
#define LINE_SENSOR_MODE 1

/* ===================== UART FOR ESP32 ===================== */
// STM32 PB6 = TX, PB7 = RX
#if UART_TUNING
HardwareSerial TuningUART(PB7, PB6);   // RX, TX
#endif

/* ===================== LINE ANALYSIS ===================== */
#define ALGO_MOST_CENTERED 1
#define ALGO_LARGEST       2
#define ALGO_NORMAL        3
#define LINE_ANALYSIS_ALGORITHM ALGO_MOST_CENTERED

/* ===================== PIN DEFINITIONS ===================== */
#define MOTOR_A_PWM PA1
#define MOTOR_A_IN1 PB8
#define MOTOR_A_IN2 PB9

#define MOTOR_B_PWM PA6
#define MOTOR_B_IN1 PA5
#define MOTOR_B_IN2 PA4

#define UEN_PIN PA15
#define STBY_PIN PB12
#define JUNCTION_PULSE PB4

/* ===================== SERIAL ===================== */
const int SENSOR_BAUD_RATE = 230400;
const int DEBUG_BAUD_RATE  = 115200;

/* ===================== PID VARIABLES ===================== */
float Kp = 5.150;
float Kd = 4.550;

int SLOW_SPEED = 130;
int baseSpeed  = 180;
int maxSpeed   = 255;
int minSpeed   = -255;
int setPoint   = 35;
int junctionSpeed = 120;

int error = 0;
int lastError = 0;

unsigned long previousTime = 0;

/* ===================== LINE STATE ===================== */
const int LAST_END_LEFT    = 0;
const int LAST_END_RIGHT   = 1;
const int LAST_END_UNKNOWN = 2;

int last_end = LAST_END_UNKNOWN;
unsigned long last_end_update_time = 0;
unsigned int LAST_END_TIMEOUT_MS = 110;

unsigned long line_lost_time = 0;
unsigned int DASHED_LINE_GRACE_PERIOD_MS = 180;

int LINE_LOST_TURN_SPEED  = 150;
int LINE_LOST_PIVOT_SPEED = -150;

/* ===================== STRUCTS ===================== */
enum LineFeature {
  NORMAL_LINE,
  JUNCTION,
  LINE_LOST
};

struct LineAnalysisResult {
  int position;
  LineFeature featureType;
  bool isComplex;
};

/* ===================== FUNCTION DECLARATIONS ===================== */
LineAnalysisResult analyzeSensorData(uint8_t rawSensorByte);
void motorspeed(int rightmotorspeed, int leftmotorspeed);
void handleLineLost();
void pidTurn(int error, bool isComplex);
void processCommand(char* command);

#if UART_TUNING
void handleUARTCommands();
#endif

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(DEBUG_BAUD_RATE);
  delay(1000);

#if USB_DEBUG
  Serial.println("STM32 UART Line Follower Init");
#endif

#if UART_TUNING
  TuningUART.begin(115200);
#if USB_DEBUG
  Serial.println("UART tuning on PB6 / PB7");
#endif
#endif

  Serial1.begin(SENSOR_BAUD_RATE);   // Line sensor UART

  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);

  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);

  pinMode(UEN_PIN, OUTPUT);
  digitalWrite(UEN_PIN, LOW);

  pinMode(JUNCTION_PULSE, INPUT);

  previousTime = micros();
}

/* ===================== LOOP ===================== */
void loop() {

#if UART_TUNING
  handleUARTCommands();   // ESP32 → STM32 tuning
#endif

  if (last_end != LAST_END_UNKNOWN &&
      millis() - last_end_update_time > LAST_END_TIMEOUT_MS) {
    last_end = LAST_END_UNKNOWN;
  }

  if (Serial1.available()) {
    uint8_t rawSensorByte = Serial1.read();
    LineAnalysisResult lineState = analyzeSensorData(rawSensorByte);

    switch (lineState.featureType) {

      case NORMAL_LINE: {
        line_lost_time = 0;
        error = lineState.position - setPoint;

        bool on_left_edge  = rawSensorByte & 0b00000001;
        bool on_right_edge = rawSensorByte & 0b10000000;

        if (on_left_edge) {
          last_end = LAST_END_LEFT;
          last_end_update_time = millis();
        } else if (on_right_edge) {
          last_end = LAST_END_RIGHT;
          last_end_update_time = millis();
        }

        pidTurn(error, lineState.isComplex);
        break;
      }

      case JUNCTION:
        motorspeed(junctionSpeed, junctionSpeed);
        break;

      case LINE_LOST:
        if (last_end == LAST_END_UNKNOWN) {
          if (line_lost_time == 0) {
            line_lost_time = millis();
          } else if (millis() - line_lost_time > DASHED_LINE_GRACE_PERIOD_MS) {
            handleLineLost();
          }
        } else {
          handleLineLost();
        }
        break;
    }
  }
}

/* ===================== UART COMMAND HANDLER ===================== */
#if UART_TUNING
void handleUARTCommands() {
  static char cmdBuf[32];
  static uint8_t idx = 0;

  while (TuningUART.available()) {
    char c = TuningUART.read();

    if (c == '\n' || c == '\r') {
      if (idx > 0) {
        cmdBuf[idx] = '\0';
        processCommand(cmdBuf);
        idx = 0;
      }
    } else if (idx < sizeof(cmdBuf) - 1) {
      cmdBuf[idx++] = c;
    }
  }
}
#endif

/* ===================== COMMAND PARSER ===================== */
void processCommand(char* command) {
  char* sep = strchr(command, ':');
  if (!sep) return;

  *sep = '\0';
  char* val = sep + 1;

  if (!strcmp(command, "KP")) Kp = atof(val);
  else if (!strcmp(command, "KD")) Kd = atof(val);
  else if (!strcmp(command, "BS")) baseSpeed = atoi(val);
  else if (!strcmp(command, "MS")) maxSpeed = atoi(val);
  else if (!strcmp(command, "MN")) minSpeed = atoi(val);
  else if (!strcmp(command, "SS")) SLOW_SPEED = atoi(val);
  else if (!strcmp(command, "SP")) setPoint = atoi(val);
  else if (!strcmp(command, "LT")) LAST_END_TIMEOUT_MS = atoi(val);
  else if (!strcmp(command, "DG")) DASHED_LINE_GRACE_PERIOD_MS = atoi(val);
  else if (!strcmp(command, "TS")) LINE_LOST_TURN_SPEED = atoi(val);
  else if (!strcmp(command, "PS")) LINE_LOST_PIVOT_SPEED = atoi(val);
  else if (!strcmp(command, "JS")) junctionSpeed = atoi(val);
}

/* ===================== LINE ANALYSIS ===================== */
LineAnalysisResult analyzeSensorData(uint8_t raw) {
  LineAnalysisResult r;
  r.position = 35;
  r.isComplex = false;

  if (raw == 0xFF) { r.featureType = JUNCTION; return r; }
  if (raw == 0x00) { r.featureType = LINE_LOST; return r; }

  int weights[8] = {0,10,20,30,40,50,60,70};
  int sum = 0, cnt = 0;

  for (int i = 0; i < 8; i++) {
    if (raw & (1 << i)) {
      sum += weights[i];
      cnt++;
    }
  }

  if (cnt > 0) r.position = sum / cnt;
  r.featureType = NORMAL_LINE;
  return r;
}

/* ===================== PID ===================== */
void pidTurn(int err, bool isComplex) {
  unsigned long now = micros();
  float dt = (now - previousTime) / 1e6;
  previousTime = now;

  if (dt <= 0 || dt > 0.3) dt = 0;

  float derivative = dt > 0 ? (err - lastError) / dt : 0;
  int correction = (Kp * err) + (Kd * derivative);
  lastError = err;

  int base = isComplex ? SLOW_SPEED : baseSpeed;
  int left  = constrain(base + correction, -maxSpeed, maxSpeed);
  int right = constrain(base - correction, -maxSpeed, maxSpeed);

  motorspeed(right, left);
}

/* ===================== LINE LOST ===================== */
void handleLineLost() {
  if (last_end == LAST_END_RIGHT)
    motorspeed(LINE_LOST_PIVOT_SPEED, LINE_LOST_TURN_SPEED);
  else if (last_end == LAST_END_LEFT)
    motorspeed(LINE_LOST_TURN_SPEED, LINE_LOST_PIVOT_SPEED);
}

/* ===================== MOTOR DRIVER ===================== */
void motorspeed(int right, int left) {

  digitalWrite(MOTOR_B_IN1, right > 0);
  digitalWrite(MOTOR_B_IN2, right < 0);
  analogWrite(MOTOR_B_PWM, abs(right));

  digitalWrite(MOTOR_A_IN1, left > 0);
  digitalWrite(MOTOR_A_IN2, left < 0);
  analogWrite(MOTOR_A_PWM, abs(left));
}
