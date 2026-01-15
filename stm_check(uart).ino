#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

#define WEBSERVER 1
#define USB_DEBUG 1
#define LINE_SENSOR_MODE 1

#define ALGO_MOST_CENTERED 1
#define ALGO_LARGEST 2
#define ALGO_NORMAL 3
#define LINE_ANALYSIS_ALGORITHM ALGO_MOST_CENTERED

// ---------------- UART ----------------
#if WEBSERVER
#define ESP_BAUD 115200
char command_buffer[32];
uint8_t cmdIndex = 0;
#endif

// ---------------- PINS ----------------
#define MOTOR_A_PWM PA1
#define MOTOR_A_IN1 PB8
#define MOTOR_A_IN2 PB9
#define MOTOR_B_PWM PA6
#define MOTOR_B_IN1 PA5
#define MOTOR_B_IN2 PA4
#define UEN_PIN PA15
#define STBY_PIN PB12
#define JUNCTION_PULSE PB4

const int SENSOR_BAUD_RATE = 230400;
const int DEBUG_BAUD_RATE = 115200;

// ---------------- PID ----------------
float Kp = 5.150;
float Kd = 4.550;
int SLOW_SPEED = 130;
int baseSpeed = 180;
int maxSpeed = 255;
int minSpeed = -255;
int setPoint = 35;
int junctionSpeed = 120;

int lastError = 0;
int error = 0;
unsigned long previousTime = 0;

// ---------------- LINE LOST ----------------
const int LAST_END_LEFT = 0;
const int LAST_END_RIGHT = 1;
const int LAST_END_UNKNOWN = 2;
int last_end = LAST_END_UNKNOWN;

unsigned long last_end_update_time = 0;
unsigned int LAST_END_TIMEOUT_MS = 110;
unsigned long line_lost_time = 0;
unsigned int DASHED_LINE_GRACE_PERIOD_MS = 180;

int LINE_LOST_TURN_SPEED = 150;
int LINE_LOST_PIVOT_SPEED = -150;

// ---------------- STRUCTS ----------------
enum LineFeature { NORMAL_LINE, JUNCTION, LINE_LOST };

struct LineAnalysisResult {
  int position;
  LineFeature featureType;
  bool isComplex;
};

// ---------------- PROTOTYPES ----------------
void motorspeed(int r, int l);
void pidTurn(int error, bool isComplex);
void handleLineLost();
LineAnalysisResult analyzeSensorData(uint8_t raw);
void processCommand(char* cmd);
void readUART();

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(DEBUG_BAUD_RATE);
  delay(1000);
  Serial.println("STM32 UART PID Receiver");

#if WEBSERVER
  Serial2.begin(ESP_BAUD); // PB6 TX, PB7 RX
#endif

  Serial1.begin(SENSOR_BAUD_RATE); // LSA08

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

// ---------------- LOOP ----------------
void loop() {
#if WEBSERVER
  readUART();
#endif

  if (Serial1.available()) {
    uint8_t raw = Serial1.read();
    LineAnalysisResult line = analyzeSensorData(raw);

    if (line.featureType == NORMAL_LINE) {
      error = line.position - setPoint;
      pidTurn(error, line.isComplex);
    }
    else if (line.featureType == JUNCTION) {
      motorspeed(junctionSpeed, junctionSpeed);
    }
    else {
      handleLineLost();
    }
  }
}

// ---------------- UART RX ----------------
void readUART() {
  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '\n' || c == '\r') {
      command_buffer[cmdIndex] = '\0';
      processCommand(command_buffer);
      cmdIndex = 0;
    } else if (cmdIndex < sizeof(command_buffer) - 1) {
      command_buffer[cmdIndex++] = c;
    }
  }
}

// ---------------- PROCESS CMD ----------------
void processCommand(char* cmd) {
  char* sep = strchr(cmd, ':');
  if (!sep) return;

  *sep = '\0';
  char* val = sep + 1;

#if USB_DEBUG
  Serial.print("RX ");
  Serial.print(cmd);
  Serial.print(": ");
  Serial.println(val);
#endif

  if (!strcmp(cmd, "KP")) Kp = atof(val);
  else if (!strcmp(cmd, "KD")) Kd = atof(val);
  else if (!strcmp(cmd, "BS")) baseSpeed = atoi(val);
  else if (!strcmp(cmd, "MS")) maxSpeed = atoi(val);
  else if (!strcmp(cmd, "MN")) minSpeed = atoi(val);
  else if (!strcmp(cmd, "SS")) SLOW_SPEED = atoi(val);
  else if (!strcmp(cmd, "SP")) setPoint = atoi(val);
  else if (!strcmp(cmd, "LT")) LAST_END_TIMEOUT_MS = atoi(val);
  else if (!strcmp(cmd, "DG")) DASHED_LINE_GRACE_PERIOD_MS = atoi(val);
  else if (!strcmp(cmd, "TS")) LINE_LOST_TURN_SPEED = atoi(val);
  else if (!strcmp(cmd, "PS")) LINE_LOST_PIVOT_SPEED = atoi(val);
  else if (!strcmp(cmd, "JS")) junctionSpeed = atoi(val);
}

// ---------------- PID ----------------
void pidTurn(int error, bool isComplex) {
  unsigned long now = micros();
  float dt = (now - previousTime) / 1000000.0;
  previousTime = now;

  if (dt <= 0 || dt > 0.3) dt = 0;

  float d = (dt > 0) ? (error - lastError) / dt : 0;
  int correction = (Kp * error) + (Kd * d);
  lastError = error;

  int dynBase = isComplex ? SLOW_SPEED : baseSpeed;

  int left = constrain(dynBase + correction, -maxSpeed, maxSpeed);
  int right = constrain(dynBase - correction, -maxSpeed, maxSpeed);

  motorspeed(right, left);
}

// ---------------- LINE LOST ----------------
void handleLineLost() {
  if (last_end == LAST_END_RIGHT)
    motorspeed(LINE_LOST_PIVOT_SPEED, LINE_LOST_TURN_SPEED);
  else if (last_end == LAST_END_LEFT)
    motorspeed(LINE_LOST_TURN_SPEED, LINE_LOST_PIVOT_SPEED);
}

// ---------------- MOTOR ----------------
void motorspeed(int r, int l) {
  digitalWrite(MOTOR_B_IN1, r > 0);
  digitalWrite(MOTOR_B_IN2, r < 0);
  analogWrite(MOTOR_B_PWM, abs(r));

  digitalWrite(MOTOR_A_IN1, l > 0);
  digitalWrite(MOTOR_A_IN2, l < 0);
  analogWrite(MOTOR_A_PWM, abs(l));
}
