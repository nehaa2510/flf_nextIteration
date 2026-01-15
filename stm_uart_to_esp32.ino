#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <HardwareSerial.h>

// ---- FORCE CREATE Serial2 if core doesn't provide it ----
HardwareSerial Serial2(USART2);

// ================= CONFIG =================
#define WEBSERVER 1
#define USB_DEBUG 0
#define LINE_SENSOR_MODE 1

#define ALGO_MOST_CENTERED 1
#define LINE_ANALYSIS_ALGORITHM ALGO_MOST_CENTERED

#if WEBSERVER
#define ESP_UART_BAUD 115200
char command_buffer[32];
uint8_t cmdIndex = 0;
#define ESPSerial Serial2   // ✅ USART2 (PA2 / PA3)
#endif

// --- Pin Definitions ---
#define MOTOR_A_PWM PA1
#define MOTOR_A_IN1 PB8
#define MOTOR_A_IN2 PB9
#define MOTOR_B_PWM PA6
#define MOTOR_B_IN1 PA5
#define MOTOR_B_IN2 PA4
#define UEN_PIN PA15
#define STBY_PIN PB12
#define JUNCTION_PULSE PB4

// --- Serial Baud Rates ---
const int SENSOR_BAUD_RATE = 230400;
const int DEBUG_BAUD_RATE  = 115200;

// --- PID ---
float Kp = 5.150;
float Kd = 4.550;
int SLOW_SPEED = 130;
int baseSpeed = 180;
int maxSpeed = 255;
int setPoint = 35;
int minSpeed = -255;
int junctionSpeed = 120;

int lastError = 0;
int error = 0;
unsigned long previousTime = 0;

// --- Line memory ---
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

enum LineFeature { NORMAL_LINE, JUNCTION, LINE_LOST };

struct LineAnalysisResult {
  int position;
  LineFeature featureType;
  bool isComplex;
};

// ---- Function prototypes (UNCHANGED) ----
void motorspeed(int rightmotorspeed, int leftmotorspeed);
void pidTurn(int error, bool isComplex);
void handleLineLost();
void processCommand(char* command);
LineAnalysisResult analyzeSensorData(uint8_t raw);

// ================= SETUP =================
void setup() {
  Serial.begin(DEBUG_BAUD_RATE);
  delay(1500);

#if WEBSERVER
  ESPSerial.begin(ESP_UART_BAUD);   // ✅ UART instead of I2C
#endif

  Serial1.begin(SENSOR_BAUD_RATE);  // LSA08

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

// ================= analyzeSensorData =================
LineAnalysisResult analyzeSensorData(uint8_t rawSensorByte) {
    LineAnalysisResult result;
    result.position = 255;
    result.isComplex = false;

    if (rawSensorByte == 0xFF) {
        result.featureType = JUNCTION;
        result.position = 35;
        return result;
    }
    if (rawSensorByte == 0x00) {
        result.featureType = LINE_LOST;
        return result;
    }

    int weights[8] = {0,10,20,30,40,50,60,70};
    uint8_t sensors[8];
    for (int i = 0; i < 8; i++) sensors[i] = (rawSensorByte >> i) & 1;

    struct Block { int start; int len; };
    Block blocks[8];
    int block_count = 0;

    for (int i = 0; i < 8; ) {
        if (sensors[i]) {
            blocks[block_count].start = i;
            blocks[block_count].len = 0;
            while (i < 8 && sensors[i]) {
                blocks[block_count].len++;
                i++;
            }
            block_count++;
        } else i++;
    }

    if (block_count > 1) result.isComplex = true;
    if (block_count == 0) {
        result.featureType = LINE_LOST;
        return result;
    }

    int best = 0;
    float minDist = 100;
    for (int i = 0; i < block_count; i++) {
        float c = blocks[i].start + (blocks[i].len - 1) / 2.0;
        float d = abs(c - 3.5);
        if (d < minDist) {
            minDist = d;
            best = i;
        }
    }

    int sum = 0, cnt = 0;
    for (int i = blocks[best].start; i < blocks[best].start + blocks[best].len; i++) {
        sum += weights[i];
        cnt++;
    }

    result.position = sum / cnt;
    result.featureType = NORMAL_LINE;
    return result;
}

// ================= LOOP =================
void loop() {

#if WEBSERVER
  while (ESPSerial.available()) {
    char c = ESPSerial.read();
    if (c == '\n' || c == '\r') {
      command_buffer[cmdIndex] = '\0';
      processCommand(command_buffer);
      cmdIndex = 0;
    } else if (cmdIndex < sizeof(command_buffer) - 1) {
      command_buffer[cmdIndex++] = c;
    }
  }
#endif

  if (last_end != LAST_END_UNKNOWN &&
      millis() - last_end_update_time > LAST_END_TIMEOUT_MS) {
    last_end = LAST_END_UNKNOWN;
  }

  if (Serial1.available()) {
    uint8_t raw = Serial1.read();
    LineAnalysisResult line = analyzeSensorData(raw);

    switch (line.featureType) {
      case NORMAL_LINE: {
        line_lost_time = 0;

        if (raw & 0x01) {
          last_end = LAST_END_LEFT;
          last_end_update_time = millis();
        } else if (raw & 0x80) {
          last_end = LAST_END_RIGHT;
          last_end_update_time = millis();
        }

        error = line.position - setPoint;
        pidTurn(error, line.isComplex);
        break;
      }

      case JUNCTION:
        motorspeed(junctionSpeed, junctionSpeed);
        break;

      case LINE_LOST:
        handleLineLost();
        break;
    }
  }
}

// ================= UART COMMAND =================
void processCommand(char* command) {
    char* separator = strchr(command, ':');
    if (!separator) return;
    *separator = '\0';
    char* valueStr = separator + 1;

    if      (!strcmp(command,"KP")) Kp = atof(valueStr);
    else if (!strcmp(command,"KD")) Kd = atof(valueStr);
    else if (!strcmp(command,"BS")) baseSpeed = atoi(valueStr);
    else if (!strcmp(command,"MS")) maxSpeed = atoi(valueStr);
    else if (!strcmp(command,"MN")) minSpeed = atoi(valueStr);
    else if (!strcmp(command,"SS")) SLOW_SPEED = atoi(valueStr);
    else if (!strcmp(command,"SP")) setPoint = atoi(valueStr);
    else if (!strcmp(command,"LT")) LAST_END_TIMEOUT_MS = atoi(valueStr);
    else if (!strcmp(command,"DG")) DASHED_LINE_GRACE_PERIOD_MS = atoi(valueStr);
    else if (!strcmp(command,"TS")) LINE_LOST_TURN_SPEED = atoi(valueStr);
    else if (!strcmp(command,"PS")) LINE_LOST_PIVOT_SPEED = atoi(valueStr);
    else if (!strcmp(command,"JS")) junctionSpeed = atoi(valueStr);
}

// ================= MOTOR FUNCTIONS (UNCHANGED) =================
void handleLineLost() {
    if (last_end == LAST_END_RIGHT) {
        motorspeed(LINE_LOST_PIVOT_SPEED, LINE_LOST_TURN_SPEED);
    } else if (last_end == LAST_END_LEFT) {
        motorspeed(LINE_LOST_TURN_SPEED, LINE_LOST_PIVOT_SPEED);
    }
}

void pidTurn(int error, bool isComplex) {
    unsigned long currentTime = micros();
    float deltaTime = (currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;

    if (deltaTime <= 0.0 || deltaTime > 0.3) deltaTime = 0.0;

    float derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
    int motorSpeed = round((Kp * error) + (Kd * derivative));
    lastError = error;

    int dynamicBaseSpeed = isComplex ? SLOW_SPEED + 1 : baseSpeed;

    int currentSpeed = map(abs(error), 0, setPoint / 1.7,
                           dynamicBaseSpeed, SLOW_SPEED);
    currentSpeed = constrain(currentSpeed, SLOW_SPEED, dynamicBaseSpeed);

    int leftMotorSpeed  = constrain(currentSpeed + motorSpeed, -maxSpeed, maxSpeed);
    int rightMotorSpeed = constrain(currentSpeed - motorSpeed, -maxSpeed, maxSpeed);

    motorspeed(rightMotorSpeed, leftMotorSpeed);
}


void motorspeed(int rightmotorspeed, int leftmotorspeed) {
    if (rightmotorspeed > 0) {
        digitalWrite(MOTOR_B_IN1, HIGH);
        digitalWrite(MOTOR_B_IN2, LOW);
    } else if (rightmotorspeed < 0) {
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, HIGH);
    } else {
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, LOW);
    }
    analogWrite(MOTOR_B_PWM, constrain(abs(rightmotorspeed), 0, 255));

    if (leftmotorspeed > 0) {
        digitalWrite(MOTOR_A_IN1, HIGH);
        digitalWrite(MOTOR_A_IN2, LOW);
    } else if (leftmotorspeed < 0) {
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, HIGH);
    } else {
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, LOW);
    }
    analogWrite(MOTOR_A_PWM, constrain(abs(leftmotorspeed), 0, 255));
}
