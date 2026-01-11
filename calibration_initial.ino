#define NUM_SENSORS 8
#define ALPHA 0.3   // smoothing factor (0 < ALPHA < 1)

// ADC pins (Arduino STM32 core names)
const uint8_t sensorPins[NUM_SENSORS] = {
  PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7
};

uint16_t rawValues[NUM_SENSORS];
float filteredValues[NUM_SENSORS];
uint16_t minVal[NUM_SENSORS];
uint16_t maxVal[NUM_SENSORS];
uint16_t threshold[NUM_SENSORS];

bool calibrated = false;
unsigned long calibStartTime;
#define CALIB_TIME 20000   // 10 seconds

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // Initialize filter with first readings
 for (int i = 0; i < NUM_SENSORS; i++) {
    minVal[i] = 4095;
    maxVal[i] = 0;

    rawValues[i] = analogRead(sensorPins[i]);
    filteredValues[i] = rawValues[i];
  }

  calibStartTime = millis();
  Serial.println("Calibration started");

  Serial.println("STM32F411 8-Sensor ADC with EMA Filter");
}

void loop() {

  // Read & filter
  for (int i = 0; i < NUM_SENSORS; i++) {
    rawValues[i] = analogRead(sensorPins[i]);
    filteredValues[i] =
      ALPHA * rawValues[i] +
      (1.0 - ALPHA) * filteredValues[i];
  }

  //CALIBRATION 
  if (!calibrated) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      uint16_t val = filteredValues[i];
      if (val < minVal[i]) minVal[i] = val;
      if (val > maxVal[i]) maxVal[i] = val;
    }

    if (millis() - calibStartTime >= CALIB_TIME) {
      calibrated = true;

      Serial.println("Calibration done!");
      for (int i = 0; i < NUM_SENSORS; i++) {
        threshold[i] = (minVal[i] + maxVal[i]) / 2;
        Serial.print("S");
        Serial.print(i);
        Serial.print(" Th:");
        Serial.println(threshold[i]);
      }
    }

    delay(20);
    return;  
  }

  
for (int i = 0; i < NUM_SENSORS; i++) {

  bool isBlack = (filteredValues[i] > threshold[i]);

  Serial.print("S");
  Serial.print(i);
  Serial.print(":");
  Serial.print((int)filteredValues[i]);  // analog value
  Serial.print("[");

  if (isBlack)
    Serial.print("1");   // BLACK
  else
    Serial.print("0");   // WHITE

  Serial.print("]  ");
}
Serial.println();

delay(20);
}

// higher values on black
