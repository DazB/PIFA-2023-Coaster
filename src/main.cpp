#include <Arduino.h>
#include <VarSpeedServo.h> 

#define PIN_SENSOR_LOAD 2
#define PIN_SENSOR_TOP 3

// Sensor struct
typedef struct {
  uint8_t pin;
  uint16_t counterOn = 0;
  uint16_t counterOff = 0;
  bool sensorOn = false;
  bool sensorOnOld = false;
} Sensor;

// IR Sensors
Sensor sensorLoad;
Sensor sensorTop;

// Serial buffer
char rxBuffer[32];            // an array to store the received data
boolean newDataFlag = false;  // flag for new data

// Servo
VarSpeedServo servoGate;  // create servo object to control a servo

// Function prototypes
void sensorDebounce(Sensor* sensor);

/**
 * Setup code is run once at beginning 
 */
void setup() {
  // Setup serial 
  Serial.begin(9600);
  // Setup sensors
  sensorLoad.pin = PIN_SENSOR_LOAD;
  sensorTop.pin = PIN_SENSOR_TOP;
  pinMode(sensorLoad.pin, INPUT);
  pinMode(sensorTop.pin, INPUT);
  // Setup servo
  servoGate.attach(9); // Attaches servo to pin 9

}

/** 
 * Main loop 
 */
void loop() {
  // If we have incoming serial data
  if (newDataFlag) {
    newDataFlag = false;
    // If open gate
    if (memcmp(rxBuffer, "OP", 2) == 0) {
      servoGate.write(40, 20);
    }
    // If close gate
    if (memcmp(rxBuffer, "CL", 2) == 0) {
      servoGate.write(250);
    }
    // If Watchdog 1
    if (memcmp(rxBuffer, "W1", 2) == 0) {
      // Send WD
      Serial.print("W0\r");
    }
    // If Watchdog 0
    if (memcmp(rxBuffer, "W0", 2) == 0) {
      // Send WD
      Serial.print("W1\r");
    }
  }

  // Compare old sensor reading to current one. On rising edge, send high.
  // On falling, send low.
  sensorDebounce(&sensorLoad);
  sensorDebounce(&sensorTop);

  if (sensorLoad.sensorOn != sensorLoad.sensorOnOld) {
    if (sensorLoad.sensorOn) {
      Serial.print("S1H\r");
      sensorLoad.sensorOnOld = true;  
    }
    else {
      Serial.print("S1L\r");
      sensorLoad.sensorOnOld = false;  
    }
  }

  if (sensorTop.sensorOn != sensorTop.sensorOnOld) {
    if (sensorTop.sensorOn) {
      Serial.print("S2H\r");
      sensorTop.sensorOnOld = true;  
    }
    else {
      Serial.print("S2L\r");
      sensorTop.sensorOnOld = false;  
    }
  }
}

/** 
 * Sensor debounce. Turns on immediately, debounce off.
 * Uses counter to debounce sensor input turning off
 */
void sensorDebounce(Sensor* sensor) {
  if (digitalRead(sensor->pin) == 0) {
    sensor->counterOff = 0;
    sensor->sensorOn = true;
  }
  else {
    if (sensor->counterOff == 5000) {
      sensor->sensorOn = false;
    }
    else {
      sensor->counterOff++;
    }
  }
}


/** 
 * Implicitly called at the end of loop() when serial data is available
 */
void serialEvent() {
  static uint8_t rxIndex = 0;
  // Read serial character
  char rx = Serial.read();
  // If not termining character
  if (rx != '\r') {
    // Add to buffer
    rxBuffer[rxIndex] = rx;
    rxIndex++;
  }
  else {
    // Terminating character. Reset input, set flag
    rxIndex = 0;
    newDataFlag = true;
  }
}
