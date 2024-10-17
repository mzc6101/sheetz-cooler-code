#include "DFRobot_TMF8x01.h"
#include <Wire.h>

#define TCA9548A_ADDR_0 0x70  // First TCA9548A I2C address
#define TCA9548A_ADDR_1 0x71  // Second TCA9548A I2C address
#define EN       -1           // EN pin of TMF8801 is not used
#define INT      -1           // INT pin of TMF8801 is not used

// Create sensor instances
DFRobot_TMF8801 tof0(EN, INT);  // Sensor on multiplexer at 0x70, channel 0
DFRobot_TMF8801 tof1(EN, INT);  // Sensor on multiplexer at 0x70, channel 1
DFRobot_TMF8801 tof2(EN, INT);  // Sensor on multiplexer at 0x71, channel 2
DFRobot_TMF8801 tof7(EN, INT);  // Sensor on multiplexer at 0x71, channel 7

// Struct to hold sensor information
struct SensorInfo {
  DFRobot_TMF8801* sensor;
  uint8_t multiplexerAddress;
  uint8_t channel;
  uint8_t caliDataBuf[14];
  int ledPinOver200mm;
  int ledPinUnder200mm;
  float distance;
  const char* name;
};

// Sensor configurations
SensorInfo sensors[] = {
  { &tof0, TCA9548A_ADDR_0, 0, {0}, 2, 3, 0.0, "Sensor0" },  // Sensor on 0x70, channel 0
  { &tof1, TCA9548A_ADDR_0, 1, {0}, 4, 5, 0.0, "Sensor1" },  // Sensor on 0x70, channel 1
  { &tof2, TCA9548A_ADDR_1, 2, {0}, 6, 7, 0.0, "Sensor2" },  // Sensor on 0x71, channel 2
  { &tof7, TCA9548A_ADDR_1, 7, {0}, 8, 9, 0.0, "Sensor7" }   // Sensor on 0x71, channel 7
};

// Function to select the specific channel on the TCA9548A
void tcaSelect(uint8_t addr, uint8_t channel) {
  if (channel > 7) {
    Serial.println("Invalid channel selection!");
    return;
  }
  Wire.beginTransmission(addr);
  Wire.write(1 << channel);
  if (Wire.endTransmission() != 0) {
    Serial.println("Error selecting channel " + String(channel) + " on multiplexer at address 0x" + String(addr, HEX));
  } else {
    Serial.println("Channel " + String(channel) + " selected on multiplexer at address 0x" + String(addr, HEX));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Initialize I2C communication

  // Initialize LED pins for each sensor
  for (int i = 0; i < sizeof(sensors)/sizeof(SensorInfo); i++) {
    pinMode(sensors[i].ledPinOver200mm, OUTPUT);
    pinMode(sensors[i].ledPinUnder200mm, OUTPUT);
  }

  // Initialize and calibrate each sensor
  for (int i = 0; i < sizeof(sensors)/sizeof(SensorInfo); i++) {
    SensorInfo* s = &sensors[i];

    Serial.print("Initializing ");
    Serial.println(s->name);

    // Select the appropriate multiplexer and channel
    tcaSelect(s->multiplexerAddress, s->channel);
    delay(100);

    // Initialize sensor
    if (s->sensor->begin() != 0) {
      Serial.println("Failed to initialize " + String(s->name));
    } else {
      Serial.println(String(s->name) + " initialized.");
    }

    // Calibrate sensor
    Serial.print("Calibrating ");
    Serial.println(s->name);

    bool calibrationSuccess = false;
    unsigned long startTime = millis();

    while (!s->sensor->getCalibrationData(s->caliDataBuf, sizeof(s->caliDataBuf))) {
      Serial.print(".");
      delay(1000);
      if (millis() - startTime > 5000) {
        Serial.println("\nCalibration timeout on " + String(s->name));
        break;
      }
    }

    if (millis() - startTime <= 5000) {
      calibrationSuccess = true;
      Serial.println(" Calibration complete for " + String(s->name));

      // Apply calibration data and start measurement
      s->sensor->setCalibrationData(s->caliDataBuf, sizeof(s->caliDataBuf));
      if (s->sensor->startMeasurement() != 0) {
        Serial.println("Failed to start measurement on " + String(s->name));
      } else {
        Serial.println("Measurement started on " + String(s->name));
      }

      // Optional: Print calibration data
      Serial.print("Calibration Data for ");
      Serial.print(s->name);
      Serial.print(": ");
      for (int j = 0; j < 14; j++) {
        Serial.print(s->caliDataBuf[j], HEX);
        Serial.print(" ");
      }
      Serial.println();

    } else {
      Serial.println("Failed to calibrate " + String(s->name));
    }
  }
}

void loop() {
  for (int i = 0; i < sizeof(sensors)/sizeof(SensorInfo); i++) {
    SensorInfo* s = &sensors[i];

    // Select the appropriate multiplexer and channel
    tcaSelect(s->multiplexerAddress, s->channel);
    delay(100);

    // Check if the sensor data is ready
    if (s->sensor->isDataReady()) {
      s->distance = s->sensor->getDistance_mm();  // Get distance

      Serial.print(s->name);
      Serial.print(" Distance = ");
      Serial.print(s->distance);
      Serial.println(" mm");

      // Control LEDs based on distance
      if (s->distance > 200) {
        digitalWrite(s->ledPinOver200mm, HIGH);
        digitalWrite(s->ledPinUnder200mm, LOW);
      } else {
        digitalWrite(s->ledPinOver200mm, LOW);
        digitalWrite(s->ledPinUnder200mm, HIGH);
      }
    } else {
    
      Serial.print("No data available on ");
      Serial.println(s->name);
    }
    delay(200);  // Wait before switching to the next sensor
  }
}