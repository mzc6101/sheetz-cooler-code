#include "DFRobot_TMF8x01.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

// I2C Multiplexer Address
#define TCA9548A_ADDR 0x70  // Address of the single TCA9548A I2C multiplexer
#define EN       -1         // EN pin of TMF8801 is not used
#define INT      -1         // INT pin of TMF8801 is not used

// NeoPixel Configuration
#define NEOPIXEL_PIN    6    // Pin connected to NeoPixel data
#define NEOPIXEL_COUNT  3    // Number of NeoPixels (one per sensor)

// Create NeoPixel strip object
Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Create sensor instances for channels 0 to 7 on the same multiplexer
DFRobot_TMF8801 tof0(EN, INT);
DFRobot_TMF8801 tof1(EN, INT);
DFRobot_TMF8801 tof2(EN, INT);
DFRobot_TMF8801 tof3(EN, INT);
DFRobot_TMF8801 tof4(EN, INT);
DFRobot_TMF8801 tof5(EN, INT);
DFRobot_TMF8801 tof6(EN, INT);
DFRobot_TMF8801 tof7(EN, INT);

// Struct to hold sensor information
struct SensorInfo {
  DFRobot_TMF8801* sensor;
  uint8_t channel;
  uint8_t caliDataBuf[14];
  float distance;
  const char* name;
};

// Sensor configurations for channels 0 to 7
SensorInfo sensors[] = {
  { &tof0, 0, {0}, 0.0, "Sensor0" },
  { &tof1, 1, {0}, 0.0, "Sensor1" },
  { &tof2, 2, {0}, 0.0, "Sensor2" },
  { &tof3, 3, {0}, 0.0, "Sensor3" },
  { &tof4, 4, {0}, 0.0, "Sensor4" },
  { &tof5, 5, {0}, 0.0, "Sensor5" },
  { &tof6, 6, {0}, 0.0, "Sensor6" },
  { &tof7, 7, {0}, 0.0, "Sensor7" }
};

// Mapping of NeoPixels to Sensors
// NeoPixel0 -> Sensor6
// NeoPixel1 -> Sensor7
// NeoPixel2 -> Sensor0 (optional)
const uint8_t neoPixelMapping[NEOPIXEL_COUNT] = {6, 7, 0};

// Function to select the specific channel on the TCA9548A
void tcaSelect(uint8_t channel) {
  if (channel > 7) {
    Serial.println("Invalid channel selection!");
    return;
  }
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  if (Wire.endTransmission() != 0) {
    Serial.println("Error selecting channel " + String(channel) + " on multiplexer.");
  } else {
    Serial.println("Channel " + String(channel) + " selected on multiplexer.");
  }
}

// Function to set NeoPixel color based on distance (Flipped Logic)
void setPixelColorByDistance(uint8_t pixel, float distance) {
  uint32_t color;
  if (distance < 300.0) {
    color = strip.Color(0, 255, 0); // Green
  } else if (distance >= 300.0 && distance <= 499.0) {
    color = strip.Color(255, 255, 0); // Yellow
  } else {
    color = strip.Color(255, 0, 0); // Red
  }
  strip.setPixelColor(pixel, color);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Initialize I2C communication

  // Initialize NeoPixel strip
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  // Initialize and calibrate each sensor
  for (int i = 0; i < sizeof(sensors) / sizeof(SensorInfo); i++) {
    SensorInfo* s = &sensors[i];

    Serial.print("Initializing ");
    Serial.println(s->name);

    // Select the appropriate channel
    tcaSelect(s->channel);
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

  // Initialize all NeoPixels to off
  strip.clear();
  strip.show();
}

void loop() {
  for (int i = 0; i < sizeof(sensors) / sizeof(SensorInfo); i++) {
    SensorInfo* s = &sensors[i];

    // Select the appropriate channel
    tcaSelect(s->channel);
    delay(100);

    // Check if the sensor data is ready
    if (s->sensor->isDataReady()) {
      s->distance = s->sensor->getDistance_mm();  // Get distance

      Serial.print(s->name);
      Serial.print(" Distance = ");
      Serial.print(s->distance);
      Serial.println(" mm");

      // Update NeoPixel if it's mapped
      for (uint8_t pix = 0; pix < NEOPIXEL_COUNT; pix++) {
        if (s->channel == neoPixelMapping[pix]) {
          setPixelColorByDistance(pix, s->distance);
          strip.show();
        }
      }

    } else {
      Serial.print("No data available on ");
      Serial.println(s->name);
    }
    delay(200);  // Wait before switching to the next sensor
  }

  // Optional: Add a small delay to avoid flooding the I2C bus
  delay(1000);
}
