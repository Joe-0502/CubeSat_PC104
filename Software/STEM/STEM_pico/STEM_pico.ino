#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050_tockn.h>
#include <EEPROM.h>  // Use Pico's EEPROM emulation library

#define SEALEVELPRESSURE_HPA (1013.25)
#define EEPROM_SIZE 64  // Define EEPROM size (bytes), adjust as needed

// --- Pin Definitions (Match your wiring!) ---
// Serial1 (UART0) - To Raspberry Pi
const int RPI_TX_PIN = 0;  // Pico GP0 -> Pi RX
const int RPI_RX_PIN = 1;  // Pico GP1 <- Pi TX
// Serial2 (UART1) - For Debugging
const int DBG_TX_PIN = 8;  // Pico GP8 -> USB-Serial TX
const int DBG_RX_PIN = 9;  // Pico GP9 <- USB-Serial RX
// I2C0 Pins (BME280, MPU6050)
const int I2C0_SDA_PIN = 12;  // Pico GP12
const int I2C0_SCL_PIN = 13;  // Pico GP13
// LED Pins
const int greenLedPin = 19;  // Pico GP19
const int blueLedPin = 18;   // Pico GP18
// Analog Pins
const int analogTempPin = A2;  // Pico GP28

Adafruit_BME280 bme;
MPU6050 mpu6050(Wire);  // Uses default Wire (I2C0)

int bmePresent;

int first_time = true;  // Flag for first debug print over Serial2
int first_read = true;  // Flag for initial acceleration reading

// --- Temperature Calibration (RECALIBRATE FOR PICO 3.3V ADC!) ---
// These values MUST be re-calibrated for Pico's 10-bit ADC (0-1023) at 3.3V
float T2 = 25.0;   // Temperature data point 1 (e.g., 25°C)
float R2 = 671.0;  // Reading data point 1 (at T2 on Pico ADC 0-1023) - EXAMPLE VALUE!
float T1 = 15.5;   // Temperature data point 2 (e.g., 15.5°C)
float R1 = 695.0;  // Reading data point 2 (at T1 on Pico ADC 0-1023) - EXAMPLE VALUE!
// --- End Calibration ---

int sensorValue;
float Temp;
float rest;

// --- Placeholders from File 1 ---
float flon = 0.0, flat = 0.0, flalt = 0.0;  // GPS placeholders
char sensor_end_flag[] = "_END_FLAG_";
char sensor_start_flag[] = "_START_FLAG_";

// --- Function Prototypes ---
void eeprom_word_write(int addr, int val);
short eeprom_word_read(int addr);
void blink_setup();
void blink(int length);
void led_set(int ledPin, bool state);
int read_analog();

// --- Payload Extension Declarations (Add weak attribute if definition might be missing) ---
extern void payload_setup();
extern void payload_loop();
// --- End Function Prototypes ---


void setup() {

  // Initialize Serial2 (Debug Port)
  Serial2.setTX(DBG_TX_PIN);
  Serial2.setRX(DBG_RX_PIN);
  Serial2.begin(115200);
  Serial2.println("UART1 (Debug): Starting!");

  // Initialize Serial1 (RPi Port)
  Serial1.setTX(RPI_TX_PIN);
  Serial1.setRX(RPI_RX_PIN);
  Serial1.begin(115200);
  Serial2.println("UART1 (Debug): Serial1 (RPi) Initialized.");

  // Initialize I2C0
  Wire.setSCL(I2C0_SCL_PIN);
  Wire.setSDA(I2C0_SDA_PIN);
  Wire.begin();
  Serial2.println("UART1 (Debug): I2C0 Initialized.");

  // Initialize EEPROM Emulation
  EEPROM.begin(EEPROM_SIZE);
  Serial2.print("UART1 (Debug): EEPROM Emulation Initialized. Size: ");
  Serial2.println(EEPROM.length());

  // Setup LEDs
  blink_setup();
  Serial2.println("UART1 (Debug): LEDs Initialized.");

  // Startup blink sequence
  blink(500);
  delay(250);
  blink(500);
  delay(250);
  led_set(greenLedPin, HIGH);
  delay(250);
  led_set(greenLedPin, LOW);
  led_set(blueLedPin, HIGH);
  delay(250);
  led_set(blueLedPin, LOW);
  Serial2.println("UART1 (Debug): LED Test Complete.");

  // Initialize BME280
  if (bme.begin(0x76, &Wire)) {
    bmePresent = 1;
    Serial2.println("UART1 (Debug): BME280 Found!");
  } else {
    Serial2.println("UART1 (Debug): Could not find BME280 sensor, check wiring/address!");
    bmePresent = 0;
  }

  // Initialize MPU6050
  mpu6050.begin();
  Serial2.println("UART1 (Debug): MPU6050 Initializing...");

  // Gyro Offset Calibration (Same as before, EEPROM handling is correct)
  if (eeprom_word_read(0) == 0xA07) {
    Serial2.println("UART1 (Debug): Reading gyro offsets from EEPROM");
    // ... (reading code is fine) ...
    mpu6050.setGyroOffsets(((float)eeprom_word_read(1)) / 100.0, ((float)eeprom_word_read(2)) / 100.0, ((float)eeprom_word_read(3)) / 100.0);
  } else {
    Serial2.println("UART1 (Debug): Calculating gyro offsets and storing in EEPROM (Flash!)");
    mpu6050.calcGyroOffsets(true);
    Serial2.println("UART1 (Debug): Offset calculation complete.");
    eeprom_word_write(0, 0xA07);
    eeprom_word_write(1, (int)(mpu6050.getGyroXoffset() * 100.0 + 0.5));
    eeprom_word_write(2, (int)(mpu6050.getGyroYoffset() * 100.0 + 0.5));
    eeprom_word_write(3, (int)(mpu6050.getGyroZoffset() * 100.0 + 0.5));
    Serial2.println("UART1 (Debug): Committing offsets to EEPROM...");
    if (!EEPROM.commit()) {
      Serial2.println("UART1 (Debug): ERROR! EEPROM commit failed during calibration.");
    } else {
      Serial2.println("UART1 (Debug): Gyro offsets committed to EEPROM.");
    }
    // ... (readback code is fine) ...
  }

  Serial2.println("UART1 (Debug): Done with Setup()");
}

void loop() {

  // --- MODIFICATION: Send data periodically instead of waiting for Serial1 input ---
  // This block will now execute roughly every 100ms + processing time

  blink(20);  // Blink onboard LED quickly to show activity

  // --- MODIFICATION: Restore output format similar to File 1 ---
  Serial1.print(sensor_start_flag);  // Add start flag

  if (bmePresent) {
    Serial1.print("OK BME280 ");
    Serial1.print(bme.readTemperature());
    Serial1.print(" ");
    Serial1.print(bme.readPressure() / 100.0F);
    Serial1.print(" ");
    Serial1.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial1.print(" ");
    Serial1.print(bme.readHumidity());
  } else {
    Serial1.print("OK BME280 0.0 0.0 0.0 0.0");
  }
  mpu6050.update();

  Serial1.print(" MPU6050 ");
  Serial1.print(mpu6050.getGyroX());
  Serial1.print(" ");
  Serial1.print(mpu6050.getGyroY());
  Serial1.print(" ");
  Serial1.print(mpu6050.getGyroZ());
  Serial1.print(" ");
  Serial1.print(mpu6050.getAccX());
  Serial1.print(" ");
  Serial1.print(mpu6050.getAccY());
  Serial1.print(" ");
  Serial1.print(mpu6050.getAccZ());

  sensorValue = read_analog();  // Reads analogTempPin (A2/GP28)

  // --- IMPORTANT: Recalibrate T1, R1, T2, R2 for Pico ---
  Temp = T1 + (sensorValue - R1) * ((T2 - T1) / (R2 - R1));
  // ---

  // --- MODIFICATION: Restore GPS and TMP placeholders ---
  Serial1.print(" GPS ");
  Serial1.print(flat, 4);  // GPS placeholders
  Serial1.print(" ");
  Serial1.print(flon, 4);
  Serial1.print(" ");
  Serial1.print(flalt, 2);
  Serial1.print(" TMP ");
  Serial1.print(Temp);

  // --- MODIFICATION: Call payload_loop if it exists ---
  if (payload_loop) {
    payload_loop();  // Call payload extension function
  }

  // --- MODIFICATION: Add end flag ---
  Serial1.println(sensor_end_flag);  // Use println to end the line


  // --- Motion Detection Logic (remains the same) ---
  float rotation = sqrt(pow(mpu6050.getGyroX(), 2) + pow(mpu6050.getGyroY(), 2) + pow(mpu6050.getGyroZ(), 2));
  float acceleration = sqrt(pow(mpu6050.getAccX(), 2) + pow(mpu6050.getAccY(), 2) + pow(mpu6050.getAccZ(), 2));

  if (first_read == true) {
    first_read = false;
    rest = acceleration;  // Capture baseline acceleration
    Serial2.print("UART1 (Debug): Baseline acceleration set: ");
    Serial2.println(rest);
  }

  // Update external LEDs based on motion
  if (acceleration > 1.2 * rest || acceleration < 0.8 * rest)
    led_set(greenLedPin, HIGH);
  else
    led_set(greenLedPin, LOW);

  if (rotation > 5)
    led_set(blueLedPin, HIGH);
  else
    led_set(blueLedPin, LOW);


  // --- Check for command from Debug Port (Serial2) ---
  // This part remains the same as your File 2
  if (Serial2.available() > 0) {
    blink(50);  // Blink onboard LED
    char result = Serial2.read();
    Serial2.print("UART1 (Debug): Received command: ");
    Serial2.println(result);

    if (result == 'R') {  // Reset command
      Serial2.println("UART1 (Debug): Command 'R' received. Re-running setup...");
      first_read = true;
      first_time = true;
      setup();
    } else if (result == 'C') {  // Clear EEPROM command
      Serial2.println("UART1 (Debug): Command 'C' received. Clearing gyro offsets...");
      eeprom_word_write(0, 0x00);
      Serial2.println("UART1 (Debug): Committing EEPROM clear...");
      if (!EEPROM.commit()) {
        Serial2.println("UART1 (Debug): ERROR! EEPROM commit failed.");
      } else {
        Serial2.println("UART1 (Debug): EEPROM cleared.");
      }
      first_time = true;
    } else if (result == '?') {  // Query command
      Serial2.println("UART1 (Debug): Command '?' received. Printing debug info...");
      first_time = true;
    }

    if (first_time == true) {
      first_time = false;  // Reset flag
      Serial2.print("UART1 (Debug): ");
      // ... (Debug print section remains the same) ...
      Serial2.println();  // End debug line
    }
  }

  // --- MODIFICATION: Adjust delay for periodic sending ---
  // delay(100); // Original delay
  delay(500);  // Send data roughly every 500ms (adjust as needed)
}

// --- Function Definitions ---
// These functions (eeprom_word_write/read, blink_setup, blink, led_set, read_analog)
// are correctly implemented for Pico in File 2 and don't need changes based on File 1.
// Make sure the pin numbers inside them match your Pico setup.

void eeprom_word_write(int addr, int val) {
  EEPROM.write(addr * 2, lowByte(val));
  EEPROM.write(addr * 2 + 1, highByte(val));
}

short eeprom_word_read(int addr) {
  byte low = EEPROM.read(addr * 2);
  byte high = EEPROM.read(addr * 2 + 1);
  return ((short)((high << 8) | low));
}

void blink_setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
}

void blink(int length) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(length);
  digitalWrite(LED_BUILTIN, LOW);
}

void led_set(int ledPin, bool state) {
  digitalWrite(ledPin, state ? HIGH : LOW);
}

int read_analog() {
  sensorValue = analogRead(analogTempPin);
  return (sensorValue);
}
