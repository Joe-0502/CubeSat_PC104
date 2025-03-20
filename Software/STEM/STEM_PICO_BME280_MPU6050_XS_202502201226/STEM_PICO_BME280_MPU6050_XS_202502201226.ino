

// code for Pro Micro or STM32 on the CubeSat Simulator STEM Payload board
// answers "OK" on the serial port Serial1 when queried by the Pi

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050_tockn.h>
#include <EEPROM.h>
#define SEALEVELPRESSURE_HPA (1013.25)


// Code modified for STEM board with PICO 

// PI_TXD       : GPIO0   using Serial1()
// PI_RXD       : GPIO1
// TX2          : GPIO8   using Serial2()
// RX2          : GPIO9
// LED1(G)      : GPIO17
// LED2(B)      : GPIO16
// LED(BUILT_IN): GPIO14

// I2C0_SDA     : GPIO12
// I2C0_SCL     : GPIO13
// I2C1_SDA     : GPIO6
// I2C1_SCL     : GPIO7

Adafruit_BME280 bme;
MPU6050 mpu6050(Wire);


long timer = 0;
int bmePresent;
int RXLED = LED_BUILTIN;  // The RX LED has a defined Arduino pin
int greenLED = 19;
int blueLED = 18;
int Sensor1 = 0;
float Sensor2 = 0;
void eeprom_word_write(int addr, int val);
short eeprom_word_read(int addr);
int first_time = true;
int first_read = true;

float T2 = 25; // Temperature data point 1
float R2 = 671; // Reading data point 1
float T1 = 15.5; // Temperature data point 2
float R1 = 695; // Reading data point 2


int sensorValue;
float Temp;
float rest;
  
void setup() {

  Serial2.begin(115200); // Serial Monitor for testing
  Serial1.begin(115200);  // UART1 faster speed
  Serial2.println("UART1: Starting!");

  Wire.setSCL(13);
  Wire.setSDA(12);
  Wire.begin();
  
  blink_setup();
 
  blink(500);
  delay(250);
  blink(500);
  delay(250);
  led_set(greenLED, HIGH);
  delay(250);
  led_set(greenLED, LOW);
  led_set(blueLED, HIGH);
  delay(250);
  led_set(blueLED, LOW);

  if (bme.begin(0x76)) {
    bmePresent = 1;
  } else {
    Serial2.println("UART1: Could not find a valid BME280 sensor, check wiring!");
    bmePresent = 0;
  }
 
  mpu6050.begin();
 
  if (eeprom_word_read(0) == 0xA07)
  {
    Serial2.println("UART1: Reading gyro offsets from EEPROM\n");
 
    float xOffset = ((float)eeprom_word_read(1)) / 100.0;
    float yOffset = ((float)eeprom_word_read(2)) / 100.0;
    float zOffset = ((float)eeprom_word_read(3)) / 100.0;
 
    Serial2.println(xOffset, DEC);
    Serial2.println(yOffset, DEC);
    Serial2.println(zOffset, DEC);
 
    mpu6050.setGyroOffsets(xOffset, yOffset, zOffset);
  }
  else
  {
    Serial2.println("UART1: Calculating gyro offsets and storing in EEPROM\n");
 
    mpu6050.calcGyroOffsets(true);
 
    eeprom_word_write(0, 0xA07);
    eeprom_word_write(1, (int)(mpu6050.getGyroXoffset() * 100.0) + 0.5);
    eeprom_word_write(2, (int)(mpu6050.getGyroYoffset() * 100.0) + 0.5);
    eeprom_word_write(3, (int)(mpu6050.getGyroZoffset() * 100.0) + 0.5);
 
    Serial2.println(eeprom_word_read(0), HEX);
    Serial2.println(((float)eeprom_word_read(1)) / 100.0, DEC);
    Serial2.println(((float)eeprom_word_read(2)) / 100.0, DEC);
    Serial2.println(((float)eeprom_word_read(3)) / 100.0, DEC);
  }
/**/      
  Serial2.println("UART1: Done with Setup()");
}
 
void loop() {
 
  if (Serial1.available() > 0) {
    blink(50);
    char result = Serial1.read();
//  Serial1.println(result);
//  Serial1.println("OK");
 
//    if (result == '?')
    {
      if (bmePresent) {
        Serial1.print("OK BME280 ");
        Serial1.print(bme.readTemperature());
        Serial1.print(" ");
        Serial1.print(bme.readPressure() / 100.0F);
        Serial1.print(" ");
        Serial1.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        Serial1.print(" ");
        Serial1.print(bme.readHumidity());
      } else
      {
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
 
    sensorValue = read_analog();
    Sensor2 = analogRead(1)*3.3/1024;
//  Serial.println(sensorValue);  
    Temp = T1 + (sensorValue - R1) *((T2 - T1)/(R2 - R1));
 
    Serial1.print(" XS ");
    Serial1.print(Temp);   
    Serial1.print(" ");
    Serial1.println(Sensor2);              
     
    float rotation = sqrt(mpu6050.getGyroX()*mpu6050.getGyroX() + mpu6050.getGyroY()*mpu6050.getGyroY() + mpu6050.getGyroZ()*mpu6050.getGyroZ()); 
    float acceleration = sqrt(mpu6050.getAccX()*mpu6050.getAccX() + mpu6050.getAccY()*mpu6050.getAccY() + mpu6050.getAccZ()*mpu6050.getAccZ()); 
//    Serial.print(rotation);
//    Serial.print(" ");
//    Serial.println(acceleration);
 
    if (first_read == true) {
      first_read = false;
      rest = acceleration;
    }
 
    if (acceleration > 1.2 * rest)
        led_set(greenLED, HIGH);
    else
        led_set(greenLED, LOW);
        
    if (rotation > 5)
        led_set(blueLED, HIGH);
    else
        led_set(blueLED, LOW);
    }
   
  }

  if (Serial2.available() > 0) {
    blink(50);
    char result = Serial2.read();
//    Serial.println(result);
//    Serial.println("OK");
//    Serial.println(counter++); 
   
  if (result == 'R') {
      Serial2.println("UART1: OK");
      delay(100);
      first_read = true;
      setup();
    }
  else if (result == 'C') {
      Serial2.println("UART1: Clearing stored gyro offsets in EEPROM\n");
      eeprom_word_write(0, 0x00);
      first_time = true;
      setup();
    }  
   
 if ((result == '?') || first_time == true)
    {
      first_time = false;
      if (bmePresent) {
        Serial2.print("UART1: OK BME280 ");
        Serial2.print(bme.readTemperature());
        Serial2.print(" ");
        Serial2.print(bme.readPressure() / 100.0F);
        Serial2.print(" ");
        Serial2.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        Serial2.print(" ");
        Serial2.print(bme.readHumidity());
      } else
      {
        Serial2.print("UART1: OK BME280 0.0 0.0 0.0 0.0");
      }
      mpu6050.update();
 
      Serial2.print(" MPU6050 ");
      Serial2.print(mpu6050.getGyroX());
      Serial2.print(" ");
      Serial2.print(mpu6050.getGyroY());
      Serial2.print(" ");
      Serial2.print(mpu6050.getGyroZ());
 
    Serial2.print(" ");
    Serial2.print(mpu6050.getAccX());   
    Serial2.print(" ");
    Serial2.print(mpu6050.getAccY());   
    Serial2.print(" ");
    Serial2.print(mpu6050.getAccZ());   
    
    sensorValue = read_analog();
    Sensor2 = analogRead(A1)*3.3/512;

    Temp = T1 + (sensorValue - R1) *((T2 - T1)/(R2 - R1));
 
    Serial2.print(" XS ");
    Serial2.print(Temp);   
    Serial2.print(" ");
    Serial2.print(Sensor2);              
    Serial2.print(" (");  
    Serial2.print(sensorValue);   
    Serial2.println(")"); 
  
    float rotation = sqrt(mpu6050.getGyroX()*mpu6050.getGyroX() + mpu6050.getGyroY()*mpu6050.getGyroY() + mpu6050.getGyroZ()*mpu6050.getGyroZ()); 
    float acceleration = sqrt(mpu6050.getAccX()*mpu6050.getAccX() + mpu6050.getAccY()*mpu6050.getAccY() + mpu6050.getAccZ()*mpu6050.getAccZ()); 
//    Serial.print(rotation);
//    Serial.print(" ");
//    Serial.println(acceleration);
 
    if (first_read == true) {
      first_read = false;
      rest = acceleration;
    }
 
    if (acceleration > 1.2 * rest)
        led_set(greenLED, HIGH);
    else
        led_set(greenLED, LOW);
        
    if (rotation > 5)
        led_set(blueLED, HIGH);
    else
        led_set(blueLED, LOW);
    }
  }  
  delay(100);
}
 
void eeprom_word_write(int addr, int val)
{
  EEPROM.write(addr * 2, lowByte(val));
  EEPROM.write(addr * 2 + 1, highByte(val));
}
 
short eeprom_word_read(int addr)
{
  return ((EEPROM.read(addr * 2 + 1) << 8) | EEPROM.read(addr * 2));
}
 
void blink_setup() 
{
  // initialize digital pin PB1 as an output.
  pinMode(RXLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
 
}
 
void blink(int length)
{
  digitalWrite(RXLED, HIGH);   // turn the LED on (HIGH is the voltage level)
 
  delay(length);              // wait for a lenth of time
 
  digitalWrite(RXLED, LOW);    // turn the LED off by making the voltage LOW

}
 
void led_set(int ledPin, bool state)
{
  if (ledPin == greenLED)
    digitalWrite(greenLED, state);
  else if (ledPin == blueLED)
    digitalWrite(blueLED, state);    

}

int read_analog()
{
    int sensorValue;
    sensorValue = analogRead(A2);

    return(sensorValue); 
}
