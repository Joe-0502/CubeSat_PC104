#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

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


#define DELAY         500
#define LED_ONBOARD   14        //onboard led
#define LED_G         17        //Green LED
#define LED_B         16        //Blue LED


Adafruit_BME280 bme;



void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_G,  OUTPUT);
  pinMode(LED_B,  OUTPUT);


  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(LED_G,  LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_B,  LOW);   // turn the LED on (HIGH is the voltage level)

  Serial1.begin(115200);
  Serial2.begin(115200);

  Wire.setSCL(13);
  Wire.setSDA(12);
  Wire.begin();
  
  if (!bme.begin(0x76)) {
    Serial1.println("UART0: Could not find a valid BME280 sensor, check wiring!");
    Serial2.println("UART1: Could not find a valid BME280 sensor, check wiring!");
    digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW
    digitalWrite(LED_G,  HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(LED_B,  HIGH);   // turn the LED on (HIGH is the voltage level)
    while (1);
  }
}

void loop() {
  Serial1.print("UART0: emperature = ");
  Serial1.print(bme.readTemperature());
  Serial1.println("*C");

  Serial1.print("UART0: Pressure = ");
  Serial1.print(bme.readPressure() / 100.0F);
  Serial1.println("hPa");

  Serial1.print("UART0: Approx. Altitude = ");
  Serial1.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial1.println("m");

  Serial1.print("UART0: Humidity = ");
  Serial1.print(bme.readHumidity());
  Serial1.println("%");

  Serial1.println();
  
  
  Serial2.print("UART1: emperature = ");
  Serial2.print(bme.readTemperature());
  Serial2.println("*C");

  Serial2.print("UART1: Pressure = ");
  Serial2.print(bme.readPressure() / 100.0F);
  Serial2.println("hPa");

  Serial2.print("UART1: Approx. Altitude = ");
  Serial2.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial2.println("m");

  Serial2.print("UART1: Humidity = ");
  Serial2.print(bme.readHumidity());
  Serial2.println("%");

  Serial2.println();

  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW
  digitalWrite(LED_G,  HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_B,  HIGH);   // turn the LED on (HIGH is the voltage level)

  delay(DELAY); // Wait 5 seconds for next scan

  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_G,  LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_B,  LOW);   // turn the LED on (HIGH is the voltage level)  
  delay(1000);
}
