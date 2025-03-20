// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//


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

#include <Wire.h>
//                      RX    TX
////HardwareSerial Serial2(PA3, PA2);
//            SDA  SCL
////TwoWire Wire1(PB7, PB6);

#define DELAY 500
#define LED_ONBOARD   14        //onboard led
#define LED_G   17        //Green LED
#define LED_B   16        //Blue LED


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_G,  OUTPUT);
  pinMode(LED_B,  OUTPUT);


  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(LED_G,  LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_B,  LOW);   // turn the LED on (HIGH is the voltage level)
    
  Wire.setSCL(13);
  Wire.setSDA(12);
  Wire.begin();

  Wire1.setSCL(7);
  Wire1.setSDA(6);
  Wire1.begin();

  Serial1.begin(115200);
  Serial2.begin(115200);
                                                                              Serial2.begin(115200);
////  while (!Serial); // Leonardo: wait for serial monitor
  Serial1.println("\nUART0: I2C Scanner");
  Serial2.println("\nUART1: I2C Scanner");
}

void loop() {
  int nDevices = 0;

  Serial1.println("UART0: Scanning...");
  Serial2.println("UART1: Scanning...");

  for (byte address = 0x43; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.

    
    Wire.beginTransmission(address);
    Serial1.print(address, HEX);
    Serial2.print(address, HEX);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial1.println();      
      Serial2.println();
      Serial1.print("I2C device found at address 0x");
      Serial2.print("I2C device found at address 0x");
      if (address < 16) {
        Serial1.print("0");
        Serial2.print("0");
      }
      Serial1.print(address, HEX);
      Serial1.println("  !");
      
      Serial2.print(address, HEX);
      Serial2.println("  !");

      ++nDevices;


      
    } else if (error == 4) {

      Serial1.println();
      Serial1.print("UART0: Unknown error at address 0x");
      Serial2.println();
      Serial2.print("UART1: Unknown error at address 0x");
      if (address < 16) {
        Serial1.print("0");
        Serial2.print("0");
      }
      Serial1.println(address, HEX);
      Serial2.println(address, HEX);
    }

  }
  if (nDevices == 0) {
    Serial1.println();
    Serial1.println("UART0: No I2C devices found\n");
    
    Serial2.println();
    Serial2.println("UART1: No I2C devices found\n");
  } else {
    Serial1.println();
    Serial1.println("UART0: done\n");

    Serial2.println();
    Serial2.println("UART1: done\n");
  }
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW
  digitalWrite(LED_G,  HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_B,  HIGH);   // turn the LED on (HIGH is the voltage level)

  delay(DELAY); // Wait 5 seconds for next scan

  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_G,  LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_B,  LOW);   // turn the LED on (HIGH is the voltage level)



  
  delay(5000); // Wait 5 seconds for next scan
}
