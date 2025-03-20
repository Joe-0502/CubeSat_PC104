/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://arduino.cc
  This example code is in the public domain.
  modified 8 May 2014
  by Scott Fitzgerald
  
  Modified by Roger Clark. www.rogerclark.net for Maple mini 25th April 2015 , where the LED is on PC13
  
 */

 /* Pro Micro Test Code
   by: Nathan Seidle
   modified by: Jim Lindblom
   SparkFun Electronics
   date: September 16, 2013
   license: Public Domain - please use this code however you'd like.
   It's provided as a learning tool.
   This code is provided to show how to control the SparkFun
   ProMicro's TX and RX LEDs within a sketch. It also serves
   to explain the difference between Serial.print() and
   Serial1.print().
*/

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


                                                                                                        
#define DELAY 1000
#define LED_ONBOARD   14        //onboard led
#define LED_G   17        //Green LED
#define LED_B   16        //Blue LED

// the setup function runs once when you press reset or power the board
void setup() {
//#if defined(ARDUINO_ARCH_STM32F0) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_STM32L4)  
  // initialize digital pin PB1 as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_G,  OUTPUT);
  pinMode(LED_B,  OUTPUT);

  Serial1.begin(115200);
  Serial2.begin(115200);
  

}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_G,  LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_B,  LOW);   // turn the LED on (HIGH is the voltage level)

  Serial1.println("UART0: LED is off!");
  Serial2.println("UART1: LED is off!");

  delay(DELAY);              // wait for a second

//#if defined(ARDUINO_ARCH_STM32F0) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_STM32L4)

  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW
  digitalWrite(LED_G,  HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_B,  HIGH);   // turn the LED on (HIGH is the voltage level)

  Serial1.println("UART0: LED is on!");
  Serial2.println("UART1: LED is on!");

  delay(DELAY);              // wait for a second
}
