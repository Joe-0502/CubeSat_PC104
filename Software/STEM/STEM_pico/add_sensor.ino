// put your library includes here
#include "Arduino.h"

// put your globals here
int sensor1Value = 111;
int sensor2Value = 0;
unsigned long lastUpdateTime = 0;  // Declare a variable to store the last update time

// put your setup code here
void payload_setup() {

  
}

// put your loop code here
// Very Important: only use print, not println!!
void payload_loop() {

  unsigned long currentTime = millis();  // Get current time in milliseconds

  // Check if 2 seconds (2000 milliseconds) have passed
  if (currentTime - lastUpdateTime >= 2000) {
    sensor2Value += 2;             // Increment Sensor1 value by 2
    lastUpdateTime = currentTime;  // Update last update time
  }

  //You need to pass a value before you can start passing the Sensor value.
  Serial1.print(" ");
  Serial1.print("1");
  Serial1.print(" ");
  // Sensor1
  Serial1.print(sensor1Value); // Print the analog value of the photoresistor
  Serial1.print(" ");
  // Sensor2
  Serial1.print(sensor2Value);
  Serial1.print(" ");
  // Sensor3
  Serial1.print("437");
}

