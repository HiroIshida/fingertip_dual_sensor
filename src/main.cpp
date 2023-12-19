#include "Arduino.h"
#include "ros.h"
#include <i2c_t3.h>     // Use <i2c_t3.h> for Teensy and <Wire.h> for Arduino
#include <SparkFun_LPS25HB_Arduino_Library.h> 
#include <math.h>
#include <ros.h>
#include <array>
#include <string.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "fingertip_dual_sensor/DualPressureStamped.h"
#include <ros/time.h>

#define LOOP_TIME 80  // loop duration in ms

LPS25HB sensor1, sensor2; // Create an object of the LPS25HB class

// ros
ros::NodeHandle  nh;
std_msgs::Float32 hoge;
fingertip_dual_sensor::DualPressureStamped pstamped;
ros::Publisher pub_dualpressure("dpressures", &pstamped);

void setup()
{

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub_dualpressure);

  while(!nh.connected())
  {
    nh.spinOnce();
  }

  Serial.begin(9600);
  Serial.println("LPS25HB Pressure Sensor Example 3 - Checking the Connection");
  Serial.println();

  Wire.begin();
  Wire1.begin();
  sensor1.begin(Wire, LPS25HB_I2C_ADDR_DEF); 
  sensor2.begin(Wire1, LPS25HB_I2C_ADDR_DEF); 
}

void common_procedure(LPS25HB& sensor, std_msgs::Float32& floatmsg){
  if (sensor.isConnected() == true)
  {
    if (sensor.getStatus() == 0x00)
    {
      sensor.begin();
    } // If it is connected but not responding (for example after a hot-swap) then it may need to be re-initialized
    Serial.print("Connected. Sensor Status: ");
    Serial.print(sensor.getStatus(), HEX); // Read the sensor status, the datasheet can explain what the various codes mean
    Serial.print(", Pressure (hPa): ");
    Serial.print(sensor.getPressure_hPa()); // Get the pressure reading in hPa as determined by dividing the number of ADC counts by 4096 (according to the datasheet)
    hoge.data = sensor.getPressure_hPa();
    floatmsg.data = sensor.getPressure_hPa();
    Serial.print(", Temperature (degC): ");
    Serial.println(sensor.getTemperature_degC()); // Get the temperature in degrees C by dividing the ADC count by 480
  }
  else
  {
    Serial.println("Disconnected");
    sensor.begin();
  }
}

void loop()
{

    auto time = millis();

    pstamped.header.stamp = nh.now(); // different from roscpp!
    Serial.println("testing sensor1\n");
    common_procedure(sensor1, pstamped.pressure1);

    Serial.println("testing sensor2\n");
    common_procedure(sensor2, pstamped.pressure2);
    pub_dualpressure.publish(&pstamped);

    //delay(100);
    while (millis() < time + LOOP_TIME); // enforce constant loop time
    nh.spinOnce();
}
