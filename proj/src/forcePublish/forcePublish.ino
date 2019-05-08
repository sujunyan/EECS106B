#include "HX711.h" //force sensor specific library
#include <ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>


//define the pins
HX711 scale(5, 6);
 
//define the constants
float calibration_factor = 154; // this calibration factor is adjusted according to my load cell
float units;
float newtons;
unsigned long range_timer;
 
//variables
float range;

ros::NodeHandle nh;
 
//sensor_msgs::Range range_msg;
std_msgs::Float32 range_msg;
 
ros::Publisher pub("/force", &range_msg);
 
void setup() {
   Serial.begin(9600);
   
   scale.set_scale();
   scale.tare();  //Reset the scale to 0

   nh.getHardware()->setBaud(9600);
   nh.initNode();
   nh.advertise(pub);
}
 
void loop() {
  scale.set_scale(calibration_factor); //Adjust to this calibration factor

  Serial.print("Reading: ");
  units = scale.get_units(), 10;
  if (units < 0)
  {
    units = 0.00;
  }
  newtons = units * 0.00980665;
  Serial.print(units);
  Serial.print(" grams"); 
  Serial.print(" calibration_factor: ");
  Serial.print(calibration_factor);
  Serial.println();

  unsigned long currentMillis = millis();
  if (currentMillis-range_timer >= 20) //publish every 50 milliseconds
  {
    range_timer = currentMillis+50;
    range_msg.data = newtons;
    pub.publish(&range_msg);
  }
   
   nh.spinOnce();

}
