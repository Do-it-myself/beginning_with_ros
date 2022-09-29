#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <HCSR04.h>

char frameid[]="/ultrasound";
const int trigPin = 7, echoPin = 8;
UltraSonicDistanceSensor sensor(trigPin, echoPin);

sensor_msgs::Range message;

ros::NodeHandle nodeHandle;
ros::Publisher publisher("ultrasound", &message);


void setup() {
  nodeHandle.initNode();
  nodeHandle.advertise(publisher);

  message.header.frame_id = frameid;
  message.radiation_type = sensor_msgs::Range::ULTRASOUND;
  message.field_of_view = 0.1; //fake
  message.min_range = 0.002; //2 cm
  message.max_range = 0.150; //150 cm

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  message.range = sensor.measureDistanceCm();
  message.header.stamp = nodeHandle.now();
  
  publisher.publish(&message);

  nodeHandle.spinOnce();
  delay(500);
}
