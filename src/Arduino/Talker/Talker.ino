# include <ros.h>
# include <std_msgs/String.h>

std_msgs::String message;
char hello[13]="Hello World!";

ros::NodeHandle nodeHandle; // = "ros.xxx" in program
ros::Publisher publisher("talk_arduino", &message);

void setup() {
  nodeHandle.initNode(); 
  nodeHandle.advertise(publisher); // publisher & advertise separated
}

// equivalent to while (ros::ok())
void loop() {
  message.data = hello;
  publisher.publish(&message);
  nodeHandle.spinOnce();
  delay(1000); // = rate.sleep();
}
