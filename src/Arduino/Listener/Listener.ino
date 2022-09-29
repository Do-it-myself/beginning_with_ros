# include <ros.h>
# include <std_msgs/Empty.h>

void callback_function(const std_msgs::Empty &message) {
  digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN));
}

ros::NodeHandle nodeHandle;
ros::Subscriber<std_msgs::Empty> subscriber("listen_arduino", &callback_function);

void setup() {
  nodeHandle.initNode();
  nodeHandle.subscribe(subscriber);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  nodeHandle.spinOnce();
  delay(1);
}
