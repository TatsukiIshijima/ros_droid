#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <Servo.h>

const int REIGHT_INFRARED_ANALOG_PIN = 0;
const int LEFT_INFRARED_ANALOG_PIN = 1;
const int CENTER_INFRARED_ANALOG_PIN = 2;
const int PWMA = 3;
const int SERVO_PIN = 9;
const int AIN2 = 11;
const int AIN1 = 12;
const int LED_PIN = 13;

std_msgs::Float32MultiArray infrareds_msg;

ros::NodeHandle nodeHandle;
ros::Publisher infrared_pub("infrared", &infrareds_msg);

unsigned long range_timer;

Servo servo;

// $ rostopic pub led std_msgs/Bool (true or false) で確認できる
void blink(const std_msgs::Bool& is_on) {
  if (is_on.data) {
    digitalWrite(LED_PIN, true);
  } else {
    digitalWrite(LED_PIN, false);
  }
}

ros::Subscriber<std_msgs::Bool> led_sub("led", &blink);

// $ rostopic echo /infrared で確認できる
void measure() {
  int right_value = analogRead(REIGHT_INFRARED_ANALOG_PIN);
  int left_value = analogRead(LEFT_INFRARED_ANALOG_PIN);
  int center_value = analogRead(CENTER_INFRARED_ANALOG_PIN);
  infrareds_msg.data[0] = toMeter(right_value);
  infrareds_msg.data[1] = toMeter(center_value);
  infrareds_msg.data[2] = toMeter(left_value);
}

float toMeter(int value) {
  float voltage = 5.0 * value / 1024;
  float range = 26.757 * pow(voltage, -1.236);
  return range / 100;
}

// $ rostopic pub servo std_msgs/UInt8 (0 ~ 180) で確認できる
void steering(const std_msgs::UInt8& angle) {
  if (angle.data < 0 || angle.data > 180) {
    return;
  }
  servo.write(angle.data);
}

ros::Subscriber<std_msgs::UInt8> servo_sub("servo", &steering);

void accelerate(const std_msgs::Int8& velocity) {
  if (velocity.data < -128 || velocity.data > 127 ) {
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,LOW);
    analogWrite(PWMA, velocity.data);
    return;
  }
  if (velocity.data > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else if (velocity.data < 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,LOW);
  }
  analogWrite(PWMA, velocity.data);
}

ros::Subscriber<std_msgs::Int8> dc_motor_sub("dc_motor", &accelerate);

void init_pin() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  servo.attach(SERVO_PIN);
}

void setup() {

  init_pin();

  infrareds_msg.data = (float*)malloc(sizeof(float) * 3);
  infrareds_msg.data_length = 3;

  nodeHandle.initNode();
  nodeHandle.subscribe(led_sub);
  nodeHandle.advertise(infrared_pub);
  nodeHandle.subscribe(servo_sub);
  nodeHandle.subscribe(dc_motor_sub);
}

void loop() {
  // 配信はセンサーの値が取得できてから 50 millsec 間を開けて行う
  if ((millis() - range_timer) > 50) {
    measure();
    infrared_pub.publish(&infrareds_msg);
    range_timer = millis();
  }
  
  nodeHandle.spinOnce();
  delay(1);
}
