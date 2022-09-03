#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
std_msgs::Int64 speed_motor;

ros::Publisher speedMotorValue("VelocityValue", &speed_motor);

// Defining the Motor Pins and Motor Variables for motor
#define CW 1
#define CCW 2
#define enablePin 3
#define hallPin 2
#define directionPin 4
#define speedPin 5
#define SPEED_TIMEOUT 500 // // Time (mS)used to determine wheel is not spinning
#define UPDATE_TIME 1 // Time (mS) - 0.001s used to measure speed of motor
#define WHEEL_DIRAMETER_CM  16.5 //Motor wheel diameter (centimeter)
#define WHEEL_CIRCUMFERENCE_CM 51.87 // Motor wheel circumference (centimeters)

// Variable used in evenCounter() function
volatile int64_t counter = 0;
// Variable used in readSpeed() function
static unsigned long lastTime_mS = 0;
// Define a function to move the motor according to the direction and PWM signal
void runMotor(int16_t direct, int16_t PWM)
{
  if (direct == CW)
  {
    digitalWrite(enablePin, HIGH);
    digitalWrite(directionPin, LOW);
    analogWrite(speedPin, PWM);
  }
  else if (direct == CCW)
  {
    digitalWrite(enablePin, HIGH);
    digitalWrite(directionPin, HIGH);
    analogWrite(speedPin, PWM);
  }
  else
  {
    digitalWrite(enablePin, LOW);
    digitalWrite(directionPin, LOW);
    analogWrite(speedPin, PWM);
  }
}
//create a function to move the motor according to the sign of PWM value give to it
void pwm_input(const std_msgs::Int16& pwm_value)
{
  int PWM = 0;
  PWM = pwm_value.data;
  if (PWM > 0)
  {
    runMotor(CCW, PWM);
  }
  else
  {
    runMotor(CW, abs(PWM));
  }
}
ros::Subscriber<std_msgs::Int16> PWM("PWM_Value", &pwm_input);
// reading the encoder values
void eventCounter()
{
  counter++;
}
void setup() {
  // Setup ROS
  nh.initNode();
  nh.advertise(speedMotorValue);
  nh.subscribe(PWM);
  // Set pin directions
  pinMode(enablePin, OUTPUT);
  pinMode(hallPin, INPUT_PULLUP);
  pinMode(directionPin, OUTPUT);
  pinMode(speedPin, OUTPUT);
  // attach hall pin for interrupt
  attachInterrupt(digitalPinToInterrupt(hallPin), eventCounter, CHANGE);

  // Set initial pin states
  digitalWrite(enablePin, LOW);
  digitalWrite(directionPin, LOW);
  digitalWrite(speedPin, 0);
  lastTime_mS = millis();
  counter = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.loginfo("VelocityValue");
  
  speed_motor.data = readSpeed();
  speedMotorValue.publish(&speed_motor);
  nh.spinOnce();
}
volatile int64_t readSpeed()
{
  volatile int64_t rpm = 0.0 ; // wheel speed in revolutions per minutes

  unsigned long periodTime_mS = millis() - lastTime_mS;
  if (periodTime_mS > UPDATE_TIME)
  {
    detachInterrupt(digitalPinToInterrupt(hallPin));
    // Calculate the RPM of motor
    rpm = (volatile int64_t) 60000 * counter / periodTime_mS / 45;

    // If RPM is excessively high the ignore it;
    if (rpm > 5000) rpm = 0;

    lastTime_mS = millis();
    counter = 0;
    attachInterrupt(digitalPinToInterrupt(hallPin), eventCounter, CHANGE);
    return rpm;
  }
  return -1;
}
