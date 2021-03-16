#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#define encodPinA1      2                       // Quadrature encoder A pin
#define encodPinB1      3                       // Quadrature encoder B pin
#define M1              9                       // PWM outputs to L298N H-Bridge motor driver module
#define M2              8
#define M3              10                       // PWM outputs to L298N H-Bridge motor driver module
#define M4              11

#define LOOPTIME        100

double kpL = 0 , kiL = 200000 , kdL = 0;             // modify for optimal performance
double inputL = 0, outputL = 0, setpointL = 0;
double kpR = 0, kiR = 200000, kdR = 0;             // modify for optimal performance
double inputR = 0, outputR = 0, setpointR = 0;
long temp;
volatile long encoderPosL = 0, encoderPosR = 0;
double v_L, v_R;
unsigned long lastMilli = 0;
PID myPIDL(&inputL, &outputL, &setpointL, kpL, kiL, kdL, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'
PID myPIDR(&inputR, &outputR, &setpointR, kpR, kiR, kdR, DIRECT);
int distancefr=0, distanceleft=0, distanceright=0, durationfr=0, durationleft=0, durationright=0;
void encoderL();
void encoderR();

const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;

const float diameter = 0.065;                   //Wheel radius, in m
const float wheelbase = 0.20;               //Wheelbase, in m

float speed_req = 0;                         //Desired linear speed for the robot, in m/s
float angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

float speed_req_left = 0;                    //Desired speed for left wheel in m/s                //Actual speed for left wheel in m/s
//float speed_cmd_left = 0;                    //Command speed for left wheel in m/s

float speed_req_right = 0;                   //Desired speed for right wheel in m/s


ros::NodeHandle nh;
//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {                                                //Reset the counter for number of main loops without communication
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message

 speed_req_left = speed_req - angular_speed_req * (wheelbase / 2); //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
 speed_req_right = speed_req + angular_speed_req * (wheelbase / 2); //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);
void setup() {
    Serial.begin(57600);
    pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
    pinMode(encodPinB1, INPUT_PULLUP);
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT); 
    pinMode(M3, OUTPUT); 
    pinMode(M4, OUTPUT); // quadrature encoder input A
    attachInterrupt(0, encoderL, FALLING);
    attachInterrupt(1, encoderR, FALLING); // update encoder position v
    nh.initNode();                            //init ROS node
    nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
    nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
    myPIDL.SetMode(AUTOMATIC);
    myPIDL.SetSampleTime(1);
    myPIDL.SetOutputLimits(0, 255);
    myPIDR.SetMode(AUTOMATIC);
    myPIDR.SetSampleTime(1);
    myPIDR.SetOutputLimits(0, 255); 
}

void loop() {
  nh.spinOnce();
  if ((millis() - lastMilli) >= LOOPTIME)
  {
    lastMilli = millis();
    if (speed_req_left < 0) {
      setpointL =-speed_req_left;
       inputL = (encoderPosL / 850.00) * 10 * 3.14 * diameter;
      encoderPosL = 0;
      myPIDL.Compute();
      analogWrite(M1, 255-outputL);                           // drive motor CW
      digitalWrite(M2, HIGH);
    }
    if (speed_req_left >= 0) {
      setpointL = (speed_req_left);
      inputL = (encoderPosL / 850.00) * 10 * 3.14 * diameter ;
       encoderPosL = 0;
      myPIDL.Compute();
      analogWrite(M1, outputL);                             // drive motor CW
      digitalWrite(M2, LOW);
    }
    if (speed_req_right < 0) {
      setpointR =-speed_req_right;
      inputR = (encoderPosR / 850.00) * 10 * 3.14 * diameter;
       encoderPosR = 0;
      myPIDR.Compute();
       analogWrite(M3, outputR);                           // drive motor CW
      digitalWrite(M4, LOW);   
    }

    if (speed_req_right >= 0) {
      setpointR = (speed_req_right);
      inputR = (encoderPosR / 850.00) * 10 * 3.14 * diameter;
       encoderPosR = 0;
      myPIDR.Compute(); 
      analogWrite(M3, 255-outputR);                             // drive motor CW
      Serial.println(inputR);
      digitalWrite(M4, HIGH);
    }
  }
  
}
void encoderL()  {                                     // pulse and direction, direct port reading to save cycles
  if (digitalRead(encodPinA1) == LOW )  encoderPosL++;
  
}
void encoderR()  {                                     // pulse and direction, direct port reading to save cycles
  if (digitalRead(encodPinB1) == LOW )  encoderPosR++;          // if(digitalRead(encodPinB1)==HIGH)   count ++;            // if(digitalRead(encodPinB1)==LOW)   count --;
}
