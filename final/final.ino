#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define DEBUG true

enum message_type {
  SERVO_CONTROL = 0,
  READ_IMU = 1,
  READ_LIDAR = 2
};


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

int cnt = 0;
char data[4];
bool msgStart = true;
bool msgEnd = false;
int servo_num = 0;
int servo_val = 90;
int baudrate = 115200;

int servos_milis_prev = 0;
int servos_update_tick_mili = 50;



int large_difference = 30;
int mid_difference = 15;


const int PWM_MIN = 102;
const int PWM_MAX = 512;
const int MID = (PWM_MIN + PWM_MAX) / 2;

float current_state_of_joint[12];
float desired_state_of_joint[12];


message_type current_msg;

void reset_joints()
{
  for (int i = 0; i < 12; ++i)
  {
    pwm.setPWM(i, 0, MID);  
    current_state_of_joint[i] = MID;
    desired_state_of_joint[i] = MID;
  }
}

float smooth_degree(float degree, float prev_degree) {
  float difference = abs(degree - prev_degree);
  float smoothingFactor = 0.99; // Default smoothing factor for small differences

  // Adjust smoothing factor based on the difference
  if (difference > large_difference) {
    smoothingFactor = 0.70; // Less smoothing for large differences
  } else if (difference > mid_difference) {
    smoothingFactor = 0.85; // Moderate smoothing for medium differences
  }

  return (degree * (1 - smoothingFactor)) + (prev_degree * smoothingFactor);
}


void update_smoothly_joints()
{
  int current_time= millis();
  if (current_time - servos_milis_prev > servos_update_tick_mili)
  {
  for (int i = 0; i < 12; ++i)
  {
     if (desired_state_of_joint[i] == current_state_of_joint[i])
       continue;
     
     int toset = smooth_degree(desired_state_of_joint[i], current_state_of_joint[i]);
     current_state_of_joint[i] = toset;
     pwm.setPWM(i, 0, toset);
     #ifdef DEBUG
      Serial.print("servo ");
      Serial.print(i);
      Serial.print(" is set to ");
      Serial.print(toset);
      Serial.print(" ");
      Serial.println(current_state_of_joint[i]);
     #endif 
  }
    servos_milis_prev = current_time;
  }
}



void decode_first_byte(byte b)
{
  if (b == 0)
  {
    current_msg = SERVO_CONTROL;

    #ifdef DEBUG
      Serial.println("SERVO MSG");
    #endif
   
  }
  else if (b == 1)
  {
    current_msg = READ_IMU;
    #ifdef DEBUG
      Serial.println("IMU");
    #endif

  }
  else if (b == 2)
  {
    current_msg = READ_LIDAR;
    #ifdef DEBUG
      Serial.println("LIDAR");
    #endif
  }
  else
  {
    #ifdef DEBUG
      Serial.println("ERROR");
    #endif
    msgEnd = true;
  }
}

void decode_next_byte(byte b)
{
  if (current_msg == SERVO_CONTROL)
  {
    if (cnt == 1)
    {
      servo_num = b;
      #ifdef DEBUG
        Serial.print("servo=");
        Serial.println(servo_num);
      #endif

    }
    else if (cnt == 2)
    { 
      servo_val = b;
    }
    else if (cnt == 3)
    {
      servo_val <<= 8;
      servo_val += b;
      msgEnd = true;
      #ifdef DEBUG
        Serial.print("val=");
        Serial.println(servo_val);
      #endif      
    }
  }
}

void decode_last_byte(byte b )
{
  if (current_msg == SERVO_CONTROL)
  {
      desired_state_of_joint[servo_num] = servo_val; 
      #ifdef DEBUG
        Serial.print("setting servo : val=");
        Serial.print(servo_val);
        Serial.print(",num=");
        Serial.println(servo_num);
      #endif
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(baudrate);
  Serial2.begin(baudrate);
  pwm.begin();
  pwm.setPWMFreq(50); //set frequency 50
  servos_milis_prev = 0;

  reset_joints();
  
}

void loop() {
  // put your main code here, to run repeate  dly:
  // Check if data is available to read from Serial2
  if (Serial2.available() > 0) {
    // Read a byte from Serial2
    byte data = Serial2.read();

    if (msgStart)
    {
      decode_first_byte(data);
      msgStart = false;
      cnt = 0;
    }
    else 
    {
      cnt += 1;
      decode_next_byte(data);
    }

    if (msgEnd)
    {
      decode_last_byte(data);
      msgEnd = false;
      msgStart = true; 
    }
  }
  update_smoothly_joints();
}
