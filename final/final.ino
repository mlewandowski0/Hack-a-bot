#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>


Adafruit_MPU6050 mpu;

#define DEBUG true
//#define SERVOS

enum message_type {
  SERVO_CONTROL = 0,
  READ_IMU = 1,
  READ_LIDAR = 2
};

const int numServos = 12; // Define the number of servos
Servo servos[numServos]; // Array of servo objects
s

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

const float corr_acc_x = 2.0226804123711344;
const float corr_acc_y = 1.9979633401221997;
const float corr_acc_z = 1.5069124423963134;


int large_difference = 30;
int mid_difference = 15;


const int PWM_MIN = 102;
const int PWM_MAX = 512;
const int MID = (PWM_MIN + PWM_MAX) / 2;

float current_state_of_joint[12];
float desired_state_of_joint[12];


message_type current_msg;


const int E1 = 3; ///<Motor1 Speed
const int E2 = 11;///<Motor2 Speed
const int E3 = 5; ///<Motor3 Speed
const int E4 = 6; ///<Motor4 Speed

const int M1 = 4; ///<Motor1 Direction
const int M2 = 12;///<Motor2 Direction
const int M3 = 8; ///<Motor3 Direction
const int M4 = 7; ///<Motor4 Direction


void M1_advance(uint8_t Speed) ///<Motor1 Advance
{
 digitalWrite(M1,LOW);
 analogWrite(E1,Speed);
}
void M2_advance(uint8_t Speed) ///<Motor2 Advance
{
 digitalWrite(M2,HIGH);
 analogWrite(E2,Speed);
}
void M3_advance(uint8_t Speed) ///<Motor3 Advance
{
 digitalWrite(M3,LOW);
 analogWrite(E3,Speed);
}
void M4_advance(uint8_t Speed) ///<Motor4 Advance
{
 digitalWrite(M4,HIGH);
 analogWrite(E4,Speed);
}

void M1_back(uint8_t Speed) ///<Motor1 Back off
{
 digitalWrite(M1,HIGH);
 analogWrite(E1,Speed);
}
void M2_back(uint8_t Speed) ///<Motor2 Back off
{
 digitalWrite(M2,LOW);
 analogWrite(E2,Speed);
}
void M3_back(uint8_t Speed) ///<Motor3 Back off
{
 digitalWrite(M3,HIGH);
 analogWrite(E3,Speed);
}
void M4_back(uint8_t Speed) ///<Motor4 Back off
{
 digitalWrite(M4,LOW);
 analogWrite(E4,Speed);
}


void right_motor_forward(uint8_t Speed){
    M1_advance(Speed);
    M4_advance(Speed); 
}


void left_motor_forward(uint8_t Speed){
    M2_advance(Speed);
    M3_advance(Speed); 
}


void all_motor_backward(uint8_t Speed){
    M1_back(Speed);
    M4_back(Speed); 
}

void all_motor_backward(uint8_t Speed){
    M1_back(Speed);
    M4_back(Speed); 
}


void new_setPWM(int i, int on, int off) {
  if (i >= 0 && i < numServos) {
    int pin = i + 9; // Assuming servos start from pin 9
    int pwmPeriod = 4095; // Max value for the tick

    // Calculate the duty cycle for on and off ticks
    int onTime = map(on, 0, pwmPeriod, 0, 255);
    int offTime = map(off, 0, pwmPeriod, 0, 255);

    // Set PWM using analogWrite
    analogWrite(pin, onTime);
    delayMicroseconds(50); // Small delay to ensure the PWM is set properly
    analogWrite(pin, offTime);
  }
}

void setPWMfrequency(int pin, int frequency) {
  // Calculate the appropriate values for Timer1 registers
  uint8_t prescalerBits = 0;
  uint16_t pwmPeriod;

  // Choose the appropriate prescaler to achieve the desired frequency
  if (frequency >= 31250) {
    prescalerBits = 0b001; // No prescaling
    pwmPeriod = 65535; // 16MHz / 1 (no prescaling) / desired frequency - 1
  } else if (frequency >= 3906) {
    prescalerBits = 0b010; // Prescaler of 8
    pwmPeriod = 65535; // 16MHz / 8 / desired frequency - 1
  } else if (frequency >= 488) {
    prescalerBits = 0b011; // Prescaler of 64
    pwmPeriod = 65535; // 16MHz / 64 / desired frequency - 1
  } else if (frequency >= 61) {
    prescalerBits = 0b100; // Prescaler of 256
    pwmPeriod = 65535; // 16MHz / 256 / desired frequency - 1
  } else if (frequency >= 15) {
    prescalerBits = 0b101; // Prescaler of 1024
    pwmPeriod = 65535; // 16MHz / 1024 / desired frequency - 1
  } else {
    // Frequency too low
    return;
  }





void reset_joints()
{
  for (int i = 0; i < 12; ++i)
  {
    #ifdef SERVOS
    pwm.setPWM(i, 0, MID);  
    #endif 
    current_state_of_joint[i] = MID;
    desired_state_of_joint[i] = MID;
  }
}

void setupMPU6050()
{
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
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
    #ifdef SERVOS
     pwm.setPWM(i, 0, toset);
    #endif SERVOS
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
ma


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
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

        
    #ifdef DEBUG
      Serial.println("IMU");
      Serial.print(a.acceleration.x * corr_acc_x);
      Serial.print(",");
      Serial.print(a.acceleration.y * corr_acc_y);
      Serial.print(",");
      Serial.print(a.acceleration.z * corr_acc_z);
      Serial.print(",");        
      Serial.print(g.gyro.x);
      Serial.print(",");
      Serial.print(g.gyro.y);
      Serial.print(",");
      Serial.print(g.gyro.z);
      Serial.println("");
   #endif
      Serial2.print(a.acceleration.x * corr_acc_x);
      Serial2.print(",");
      Serial2.print(a.acceleration.y * corr_acc_y);
      Serial2.print(",");
      Serial2.print(a.acceleration.z * corr_acc_z);
      Serial2.print(",");        
      Serial2.print(g.gyro.x);
      Serial2.print(",");
      Serial2.print(g.gyro.y);
      Serial2.print(",");
      Serial2.print(g.gyro.z);
      Serial2.println("");

    msgEnd = true;
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
  #ifdef SERVOS
   pwm.begin();
  pwm.setPWMFreq(50); //set frequency 50
  #endif
  servos_milis_prev = 0;  
  //reset_joints();

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  setupMPU6050();

  for (int i = 0; i < numServos; i++) {
    servos[i].attach(i + 9); // Attach servos to pins 9, 10, 11 (adjust pins as needed)
  }
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
  //update_smoothly_joints();
  // Take a measurement with receiver bias correction and print to serial terminal
}
