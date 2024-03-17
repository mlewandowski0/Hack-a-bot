/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  to calibrate the frequency of the oscillator clock of the PCA9685.

  CAUTION: DO NOT CONNECT ANY VOLTAGE HIGHER THAN THE BOARD LIMITS.
  For 3.3V boards, like the ESP8266, remove any 5V input. The setup will
  feed the voltage back into the board to measure the frequency.
  KABOEM, SMOKE if you use too much VOLTAGE.

  Connect the PCA9685 with I2C (Ground, VCC, SCL, SCA) and apply
  voltage on V+. See above not higher than board limits.
  Connect the signal (yellow pin, PWM) of the PCA9685 to your board:
  Default is pin 3, last of first block.
  Default is pin 14 (of your ESP8266).

  Formula for prescale to get the targetted frequency (=update_rate) is:
  prescale = round ( osc_clock / 4096 * update_rate) - 1
  rewritten: osc_clock = (prescale + 1) * 4096 * update_rate
  We will measure the real update_rate to assert the real osc_clock.

  ***************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PIN_SERVO_RLHJ 0 //
#define PIN_SERVO_RLBL 1
#define PIN_SERVO_RLSL 2 


const int PWM_MIN = 102;
const int PWM_MAX = 512;
const float degree_to_PWM = float(PWM_MAX - PWM_MIN)/ 180.0;
const float ddeg = 1;
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

float curr_RLHJ = 90;
float prev_RLHJ = 90;
float curr_RLBL = 90;
float prev_RLBL = 90;
float curr_RLSL = 90;
float prev_RLSL = 90;

void write_degree(float degree)
{
  int toset = PWM_MIN + degree * degree_to_PWM;
  //pwm.setPWM(PIN_SERVO_RLHJ, 0, toset);
  pwm.setPWM(PIN_SERVO_RLBL, 0, toset);
  //pwm.setPWM(PIN_SERVO_RLSL, 0, toset);
  delay(10);
}



//equation to calculate smooth pulse from current and previous values.
/*
float smooth_degree(float degree, float prev_degree){
  return (degree* 0.01) + (prev_degree * 0.99);
}
*/
float smooth_degree(float degree, float prev_degree) {
  float difference = abs(degree - prev_degree);
  float smoothingFactor = 0.99; // Default smoothing factor for small differences

  // Adjust smoothing factor based on the difference
  if (difference > 30) {
    smoothingFactor = 0.70; // Less smoothing for large differences
  } else if (difference > 15) {
    smoothingFactor = 0.85; // Moderate smoothing for medium differences
  }
  // For differences <= 15, use the default smoothing factor

  return (degree * (1 - smoothingFactor)) + (prev_degree * smoothingFactor);
}
void sequencer (float delta_leg, float prev_val, float curr_val){
  write_degree(smooth_degree(90.0, prev_val));
  delay(600);
  //joint 1 = 5 degree
  for (int i = -1; i < 2; i++){
    curr_val = 90.0 + i*delta_leg;
    write_degree(smooth_degree(curr_val, prev_val));
    prev_val = curr_val;
    delay(600);
  }
}

void setup() {
  pwm.begin();
  pwm.setPWMFreq(50); //set frequency 50
}
//2048 is too large. 0.5 ms for 0 degree, 2.5 ms for 180 degree
//min = 102 left 90, 512 right (max) - midpoint 300~
//30 degree oscillation for both sides from midpoint.
//5 degree for horizontal joint. 20 degree for big leg. 15 degree 2nd small leg.
//0 is hj, 1 is bl, 2 is sl.

int serial_set_angle = 90;

//decrease step interval for smoother movement of the servo
void loop() {
  //write_degree(90.0);
  //delay(300);
  //sequencer(5, prev_RLHJ, curr_RLHJ);
  if Serial.available(){
    serial_set_angle = Serial.read();
    sequencer(20, prev_RLBL, serial_set_angle);
  }
  
  //sequencer(10, prev_RLSL, curr_RLSL);
  /*for (int i = -1; i < 2; i++){
    write_degree(90.0 + i*30);
    delay(300);
  }
 */
}
