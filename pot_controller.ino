/* Potentiometer controlled robot arm
 * By Wei Shi
 * 
 * This code supports 6 degrees of freedom
 * SERVO 0 is the claw
 * 
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> //uses adafruit servo shield

// This sets default address 0x40
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// The above sets address to 0x41
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// these constants are for use in the Adafruit API
// to control the servos

// max pulse length is 4096
// claw
#define SERVO_0_MIN 0
#define SERVO_0_MAX 270
//#define SERVO_0_MIN 2468
//#define SERVO_0_MAX 2578

// wrist
#define SERVO_1_MIN 125
#define SERVO_1_MAX 550
//#define SERVO_1_MIN 2042
//#define SERVO_1_MAX 2467

#define SERVO_2_MIN 100
#define SERVO_2_MAX 500
//#define SERVO_2_MIN 1653
//#define SERVO_2_MAX 2041

// 500
#define SERVO_3_MIN 0
#define SERVO_3_MAX 500

// 350
#define SERVO_4_MIN 350
#define SERVO_4_MAX 700

// 430
#define SERVO_5_MIN 120
#define SERVO_5_MAX 550

// servo # (0-15)
uint8_t servonum = 15;

// analog input 4 and 5 are used for SDA and SCL in I2C
unsigned int servo5_potpin = 8;
unsigned int servo4_potpin = 7;
unsigned int servo3_potpin = 3;
unsigned int servo2_potpin = 2;
unsigned int servo1_potpin = 1;
unsigned int servo0_potpin = 0;


// pins for HC-SR04 ultrasonic sensors
// left
const int trig_0 = 2;
const int echo_0 = 3;

// front
const int trig_1 = 4;
const int echo_1 = 5;

// right
const int trig_2 = 6;
const int echo_2 = 7;

//potentiometer analog signal post-processing variables for each servo (0-5)
const unsigned int num_readings = 25;

unsigned int read_index_5 = 0;
unsigned int readings_5[num_readings];
unsigned int total_5 = 0;
unsigned int prev_value_5;

unsigned int read_index_4 = 0;
unsigned int readings_4[num_readings];
unsigned int total_4 = 0;
unsigned int prev_value_4;

unsigned int read_index_3 = 0;
unsigned int readings_3[num_readings];
unsigned int total_3 = 0;
unsigned int prev_value_3;

unsigned int read_index_2 = 0;
unsigned int readings_2[num_readings];
unsigned int total_2 = 0;
unsigned int prev_value_2;

unsigned int read_index_1 = 0;
unsigned int readings_1[num_readings];
unsigned int total_1 = 0;
unsigned int prev_value_1;

unsigned int read_index_0 = 0;
unsigned int readings_0[num_readings];
unsigned int total_0 = 0;
unsigned int prev_value_0;

void setup() {
  pinMode(trig_0, OUTPUT);
  pinMode(echo_0, INPUT);  
  pinMode(trig_1, OUTPUT);
  pinMode(echo_1, INPUT); 
  pinMode(trig_2, OUTPUT);
  pinMode(echo_2, INPUT);     

  // initialize potentiometer readings array for each servo
  for(unsigned int i = 0; i < num_readings; ++i) {
    readings_5[i] = analogRead(servo5_potpin);
    readings_4[i] = analogRead(servo4_potpin);
    readings_3[i] = analogRead(servo3_potpin); 
    readings_2[i] = analogRead(servo2_potpin);
    readings_1[i] = analogRead(servo1_potpin);
    readings_0[i] = analogRead(servo0_potpin);

    total_5 += readings_5[i];
    total_4 += readings_4[i];    
    total_3 += readings_3[i];
    total_2 += readings_2[i];
    total_1 += readings_1[i];
    total_0 += readings_0[i];
  }
  
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates

  Serial.begin(9600);

}

// test function to open and close servo
void openCloseServo(uint8_t servo_number, unsigned int mi, unsigned int ma) {
  pwm.setPWM(servo_number, 0, ma);
  delay(1000);
  pwm.setPWM(servo_number, 0, mi);
  delay(1000);
  
}
// reverses direction of reading for potentiometers
unsigned int reverse(unsigned int low, unsigned int high, unsigned int value) {
  unsigned int temp = value - low;
  temp = high - temp;
  return temp;
  
}
// controls a given servo sn
void controlServo(unsigned int pot_pin, uint8_t sn, unsigned int mi, unsigned int ma) {
  unsigned int value = analogRead(pot_pin);
  value = map(value, 0, 1023, mi, ma);
  Serial.println(value);
  pwm.setPWM(sn, 0, value);
}
unsigned int smoothSignal(unsigned int pot_pin, unsigned int& total, 
                          unsigned int readings[], unsigned int& read_index, 
                          const unsigned int& num_readings) {
  total = total - readings[read_index];
  readings[read_index] =  analogRead(pot_pin);
  total = total + readings[read_index];
  read_index++;
  if(read_index >= num_readings) {
    read_index = 0;
  }
  return  (total/num_readings);

}
bool leftGood(unsigned int min_distance_cm) {
  
  long duration_micro;
  int distance_cm;
  digitalWrite(trig_0, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_0, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_0, LOW);

  duration_micro = pulseIn(echo_0, HIGH);
  distance_cm = duration_micro*0.0343/2;
  //Serial.println(distance_cm); 

  if(distance_cm <= min_distance_cm)
    return false;
    
  return true;
  
}
bool frontGood(unsigned int min_distance_cm) {
  long duration_micro;
  int distance_cm;
  digitalWrite(trig_1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_1, LOW);

  duration_micro = pulseIn(echo_1, HIGH);
  distance_cm = duration_micro*0.0343/2;
  //Serial.println(distance_cm); 

  if(distance_cm <= min_distance_cm)
    return false;
    
  return true;
  
}
bool rightGood(unsigned int min_distance_cm) {
  
  long duration_micro;
  int distance_cm;
  digitalWrite(trig_2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_2, LOW);

  duration_micro = pulseIn(echo_2, HIGH);
  distance_cm = duration_micro*0.0343/2;

  //Serial.println(distance_cm); 
  if(distance_cm <= min_distance_cm)
    return false;
    
  return true;
  
}


void controlServo5(unsigned int pot_pin) {
  unsigned int value = smoothSignal(pot_pin, total_5, readings_5, read_index_5, num_readings);
  value = map(value, 0, 1023, SERVO_5_MIN, SERVO_5_MAX);
  //Serial.print("s5 ");
  //Serial.println(value);

  
  if(!rightGood(10)){
    //Serial.print("s5 (!rg)");
    //Serial.println(value);
    //value = reverse(SERVO_5_MIN, SERVO_5_MAX, value);
    if(value < prev_value_5) {
      return;
    }
    


    // right is blocked check value to see if it is value for turning left
    pwm.setPWM(5, 0, value);
    prev_value_5 = value;
  }

  
  if(!leftGood(10)){
        
    //Serial.print("s5 (!lg)");
    //Serial.println(value);

    //value = reverse(SERVO_5_MIN, SERVO_5_MAX, value);   
    if(value > prev_value_5) {
      return;
    }

    // left is blocked check value to see if it is value for turning right
    pwm.setPWM(5, 0, value);   
    prev_value_5 = value;
 
  }
  else {
    //value = reverse(SERVO_5_MIN, SERVO_5_MAX, value);
    //Serial.print("s5 ");
    //Serial.println(value);
    pwm.setPWM(5, 0, value);
    prev_value_5 = value;
  }
}
void controlServo4(unsigned int pot_pin) {
  unsigned int value = smoothSignal(pot_pin, total_4, readings_4, read_index_4, num_readings);
  value = map(value, 0, 1023, SERVO_4_MIN, SERVO_4_MAX);
  
  if(!frontGood(15)){
    //Serial.print("s4 (!fg)");
    //Serial.println(value);

    if(value < prev_value_4) {
      return;
    }    
    
    pwm.setPWM(4, 551, 551 + value);
    prev_value_4 = value;
   
  }
  else {
    //Serial.print("s4 ");
    //Serial.println(value);
    pwm.setPWM(4, 551, 551 + value);
    prev_value_4 = value;

 }
}
void controlServo3(unsigned int pot_pin) {
  unsigned int value = smoothSignal(pot_pin, total_3, readings_3, read_index_3, num_readings);
  value = map(value, 0, 1023, SERVO_3_MIN, SERVO_3_MAX);
  value = reverse(SERVO_3_MIN, SERVO_3_MAX, value);
  //Serial.print("s3 ");
  //Serial.println(value);
  pwm.setPWM(3, 1152, 1152 + value);
}
void controlServo2(unsigned int pot_pin) {
  unsigned int value = smoothSignal(pot_pin, total_2, readings_2, read_index_2, num_readings);
  value = map(value, 0, 1023, SERVO_2_MIN, SERVO_2_MAX);
  //Serial.print("s2 ");
  //Serial.println(value);
  pwm.setPWM(2, 1652, 1652 + value);

}
void controlServo1(unsigned int pot_pin) {
  unsigned int value = smoothSignal(pot_pin, total_1, readings_1, read_index_1, num_readings);
  value = map(value, 0, 1023, SERVO_1_MIN, SERVO_1_MAX);
  //Serial.print("s1 ");
  //Serial.println(value);
  pwm.setPWM(1, 1952, 1952 + value);
}
void controlServo0(unsigned int pot_pin) {
  unsigned int value = smoothSignal(pot_pin, total_0, readings_0, read_index_0, num_readings);
  value = map(value, 0, 1023, SERVO_0_MIN, SERVO_0_MAX);
  //Serial.print("s0 ");
  //Serial.println(value);
  pwm.setPWM(0, 2502, 2502 + value);
}

// test function for testing servos
void continuousMove(uint8_t servo_num, unsigned int smin, unsigned int smax) {

  for (uint16_t pulselen = smin; pulselen < smax; pulselen++) {
    pwm.setPWM(servo_num, 0, pulselen);
  }
  delay(500);
  for (uint16_t pulselen = smax; pulselen > smin; pulselen--) {
    pwm.setPWM(servo_num, 0, pulselen);
  }
  delay(500);
  
}


void loop() {
  controlServo5(servo5_potpin);
  controlServo4(servo4_potpin);
  controlServo3(servo3_potpin);
  controlServo2(servo2_potpin);
  controlServo1(servo1_potpin);
  controlServo0(servo0_potpin);    
  
  delay(20);
}
