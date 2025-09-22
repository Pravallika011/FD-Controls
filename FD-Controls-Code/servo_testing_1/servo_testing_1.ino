#include <PWMServo.h>
#include <SD.h>

PWMServo s1,s2,s3,s4;
int servo_angles[4] = {0,0,0,0};
float servo_current[4] = {0,0,0,0};
float reading_time = 0; //ms
float current_reading_timer = 0;
float actuating_time = 500*1000; //ms
float current_actuating_timer = 0;
float prevmillis;
float dt;

int dir = 1;

void move_servos_to_angle(int angle){
  s1.write(angle);
  s2.write(angle);
  s3.write(angle);
  s4.write(angle);
}

void setup() {
  prevmillis = micros();
  Serial.begin(9600);
  s1.attach(7);
  s2.attach(6);
  s3.attach(5);
  s4.attach(4);
}

void loop() {
  dt = micros() - prevmillis;
  prevmillis = micros();

  current_reading_timer += dt;
  current_actuating_timer += dt;

  if(current_actuating_timer > actuating_time){
  move_servos_to_angle(45*dir);
  current_actuating_timer = 0;
  dir *= -1;
  }

  //printing output
  if(current_reading_timer > reading_time){
  // Serial.print("Servo Currents: ");
  for(int i=0; i<4; i++){
    Serial.print(servo_current[i]);
    Serial.print(" ");
  }
  Serial.println(); 

  for(int i = 0; i<4; i++){
  float current = (analogRead(27-i)*5/1023.0 - 2.5)/0.1;
  servo_current[i] = current;
  current_reading_timer = 0;
  }
  }
}

