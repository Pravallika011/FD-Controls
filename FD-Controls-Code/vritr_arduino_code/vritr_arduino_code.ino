#include <Wire.h> // For SPI comunications
#include <PWMServo.h> // for servo actuation
#include <SPI.h> // For spi pins
#include <SD.h> // To write to SD card
#include <MS5611.h> // for pressure sensor`
#include <math.h>

//Final Compiled Code For the Canard Plus airbrake system

//variables for sensors and actuators
File myFile;
PWMServo s1,s2,s3,s4;
MS5611 baro(0x77);

#define datafile "datalog1.txt"
#define tablefile "lookup.txt" 
float lookup[10500];
#define setpoint 0
#define samp_time 0.01

unsigned long previousmillis, currentmillis, startmillis;
const int looptime = 15;

int prevposition[4] = {0,0,0,0};
int servo[4] = {0,0,0,0};
float servocurrent[4] = {0,0,0,0};
int neutral[4] = {89,53,79,50};

float prev_gyrox[2] = {0,0};
float prev_gyroy[2] = {0,0};
float prev_gyroz[2] = {0,0};
float prev_accz[2] = {0,0};
float accx, accy, accz;
float gyrox, gyroy, gyroz;
float fgyrox[3], fgyroy[3], fgyroz[3], faccz[3] = {0,0,0};

float temp;
float pressure;
float density;

float zeroangle;
float prev_angle[2] = {0,0};
uint16_t rawangle;
float angle,absoluteangle;
int turns = 0;

float motorintegral[2] = {0,0};
float motorerror[3] = {0,0,0};
float brakeangle;

float b0 = 0.02008;
float b1 = 0.04016;
float b2 = 0.02008;
float a1 = -1.561;
float a2 = 0.6414;

float inclination;

// For airbrakes
float vertical_velocity = 0;
float velocity_prev = 0;
float altitude = 0;
float altitude_pre = 0;
float airbrake_theta = 0;
float prediced_apogee = 0;
float target_apogee = 3048; //hope the units are meters
float airbrake_gain = 1;
float ab_max = 60;
float motor_angle_slope = 1200/73;

//  for canards
float roll_rate = 0;
float roll_angle = 0;
float roll_limit = 0; // rad per sec
float max_roll_rate = 31.416; // 5 rps
int canard_actutation_time = 5; //sec
float max_delta = 0.12217; //rad
float sliding_cons = 2.0;
float roll_damping = -0.1;
float dyn_pressure = 0;
float c_lv = -0.22;
float L_p = 0;
float S_ref = 0.02;
float L_ref = 4.83;
float Ixx = 85.85;
float q_breakpoints[2] = {1e4, 7e5};
float Cl_values[2]     = {0.005, 0.02};
float cl_delta = 0;
float L_delta = 0;
const float rho_base = 0.08;
const float k_base = 0.8;
float rho_adapted = 0;
float k_adapted = 0;
float control_delta = 0;
float dyn_p_norm = 0;


//for failsafes
int detect_launch = 0; //1 is true
int detect_burnout = 0; // 1 is true
int launch_g = 3*9.81;
int lauch_altitude = 15;
int burnout_altitude = 1500;
int launch_time = 0;
int start_time = 0;
int airbrake_disable = 0;

float pitch = 0;
float yaw = 0;
float roll = 0;


bool canard_actuate = true;
bool airbrake_actuate = true;


void datagenerator(void){
  // altitude 
  float sea_pressure = 101325;
  altitude_pre = altitude;
  altitude = ((temp + 273)/0.0065)*(1.0 - pow((pressure*100/sea_pressure),0.1903));
  float alt_veloc = (altitude - altitude_pre)/(0.001*looptime);
  float acc_velocity = looptime*0.001*(faccz[0] - 9.81);
  vertical_velocity = 0.01*alt_veloc + 9.99*acc_velocity;

  // for orientation
  pitch += fgyrox[0] * looptime*0.001;
  yaw  += fgyroy[0] * looptime*0.001;
  roll += fgyroz[0]*looptime*0.001;
  roll_rate = fgyroz[0];

  // Calculate net inclination angle (from vertical), ignoring roll
  inclination = sqrt(pitch * pitch + yaw * yaw);
}

bool pitch_check(void){
  if( inclination > 25){
    return true;
  } else{
    return false;
  }
}

bool load_check(void){
  currentsense();
  int average = 0;
  int error = 0;
  for(int i=0;i<4;i++){
    average += servocurrent[i];
  }
  average = average/4;
  for(int i=0;i<4;i++){
    error += abs( average - servocurrent[i]);
  }
  if (error > 2){  //error is greater than 2 amps
    return true;
  }
  else return false;
}

void canard_datas(void){ // density, not accounting for pressure
  density = pressure*100/(287.05*(temp +273));
  dyn_pressure = 0.5*density*vertical_velocity*vertical_velocity;

  if (dyn_pressure <= q_breakpoints[0]){
    cl_delta = Cl_values[0];
  }
  else if (dyn_pressure >= q_breakpoints[1]){
    cl_delta = Cl_values[1];
  }
  else{
    cl_delta = Cl_values[0] + (Cl_values[1] - Cl_values[0])*(dyn_pressure - q_breakpoints[0]) / (q_breakpoints[1] - q_breakpoints[0]);
  }
  L_delta = dyn_pressure*S_ref*L_ref*cl_delta/Ixx;
  L_p = dyn_pressure*S_ref*L_ref*c_lv/Ixx;
  dyn_p_norm = dyn_pressure/700000.0;
  if (dyn_p_norm<0.01){
    dyn_p_norm = 0.01;
  }
  else if(dyn_p_norm >1.0){
    dyn_p_norm = 1.0;
  }
  //rho_adapted = rho_base/(dyn_p_norm + 1e-3);
  k_adapted = k_base*dyn_p_norm;
}
int detect_roll_reversal(){
  if(abs(fgyroz[1]) - abs(fgyroz[0]) >= 0){
    return -1;
  }else{
    return 1;
  }
}

void canard_control(void){
  canard_datas();
  float canard_error = roll_limit - abs(roll_rate)*3.14/180;
  rho_adapted = rho_base*(1+6*abs(canard_error))/(dyn_p_norm + 0.001);
  sliding_cons = 4 +1.5*(vertical_velocity/300)+3*abs(canard_error);
  float sigma = canard_error + sliding_cons*roll;
  float sigma_smooth = sigma /(abs(sigma) + k_adapted + 1e-5);
  control_delta = detect_roll_reversal()*(roll_limit - roll_damping*roll_rate + sliding_cons*canard_error+ rho_adapted*sigma_smooth)/(L_delta+1e-5);
  if (canard_error < 0.1){
    control_delta = 0;
  } else if (control_delta>max_delta){
    control_delta = max_delta;
  }
  else if(control_delta< - max_delta){
    control_delta = -max_delta;
  }
}






void get_predicted_altitude(void){
  int velocity_index = int(vertical_velocity/20) + 1;
  int altitude_index = int(altitude/35);
  int airbrake_index = int(airbrake_theta/15);
  int table_index = altitude_index*35 + velocity_index*20 + airbrake_index*15;
  prediced_apogee = lookup[table_index];

}

void airbrake_control(void){
  float airbrake_error = prediced_apogee - target_apogee;
  float dyn_gain = airbrake_gain*(1 + 0.3*(vertical_velocity/100)*(vertical_velocity/100));
  float control_ab = 1.0 - exp(-dyn_gain*abs(airbrake_error));
  if (airbrake_error > 25){
    airbrake_theta = airbrake_theta + control_ab*ab_max;
  } else if (airbrake_error < 0){
    airbrake_theta = 0;
  }
  if (airbrake_theta > ab_max){
    airbrake_theta = ab_max;
  }
}


void filtered(void) { // Noise filtering Gyros
  fgyrox[0] = b0*gyrox + b1*prev_gyrox[0] + b2*prev_gyrox[1] - a1*fgyrox[1] - a2*fgyrox[2];
  fgyrox[2] = fgyrox[1];
  fgyrox[1] = fgyrox[0];
  prev_gyrox[1] = prev_gyrox[0];
  prev_gyrox[0] = gyrox;
  fgyroy[0] = b0*gyroy + b1*prev_gyroy[0] + b2*prev_gyroy[1] - a1*fgyroy[1] - a2*fgyroy[2];
  fgyroy[2] = fgyroy[1];
  fgyroy[1] = fgyroy[0];
  prev_gyroy[1] = prev_gyroy[0];
  prev_gyroy[0] = gyroy;
  fgyroz[0] = b0*gyroz + b1*prev_gyroz[0] + b2*prev_gyroz[1] - a1*fgyroz[1] - a2*fgyroz[2];
  fgyroz[2] = fgyroz[1];
  fgyroz[1] = fgyroz[0];
  prev_gyroz[1] = prev_gyroz[0];
  prev_gyroz[0] = gyroz;
  faccz[0] = b0*accz + b1*prev_accz[0] + b2*prev_accz[1] -a1*faccz[1] - a2*faccz[2];
  faccz[2] = faccz[1];
  faccz[1] = faccz[0];
  prev_accz[1] = prev_accz[0];
  prev_accz[0] = accz;

}

void currentsense(void) {
  float constant = 0;
  for (int i=0;i<4;i++) {
    servocurrent[i] = analogRead(27-i)*constant;
  }
}

void writefile() {
  char buffer[100];
  sprintf(buffer, "%.3f, %.2f, %.2f, %.2f, %.2f, %.2f",float(currentmillis),fgyroz[0],inclination,vertical_velocity,altitude,control_delta);
  myFile = SD.open(datafile, FILE_WRITE);
  myFile.println(buffer);
  myFile.close();
}


void setzeroangle(void) {
  Wire.beginTransmission(0x36);
  Wire.write(0x0E);
  Wire.endTransmission();
  Wire.requestFrom(0x36,2);
  rawangle = Wire.read()<<8|Wire.read();
  angle = rawangle*0.087890625;
  zeroangle = angle;
}

void calculateangle(void) {
  Wire.beginTransmission(0x36);
  Wire.write(0x0E);
  Wire.endTransmission();
  Wire.requestFrom(0x36,2);
  rawangle = Wire.read()<<8|Wire.read();
  angle = rawangle*0.087890625;
  prev_angle[1] = prev_angle[0];
  prev_angle[0] = angle;
  if (prev_angle[1]>prev_angle[0] && (prev_angle[1]-prev_angle[0])>200) {
    turns+=1;
  }
  if (prev_angle[1]<prev_angle[0] && (prev_angle[0]-prev_angle[1])>200) {
    if (turns>0) {
      turns-=1;
    }
  }
  absoluteangle = turns*360 + angle - zeroangle;
}

void drivemotor(void) {
  motorerror[0] = brakeangle - absoluteangle;
  motorintegral[0] = (0.001*looptime/2)*(motorerror[0]+(2*motorerror[1])+motorerror[2]) + motorintegral[1];
  float motorderivative = (motorerror[0]-motorerror[1])/0.01;
  float ut = 0.01*motorerror[0] + 0.02*motorintegral[0] + 0.02*motorderivative;
  motorintegral[1] = motorintegral[0];
  ut = constrain(ut,-100,100);
  if (ut>=0) {
    digitalWrite(15,HIGH);
  }
  else if (ut<0) {
    digitalWrite(15,LOW);
  }
  int pwm = abs((ut/100)*256);
  if (pwm>15) {
    analogWrite(14,pwm);
  }
  else {
    analogWrite(14,0);
  }
}


void calculatebaro(void) {
  digitalWrite(LED_BUILTIN, HIGH);
  baro.setOversampling(OSR_ULTRA_LOW);
  baro.read();
  temp = baro.getTemperature();
  pressure = baro.getPressure();
  digitalWrite(LED_BUILTIN,LOW);
}

void setupbno(void) {
  Wire.beginTransmission(0x28);
  Wire.write(0x3E); //power mode
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x3D); //operation mode
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x3B); //units
  Wire.write(0x00);
  Wire.endTransmission();
}

void getaccel(void) {
  Wire.beginTransmission(0x28);
  Wire.write(0x08); //accel settings
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x08); //accel data starting register
  Wire.endTransmission();
  Wire.requestFrom(0x28, 6);
  int16_t AccelX = Wire.read() | Wire.read() << 8;
  int16_t AccelY = Wire.read() | Wire.read() << 8;
  int16_t AccelZ = Wire.read() | Wire.read() << 8;
  accx = (float)AccelX/100;
  accy = (float)AccelY/100;
  accz = (float)AccelZ/100;
}

void getgyro(void) {
  Wire.beginTransmission(0x28);
  Wire.write(0x0A); //gyro settings 1
  Wire.write(0x28);
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x0B); //gyro settings 2
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x14); //gyro data starting register
  Wire.endTransmission();
  Wire.requestFrom(0x28,6);
  int16_t GyroX = Wire.read() | Wire.read() << 8;
  int16_t GyroY = Wire.read() | Wire.read() << 8;
  int16_t GyroZ = Wire.read() | Wire.read() << 8;
  gyrox = (float)GyroX/16;
  gyroy = (float)GyroY/16;
  gyroz = (float)GyroZ/16;
}


void setup() {
  SD.begin(BUILTIN_SDCARD);
  startmillis = millis();
  myFile = SD.open(datafile, FILE_WRITE);
  myFile.println("Setup Started");
  myFile.close();
  s1.attach(7);
  s2.attach(6);
  s3.attach(5);
  s4.attach(4);
  pinMode(13,OUTPUT); //led
  pinMode(27,INPUT); //current sense 1
  pinMode(26,INPUT); //current sense 2
  pinMode(25,INPUT); //current sense 3
  pinMode(24,INPUT); //current sense 4
  pinMode(15,OUTPUT); //motor direction
  pinMode(14,OUTPUT); //motor pwm
  analogWrite(14,0);
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  setzeroangle();
  setupbno();
  baro.begin();
  myFile = SD.open(datafile, FILE_WRITE);
  currentmillis = millis();
  myFile.print("Setup Finished - ");
  myFile.println((currentmillis-startmillis)/1000.0);
  myFile.close();
  previousmillis = currentmillis = millis();
  s1.write(neutral[0] + 45);
  s2.write(neutral[1] + 45);
  s3.write(neutral[2] + 45);
  s4.write(neutral[3] + 45);
  delay(500);
  s1.write(neutral[0]);
  s2.write(neutral[1]);
  s3.write(neutral[2]);
  s4.write(neutral[3]);
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentmillis = millis();
  if (currentmillis-previousmillis>looptime){
    previousmillis = currentmillis;
    getgyro();
    getaccel();
    filtered();
    calculatebaro();
    if (detect_launch>=5){
      datagenerator();
    }
    if (detect_launch<5 ){
      if (gyroz>launch_g || altitude > lauch_altitude){
        detect_launch += 1;
      }
    if (detect_launch == 5){
        launch_time = currentmillis;
      }
    } else if (detect_burnout < 5){
      if ((currentmillis - launch_time) > 5000 || altitude > burnout_altitude){
        detect_burnout += 1;
      }
      if (detect_burnout == 5) {
        start_time = currentmillis;
    }
    } else {
    int actuation_time = currentmillis - start_time;
    if (actuation_time < 15000 && detect_burnout >= 5){
      if (!pitch_check() && !load_check() && canard_actuate){
        canard_control();
        float deflection_angle = control_delta*180/3.14;
        servo[0] = neutral[0] + deflection_angle;
        servo[1] = neutral[1] + deflection_angle;
        servo[2] = neutral[2] + deflection_angle;
        servo[3] = neutral[3] + deflection_angle;
      }else{
        servo[0] = neutral[0];
        servo[1] = neutral[1];
        servo[2] = neutral[2];
        servo[3] = neutral[3];
        canard_actuate = false;
      }
    s1.write(servo[0]);
    s2.write(servo[1]);
    s3.write(servo[2]);
    s4.write(servo[3]);
    }
    }
    writefile();
  }
}