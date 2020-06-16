// Yasteer Sewpersad Drone Build Version2
/* System Components:
 *  4x XXD HW30A ESC
 *  4x RacerStar BR2212 1000KV Brushless Motor
 *  1x LiPo Battery (11.1V @ 2200mah)
 *  1x Arduino Uno
 *  1x Xpower 2200mah Battery
 */

#include <Servo.h>
#include <Wire.h>
#include <Math.h>
#include <LiquidCrystal_I2C.h>

//float rad_to_deg = 180/3.141592654;

// LCD Object & Variables
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3, POSITIVE); 

int analogPin = 0;
float ADC_Voltage; // ADC allows for a maximum of 5V to be read as a 10 Bit number. 
float Calculated_Voltage, Vin;
float Lowest_Voltage = 3*3.5; // 3s battery, lowest voltage per cell is 3.5
float Nominal_Voltage = 11.1;
float Diode_Volt_Drop = 0.70;

// Servo Objects
float time, timePrev, Elapsed_Time;
Servo M1,M2,M3,M4;
int esc_1, esc_2, esc_3, esc_4;

// Tx & Rx Variables
byte last_val1, last_val2, last_val3, last_val4;
unsigned long timeA, timeB, timeC, timeD, timeE, timeF, timeG, timeH, Current_Time;
volatile int receiver_input_1, receiver_input_2, receiver_input_3, receiver_input_4; // Volatile keyword tells controller to store value in RAM and not in a register. Typically used in ISRs. 
int throttle;

// Gyro & Accel Variables
const int MPU_Address = 0x68;
int16_t AccelX_Raw, AccelY_Raw, AccelZ_Raw, GyroX_Raw, GyroY_Raw, GyroZ_Raw; // 16-bit variable required to store output of MPU-6050
float Accel_Angle[3], Gyro_Angle[3], Coupled_Roll_Angle, Coupled_Pitch_Angle, Total_Angle[3]; // Arrays to store X,Y & Z data from the device. 
int Refresh_Rate = 250;

// PID Variables
int desired_angle = 0;
float PID_ROLL, PID_YAW, PID_PITCH;
double Roll_Kp = 1.3;
double Roll_Ki = 0.04;
double Roll_Kd = 18;
double Pitch_Kp = Roll_Kp;
double Pitch_Ki = Roll_Ki;
double Pitch_Kd = Roll_Kd;
double Yaw_Kp = 4;
double Yaw_Ki = 0.02;
double Yaw_Kd = 0;

int max_PID_roll = 400;
int max_PID_yaw = 400;
int max_PID_pitch = max_PID_roll;
double Roll_Setpoint, Yaw_Setpoint, Pitch_Setpoint, Roll_Error, Pitch_Error, Yaw_Error, PID_Previous_Roll_Error, PID_Previous_Pitch_Error, PID_Previous_Yaw_Error;
double Roll_P, Yaw_P , Pitch_P, Roll_I, Yaw_I , Pitch_I, Roll_D, Yaw_D, Pitch_D;

// Refresh Rate Measurements
unsigned long Loop_Timer_Start;

void setup() {
  // put your setup code here, to run once:
    
    initTransmitter();
    initMotors();
    
    Wire.begin();
    Wire.beginTransmission(MPU_Address);
    Wire.write(0x6B); // Move pointer to the Power Management Register
    Wire.write(0);
    Wire.endTransmission(true); // Keep default scaling on Accelerometer and Gyroscope -> 1g = 16384 units.

    lcd.begin(16,2);
    lcd.clear();
    lcd.setBacklight(LOW);
    
    Serial.begin(9600);
    time = millis(); // Start the timer.
    M1.writeMicroseconds(1000);
    M2.writeMicroseconds(1000);
    M3.writeMicroseconds(1000);
    M4.writeMicroseconds(1000);
    delayMicroseconds(10000); 
    Serial.println("Setup Complete.");
}

void loop() {
  // put your main code here, to run repeatedly:
  Loop_Timer_Start = micros(); // Start the timer for measuring the refresh rate.
  if(((int)time % 1000) == 0){
    lcd.clear();
    ADC_Voltage = analogRead(analogPin);
    Calculated_Voltage = (ADC_Voltage * 0.0049) + (Diode_Volt_Drop) - 0.4; // 1 unit in ADC = 0.0049 V
    Vin = Calculated_Voltage*((1000+2000)/1000); // Solve voltage divider equation with resistor values used in the circuit. 
    lcd.print(String("Cell Volts:") + String(Vin));
    if(Vin < Lowest_Voltage) {
      lcd.setBacklight(HIGH);
      lcd.setCursor(0,1);
      lcd.print(String("Dead Battery!"));
    }
  }
  
  timePrev = time;
  time = millis();
  Elapsed_Time = (time - timePrev)/1000;

  // Read in the accelerometer data:
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x3B); // Move pointer to the first output register of the IMU device.
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_Address, 6, true);
  AccelX_Raw =  Wire.read()<<8 | Wire.read(); // Read & combine register values using bit-shifting. Bit-shifting allows the fist bit to be set as high and the second bit to be set as low.  
  AccelY_Raw =  Wire.read()<<8 | Wire.read();
  AccelZ_Raw =  Wire.read()<<8 | Wire.read(); //Division by 16384 to convert reading to g.

  // Reference: https://engineering.stackexchange.com/questions/3348/calculating-pitch-yaw-and-roll-from-mag-acc-and-gyro-data
  Accel_Angle[0] = 180*atan((AccelY_Raw/16384.0)/(sqrt(pow(AccelX_Raw/16384.0,2)+pow(AccelZ_Raw/16384.0,2))))/M_PI; // Roll Measurement From Accelerometer(Degrees)
  Accel_Angle[1] = 180*atan((AccelX_Raw/16384.0)/(sqrt(pow(AccelY_Raw/16384.0,2)+pow(AccelZ_Raw/16384.0,2))))/M_PI; // Pitch Measurement From Accelerometer(Degrees)

  // Read in the gyroscope data:
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_Address, 6, true);
  GyroX_Raw = Wire.read()<<8 | Wire.read();
  GyroY_Raw = Wire.read()<<8 | Wire.read();
  GyroZ_Raw = Wire.read()<<8 | Wire.read();

  Gyro_Angle[0] += GyroX_Raw/Refresh_Rate/131; // Integrate gyro readings
  Gyro_Angle[1] += GyroY_Raw/Refresh_Rate/131;
  Gyro_Angle[2] += (GyroZ_Raw)/(float)Refresh_Rate/((float)131);

  // Reference: MPU-6050 6dof IMU tutorial for auto-leveling quadcopters with Arduino source code - Joop Brokking.
  Gyro_Angle[0] -= Gyro_Angle[1] * sin(Gyro_Angle[2]*(M_PI/180));  // Couple gyro readings.
  Gyro_Angle[1] += Gyro_Angle[0] * sin(Gyro_Angle[2]*(M_PI/180));
  
  // Combine Sensors Data:
  Total_Angle[0] = 0.98*(Total_Angle[0] + Gyro_Angle[0]) + 0.02*(Accel_Angle[0]);
  Total_Angle[1] = 0.98*(Total_Angle[1] + Gyro_Angle[1]) + 0.02*(Accel_Angle[1]);
  Total_Angle[2] = Gyro_Angle[2];

  // Scale Receiver Outputs For Faster Roll, Pitch & Yaw Rates
  Roll_Setpoint = 0; 
  if(receiver_input_1 > 1508) Roll_Setpoint = (receiver_input_1 - 1508)/3.0; // 3.0 Controls roll rate --> Joop Broking YMFC Video
  else if(receiver_input_1 < 1492) Roll_Setpoint = (receiver_input_1 - 1492)/3.0; // Gap between 1492 & 1508 creates a deadband to prevent noise affecting calculation.

  Pitch_Setpoint = 0;
  if(receiver_input_2 > 1508) Pitch_Setpoint = (receiver_input_2 - 1508)/3.0;
  else if(receiver_input_2 < 1492) Pitch_Setpoint = (receiver_input_2 - 1492)/3.0;

  Yaw_Setpoint = 0;
  if(receiver_input_4 > 1508) Yaw_Setpoint = (receiver_input_4 - 1508)/3.0;
  else if(receiver_input_4 < 1492) Yaw_Setpoint = (receiver_input_4 - 1492)/3.0;
  
  Compute_PID(); // Function Call.
  
  throttle = receiver_input_3;
  if(throttle > 1800) throttle = 1800; // Set a limit for maximum throttle so controllers can maintain safe operation. 
  
  esc_1 = throttle + PID_ROLL - PID_YAW - PID_PITCH; // Addition/Subtraction neccessary to maintain balance of torque.
  esc_2 = throttle + PID_ROLL + PID_YAW + PID_PITCH;
  esc_3 = throttle - PID_ROLL + PID_YAW - PID_PITCH;
  esc_4 = throttle - PID_ROLL - PID_YAW + PID_PITCH;

  esc_1 += esc_1*(Nominal_Voltage - Vin)/10; // Compensate motors for reducing battery voltage.
  esc_2 += esc_2*(Nominal_Voltage - Vin)/10;
  esc_3 += esc_3*(Nominal_Voltage - Vin)/10;
  esc_4 += esc_4*(Nominal_Voltage - Vin)/10;

  if(esc_1 < 1000) esc_1 = 1200; // Lower limit set to prevent motors from turning off
  if(esc_1 > 2000) esc_1 = 2000;
  
  if(esc_2 < 1000) esc_2 = 1200;
  if(esc_2 > 2000) esc_2 = 2000;
  
  if(esc_3 < 1000) esc_3 = 1200;
  if(esc_3 > 2000) esc_3 = 2000;
  
  if(esc_4 < 1000) esc_4 = 1200;
  if(esc_4 > 2000) esc_4 = 2000;

  //M1.writeMicroseconds(esc_1);
  //M2.writeMicroseconds(esc_2);
  //M3.writeMicroseconds(esc_3);
  //M4.writeMicroseconds(esc_4);
   
  //if((micros() - Loop_Timer_Start) > 4050)Serial.println("Over 4ms!");
} // End of Void Loop  

void initTransmitter() {
   PCICR  |= (1<<PCIE0); // Enable Interupts
   PCMSK0 |= (1<<PCINT0); // Arduino Pin 8 triggers interupt on state change.
   PCMSK0 |= (1<<PCINT1); // Arduino Pin 9 ""                               ""
   PCMSK0 |= (1<<PCINT2); // Arduino Pin 10 ""                              ""
   PCMSK0 |= (1<<PCINT3); // Arduino Pin 11 ""                              ""
}

void initMotors() {
  M1.attach(4, 1000, 2000);
  M2.attach(5, 1000, 2000);
  M3.attach(6, 1000, 2000);
  M4.attach(7, 1000, 2000);
}

ISR (PCINT0_vect) { // ISR must determine time of each square wave. This can then be used to control the ESC's. 
  // Channel 1
  if((last_val1 == 0) && (PINB & B00000001)) { // && is logical AND while & is bitwise &. The second bracket checks channel 1 on Port B for input. 
    last_val1 = 1;
    timeA = micros(); // Fetch time of rise.     
  }
  else if((last_val1 == 1) && !(PINB & B00000001)) { // Check if value changes from high to low. PINB & B00000001 is the same as digitalRead() but is faster --> Joop Brokking
    last_val1 = 0;
    receiver_input_1 = micros() - timeA; // Horizontal direction on RHS Gamble.
  }
  // Channel 2
  if((last_val2 == 0) && (PINB & B00000010)) { // && is logical AND while & is bitwise &. The second bracket checks channel 1 on Port B for input. 
    last_val2 = 1;
    timeC = micros(); // Fetch time of rise. 
  }
  else if((last_val2 == 1) && !(PINB & B00000010)) { // Check if value changes from high to low. PINB & B00000001 is the same as digitalRead() but is faster --> Joop Brokking
    last_val2 = 0;
    receiver_input_2 = micros() - timeC; // Vertical direction in RHS Gamble.
  }
  // Channel 3
  if((last_val3 == 0) && (PINB & B00000100)) { // && is logical AND while & is bitwise &. The second bracket checks channel 1 on Port B for input. 
    last_val3 = 1;
    timeE = micros(); // Fetch time of rise. 
  }
  else if((last_val3 == 1) && !(PINB & B00000100)) { // Check if value changes from high to low. PINB & B00000001 is the same as digitalRead() but is faster --> Joop Brokking
    last_val3 = 0;
    receiver_input_3 = micros() - timeE; // Vertical direction in LHS Gamble.
  }
  // Channel 4
  if((last_val4 == 0) && (PINB & B00001000)) { // && is logical AND while & is bitwise &. The second bracket checks channel 1 on Port B for input. 
    last_val4 = 1;
    timeG = micros(); // Fetch time of rise. 
  }
  else if((last_val4 == 1) && !(PINB & B00001000)) { // Check if value changes from high to low. PINB & B00000001 is the same as digitalRead() but is faster --> Joop Brokking
    last_val4 = 0;
    receiver_input_4 = micros() - timeG; // Horizontal direction in LHS Gamble. 
  }
} // Reading receiver signals takes a long time so we don't want it to slow the reffresh rate of the controller. Hence the need for an ISR. 

void Compute_PID(){
  
  // Roll
  Roll_Error = Total_Angle[0] - Roll_Setpoint;
  Roll_P = Roll_Kp * Roll_Error;
  Roll_I += Roll_Ki * Roll_Error;
  if(Roll_I > max_PID_roll) Roll_I = max_PID_roll;
  else if(Roll_I < max_PID_roll*-1) Roll_I = max_PID_roll*-1;
  Roll_D = Roll_Kd * (Roll_Error - PID_Previous_Roll_Error);
  PID_ROLL = (Roll_P) + (Roll_I) + (Roll_D); 
  if(PID_ROLL > max_PID_roll) PID_ROLL = max_PID_roll;
  else if(PID_ROLL < max_PID_roll*-1) PID_ROLL = max_PID_roll*-1;
  PID_Previous_Roll_Error = Roll_Error;
  
  // Pitch
  Pitch_Error = Total_Angle[1] - Pitch_Setpoint;
  Pitch_P = Pitch_Kp * Pitch_Error;
  Pitch_I += Pitch_Ki * Pitch_Error;
  if(Pitch_I > max_PID_pitch) Pitch_I = max_PID_pitch;
  else if(Pitch_I < max_PID_pitch*-1) Pitch_I = max_PID_pitch*-1;
  Pitch_D = Pitch_Kd * (Pitch_Error - PID_Previous_Pitch_Error);
  PID_PITCH = (Pitch_P) + (Pitch_I) + (Pitch_D); 
  if(PID_PITCH > max_PID_pitch) PID_PITCH = max_PID_pitch;
  else if(PID_PITCH < max_PID_pitch*-1) PID_PITCH = max_PID_pitch*-1;
  PID_Previous_Pitch_Error = Pitch_Error;
  
  // Yaw
  Yaw_Error = Total_Angle[2] - Yaw_Setpoint;
  Yaw_P = Yaw_Kp * Yaw_Error;
  Yaw_I += Yaw_Ki * Yaw_Error;
  if(Yaw_I > max_PID_yaw) Yaw_I = max_PID_yaw;
  else if(Yaw_I < max_PID_yaw*-1) Yaw_I = max_PID_yaw*-1;
  Yaw_D = Yaw_Kd * (Yaw_Error - PID_Previous_Yaw_Error);
  PID_YAW = (Yaw_P) + (Yaw_I) + (Yaw_D); 
  if(PID_YAW > max_PID_yaw) PID_YAW = max_PID_yaw;
  else if(PID_YAW < max_PID_yaw*-1) PID_YAW = max_PID_yaw*-1;
  PID_Previous_Yaw_Error = Yaw_Error;
  
}
