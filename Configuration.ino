// Yasteer Sewpersad
// Quadcopter Configuration 

#include <EEPROM.h> // Non-volatile memory. Can really write to the registers a lot so will be used sparingly. 

// Tx & Rx Variables
byte last_val1, last_val2, last_val3, last_val4;
unsigned long timeA, timeB, timeC, timeD, timeE, timeF, timeG, timeH, Current_Time;
volatile int receiver_input_1, receiver_input_2, receiver_input_3, receiver_input_4,Channel_1, Channel_2, Channel_3, Channel_4; // Volatile keyword tells controller to store value in RAM and not in a register. Typically used in ISRs. 
int throttle, sample_count, Throttle_Lower_Limit, Throttle_Upper_Limit, Yaw_Lower_Limit, Yaw_Upper_Limit;
bool CenterPointCal, VerticalCal, HorizontalCal;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initTransmitter();
  Serial.println("Setup Complete");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Store The Centerpoints
  if(CenterPointCal == false){
    delay(3000);
    Serial.println("Move All Joysticks To The Center");
    Channel_1 = receiver_input_1;
    Channel_2 = receiver_input_2;
    Channel_3 = receiver_input_3;
    Channel_4 = receiver_input_4;
    
    Serial.print(F("Channel 1 center point: "));
    Serial.println(Channel_1);
    Serial.print(F("Channel 2 center point: "));
    Serial.println(Channel_2);
    Serial.print(F("Channel 3 center point: "));
    Serial.println(Channel_3);
    Serial.print(F("Channel 4 center point: "));
    Serial.println(Channel_4);

    CenterPointCal = true; 
    // EEPROM.write(Address, Data) uses Data of type byte --> 0 - 255 so multiple addresses will be required to store int data (0 - 65 535).
    // 2 bytes = 1 int in Arduino --> 2^16 = 65 535 so using two registers in EEPROM is enough to store an int type.
    // Reference: http://electronics.scriblab.de/write-an-integer-to-the-arduino-eeprom/ 
    EEPROM.write(0, Channel_1 & 0xFF); // Store first 8 bits of int.
    EEPROM.write(1, Channel_1 >> 8);  // Store second 8 bits of int.
    
    EEPROM.write(2, Channel_2 & 0xFF);
    EEPROM.write(3, Channel_2 >> 8);
    
    EEPROM.write(4, Channel_3 & 0xFF);
    EEPROM.write(5, Channel_3 >> 8);

    EEPROM.write(6, Channel_4 & 0xFF);
    EEPROM.write(7, Channel_4 >> 8);
  }
  // Configure Vertical Min & Max Points
  if(VerticalCal == false){
    Serial.println("Now move the throttle to zero.");
    delay(5000);
    Throttle_Lower_Limit = receiver_input_3;
    Serial.println("Now move the throttle to max");
    delay(5000);
    Throttle_Upper_Limit = receiver_input_3;
    EEPROM.write(8, Throttle_Lower_Limit & 0xFF);
    EEPROM.write(9, Throttle_Lower_Limit >> 8);

    EEPROM.write(10, Throttle_Upper_Limit & 0xFF);
    EEPROM.write(11, Throttle_Upper_Limit >> 8);
    
    Serial.println("Vertical Limits Set!");
    VerticalCal = true;
  }
  // Configure Horizontal Min & Max Points
  if(HorizontalCal == false){
    Serial.println(" ");
    Serial.println("Move throttle joystick to the left.");
    delay(5000);
    Yaw_Lower_Limit = receiver_input_4;
    Serial.println("Move throttle joystick to the right");
    delay(5000);
    Yaw_Upper_Limit = receiver_input_4;
    EEPROM.write(12, Yaw_Lower_Limit  & 0xFF);
    EEPROM.write(13, Yaw_Lower_Limit >> 8);
    
    EEPROM.write(14, Yaw_Upper_Limit  & 0xFF);
    EEPROM.write(15, Yaw_Upper_Limit >> 8);
    Serial.println("Great all setpoints have been captured!");
    HorizontalCal = true;
  }
}

void initTransmitter() {
   PCICR  |= (1<<PCIE0); // Enable Interupts
   PCMSK0 |= (1<<PCINT0); // Arduino Pin 8 triggers interupt on state change.
   PCMSK0 |= (1<<PCINT1); // Arduino Pin 9 ""                               ""
   PCMSK0 |= (1<<PCINT2); // Arduino Pin 10 ""                              ""
   PCMSK0 |= (1<<PCINT3); // Arduino Pin 11 ""                              ""
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
} // Reading receiver signals takes a long time so we don't want it to slow the refresh rate of the controller. Hence the need for an ISR. 
