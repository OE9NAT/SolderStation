#include <Rotary.h>

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

/* Program for solder station
 * 
 * 
 */

// variables
int temp_IST = 10; //gemessener Wert
int temp_SOLL = 380; //eingestellte temperatur
int temp_pin = A6; // read analog temperature value
int pwm_heater_pin = 9 ; // output pin D9 for opptocopler for heater
unsigned int pwm_heater_duty = 10 ; // output PWM optocopler value between 0, 65535

int temp_measure(); //funktion to measure the temp

int temp_max = 400; // max einstellbare Temperatur in grad Celcius
int temp_min = 60;  // min einstellbare Temperatur in grad Celcius & stand by temp

// inputs rotary encoder and buttens
 Rotary r = Rotary(3, 2);  // rotary encoder on hardware interup pin D2 und D3

 const int button_rott_pin = 7;  // Digital pin D7 of encoder butten
  int button_rott_state = HIGH;   // Current state of the button
  int button_rott_last = HIGH;  // Previous state of the button
  bool POWERon = false ;

  const int stand_by_pin = 8;  // Digital pin D8 of encoder butten
  bool stand_by = false ;

// display
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
int led_status = 13; // pin on board
int led_rot = A0; // Analog pin 0 (also digital pin 14)
int led_gelb = A1; // Analog pin 1 (also digital pin 15)
int led_gruen = A2; //  Analog pin 2 (also digital pin 16)



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Serial.println("Boot system");
   // all leds off
   pinMode(led_status, OUTPUT);    // sets the digital as output
   digitalWrite(led_status, LOW); //LOW or HIGH
   pinMode(led_rot, OUTPUT);    // sets the digital as output
   digitalWrite(led_rot, LOW);
   pinMode(led_gelb, OUTPUT);    // sets the digital as output
   digitalWrite(led_gelb, LOW);
   pinMode(led_gruen, OUTPUT);    // sets the digital as output
   digitalWrite(led_gruen, LOW);

   // lcd display
    lcd.begin();           // initialize the LCD 
    lcd.backlight();

    lcd.setCursor(1,0); //position 1, line 0
    lcd.print("Solder station");
    lcd.setCursor(0,1);
    lcd.print("Start v0.1");
   
   //rotory interrupt setup  
   PCICR |= (1 << PCIE2);  
   PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);  
   sei(); 

   // butten
   pinMode(button_rott_pin, INPUT_PULLUP); // butten of rotary encoder input

   // temperature read input
     pinMode(temp_pin, OUTPUT);
     pinMode(stand_by_pin, INPUT_PULLUP); // pin to read if the sation is in use
     

   //pwm output
     pinMode(pwm_heater_pin, OUTPUT);
     
  
     // Configure Timer 2 for Phase-Correct PWM
     TCCR1A = _BV(COM1A1) | _BV(WGM10);  // Fast PWM mode, Clear OC1A on compare match
     TCCR1B = _BV(CS12) | _BV(CS10);     // Prescaler = 1024
     TCNT1 = 0;  // Reset the Timer1 count value to 0
     OCR1A = 0;  // Set PWM duty cycle (0 to 255)

  Serial.println("Start of program");
  delay(100);            // waits for a second

  Serial.println("Temp SOLL, Temp IST");

}

void loop() {
  // put your main code here, to run repeatedly:
//noInterrupts();
  //read temperature Value
  temp_IST = temp_measure();
  

  // set duty cycle of PWM of heater1 
  temp_SOLL = constrain(temp_SOLL, temp_min, temp_max); // limit the range of the set temperature
  
  //                     map(temp_SOLL,room temp, maximum reachable temperature, min duty, max duty)
  pwm_heater_duty = map(temp_SOLL, 25, 500, 0, 65535);  // Map Temp to counter value linearisation
  pwm_heater_duty = constrain(pwm_heater_duty , 1 ,65530 );
  
 

  //check if pen is in the station ... stand by
  stand_by = !digitalRead(stand_by_pin);
  

  if (!POWERon) { // power off
       OCR1A = 10; //set duty scile to 0% ... switch off 4%
       lcd.setCursor(0,1);
       lcd.print("OFF");
  }
    else if (stand_by) { // stand by mode with reduces power
    OCR1A = 100;  //40%
    lcd.setCursor(0,1);
    lcd.print("by ");
  }
  else { // full power ON
    OCR1A = map(pwm_heater_duty, 0, 65535, 0,255);  // Map Temp to counter value
    lcd.setCursor(0,1);
    lcd.print("ON ");
  }
//interrupts();
  
  /*
  //int counterValue = TCNT1;  // Read the current counter value
  int pwmValue = OCR1A;  // Read the current PWM value
  int analogValue = map(OCR1A, 0, 255, 0, 255); // Map PWM value to 0-255 range
  analogWrite(A5, OCR1A);  // Output mapped value on A6
  */

  // Rotary butten press
  button_rott_state = digitalRead(button_rott_pin);
  if (button_rott_state != button_rott_last) {     // If the button was pressed (transition from HIGH to LOW)
    if (button_rott_state == LOW && button_rott_last == HIGH) {
      Serial.println("Button Pressed!");
      POWERon = !POWERon; // invert signal
      //if (!POWERon) { POWERon = true; Serial.println("POWERon = true");} else {POWERon = false; };
      button_rott_state = button_rott_state;
    }
     
  }
  button_rott_last = button_rott_state;
    

//status and debugging
  if (POWERon) { digitalWrite(led_status, HIGH); } else { digitalWrite(led_status, LOW); }
  

  digitalWrite(led_rot, HIGH); //LOW or HIGH
  delay(100);            // waits for a second
  digitalWrite(led_rot, LOW); //LOW or HIGH
 

  digitalWrite(led_gelb, HIGH); //LOW or HIGH
  delay(100);            // waits for a second
  digitalWrite(led_gelb, LOW); //LOW or HIGH
  

  digitalWrite(led_gruen, HIGH); //LOW or HIGH
  delay(100);            // waits for a second
  digitalWrite(led_gruen, LOW); //LOW or HIGH
  

  Serial.println("  \n ");
  //Serial.print(String("#  Temp ist: ") + temp_IST);
  // Serial.print(String("#  Temp Soll: ") + temp_SOLL );
  //Serial.print("#  Temp diff: ");
  //Serial.print(temp_IST - temp_SOLL);
  // Serial.println("  End loop program ");
  

   Serial.print(temp_IST);
   Serial.print(",");
   Serial.println(temp_SOLL);
   //Serial.println(temp_SOLL);

   lcd.setCursor(12,0);
   lcd.print(temp_SOLL);
   lcd.print(" "); // to overwrite the old values


   lcd.setCursor(12,1);   
   lcd.print(temp_IST);
   lcd.print("   ");

   
   
  

}

// temperature input with rotery encoder
ISR(PCINT2_vect) {  
 
   unsigned char result = r.process();  
   if (result == DIR_NONE) {  
     // do nothing  
   }  
   else if (result == DIR_CW) {  
     //temp_SOLL++;
     temp_SOLL = temp_SOLL + 5; 
     
     if (temp_SOLL > temp_max) { // maximum temperature limit
      temp_SOLL = temp_max;
      Serial.print("maximum temperature limit "); 
     }
     
     Serial.print("CLOCKWISE  ");
     Serial.print(temp_SOLL);  
   }  
   else if (result == DIR_CCW) {  
     //temp_SOLL--;
     temp_SOLL = temp_SOLL - 5; 

     if (temp_SOLL < temp_min) { // minimum temperature limit
      temp_SOLL = temp_min;
      Serial.print("minimum temperature limit "); 
     }
     
     Serial.print("COUNTERCLOCK  ");  
     Serial.print(temp_SOLL);
   }  
 }  

 // Measure ist temperature
int temp_measure() {
  OCR1A = 0; //set duty scile to 0% ... power off
  delay(10);
  //read analog Value
  temp_IST = analogRead(temp_pin);

  
  return temp_IST;
}
