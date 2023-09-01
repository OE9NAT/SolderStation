#include <Rotary.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

/* Program for solder station
 * 
 * 
 */

// variables control
  int temp_IST = 10; //gemessener Wert
  int temp_SOLL = 380; //eingestellte temperatur
  int temp_pin = A6; // read analog temperature value
  unsigned int pwm_heater_duty = 1 ; // output PWM optocopler value between 0, 65535
 // conditions
  const int stand_by_pin = 8;  // Digital pin D8 of encoder butten icon pull pin to ground
  bool stand_by = false ;
  bool POWERon = false ;

  int temp_measure(); //funktion to measure the temp
  int temp_max = 400; // max einstellbare Temperatur in grad Celcius
  int temp_min = 60;  // min einstellbare Temperatur in grad Celcius 
  int temp_delta = 20;
  int temp_standby = temp_min; // stand by temp


// inputs rotary encoder and buttens
  Rotary r = Rotary(3, 2);  // rotary encoder on hardware interup pin D2 und D3
  const int button_rott_pin = A3;  // A3 of encoder butten
  int button_rott_state = HIGH;   // Current state of the button
  int button_rott_last = HIGH;  // Previous state of the button
  
//PWM duty cylce
  const int pwm_heater_pin = 9;  // Digital pin 9 for PWM output to optocopler
  int temp = 0;          // Variable to store the value for counter

// display
  LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
  int led_status = 13; // pin on board
  int led_rot = A0; // Analog pin 0 (also digital pin 14)
  int led_gelb = A1; // Analog pin 1 (also digital pin 15)
  int led_gruen = A2; //  Analog pin 2 (also digital pin 16)

// #######################################################################################################################
void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 bps  
  Serial.println("Boot system");
  

//Setup PWM output for heater element
  pinMode(pwm_heater_pin, OUTPUT);
  // Configure Timer 1 for 16-bit Fast PWM mode with a prescaler of 1024
  TCCR1A = _BV(COM1A1) | _BV(WGM11); // Fast PWM, non-inverted mode
  //TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12) | _BV(CS10); // Prescaler of 1024 -> 237 mHz
  //TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12); // Prescaler of 256 ->1Hz
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10); // Prescaler of 64 4Hz
  //TCCR1B = _BV(CS11); // Prescaler of 8 for PWM frequency control -> 2 kHz
  ICR1 = 65535; // Set the top value for 16-bit PWM (65535)

//pre-sets on start up
  temp_SOLL = temp_min; 
  OCR1A = 1; // Set PWM duty cycle
  
//all leds off
   pinMode(led_status, OUTPUT); digitalWrite(led_status, LOW); 
   pinMode(led_rot, OUTPUT);    digitalWrite(led_rot, LOW);
   pinMode(led_gelb, OUTPUT);   digitalWrite(led_gelb, LOW);
   pinMode(led_gruen, OUTPUT);  digitalWrite(led_gruen, LOW);
  
 // lcd display
   lcd.begin();lcd.backlight();           // initialize the LCD 
   

   lcd.setCursor(1,0); lcd.print("Solder station"); //position 1, line 0
   lcd.setCursor(0,1);   lcd.print("Start v1.0");
   
 //rotory encoder interrupt setup  
   PCICR |= (1 << PCIE2);  
   PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);  
   sei(); 

 // butten
   pinMode(button_rott_pin, INPUT_PULLUP); // butten of rotary encoder input
   //attachInterrupt(digitalPinToInterrupt(20), Interupt_button_rott, RISING); // Attach an interrupt to pin 2


 // temperature read input
   pinMode(temp_pin, INPUT);// pin to read the temperature
   pinMode(stand_by_pin, INPUT_PULLUP); // pin to read if the station is in use
  
  
  
//TESTTING
  Serial.println("pwm_heater_duty, Temp_SOLL, Temp_IST");
  temp_min = 1; 
  
  Serial.println("Start of program");
  delay(100);            // waits for a second
  lcd.clear();
}





// #######################################################################################################################
void loop() {
  //read temperature Value
  temp_IST = temp_measure();
  temp_SOLL = constrain(temp_SOLL, temp_min, temp_max); // limit the range of the set temperature

  if ((temp_SOLL + temp_delta) > temp_IST){
    pwm_heater_duty = pwm_heater_duty + 1000;
  }
  else if ( (temp_SOLL - temp_delta) < temp_IST){
    pwm_heater_duty = pwm_heater_duty - 1000;
  }
  else {
    if (temp_SOLL > temp_IST){
            pwm_heater_duty++;    }
    else {  pwm_heater_duty--;    } 
  }
  //pwm_heater_duty = map(temp_IST, temp_min, temp_max, 0, 65535)

 //check if pen is in the station ... stand by
  stand_by = !digitalRead(stand_by_pin);

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

// program ______ start _______________________________________________________________
  if (!POWERon) { // power off
       OCR1A = 100; //set duty scile to 0% ... switch off 4%
       lcd.setCursor(0,1);
       lcd.print("OFF");
  }
    else if (stand_by) { // stand by mode with reduces power
    OCR1A = map(temp_standby , temp_min, temp_max, 0, 65535);  //Set PWM duty cycle
    OCR1A = 1000;
    lcd.setCursor(0,1);
    lcd.print("by ");
  }
  else { // full power ON
    OCR1A = pwm_heater_duty;  // set PWM between 0 and 65535
    lcd.setCursor(0,1);
    lcd.print("ON ");
  }
  lcd.setCursor(0,0);
  lcd.print("PWM ");
  lcd.print(OCR1A );
  

// program ______ end _______________________________________________________________

  digitalWrite(led_rot, HIGH); delay(100);   digitalWrite(led_rot, LOW);
  digitalWrite(led_gelb, HIGH);  delay(100); digitalWrite(led_gelb, LOW); 
  digitalWrite(led_gruen, HIGH);  delay(100);  digitalWrite(led_gruen, LOW);
  
   lcd.setCursor(12,0);   lcd.print(temp_SOLL);  lcd.print(" "); // to overwrite the old values
   lcd.setCursor(12,1);   lcd.print(temp_IST);  lcd.print("   ");
 
 //status and debugging
  //Serial.print(pwm_heater_duty);  Serial.print(",");
  Serial.print(temp_IST);  Serial.print(",");  Serial.println(temp_SOLL);
  if (POWERon) { digitalWrite(led_status, HIGH); } else { digitalWrite(led_status, LOW); }
}


// end of main program
// #######################################################################################################################

// temperature input with rotery encoder
ISR(PCINT2_vect) {  
   unsigned char result = r.process();  
   if (result == DIR_NONE) {  
     // do nothing  
   }  
   else if (result == DIR_CW) {  //CLOCKWISE
     //temp_SOLL++;
     temp_SOLL = temp_SOLL + 5; 
     if (temp_SOLL > temp_max) { // maximum temperature limit
      temp_SOLL = temp_max;
      Serial.print("maximum temperature limit "); 
     }
     Serial.print(temp_SOLL);  
   }  
   
   else if (result == DIR_CCW) {  //COUNTERCLOCK
     //temp_SOLL--;
     temp_SOLL = temp_SOLL - 5;
     if (temp_SOLL < temp_min) { // minimum temperature limit
      temp_SOLL = temp_min;
      Serial.print("minimum temperature limit "); 
     }
     Serial.print(temp_SOLL);
   }  
 }  

// Measure ist temperature
int temp_measure() {
  OCR1A = 0; //set duty scile to 0% ... power off
  delay(10);
  //Temp = analogRead(temp_pin);  //read analog Value
    // Calculates °C from RTD voltage divider
  double Temp = log(10000.0 * ((1024.0 / analogRead(temp_pin) - 1)));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp )) * Temp );
  //return Temp - 273.15;

  return analogRead(temp_pin);  //read analog Value
}

// interupt for pressing on the rotory encoder to poer off and on
void Interupt_button_rott() {
  Serial.print("interupt");
  POWERon = !POWERon;
  }
