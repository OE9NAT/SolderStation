#include <LiquidCrystal.h>

#define HEAT_PIN 5
#define TC_PIN A0
#define THERMISTOR_PIN A1
#define UP_BTN_PIN 6
#define DOWN_BTN_PIN 7
#define INTERRUPT_PIN 2
#define R5_RESISTANCE 100300.0
#define R6_RESISTANCE 240.3
#define VCC 5.1

int tipTempIs = 0;
int tipTempIsDisplay = 0;   // Separate Display variables
int tipTempSet = 50;
int tipTempSetDisplay = 0;  // for asynchronous updates
bool heat = false;
int mainsCycles = 0;
unsigned long buttonMillis = 0; // When last button press was registered

float getAmbientTemperature() {
  // Calculates °C from RTD voltage divider
  double Temp = log(10000.0 * ((1024.0 / analogRead(THERMISTOR_PIN) - 1)));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp )) * Temp );
  return Temp - 273.15;
}

float runningAverage(float M) {
  #define LENGTH 20
  static int values[LENGTH];
  static byte index = 0;
  static float sum = 0;
  static byte count = 0;
  sum -= values[index];
  values[index] = M;
  sum += values[index];
  index++;
  index = index % LENGTH;
  if (count < LENGTH) count++;
  return sum / count;
}

void zeroCrossingInterrupt() {
  mainsCycles++;
}

LiquidCrystal lcd(A2, A3, A4, A5, 0, 1 );

void setup()
{
  lcd.begin(16,2);
  lcd.print("Set:     ");
  lcd.print(char(223));
  lcd.print('C');
  lcd.setCursor(0,1);
  lcd.print("Is:      ");
  lcd.print(char(223));
  lcd.print('C');
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(UP_BTN_PIN, INPUT_PULLUP);
  pinMode(DOWN_BTN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), zeroCrossingInterrupt, CHANGE);
}

void loop()
{
  if (mainsCycles >= 0) // At 0 turn off heater
    digitalWrite(HEAT_PIN, LOW);

  if (mainsCycles > 6){ // Wait for 6 more mains cycles to get an undisturbed reading
    noInterrupts();
    tipTempIs = round(runningAverage(((analogRead(TC_PIN)*(VCC/1023.0))/(1+R5_RESISTANCE/R6_RESISTANCE))*43500+getAmbientTemperature()));
    if (tipTempIs < tipTempSet){ // If heat is missing
      digitalWrite(HEAT_PIN, HIGH);
      mainsCycles = sqrt(tipTempSet - tipTempIs)*-1; // Schedule next measurement
    }
    else // If no heat is missing
      mainsCycles = -4;
    interrupts();
  }
  
  if (abs(tipTempIs-tipTempIsDisplay) >= 1) // Is it time to update the display?
  {
    noInterrupts();
    tipTempIsDisplay = tipTempIs;
    lcd.setCursor(5,1);
    lcd.print("   ");
    lcd.setCursor(5,1);
    lcd.print(tipTempIsDisplay);
    interrupts();
  }

  if (abs(tipTempSet-tipTempSetDisplay) >= 1) // Is it time to update the display?
  {
    noInterrupts();
    tipTempSetDisplay = tipTempSet;
    lcd.setCursor(5,0);
    lcd.print("   ");
    lcd.setCursor(5,0);
    lcd.print(tipTempSetDisplay);
    interrupts();
  }

  if (!digitalRead(UP_BTN_PIN) && tipTempSet < 400 && millis() > buttonMillis+10){
    tipTempSet++;
    buttonMillis = millis();
  }
  if (!digitalRead(DOWN_BTN_PIN) && tipTempSet > 0 && millis() > buttonMillis+10){
    tipTempSet--;
    buttonMillis = millis();
  }
}
