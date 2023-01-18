

// Depends on the following Arduino libraries:
// - Arduino thermistor library: https://github.com/miguel5612/Arduino-ThermistorLibrary
// - U8g2lib Display library https://github.com/olikraus/u8glib

// At this example I use A thermistor, the 104nt-4 thermistor.
// you can configurate this thermistor in Configuration.h in Documents/arduino/libraries/Arduino-ThermistorLibrary/src/Configuration.h and select TEMP_SENSOR_1 value.

#include <Arduino.h>
#include <U8g2lib.h>
#include <thermistor.h>
#include <Wire.h>



//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1306_128X64_ALT0_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // same as the NONAME variant, but may solve the "every 2nd line skipped" problem
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/A5, /* data=*/A4);
thermistor therm1(A1, 0);  // Analog Pin which is connected to the 3950 temperature sensor, and 0 represents TEMP_SENSOR_0 (see configuration.h for more information).


#define PIN_SENSE 2             //where we connected the fan sense pin. Must be an interrupt capable pin (2 or 3 on Arduino Uno)
#define DEBOUNCE 0              //0 is fine for most fans, crappy fans may require 10 or 20 to filter out noise
#define FANSTUCK_THRESHOLD 500  //if no interrupts were received for 500ms, consider the fan as stuck and report 0 RPM
//Interrupt handler. Stores the timestamps of the last 2 interrupts and handles debouncing
unsigned long volatile ts1 = 0, ts2 = 0;


void setup() {
  // put your setup code here, to run once:

  u8g2.begin();
  Serial.begin(9600);  //initialize port serial at 9600 Bauds.

  pinMode(PIN_SENSE, INPUT_PULLUP);                                     //set the sense pin as input with pullup resistor
  attachInterrupt(digitalPinToInterrupt(PIN_SENSE), tachISR, FALLING);  //set tachISR to be triggered when the signal on the sense pin goes low

  //enable outputs for Timer 2
  pinMode(3, OUTPUT);  //2
  setupTimer2();
}

void loop() {

  double temp1 = therm1.analog2temp();  // read temperature



  if (temp1 < 20.0) {

    setPWM(0.0f);  //set duty to 0% on pin 3
  }

  else if ((temp1 > 20.0) && (temp1 < 22.0)) {
    setPWM(0.2f);  //set duty to 20% on pin 3
  }


  else if ((temp1 > 22.0) && (temp1 < 23.0)) {
    setPWM(0.3f);  //set duty to 40% on pin 3
  }

  else if ((temp1 > 23.0) && (temp1 < 25.0)) {
    setPWM(0.5f);  //set duty to 80% on pin 3
  }

  else if ((temp1 > 25.0) && (temp1 < 31.0)) {
    setPWM(0.7f);  //set duty to 80% on pin 3
  }


  else if ((temp1 > 31.0) && (temp1 < 33.0)) {
    setPWM(0.8f);  //set duty to 80% on pin 3
  }

  else if ((temp1 > 33.0) && (temp1 < 35.0)) {
    setPWM(0.9f);  //set duty to 80% on pin 3
  }

  else if (temp1 > 35.0) {
    setPWM(1.0f);  //set duty to 80% on pin 3
  }

  //Print temperature in port serial
  Serial.print("Temperature of thermistor : ");
  Serial.println(temp1);
  Serial.println("----------------------");
  Serial.println("RPM:");
  Serial.println(calcRPM());

  u8g2.firstPage();
  do {

    u8g2.setFont(u8g2_font_t0_13b_tf);
    u8g2.setCursor(9, 38);
    u8g2.print("UV Light Cooling");

    u8g2.setFont(u8g2_font_crox5tb_tf);
    u8g2.setCursor(9, 64);
    u8g2.print(temp1, 0);
    u8g2.setFont(u8g2_font_unifont_t_symbols);
    u8g2.drawGlyph(38, 64, 0x00b0);  // degree
    u8g2.drawStr(45, 64, "C");

    u8g2.setCursor(63, 64);
    u8g2.print("RPM:");
    u8g2.setCursor(95, 64);
    u8g2.print(calcRPM());


  } while (u8g2.nextPage());



  delay(2000);  //wait 1200 mS for next measure
}



//configure Timer 2 (pin 3) to output 25kHz PWM. Pin 11 will be unavailable for output in this mode
void setupTimer2() {
  //Set PWM frequency to about 25khz on pin 3 (timer 2 mode 5, prescale 8, count to 79)
  TIMSK2 = 0;
  TIFR2 = 0;
  TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << WGM22) | (1 << CS21);
  OCR2A = 79;
  OCR2B = 0;
}


//equivalent of analogWrite on pin 3
void setPWM(float f) {
  f = f < 0 ? 0 : f > 1 ? 1
                        : f;
  OCR2B = (uint8_t)(79 * f);
}


void tachISR() {
  unsigned long m = millis();
  if ((m - ts2) > DEBOUNCE) {
    ts1 = ts2;
    ts2 = m;
  }
}
//Calculates the RPM based on the timestamps of the last 2 interrupts. Can be called at any time.
unsigned long calcRPM() {
  if (millis() - ts2 < FANSTUCK_THRESHOLD && ts2 != 0) {
    return (60000 / (ts2 - ts1)) / 2;
  } else return 0;
}