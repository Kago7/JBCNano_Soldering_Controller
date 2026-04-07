/**
 * @file JBCNano.ino
 * @author Kago
 * @brief   Main source file for JBCNano firmware.
 * @version 1.0
 * @date 2026-04-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

/* INCLUDES */
#include <math.h>
#include <stdio.h>
#include <float.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

/* PINOUT DEFINES */
#define VMON                A0
#define TC                  A1
#define IMON                A2
#define SET_PWR_ACC         A3
#define TMON                A4
#define TFT_RST             A5
#define SET_TEMP            A6
#define SET_PWR_HEATER      A7

#define HANDLE_SENSE_2_IN   0
#define HANDLE_SENSE_1_IN   1
#define SET_PWR_HEATER_EN   2
#define SET_PWR_ACC_EN      3
#define SET_TEMP_EN         4
#define BUZZER              5
#define HEATER_LO           6
#define STAND_SENSE_IN      7
#define TIP_CHANGE_SENSE_IN 8
#define HEATER_HI           9
#define TFT_CS              10
#define MOSI                11
#define TFT_DC              12
#define SCK                 13

#define ADC_REF_VOLTAGE     5
#define ADC_NUM_COUNTS      1023
#define ADC_NUM_BITS        10

#define BUZZER_FREQ         2000
#define HEATER_FREQ         20000

#define SERIAL_BAUD         115200


/* Global fields */
volatile int set_pwr_heater = 0;
volatile int set_pwr_acc    = 0;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);


/* ISRs */

/**
 * @brief ISR to turn off power to the heater.
 * 
 */
void set_pwr_heater_en_isr() {
  set_pwr_heater = 0;
  digitalWrite(HEATER_HI, 0);
  return;
}

/**
 * @brief ISR to turn off power to the accessory.
 * 
 */
void set_pwr_acc_en_isr() {
  set_pwr_acc = 0;
  digitalWrite(HEATER_LO, 0);
  return;
}


/* Functions */

/**
 * @brief Get the voltage of the VDD bus.
 * 
 * @return float 
 */
float get_vmon() {
  return ADC_REF_VOLTAGE * (analogRead(VMON)/(float)ADC_NUM_COUNTS) * 6.1;
}

/**
 * @brief Get the iron tip thermocouple temperature in degrees C
 * 
 * @return int 
 */
int get_tc() {
  return 0;
}

/**
 * @brief Get the accessory pwr set by potentiometer mapped to 0W-100W
 * 
 * @return int 
 */
int get_pwr_acc() {
  return map(analogRead(SET_PWR_ACC), 0, ADC_NUM_COUNTS, 0, 100);
}

/**
 * @brief Get the temperature reading in degrees C from MCP9700T-E/TT
 * 
 * @return float 
 */
float get_tmon() {
  return ( ADC_REF_VOLTAGE*(analogRead(TMON)/(float)ADC_NUM_COUNTS) - 0.5) / 0.01;
}

/**
 * @brief Get the heater temperature set by potentiometer mapped to 25C-450C, rounded to nearest multiple of 5.
 * 
 * @return int 
 */
int get_temp() {
  int temp = map(analogRead(SET_TEMP), 0, ADC_NUM_COUNTS, 25, 450);
  return ((temp + 2) / 5) * 5;
}

/**
 * @brief Get the heater pwr set by potentiometer mapped to 0W-100W
 * 
 * @return int 
 */
int get_pwr_heater() {
  return map(analogRead(SET_PWR_HEATER), 0, ADC_NUM_COUNTS, 0, 100);
}

/**
 * @brief Simply beep the buzzer briefly for audio feedback.
 * 
 */
void beep() {
  tone(BUZZER, BUZZER_FREQ, 150);
}

void setup() {
  /* Initialize peripherals */
  Serial.begin(SERIAL_BAUD);
  analogReference(DEFAULT);

  /* Initialize inputs */
  pinMode(HANDLE_SENSE_1_IN  , INPUT_PULLUP);
  pinMode(HANDLE_SENSE_2_IN  , INPUT_PULLUP);
  pinMode(STAND_SENSE_IN     , INPUT_PULLUP);
  pinMode(TIP_CHANGE_SENSE_IN, INPUT_PULLUP);
  pinMode(SET_PWR_HEATER_EN  , INPUT_PULLUP);
  pinMode(SET_PWR_ACC_EN     , INPUT_PULLUP);
  pinMode(SET_TEMP_EN        , INPUT_PULLUP);

  /* Initialize outputs */
  pinMode(HEATER_LO, OUTPUT);
  digitalWrite(HEATER_LO, 0);
  pinMode(HEATER_HI, OUTPUT);
  digitalWrite(HEATER_HI, 0);

  /* Initialize interrupts */
  noInterrupts();
  attachInterrupt(digitalPinToInterrupt(SET_PWR_HEATER_EN), set_pwr_heater_en_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(SET_PWR_ACC_EN)   , set_pwr_acc_en_isr   , FALLING);

  /* Initialize 20KHz PWM on Timer1 for Heater Gate Driver */
  TCCR1A  = 0;
  TCCR1B  = 0;
  TCCR1A |= (1 << WGM11);                /* Mode 14 part 1 */
  TCCR1B |= (1 << WGM13) | (1 << WGM12); /* Mode 14 part 2 */
  TCCR1B |= (1 << CS11);                 /* Prescaler = 8 */
  ICR1    = ((F_CPU/8)/HEATER_FREQ);     /* Set output frequency */

  /* Initialize TFT */
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.print("JBC NANO");
  
  /* Allow for system settling time */
  delay(1000);

  /* Enable ISR after setup*/
  interrupts();
}

void loop() {


  /* Overtemperature lockout for tip and tmon (requires reset to escape) */
  if (get_tmon() > 40 || get_tc() > 475) {
    /* Disable pwm outputs to mosfets and beep buzzer */
    digitalWrite(HEATER_LO, 0);
    digitalWrite(HEATER_HI, 0);
    tone(BUZZER, BUZZER_FREQ);
    while(1);
  }

}
