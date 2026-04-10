/**
 * @file JBCNano.ino
 * @author Kago
 * @brief   Main source file for JBCNano firmware.
 * @version 1.0
 * @date 2026-04-07
 * 
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
#define TFT_DC              A5
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
#define TFT_RST             12
#define SCK                 13

/* CONFIG DEFINES */
#define ADC_REF_VOLTAGE     5
#define ADC_NUM_COUNTS      1023
#define ADC_NUM_BITS        10

#define BUZZER_FREQ_LO      2000
#define BUZZER_FREQ_HI      3000
#define HEATER_FREQ         20000

#define SERIAL_BAUD         115200

#define TC_CONV_FACTOR      100
#define IMON_CONV_FACTOR    1.57f

#define HEATER_MIN_TEMP     25
#define HEATER_MAX_TEMP     450
#define HEATER_MIN_PWR      0
#define HEATER_MAX_PWR      100

#define VDD_MINIMUM         10

#define TFT_SPI_SPEED       8000000

/* CARTRIDGE DEFINES */
#define KP_C115 					5
#define KI_C115 					2
#define KD_C115 					0.3
#define MAX_I_C115 				300
#define MAX_POWER_C115    25
#define RESISTANCE_C115   3.4f

#define KP_C210 					7
#define KI_C210 					4
#define KD_C210 					0.3
#define MAX_I_C210 			  300
#define MAX_POWER_C210    65
#define RESISTANCE_C210   3.4f

#define KP_C245 					8
#define KI_C245 					2
#define KD_C245 					0.5
#define MAX_I_C245 				300
#define MAX_POWER_C245    100
#define RESISTANCE_C245   3.4f


#define DEBUG


/* Global fields */
volatile bool set_pwr_heater_en = 0;
volatile bool set_pwr_acc_en    = 0;

bool uvlo = 0;

typedef enum { C115, C210, C245, NONE } eCartridgeT;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

/* ISRs */

/**
 * @brief ISR to turn off power to the heater.
 * 
 */
void set_pwr_heater_en_isr() {
  set_pwr_heater_en = 0;
  OCR1A = 0;
  return;
}

/**
 * @brief ISR to turn off power to the accessory.
 * 
 */
void set_pwr_acc_en_isr() {
  set_pwr_acc_en = 0;
  analogWrite(HEATER_LO, 0);
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
 * @brief Get the temperature reading in degrees C from MCP9700T-E/TT
 * 
 * @return float 
 */
float get_tmon() {
  return ( ADC_REF_VOLTAGE*(analogRead(TMON)/(float)ADC_NUM_COUNTS) - 0.5) * 100;
}

/**
 * @brief Get the current reading in Amps across the shunt resistor.
 * @note  Consider implementing oversampling instead.
 * @return float 
 */
float get_imon() {
  /* Average over 1ms, default analogRead~100us */
  uint16_t avg = 0;
  for (int i=0; i<10; i++) {
    avg += analogRead(IMON);
  }
  /* Compute current in Amps */
  avg = avg / 10;
  return (ADC_REF_VOLTAGE * (avg/(float)ADC_NUM_COUNTS)) * IMON_CONV_FACTOR;
}

/**
 * @brief Get the iron tip thermocouple temperature in degrees C. HEATER_HI turned OFF during this function for accuracy.
 * @note  Consider implementing oversampling instead.
 * @return int 
 */
int get_tc() {
  /* Disable and save HEATER_HI duty cycle, wait 0.5ms for TC to stabilize */
  uint16_t avg = 0;
  uint16_t duty = OCR1A;
  OCR1A = 0;
  delayMicroseconds(500);
  /* Average over 1ms, default analogRead~100us */
  for (int i=0; i<10; i++) {
    avg += analogRead(TC);
  }
  /* Restore HEATER_HI duty cycle */
  OCR1A = duty;
  /* Compute temperature in degrees C + Cold Junction Compensation */
  avg = avg / 10;
  return ((ADC_REF_VOLTAGE * (avg/(float)ADC_NUM_COUNTS)) * TC_CONV_FACTOR) + get_tmon();
}

/**
 * @brief Get the accessory pwr set by potentiometer mapped to 0W-100W
 * 
 * @return int 
 */
int get_pwr_acc() {
  return map(analogRead(SET_PWR_ACC), 0, ADC_NUM_COUNTS, HEATER_MIN_PWR, HEATER_MAX_PWR);
}

/**
 * @brief Get the heater temperature set by potentiometer mapped to 25C-450C, rounded to nearest multiple of 5.
 * 
 * @return int 
 */
int get_temp() {
  int temp = map(analogRead(SET_TEMP), 0, ADC_NUM_COUNTS, HEATER_MIN_TEMP, HEATER_MAX_TEMP);
  return ((temp + 2) / 5) * 5;
}

/**
 * @brief Get the heater pwr set by potentiometer mapped to 0W-100W
 * 
 * @return int 
 */
int get_pwr_heater() {
  return map(analogRead(SET_PWR_HEATER), 0, ADC_NUM_COUNTS, HEATER_MIN_PWR, HEATER_MAX_PWR);
}

/**
 * @brief Handle type detection logic based on handle sense 1/2 inputs.
 *        Cannot distinguish if handle is actually installed or not due to input pullups for C245 case.
 * 
 * @return eCartridgeT 
 */
eCartridgeT detect_handle_type() {
  /* Read handle sense inputs (active low) and return handle type */
  bool hs1 = !digitalRead(HANDLE_SENSE_1_IN);
  bool hs2 = !digitalRead(HANDLE_SENSE_2_IN);

  if (hs1 & hs2) {
    return NONE;
  } else if (!hs1 & !hs2) {
    return C245;
  } else if (hs1) {
    return C210;
  } else if (hs2) {
    return C115;
  }
}

/**
 * @brief Simply beep the buzzer briefly in 2 possible tones for audio feedback.
 * 
 */
void beep(bool freq_type) {
  if (freq_type) {
    tone(BUZZER, BUZZER_FREQ_HI, 150);
  } else {
    tone(BUZZER, BUZZER_FREQ_LO, 150);
  }
}

/**
 * @brief Used for value color gradients in TFT display from green (min) to red (max)
 * 
 * @param val 
 * @param min 
 * @param max 
 * @return uint16_t 
 */
uint16_t value_to_color(int val, int min, int max) {
  /* Convert value to a rgb color */
  if (val < min) val = min;
  if (val > max) val = max;
  float t = (float)(val - min)/(max - min);
  t = pow(t, 0.5);
  uint8_t r = t * 255;
  uint8_t g = (1.0 - t) * 255;
  return tft.color565(r, g, 0);
}

/**
 * @brief Update the TFT display with specific sensor data
 * 
 * @param set_temp 
 * @param actual_temp 
 * @param set_pwr_heater 
 * @param set_pwr_acc 
 * @param vmon 
 * @param tmon 
 * @param imon 
 */
void update_tft(int actual_temp, int set_temp, int set_pwr_heater, int set_pwr_acc, float vmon, float tmon, float imon) {
  /* Update display with specific sensor data */
  tft.setTextSize(1);
  tft.setCursor(0, 0);

  /* update text color green - red based on temp */
  tft.setTextColor(value_to_color(actual_temp, HEATER_MIN_TEMP, HEATER_MAX_TEMP), ST77XX_BLACK);
  tft.print("Tip Temp: ");
  tft.setTextSize(2);
  tft.print(actual_temp);
  tft.println("C");
  tft.setTextSize(1);

  /* update text color green - red based on temp, conditional update on change for set values */
  static int last_set_temp, last_set_pwr_heater, last_set_pwr_acc;
  if ( (last_set_temp != set_temp) || (last_set_pwr_heater != set_pwr_heater) || (last_set_pwr_acc != set_pwr_acc) ) {
    tft.setTextColor(value_to_color(set_temp, HEATER_MIN_TEMP, HEATER_MAX_TEMP), ST77XX_BLACK);
    tft.print("Set Temp: ");
    tft.setTextSize(2);
    tft.print(set_temp);
    tft.println("C");
    tft.setTextSize(1);
    tft.println();

    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.print("Set Pwr Tip: ");
    tft.setTextSize(2);
    tft.print(set_pwr_heater);
    tft.println("W");
    tft.setTextSize(1);

    tft.print("Set Pwr Acc: ");
    tft.setTextSize(2);
    tft.print(set_pwr_acc);
    tft.println("W");
    tft.setTextSize(1);

    Serial.println(tft.getCursorX());
    Serial.println(tft.getCursorY());
  }

  last_set_temp = set_temp;
  last_set_pwr_heater = set_pwr_heater;
  last_set_pwr_acc = set_pwr_acc;

  
  
  /* update text color green - red based on power */
  int power = vmon*imon;
  tft.setCursor(0, 72);
  tft.setTextColor(value_to_color(power, HEATER_MIN_PWR, HEATER_MAX_PWR), ST77XX_BLACK);
  tft.print("Total Power: ");
  tft.setTextSize(2);
  tft.print(power);
  tft.println("W");
  tft.println();
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);




  // tft.print("VMON: ");
  // tft.print(vmon);
  // tft.println(" V");

  // tft.print("Ambient: ");
  // tft.print(tmon);
  // tft.println(" C");

  // tft.print("Current: ");
  // tft.print(imon);
  // tft.println(" A");
}

/**
 * @brief Initial setup for JBCNano
 * 
 */
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
  tft.setSPISpeed(TFT_SPI_SPEED);
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(2);
  
  /* Allow for system settling time delay */
  beep(0);
  delay(200);
  beep(1);

  /* Enable ISR after setup*/
  interrupts();
}

/**
 * @brief Main control loop for JBCNano
 * 
 */
void loop() {
  /* UVLO - Under Voltage Lockout */
  if (get_vmon() < VDD_MINIMUM) {
    /* Disable pwm outputs to protect mosfet gate driver */
    analogWrite(HEATER_LO, 0);
    OCR1A = 0;
    uvlo = 1;
  } else {
    uvlo = 0;
  }

  /* Overtemperature lockout for tip and tmon */
  // if (get_tmon() > 40 || get_tc() > 475) {
  //   /* Disable pwm outputs to mosfets and play buzzer */
  //   digitalWrite(HEATER_LO, 0);
  //   digitalWrite(HEATER_HI, 0);
  //   tone(BUZZER, BUZZER_FREQ_LO);
  //   while(1);
  // }

  /* Loop variables */
  set_pwr_heater_en    = !digitalRead(SET_PWR_HEATER_EN);
  set_pwr_acc_en       = !digitalRead(SET_PWR_ACC_EN);
  int   actual_temp    = get_tc();
  int   set_temp       = (!digitalRead(SET_TEMP_EN)) ? get_temp() : 25;
  int   set_pwr_heater = (set_pwr_heater_en) ? get_pwr_heater() : 0;
  int   set_pwr_acc    = (set_pwr_acc_en) ? get_pwr_acc() : 0;
  float vmon           = get_vmon();
  float tmon           = get_tmon();
  float imon           = get_imon();


  /* DEBUG */
  #ifdef DEBUG
    Serial.println("loop running");
  #endif



  /* Update the TFT display */
  update_tft(actual_temp, set_temp, set_pwr_heater, set_pwr_acc, vmon, tmon, imon);
}
