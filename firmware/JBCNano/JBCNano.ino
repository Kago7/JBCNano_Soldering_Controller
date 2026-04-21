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
#include "PID_v1.h"

/* PINOUT DEFINES */
#define VMON                A0
#define TC                  A1
#define IMON                A2
#define SET_PWM_ACC         A3
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

#define TC_GAIN             226.67f
#define IMON_CONV_FACTOR    1.57f

#define TMON_OFFSET         0

#define HEATER_MIN_TEMP     25
#define HEATER_MAX_TEMP     450
#define HEATER_MIN_PWR      1
#define HEATER_MAX_PWR      100

#define TIP_STAND_TEMP      150

#define VDD_MINIMUM         9

#define TFT_SPI_SPEED       8000000

/* CARTRIDGE DEFINES */
#define KP_C115 					1
#define KI_C115 					0
#define KD_C115 					0
#define MAX_I_C115 				300
#define MAX_POWER_C115    22
#define RESISTANCE_C115   3.4f
#define TC_UV_C_C115      .00000692f

#define KP_C210 					1
#define KI_C210 					0
#define KD_C210 					0
#define MAX_I_C210 			  300
#define MAX_POWER_C210    65
#define RESISTANCE_C210   2.4f
#define TC_UV_C_C210      .00000938f

#define KP_C245 					1
#define KI_C245 					0
#define KD_C245 					0
#define MAX_I_C245 				300
#define MAX_POWER_C245    100
#define RESISTANCE_C245   2.4f
#define TC_UV_C_C245      .00002572f

/* Global fields */
volatile bool set_pwr_heater_en = 0;
volatile bool set_pwr_acc_en    = 0;

bool uvlo = 0;
bool serial_enable = 0;
char buf[10] = {0};

typedef enum { C115, C210, C245, NONE } eCartridgeT;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

double actual_temp, pid_output, set_temp;
PID pid(&actual_temp, &pid_output, &set_temp, KP_C245, KI_C245, KD_C245, DIRECT);

/* ISRs */

/**
 * @brief ISR to turn off power to the heater.
 * 
 */
void set_pwr_heater_en_isr() {
  set_pwr_heater_en = 0;
  digitalWrite(HEATER_HI, 0);
  TCCR1A &= ~(1 << COM1A1);
  if (serial_enable) {
    Serial.println("isr fired");
  }
  return;
}

/**
 * @brief ISR to turn off power to the accessory.
 * 
 */
void set_pwr_acc_en_isr() {
  set_pwr_acc_en = 0;
  analogWrite(HEATER_LO, 0);
  if (serial_enable) {
    Serial.println("isr fired");
  }
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
 * @brief Get the averaged temperature reading in degrees C from MCP9700T-E/TT
 * 
 * @return float 
 */
float get_tmon() {
  /* average 10 times */
  float tmon = 0;
  for (int i=0; i<10; i++) {
    tmon += ( ADC_REF_VOLTAGE*(analogRead(TMON)/(float)ADC_NUM_COUNTS) - 0.5) * 100;
  }
  return (tmon / 10.0) + TMON_OFFSET;
}

/**
 * @brief Get the current reading in Amps across the shunt resistor.
 * @note  Consider implementing oversampling instead.
 * @return float 
 */
float get_imon() {
  /* Average over default analogRead~100us * N */
  uint16_t avg = 0;
  for (int i=0; i<20; i++) {
    avg += analogRead(IMON);
  }
  /* Compute current in Amps */
  avg = avg / 20;
  return (ADC_REF_VOLTAGE * (avg/(float)ADC_NUM_COUNTS)) * IMON_CONV_FACTOR;
}

/**
 * @brief Get the iron tip thermocouple temperature in degrees C based on cartridge type. HEATER_HI turned OFF during this function for accuracy.
 * @note  Consider implementing oversampling instead.
 * @param handle 
 * @return int 
 */
int get_tc(eCartridgeT handle) {
  /* Force HEATER_HI output to 0 and disconnect timer, wait for TC to stabilize */
  uint16_t avg = 0;
  digitalWrite(HEATER_HI, 0);
  TCCR1A &= ~(1 << COM1A1);
  delay(2);
  /* Average over default analogRead~100us*N */
  for (int i=0; i<20; i++) {
    avg += analogRead(TC);
  }
  /* Restore HEATER_HI by reconnecting timer*/
  TCCR1A |= (1 << COM1A1);
  /* Compute temperature in degrees C + Cold Junction Compensation based on cartridge */
  avg = avg / 20;
  switch (handle) {
    case C115:
      return ((ADC_REF_VOLTAGE * (avg/(float)ADC_NUM_COUNTS)) / (TC_GAIN * TC_UV_C_C115) ) + get_tmon();
      break;
    case C210:
      return ((ADC_REF_VOLTAGE * (avg/(float)ADC_NUM_COUNTS)) / (TC_GAIN * TC_UV_C_C210) ) + get_tmon();
      break;
    case C245:
      return ((ADC_REF_VOLTAGE * (avg/(float)ADC_NUM_COUNTS)) / (TC_GAIN * TC_UV_C_C245) ) + get_tmon();
      break;
    default:
      return ((ADC_REF_VOLTAGE * (avg/(float)ADC_NUM_COUNTS)) / (TC_GAIN * TC_UV_C_C245) ) + get_tmon();
  }
}

/**
 * @brief Get the accessory pwm set by potentiometer mapped to 0%-100%
 * 
 * @return int 
 */
int get_pwm_acc() {
  return HEATER_MIN_PWR + HEATER_MAX_PWR - map(analogRead(SET_PWM_ACC), 0, ADC_NUM_COUNTS, HEATER_MIN_PWR, HEATER_MAX_PWR);
}

/**
 * @brief Get the heater temperature set by potentiometer mapped to 25C-450C, rounded to nearest multiple of 5.
 * 
 * @return int 
 */
int get_temp() {
  int temp = map(analogRead(SET_TEMP), 0, ADC_NUM_COUNTS, HEATER_MIN_TEMP, HEATER_MAX_TEMP);
  return HEATER_MAX_TEMP - ((temp + 2) / 5) * 5 + HEATER_MIN_TEMP;
}

/**
 * @brief Get the heater pwr set by potentiometer mapped to 0W-100W
 * 
 * @return int 
 */
int get_pwr_heater() {
  return HEATER_MIN_PWR + HEATER_MAX_PWR - map(analogRead(SET_PWR_HEATER), 0, ADC_NUM_COUNTS, HEATER_MIN_PWR, HEATER_MAX_PWR);
}

/**
 * @brief Handle type detection logic based on handle sense 1/2 inputs.
 *        Cannot distinguish if handle is actually installed or not due to input pullups for C245 case.
 * @param hs1
 * @param hs2
 * @return eCartridgeT 
 */
eCartridgeT detect_handle_type(bool hs1, bool hs2) {
  /* Return handle type */
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
 * @param freq_type 
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
  /* bias toward early color change */
  t = pow(t, 0.3);
  uint8_t r, g;
  if (t < 0.5) {
    /* green → yellow */ 
    float tt = t * 2.0;
    r = tt * 255;
    g = 255;
  } else {
    /* yellow → red */ 
    float tt = (t - 0.5) * 2.0;
    r = 255;
    g = (1.0 - tt) * 255;
  }
  return tft.color565(r, g, 0);
}

/**
 * @brief Return a set temp based on various tip states.
 * 
 * @param tip_change 
 * @param stand_sense
 */
int get_set_temp(bool tip_change, bool stand_sense) {
  /* Determine tip set temp based on stand inputs and potentiometer */
  int temp = (!digitalRead(SET_TEMP_EN)) ? get_temp() : 0;
  if (stand_sense && (temp > TIP_STAND_TEMP) ) temp = TIP_STAND_TEMP;
  if (tip_change) temp = HEATER_MIN_TEMP;
  /* Check if 30min. runtime exceeded */
  if ( (millis() > 1800000) ) {
    return HEATER_MIN_TEMP;
  }
  return temp;
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
void update_tft(int actual_temp, int set_temp, int set_pwr_heater, int set_pwm_acc, float vmon, float tmon, float imon, eCartridgeT handle) {
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
  static int last_set_temp       = -999;
  static int last_set_pwr_heater = -999;
  static int last_set_pwm_acc    = -999;
  if ( (last_set_temp != set_temp) || (last_set_pwr_heater != set_pwr_heater) || (last_set_pwm_acc != set_pwm_acc) ) {
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

    tft.print("Set Pwm Acc: ");
    tft.setTextSize(2);
    tft.print(set_pwm_acc);
    tft.println("%");
    tft.setTextSize(1);
  }

  last_set_temp = set_temp;
  last_set_pwr_heater = set_pwr_heater;
  last_set_pwm_acc = set_pwm_acc;

  
  
  /* update text color green - red based on power */
  int power = vmon*imon;
  tft.setCursor(0, 72);
  tft.setTextColor(value_to_color(power, HEATER_MIN_PWR, HEATER_MAX_PWR), ST77XX_BLACK);
  tft.print("Total Power: ");
  tft.setTextSize(2);
  tft.print(power);
  tft.println("W");
  tft.setTextSize(1);
  tft.println();
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);


  static int last_handle, last_vmon, last_tmon;
  if ( (last_handle != handle) || (last_vmon != (int)vmon) || (last_tmon != (int)tmon) ) {
    tft.setTextSize(1);
    tft.print("Handle:  ");
    tft.setTextSize(2);
    switch(handle) {
      case C115:
        tft.print("115");
        break;
      case C210:
        tft.print("210");
        break;
      case C245:
        tft.print("245");
        break;
      default:
        tft.print("---");
    }
    tft.println();

    tft.setTextSize(1);
    tft.print("VMON:    ");
    tft.setTextSize(2);
    tft.print(vmon);
    tft.println("V");

    tft.setTextSize(1);
    tft.print("Ambient: ");
    tft.setTextSize(2);
    tft.print((int)tmon);
    tft.println("C");

  }
  last_handle = handle;
  last_vmon   = (int)vmon;
  last_tmon   = (int)tmon;

  tft.setCursor(0, 144);
  tft.setTextSize(1);
  tft.print("Current: ");
  tft.setTextSize(2);
  tft.print(imon);
  tft.println("A");
}

/**
 * @brief Initial setup for JBCNano
 * 
 */
void setup() {
  /* Initialize peripherals, Serial only if UVLO */
  analogReference(DEFAULT);
  // if (get_vmon() < VDD_MINIMUM) {
  //   Serial.begin(SERIAL_BAUD);
  //   serial_enable = 1;
  // } else {
  //   serial_enable = 0;
  // }

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
  attachInterrupt(digitalPinToInterrupt(SET_PWR_HEATER_EN), set_pwr_heater_en_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(SET_PWR_ACC_EN)   , set_pwr_acc_en_isr   , RISING);

  /* Initialize 20KHz PWM on Timer1 for Heater Gate Driver */
  TCCR1A  = 0;                           /* Stop Timer1 */
  TCCR1B  = 0;                           /* Stop Timer1 */
  TCCR1A |= (1 << COM1A1);               /* Fast-PWM with ICR1 as TOP (Mode 14) */
  TCCR1A |= (1 << WGM11);                /* Non-inverting mode on OC1A */
  TCCR1B |= (1 << WGM12) | (1 << WGM13); /* Non-inverting mode on OC1A */
  TCCR1B |= (1 << CS10);                 /* Prescaler = 1 */
  ICR1    = (F_CPU/HEATER_FREQ) - 1;     /* Set output frequency */
  OCR1A   = 1;                           /* Start with ~0% Duty Cycle */
  TCCR1A &= ~(1 << COM1A1);              /* Initially disconnected */

  /* Initialize TFT */
  tft.setSPISpeed(TFT_SPI_SPEED);
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(2);

  /* Initialize PID Controller */
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(5);
  pid.SetOutputLimits(0, 1);
  
  /* Allow for system settling time delay */
  // beep(0);
  delay(200);
  // beep(1);

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
    /* Disable pwm outputs to protect mosfets/gate driver */
    analogWrite(HEATER_LO, 0);
    digitalWrite(HEATER_HI, 0);
    TCCR1A &= ~(1 << COM1A1);
    uvlo = 1;
  } else {
    uvlo = 0;
  }

  /* Overtemperature lockout for tip and tmon */
  // if (get_tmon() > 45 || get_tc() > 475) {
  //   /* Disable pwm outputs to mosfets and play buzzer */
  //   noInterrupts();
  //   digitalWrite(HEATER_LO, 0);
  //   digitalWrite(HEATER_HI, 0);
  //   TCCR1A &= ~(1 << COM1A1);
  //   tone(BUZZER, BUZZER_FREQ_LO);
  //   while(1);
  // }

  /* Loop variables */
  set_pwr_heater_en      = !digitalRead(SET_PWR_HEATER_EN);
  set_pwr_acc_en         = !digitalRead(SET_PWR_ACC_EN);
  bool  hs1              = !digitalRead(HANDLE_SENSE_1_IN);
  bool  hs2              = !digitalRead(HANDLE_SENSE_2_IN);
  eCartridgeT handle     = detect_handle_type(hs1, hs2);
              handle     = C210;
  bool  stand_sense      = !digitalRead(STAND_SENSE_IN);
  bool  tip_change       = !digitalRead(TIP_CHANGE_SENSE_IN);
        actual_temp      = get_tc(handle);
        set_temp         = get_set_temp(tip_change, stand_sense);
  int   set_pwr_heater   = (set_pwr_heater_en) ? get_pwr_heater() : 0;
  int   set_pwm_acc      = (set_pwr_acc_en) ? get_pwm_acc() : 0;
  float vmon             = get_vmon();
  float tmon             = get_tmon();
  float imon             = get_imon();
  int heater_power_limit = set_pwr_heater;
  double heater_hi_duty_limit = 0;


  /* Configure PID constants and minimum power limits based on handle */
  switch(handle) {
    case C115:
      pid.SetTunings(KP_C115, KI_C115, KD_C115);
      heater_power_limit = min(heater_power_limit, MAX_POWER_C115);
      heater_hi_duty_limit = heater_power_limit/(float)(pow(vmon, 2.0)/RESISTANCE_C115);
      break;
    case C210:
      pid.SetTunings(KP_C210, KI_C210, KD_C210);
      heater_power_limit = min(heater_power_limit, MAX_POWER_C210);
      heater_hi_duty_limit = heater_power_limit/(float)(pow(vmon, 2.0)/RESISTANCE_C210);
      break;
    case C245:
      pid.SetTunings(KP_C245, KI_C245, KD_C245);
      heater_power_limit = min(heater_power_limit, MAX_POWER_C245);
      heater_hi_duty_limit = heater_power_limit/(float)(pow(vmon, 2.0)/RESISTANCE_C245);
      break;
    default:
      pid.SetTunings(KP_C245, KI_C245, KD_C245);
      heater_power_limit = min(heater_power_limit, MAX_POWER_C245);
      heater_hi_duty_limit = 0;
  }

  /* Run the PID loop, and apply pwm if no uvlo */
  if (uvlo == 0) {
    /* Set HEATER_LO pwm% duty cycle */
    analogWrite(HEATER_LO, 255*set_pwm_acc/HEATER_MAX_PWR);
    /* Set HEATER_HI pwm% duty cycle */
    if (set_pwr_heater > 0) {
      /* Apply PID output limits based on max */
      pid.SetOutputLimits(0, heater_hi_duty_limit);
      /* PID to determine what duty cycle to apply to HEATER_HI */
      pid.Compute();
      /* Enable Timer to output duty cycle */
      TCCR1A |= (1 << COM1A1);
      /* Apply PID and Limit HEATER_HI duty cycle based on handle power limits; (ICR1 - 1) to limit to max 99% due to bootstrap gate driver */
      OCR1A = (ICR1 - 1)*pid_output;
    } else {
      digitalWrite(HEATER_HI, 0);
      TCCR1A &= ~(1 << COM1A1);
    }
  } else {
    /* Disable pwm outputs to protect mosfet gate driver */
    analogWrite(HEATER_LO, 0);
    digitalWrite(HEATER_HI, 0);
    TCCR1A &= ~(1 << COM1A1);
  }

  /* Update the TFT display */
  update_tft((int)actual_temp, (int)set_temp, set_pwr_heater, set_pwm_acc, vmon, tmon, imon, handle);

  /* DEBUG */
  if (serial_enable) {
    /* Print dynamic variables to serial monitor/plotter */
    Serial.print("Actual Temp:");
    Serial.print((int)actual_temp);
    Serial.print(",");
    Serial.print("Set Temp:");
    Serial.println((int)set_temp);
    Serial.print("Stand Sense:");
    Serial.println((int)stand_sense);
    Serial.print("Tip Change:");
    Serial.println((int)tip_change);
    Serial.print("HS1:");
    Serial.println((int)hs1);
    Serial.print("HS2:");
    Serial.println((int)hs2);

    Serial.print("Set PWR:");
    Serial.println((int)set_pwr_heater);

    Serial.print("Set PWM:");
    Serial.println((int)set_pwm_acc);

    Serial.print("Ambient Temp:");
    Serial.println(tmon);

    Serial.print("VMON:");
    Serial.println(vmon);

    Serial.println();
  }
}
