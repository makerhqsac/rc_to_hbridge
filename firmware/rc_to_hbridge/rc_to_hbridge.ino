#include <EnableInterrupt.h>
#include <EEPROM.h>

//#define DEBUG

/* pin configuration */
#define RC_THROTTLE_INPUT       2
#define RC_STEERING_INPUT       3
#define RC_AUX_A_INPUT         A0
#define RC_AUX_B_INPUT         A1

#define AUX_OUT_A              A5
#define AUX_OUT_B              A6

#define MOT_L_EN_OUTPUT        10
#define MOT_L_FWD_OUTPUT        8
#define MOT_L_REV_OUTPUT        6
#define MOT_R_EN_OUTPUT         9
#define MOT_R_FWD_OUTPUT        7
#define MOT_R_REV_OUTPUT        5

#define LED_RC_STATUS_OUTPUT   13

/* rc configuration */
#define MAX_SPEED             400 // max motor speed
#define RC_PULSE_CENTER      1500 // pulse width (microseconds) for RC center
#define PULSE_WIDTH_DEADBAND   25 // pulse width difference from RC_PULSE_CENTER us (microseconds) to ignore (to compensate for control centering offset)
#define PULSE_WIDTH_RANGE     400 // pulse width difference from RC_PULSE_CENTER us to be treated as full scale input
#define RC_PULSE_TIMEOUT      500 // in milliseconds

enum state_t {
  STATE_IDLE,
  STATE_FWD,
  STATE_REV,
  STATE_BREAK,
  STATE_NO_RC,
  STATE_CONFIG
};

#define RC_THROTTLE             0
#define RC_STEERING             1
#define RC_AUX_A                2
#define RC_AUX_B                3

#define RC_NUM_CHANNELS         4

#define PWM_MODE_4PIN_NOBREAK   1
#define PWM_MODE_4PIN_BREAK     2
#define PWM_MODE_6PIN_NOBREAK   3
#define PWM_MODE_6PIN_BREAK     4


#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__)
  #define USE_20KHZ_PWM
#endif

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

long last_rc_pulse = 0;
state_t state = STATE_IDLE;
int throttle = 0;
int steering = 0;
int aux_a, aux_b;
int left_speed = 0;
int right_speed = 0;

#define CONFIG_VERSION "R1X"
#define CONFIG_START 48

// runtime configurable items
struct ConfigStruct {
  char version[4]; // do not change
  // The variables of your settings
  int rc_center;
  int pwm_mode;
} config = {
  CONFIG_VERSION,
  // The default values
  RC_PULSE_CENTER, PWM_MODE_4PIN_NOBREAK
};


void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();

  throttle = (int)rc_values[RC_THROTTLE];
  steering = (int)rc_values[RC_STEERING];
  aux_a = (int)rc_values[RC_AUX_A];
  aux_b = (int)rc_values[RC_AUX_B];

  if (millis() - last_rc_pulse > RC_PULSE_TIMEOUT) {
    throttle = 0;
    steering = 0;
    aux_a = config.rc_center; // aux outputs will keep current state if value is centered
    aux_b = config.rc_center;
  }
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
  last_rc_pulse = millis();
}

void calc_throttle() { calc_input(RC_THROTTLE, RC_THROTTLE_INPUT); }
void calc_steering() { calc_input(RC_STEERING, RC_STEERING_INPUT); }
void calc_aux_a() { calc_input(RC_AUX_A, RC_AUX_A_INPUT); }
void calc_aux_b() { calc_input(RC_AUX_B, RC_AUX_B_INPUT); }


void load_config() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
    for (unsigned int t=0; t<sizeof(config); t++)
      *((char*)&config + t) = EEPROM.read(CONFIG_START + t);
}

void save_config() {
  for (unsigned int t=0; t<sizeof(config); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&config + t));
}

#ifdef DEBUG
void reset_debug(int blinks) {
  while (true) {
    blink_status(blinks);
    delay(5000);
  }
}

ISR(BADISR_vect) {
  reset_debug(10);
}
#endif

void setup() {
  unsigned char reset_src = MCUSR;   // save reset source
  MCUSR = 0x00;  // cleared for next reset detection

  pinMode(LED_RC_STATUS_OUTPUT, OUTPUT);

#ifdef DEBUG
  if(reset_src & (1<<PORF )) {
    blink_status(2);
  } else {
    if(reset_src & (1<<EXTRF)) reset_debug(6);
    if(reset_src & (1<<BORF )) reset_debug(7);
    if(reset_src & (1<<WDRF )) reset_debug(8);
  }
#endif

  pinMode(RC_THROTTLE_INPUT, INPUT);
  pinMode(RC_STEERING_INPUT, INPUT);
  pinMode(RC_AUX_A_INPUT, INPUT);
  pinMode(RC_AUX_B_INPUT, INPUT);

  load_config(); load_config();

  aux_a = config.rc_center;
  aux_b = config.rc_center;

  pinMode(MOT_L_EN_OUTPUT, OUTPUT);
  pinMode(MOT_L_FWD_OUTPUT, OUTPUT);
  pinMode(MOT_L_REV_OUTPUT, OUTPUT);
  pinMode(MOT_R_EN_OUTPUT, OUTPUT);
  pinMode(MOT_R_FWD_OUTPUT, OUTPUT);
  pinMode(MOT_R_REV_OUTPUT, OUTPUT);

  enableInterrupt(RC_THROTTLE_INPUT, calc_throttle, CHANGE);
  enableInterrupt(RC_STEERING_INPUT, calc_steering, CHANGE);
  enableInterrupt(RC_AUX_A_INPUT, calc_aux_a, CHANGE);
  enableInterrupt(RC_AUX_B_INPUT, calc_aux_b, CHANGE);

  #ifdef USE_20KHZ_PWM
      // Timer 1 configuration
      // prescaler: clockI/O / 1
      // outputs enabled
      // phase-correct PWM
      // top of 400
      //
      // PWM frequency calculation
      // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
      TCCR1A = 0b10100000;
      TCCR1B = 0b00010001;
      ICR1 = 400;
  #endif

}

void loop() {
  rc_read_values();
  calculate_speeds();
  set_aux();

  if (throttle <= 0 && steering <= 0) {
    state = STATE_NO_RC;
  }

  switch(state) {
    case STATE_IDLE:
      run_state_idle();
      break;
    case STATE_FWD:
      run_state_fwd();
      break;
    case STATE_REV:
      run_state_rev();
      break;
    case STATE_BREAK:
      run_state_break();
      break;
    case STATE_CONFIG:
      run_state_config();
      break;
    case STATE_NO_RC:
      run_state_no_rc();
      break;
  }
}

boolean is_config_mode() {
  boolean is_config = false;

  pinMode(MOT_L_REV_OUTPUT, INPUT);
  digitalWrite(MOT_L_REV_OUTPUT, HIGH); // enable internal pullup

  digitalWrite(MOT_R_REV_OUTPUT, LOW);
  if (digitalRead(MOT_L_REV_OUTPUT) == LOW) {
    is_config = true;
  }
  pinMode(MOT_L_REV_OUTPUT, OUTPUT);

  return is_config;
}

void run_state_idle() {
  set_speeds(0, 0);

  digitalWrite(LED_RC_STATUS_OUTPUT, HIGH);

  if (is_config_mode()) {
    blink_status(5);
    delay(4000);
    state = STATE_CONFIG;
  } else if (throttle > config.rc_center + PULSE_WIDTH_DEADBAND) {
    state = STATE_FWD;
  } else if (throttle < config.rc_center - PULSE_WIDTH_DEADBAND) {
    state = STATE_REV;
  }
}

void run_state_fwd() {
  set_speeds(left_speed, right_speed);

  // state transitions
  if (throttle < config.rc_center - PULSE_WIDTH_DEADBAND) {
    if (config.pwm_mode == PWM_MODE_4PIN_BREAK || config.pwm_mode == PWM_MODE_6PIN_BREAK) {
      state = STATE_BREAK;
    } else {
      state = STATE_REV;
    }
  }
}

void run_state_rev() {
  set_speeds(left_speed, right_speed);

  // state transitions
  if (throttle > config.rc_center + PULSE_WIDTH_DEADBAND) {
    state = STATE_FWD;
  }
}

void run_state_break() {
  apply_break();

  // state transitions
  if (throttle > config.rc_center) {
    state = STATE_REV;
  }
}

void run_state_config() {

  blink_status(config.pwm_mode);

  delay(4000);

  rc_read_values();

  if (steering > config.rc_center + 300) {
    int new_mode = config.pwm_mode + 1;
    if (new_mode > PWM_MODE_6PIN_BREAK) {
      new_mode = PWM_MODE_4PIN_NOBREAK;
    }
    config.pwm_mode = new_mode;
    save_config(); save_config();

  } else if (steering < config.rc_center - 300) {
    int new_mode = config.pwm_mode - 1;
    if (new_mode < PWM_MODE_4PIN_NOBREAK) {
      new_mode = PWM_MODE_6PIN_BREAK;
    }
    config.pwm_mode = new_mode;
    save_config(); save_config();
  }
}

void run_state_no_rc() {
  set_speeds(0, 0);

  if (throttle > 0 && steering > 0) {
    state = STATE_IDLE;
  }
}

void blink_status(int count) {
  digitalWrite(LED_RC_STATUS_OUTPUT, LOW);
  for (int i = 0; i < count; i++) {
    delay(500);
    digitalWrite(LED_RC_STATUS_OUTPUT, HIGH);
    delay(500);
    digitalWrite(LED_RC_STATUS_OUTPUT, LOW);
  }
}

void calculate_speeds() {

  int calc_throttle = throttle;
  int calc_steering = steering;

  if (calc_throttle > 0 && calc_steering > 0) {

    // RC signals encode information in pulse width centered on config.rc_center us (microseconds); subtract config.rc_center to get a value centered on 0
    calc_throttle -= config.rc_center;
    calc_steering -= config.rc_center;

    // apply deadband
    if (abs(calc_throttle) <= PULSE_WIDTH_DEADBAND)
      calc_throttle = 0;
    if (abs(calc_steering) <= PULSE_WIDTH_DEADBAND)
      calc_steering = 0;

    // mix throttle and steering inputs to obtain left & right motor speeds
    left_speed = ((long)calc_throttle * MAX_SPEED / PULSE_WIDTH_RANGE) - ((long)calc_steering * MAX_SPEED / PULSE_WIDTH_RANGE);
    right_speed = ((long)calc_throttle * MAX_SPEED / PULSE_WIDTH_RANGE) + ((long)calc_steering * MAX_SPEED / PULSE_WIDTH_RANGE);

    // cap speeds to max
    left_speed = min(max(left_speed, -MAX_SPEED), MAX_SPEED);
    right_speed = min(max(right_speed, -MAX_SPEED), MAX_SPEED);

  } else {
    left_speed = 0;
    right_speed = 0;
  }

}

void set_left_speed(int speed) {
  if (config.pwm_mode == PWM_MODE_4PIN_NOBREAK || config.pwm_mode == PWM_MODE_4PIN_BREAK) {
    set_left_speed_4pin(speed);
  } else {
    set_left_speed_6pin(speed);
  }
}

void set_left_speed_6pin(int speed) {
  boolean reverse = 0;

  if (speed < 0) {
    speed = -speed;
    reverse = 1;
  }

  if (speed > MAX_SPEED)
    speed = MAX_SPEED;

#ifdef USE_20KHZ_PWM
    OCR1B = speed;
#else
    analogWrite(MOT_L_EN_OUTPUT, map(speed, 0, MAX_SPEED, 0, 255));
#endif

  if (reverse) {
    digitalWrite(MOT_L_FWD_OUTPUT, LOW);
    digitalWrite(MOT_L_REV_OUTPUT, HIGH);
  } else {
    digitalWrite(MOT_L_FWD_OUTPUT, HIGH);
    digitalWrite(MOT_L_REV_OUTPUT, LOW);
  }
}

void set_left_speed_4pin(int speed) {
  boolean reverse = 0;

  if (speed < 0) {
    speed = -speed;
    reverse = 1;
  }

  if (speed > MAX_SPEED)
    speed = MAX_SPEED;

  if (reverse) {
    digitalWrite(MOT_L_FWD_OUTPUT, LOW);
    digitalWrite(MOT_L_REV_OUTPUT, HIGH);
#ifdef USE_20KHZ_PWM
    OCR1B = speed;
#else
    analogWrite(MOT_L_EN_OUTPUT, map(speed, 0, MAX_SPEED, 0, 255));
#endif

  } else {
    digitalWrite(MOT_L_FWD_OUTPUT, HIGH);
    digitalWrite(MOT_L_REV_OUTPUT, LOW);
#ifdef USE_20KHZ_PWM
    OCR1B = MAX_SPEED - speed;
#else
    analogWrite(MOT_L_EN_OUTPUT, map(speed, 0, MAX_SPEED, 255, 0));
#endif
  }
}

void set_right_speed(int speed) {
  if (config.pwm_mode == PWM_MODE_4PIN_NOBREAK || config.pwm_mode == PWM_MODE_4PIN_BREAK) {
    set_right_speed_4pin(speed);
  } else {
    set_right_speed_6pin(speed);
  }
}

void set_right_speed_6pin(int speed) {
  boolean reverse = 0;

  if (speed < 0) {
    speed = -speed;
    reverse = 1;
  }

  if (speed > MAX_SPEED)
    speed = MAX_SPEED;

#ifdef USE_20KHZ_PWM
    OCR1A = speed;
#else
    analogWrite(MOT_R_EN_OUTPUT, map(speed, 0, MAX_SPEED, 0, 255));
#endif

  if (reverse) {
    digitalWrite(MOT_R_FWD_OUTPUT, LOW);
    digitalWrite(MOT_R_REV_OUTPUT, HIGH);
  } else {
    digitalWrite(MOT_R_FWD_OUTPUT, HIGH);
    digitalWrite(MOT_R_REV_OUTPUT, LOW);
  }
}

void set_right_speed_4pin(int speed) {
  boolean reverse = 0;

  if (speed < 0) {
    speed = -speed;
    reverse = 1;
  }

  if (speed > MAX_SPEED)
    speed = MAX_SPEED;

  if (reverse) {
    digitalWrite(MOT_R_FWD_OUTPUT, LOW);
    digitalWrite(MOT_R_REV_OUTPUT, HIGH);
#ifdef USE_20KHZ_PWM
    OCR1A = speed;
#else
    analogWrite(MOT_R_EN_OUTPUT, map(speed, 0, MAX_SPEED, 0, 255));
#endif

  } else {
    digitalWrite(MOT_R_FWD_OUTPUT, HIGH);
    digitalWrite(MOT_R_REV_OUTPUT, LOW);
#ifdef USE_20KHZ_PWM
    OCR1A = MAX_SPEED - speed;
#else
    analogWrite(MOT_R_EN_OUTPUT, map(speed, 0, MAX_SPEED, 255, 0));
#endif
  }
}

void set_speeds(int left_speed, int right_speed) {
  set_left_speed(left_speed);
  set_right_speed(right_speed);
}

void set_aux() {
  if (aux_a > config.rc_center + PULSE_WIDTH_DEADBAND) {
    digitalWrite(AUX_OUT_A, HIGH);
  } else if (aux_a < config.rc_center - PULSE_WIDTH_DEADBAND) {
    digitalWrite(AUX_OUT_A, LOW);
  }
  if (aux_b > config.rc_center + PULSE_WIDTH_DEADBAND) {
    digitalWrite(AUX_OUT_B, HIGH);
  } else if (aux_b < config.rc_center - PULSE_WIDTH_DEADBAND) {
    digitalWrite(AUX_OUT_B, LOW);
  }
}

void apply_break() {

  if (config.pwm_mode == PWM_MODE_4PIN_BREAK) {
    digitalWrite(MOT_L_EN_OUTPUT, HIGH);
    digitalWrite(MOT_L_FWD_OUTPUT, HIGH);
    digitalWrite(MOT_L_REV_OUTPUT, HIGH);
    digitalWrite(MOT_R_EN_OUTPUT, HIGH);
    digitalWrite(MOT_R_FWD_OUTPUT, HIGH);
    digitalWrite(MOT_R_REV_OUTPUT, HIGH);

  } else if (config.pwm_mode == PWM_MODE_6PIN_BREAK) {
    digitalWrite(MOT_L_EN_OUTPUT, LOW);
    digitalWrite(MOT_L_FWD_OUTPUT, HIGH);
    digitalWrite(MOT_L_REV_OUTPUT, HIGH);
    digitalWrite(MOT_R_EN_OUTPUT, LOW);
    digitalWrite(MOT_R_FWD_OUTPUT, HIGH);
    digitalWrite(MOT_R_REV_OUTPUT, HIGH);
  }

}

