#include <EnableInterrupt.h>

#define RC_NUM_CHANNELS  2

#define RC_THROTTLE  0
#define RC_STEERING  1

#define RC_THROTTLE_INPUT  2
#define RC_STEERING_INPUT  3

#define MOT_L_EN_OUTPUT    10
#define MOT_L_FWD_OUTPUT   8
#define MOT_L_REV_OUTPUT   6
#define MOT_R_EN_OUTPUT    9
#define MOT_R_FWD_OUTPUT   7
#define MOT_R_REV_OUTPUT   5

#define LED_RC_STATUS_OUTPUT   13


#define MAX_SPEED             400 // max motor speed
#define PULSE_WIDTH_DEADBAND   25 // pulse width difference from 1500 us (microseconds) to ignore (to compensate for control centering offset)
#define PULSE_WIDTH_RANGE     500 // pulse width difference from 1500 us to be treated as full scale input (for example, a value of 350 means
                                  //   any pulse width <= 1150 us or >= 1850 us is considered full scale)

#define RC_PULSE_TIMEOUT     1000


#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__)
  #define USE_20KHZ_PWM
#endif

#define STATE_IDLE      0
#define STATE_FWD       1
#define STATE_REV       2
#define STATE_BREAK     3

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

long last_rc_pulse = 0;
int state = STATE_IDLE;
int throttle = 0;
int steering = 0;
int left_speed = 0;
int right_speed = 0;

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();

  throttle = (int)rc_values[RC_THROTTLE];
  steering = (int)rc_values[RC_STEERING];

  if (millis() - last_rc_pulse > RC_PULSE_TIMEOUT) {
    throttle = 0;
    steering = 0;
  }

  if (throttle > 0 && steering > 0) {
    digitalWrite(LED_RC_STATUS_OUTPUT, HIGH);
  } else {
    digitalWrite(LED_RC_STATUS_OUTPUT, LOW);
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

void setup() {
  pinMode(RC_THROTTLE_INPUT, INPUT);
  pinMode(RC_STEERING_INPUT, INPUT);

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

  pinMode(MOT_L_EN_OUTPUT, OUTPUT);
  pinMode(MOT_L_FWD_OUTPUT, OUTPUT);
  pinMode(MOT_L_REV_OUTPUT, OUTPUT);
  pinMode(MOT_R_EN_OUTPUT, OUTPUT);
  pinMode(MOT_R_FWD_OUTPUT, OUTPUT);
  pinMode(MOT_R_REV_OUTPUT, OUTPUT);

  pinMode(LED_RC_STATUS_OUTPUT, OUTPUT);

  enableInterrupt(RC_THROTTLE_INPUT, calc_throttle, CHANGE);
  enableInterrupt(RC_STEERING_INPUT, calc_steering, CHANGE);
}

void loop() {
  rc_read_values();
  calculate_speeds();

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
  }
}

void run_state_idle() {
  set_speeds(0, 0);

  if (throttle > 1500 + PULSE_WIDTH_DEADBAND) {
    state = STATE_FWD;
  } else if (throttle < 1500 - PULSE_WIDTH_DEADBAND) {
    state = STATE_REV;
  }
}

void run_state_fwd() {
  set_speeds(left_speed, right_speed);

  // state transitions
  if (throttle < 1500 - PULSE_WIDTH_DEADBAND) {
    state = STATE_BREAK;
  }
}

void run_state_rev() {
  set_speeds(left_speed, right_speed);

  // state transitions
  if (throttle > 1500 + PULSE_WIDTH_DEADBAND) {
    state = STATE_FWD;
  }
}

void run_state_break() {
  apply_break();

  // state transitions
  if (throttle > 1500 - PULSE_WIDTH_DEADBAND) {
    state = STATE_REV;
  }
}

void calculate_speeds() {

  int calc_throttle = throttle;
  int calc_steering = steering;

  if (calc_throttle > 0 && calc_steering > 0) {

    // RC signals encode information in pulse width centered on 1500 us (microseconds); subtract 1500 to get a value centered on 0
    calc_throttle -= 1500;
    calc_steering -= 1500;

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

void set_right_speed(int speed) {
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

void set_speeds(int left_speed, int right_speed) {
  set_left_speed(left_speed);
  set_right_speed(right_speed);
}

void apply_break() {
  digitalWrite(MOT_L_FWD_OUTPUT, HIGH);
  digitalWrite(MOT_L_REV_OUTPUT, HIGH);
  digitalWrite(MOT_R_FWD_OUTPUT, HIGH);
  digitalWrite(MOT_R_REV_OUTPUT, HIGH);

  int calc_throttle = throttle;
  calc_throttle -= 1500; // center == 0
  if (abs(calc_throttle) <= PULSE_WIDTH_DEADBAND)
      calc_throttle = 0;

  int speed  = calc_throttle * MAX_SPEED / PULSE_WIDTH_RANGE;

  if (speed < 0) {
    speed = -speed;
    if (speed > MAX_SPEED)
      speed = MAX_SPEED;
  } else {
    speed = 0;
  }

#ifdef USE_20KHZ_PWM
  OCR1B = speed;
  OCR1A = speed;
#else
  int pulse = map(speed, 0, MAX_SPEED, 0, 255);
  analogWrite(MOT_L_EN_OUTPUT, pulse);
  analogWrite(MOT_R_EN_OUTPUT, pulse);
#endif

}
