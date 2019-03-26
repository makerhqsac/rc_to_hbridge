
/* pin configuration */
#define RC_THROTTLE_INPUT       2
#define RC_STEERING_INPUT       3
#define RC_AUX_A_INPUT         A0
#define RC_AUX_B_INPUT         A1

#define AUX_OUT_A              A4
#define AUX_OUT_B              A5

#define MOT_L_EN_OUTPUT        10
#define MOT_L_FWD_OUTPUT        8
#define MOT_L_REV_OUTPUT        6
#define MOT_R_EN_OUTPUT         9
#define MOT_R_FWD_OUTPUT        7
#define MOT_R_REV_OUTPUT        5

#define LED_RC_STATUS_OUTPUT   13

void setup() {

  pinMode(RC_THROTTLE_INPUT, INPUT);
  pinMode(RC_STEERING_INPUT, INPUT);
  pinMode(RC_AUX_A_INPUT, INPUT);
  pinMode(RC_AUX_B_INPUT, INPUT);
  pinMode(AUX_OUT_A, INPUT);
  pinMode(AUX_OUT_B, INPUT);
  pinMode(MOT_L_EN_OUTPUT, INPUT);
  pinMode(MOT_L_FWD_OUTPUT, INPUT);
  pinMode(MOT_L_REV_OUTPUT, INPUT);
  pinMode(MOT_R_EN_OUTPUT, INPUT);
  pinMode(MOT_R_FWD_OUTPUT, INPUT);
  pinMode(MOT_R_REV_OUTPUT, INPUT);

  pinMode(LED_RC_STATUS_OUTPUT, OUTPUT);
  pinMode(1, INPUT);
  pinMode(0, INPUT);
}

void loop() {

  if (digitalRead(RC_THROTTLE_INPUT) == HIGH &&
      digitalRead(RC_STEERING_INPUT) == HIGH &&
      digitalRead(RC_AUX_A_INPUT) == HIGH &&
      digitalRead(RC_AUX_B_INPUT) == HIGH &&
      digitalRead(AUX_OUT_A) == HIGH &&
      digitalRead(AUX_OUT_B) == HIGH &&
      digitalRead(MOT_L_EN_OUTPUT) == HIGH &&
      digitalRead(MOT_L_FWD_OUTPUT) == HIGH &&
      digitalRead(MOT_L_REV_OUTPUT) == HIGH &&
      digitalRead(MOT_R_EN_OUTPUT) == HIGH &&
      digitalRead(MOT_R_FWD_OUTPUT) == HIGH &&
      digitalRead(MOT_R_REV_OUTPUT) == HIGH &&
      digitalRead(1) == HIGH &&
      digitalRead(0) == HIGH)
  {
    digitalWrite(LED_RC_STATUS_OUTPUT, HIGH);
  } else {
    digitalWrite(LED_RC_STATUS_OUTPUT, LOW);
  }
}

