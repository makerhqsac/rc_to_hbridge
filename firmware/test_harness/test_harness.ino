
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

  pinMode(RC_THROTTLE_INPUT, OUTPUT);
  pinMode(RC_STEERING_INPUT, OUTPUT);
  pinMode(RC_AUX_A_INPUT, OUTPUT);
  pinMode(RC_AUX_B_INPUT, OUTPUT);
  pinMode(AUX_OUT_A, OUTPUT);
  pinMode(AUX_OUT_B, OUTPUT);
  pinMode(MOT_L_EN_OUTPUT, OUTPUT);
  pinMode(MOT_L_FWD_OUTPUT, OUTPUT);
  pinMode(MOT_L_REV_OUTPUT, OUTPUT);
  pinMode(MOT_R_EN_OUTPUT, OUTPUT);
  pinMode(MOT_R_FWD_OUTPUT, OUTPUT);
  pinMode(MOT_R_REV_OUTPUT, OUTPUT);

  pinMode(LED_RC_STATUS_OUTPUT, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(0, OUTPUT);


  digitalWrite(RC_THROTTLE_INPUT, HIGH);
  digitalWrite(RC_STEERING_INPUT, HIGH);
  digitalWrite(RC_AUX_A_INPUT, HIGH);
  digitalWrite(RC_AUX_B_INPUT, HIGH);
  digitalWrite(AUX_OUT_A, HIGH);
  digitalWrite(AUX_OUT_B, HIGH);
  digitalWrite(MOT_L_EN_OUTPUT, HIGH);
  digitalWrite(MOT_L_FWD_OUTPUT, HIGH);
  digitalWrite(MOT_L_REV_OUTPUT, HIGH);
  digitalWrite(MOT_R_EN_OUTPUT, HIGH);
  digitalWrite(MOT_R_FWD_OUTPUT, HIGH);
  digitalWrite(MOT_R_REV_OUTPUT, HIGH);

  digitalWrite(LED_RC_STATUS_OUTPUT, HIGH);
  digitalWrite(1, HIGH);
  digitalWrite(0, HIGH);

}

void loop() {

}

