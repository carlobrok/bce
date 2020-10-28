enum pins_left {
  L_ECHO,
  L_TRIG,
  L_OUT
};

enum pins_left_mid {
  LM_ECHO,
  LM_TRIG,
  LM_OUT
};

enum pins_right_mid {
  RM_ECHO,
  RM_TRIG,
  RM_OUT
};

enum pins_right {
  R_ECHO,
  R_TRIG,
  R_OUT
};

float dist_l, dist_lm, dist_rm, dist_r;
String str_out = "";

const int buzzer_pin = 5;
unsigned long last_buzzer = 0;

float get_distance(int trigger_pin, int echo_pin, int max_cm = 100);

void setup() {
  pinMode(L_TRIG, OUTPUT);
  pinMode(LM_TRIG, OUTPUT);
  pinMode(RM_TRIG, OUTPUT);
  pinMode(R_TRIG, OUTPUT);

  pinMode(L_OUT, OUTPUT);
  pinMode(LM_OUT, OUTPUT);
  pinMode(RM_OUT, OUTPUT);
  pinMode(R_OUT, OUTPUT);

  Serial.begin(115200);
}

void loop() {
// read distances of ultrasonic sensors
  dist_l = get_distance(L_TRIG, L_ECHO);
  dist_lm = get_distance(LM_TRIG, LM_ECHO);
  dist_rm = get_distance(RM_TRIG, RM_ECHO);
  dist_r = get_distance(R_TRIG, R_ECHO);

// write distances to serial port
  str_out = String(dist_l,1) + ";" + String(dist_lm,1) + ";" + String(dist_rm,1) + ";" + String(dist_r,1);
  Serial.println(str_out);

// beep when low distance
  if((dist_l < 50 && dist_l != 0) || (dist_lm < 50 && dist_lm != 0) || (dist_rm < 50 && dist_rm != 0) || (dist_r < 50 && dist_r != 0)) {
    if(millis() - last_buzzer > 200) {
      last_buzzer = millis();
    }
    if(millis() - last_buzzer < 100) {
      tone(buzzer_pin, 590);
    } else {
      noTone(buzzer_pin);
    }
  } else {
    noTone(buzzer_pin);
  }
}

// gibt die gemessene distanz des jeweiligen Ultraschall sensors in cm zurück.
// @param trigger_pin: digitaler Triggerpin des Arduinos

float get_distance(int trigger_pin, int echo_pin, int max_cm) {
  digitalWrite(trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);
  unsigned long duration = pulseIn(echo_pin, HIGH, (max_cm*2/0.034));

  return duration*0.034/2;
}
