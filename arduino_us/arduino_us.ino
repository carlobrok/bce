const int l_echo = 0;
const int l_trig = 0;
const int l_out = 0;

const int lm_echo = 0;
const int lm_trig = 0;
const int lm_out = 0;

const int rm_echo = 0;
const int rm_trig = 0;
const int rm_out = 0;

const int r_echo = 0;
const int r_trig = 0;
const int r_out = 0;

void setup() {
  pinMode(l_trig, OUTPUT);
  pinMode(lm_trig, OUTPUT);
  pinMode(rm_trig, OUTPUT);
  pinMode(r_trig, OUTPUT);

  pinMode(l_out, OUTPUT);
  pinMode(lm_out, OUTPUT);
  pinMode(rm_out, OUTPUT);
  pinMode(r_out, OUTPUT);

  Serial.begin(115200);
}

void loop() {

}
