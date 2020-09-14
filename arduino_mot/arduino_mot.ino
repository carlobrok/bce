#include "Wire.h"
#include "Servo.h"

//#define SERIAL_OUTPUT
const int SLAVE_ADDRESS = 0x08;
const int SERVO_PIN = 9;
const int NORMAL_SPEED = 50;

// WHAT TO DO   (DO NOT USE! Only used by functions)
enum commands {
  SET_DIR_PWM,
  SET_DIR_PWM_STEER,
  SET_STEER,
  SET_STATE,
  GET_STATE
};

// direction send by
enum direction {
  MOTOR_FORWARD,
  MOTOR_BACKWARD
};

// states returned by get_state.
enum states_read {
  ON,             // read only
  OFF,
  OFF_BRAKE,
  OFF_NO_GROUND,  // read only
  OFF_OBSTACLE    // read only
};

// TODO: define pins
const int _pin_d = 0;
const int _pin_in1 = 0;
const int _pin_in2 = 0;

Servo steering_servo(SERVO_PIN);

int direction = MOTOR_FORWARD, speed = 0, state = OFF;
double steer_angle = 0;




void fwd(int speed = NORMAL_SPEED);
void bwd(int speed = NORMAL_SPEED);
void off(bool brake = false);
void update_mot(int dir, int pwm);

void steer(double angle);


template<class T>
void print(T message) {
#ifdef SERIAL_OUTPUT
  Serial.print(message);
#endif
}

template<class T>
void println(T message) {
#ifdef SERIAL_OUTPUT
  Serial.println(message);
#endif
}

inline void recieveError() {
  println("WARNING: Recieved wrong/corrupted data");
}

// TODO: debug the check_length function
template <size_t N>
void check_length(int (&inbytes)[N], size_t length_needed) {
  if(needed != N) recieveError();
}


void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  pinMode(LED_BUILTIN, OUTPUT);

#ifdef SERIAL_OUTPUT
  Serial.begin(115200);
#endif
  println("Motoren / Servo Arduino");
  print("Adresse: "); Serial.println(SLAVE_ADDRESS);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(3000 - 100);
}

void receiveEvent(int byte_amount) {
  // TODO: debug receiving
  print("Anzahl: ");
  println(byte_amount);


  if (byte_amount == 2) {
    recieveError();
    return;
  }

  digitalWrite(LED_BUILTIN, HIGH);

  int command = Wire.read();
  int puffer_b = Wire.read();

  print("Comand: ");
  print(command);
  print("\tBuffer: ");
  print(puffer_b);

  int inbytes[byte_amount - 2];

  int index = 0;
  print("  data: [");
  while (Wire.available()/* && index < byte_amount - 2*/) {
    inbytes[index++] = Wire.read();
    print(inbytes[index - 1]);
    if(Wire.available() > 0)
      print(";");
  }
  println("]");

  switch (command) {
    case SET_DIR_PWM:
      println("SET_DIR_PWM");
      check_length(inbytes, 2);
      int dir = inbytes[0];
      int pwm = inbytes[1];

      update_mot(dir, pwm);

      break;

    case SET_DIR_PWM_STEER:
      println("SET_DIR_PWM_STEER");
      check_length(inbytes, 3);
      int dir = inbytes[0];
      int pwm = inbytes[1];
      int angle = inbytes[2];

      update_mot(dir, pwm);
      steer(angle);

      break;

    case SET_STEER:
      // TODO: calculate angle from x bytes received by i2c

      println("SET_STEER");
      check_length(inbytes, 1);
      int angle = inbytes[0];

      steer(angle);

      break;

    case SET_STATE:
      println("SET_STEER");
      check_length(inbytes, 1);
      int state = inbytes[0];

      if (state == OFF) {
        off();
      } else if (state == OFF_BRAKE) {
        off(true);
      }

      break;

    default:
      recieveError();
      break;
  }

  digitalWrite(LED_BUILTIN, LOW);
}

void requestEvent() {
  // TODO: debug state sending
  Wire.write(state);
}


// Motor functions
// TODO: PID controller

void fwd(int speed) {
  digitalWrite(_pin_in1, LOW);
  digitalWrite(_pin_in2, HIGH);
  analogWrite(_pin_d, speed);
}

void bwd(int speed) {
  digitalWrite(_pin_in1, HIGH);
  digitalWrite(_pin_in2, LOW);
  analogWrite(_pin_d, speed);
}

void off(bool brake = false) {
  digitalWrite(_pin_in1, LOW);
  digitalWrite(_pin_in2, LOW);
  analogWrite(_pin_d, 0);
}

void update_mot(int dir, int pwm) {
  if (dir == MOTOR_FORWARD) {
    fwd(pwm);
  } else if (dir == MOTOR_BACKWARD) {
    bwd(pwm);
  } else {
    println("Error: Unknown direction");
  }
}

// Steering function

void steer(double angle) {
  // TODO: calculate pwm from angle
  int servo_angle = angle;


  steering_servo.set(servo_angle);
}
