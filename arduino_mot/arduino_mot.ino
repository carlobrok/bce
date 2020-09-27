#include "Wire.h"
#include "Servo.h"

#define SERIAL_OUTPUT
const int SLAVE_ADDRESS = 0x08;
const int SERVO_PIN = 3;
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

// REVIEW: define pins
const int d2_m1 = 5;
const int d2_m2 = 6;
const int in1_m1 = 7;
const int in2_m1 = 8;
const int in1_m2 = 9;
const int in2_m2 = 10;

Servo steering_servo;

int direction = MOTOR_FORWARD, speed = 0, state = OFF;
double steer_angle = 0;



// === Function declaration ====

void fwd(int speed = NORMAL_SPEED);
void bwd(int speed = NORMAL_SPEED);
void off(bool brake = false);
void update_mot(int dir, int pwm);

void steer(float angle);


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
void check_length(int array_size, int length_needed) {
  if(length_needed != array_size) recieveError();
}

// end: function declaration ====


void setup() {
#ifdef SERIAL_OUTPUT
  Serial.begin(115200);
#endif
  println("motor / steering Arduino");
  println("init i2c com..");
  // Setup i2c communication
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  print("address: "); println(SLAVE_ADDRESS);

  // initialize Servo
  println("init servo..");
  steering_servo.attach(SERVO_PIN);
  steer(0);

  // init led
  println("init pinouts..");
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize pins for motor driver
  pinMode(d2_m1, OUTPUT);
  pinMode(d2_m2, OUTPUT);
  pinMode(in1_m1, OUTPUT);
  pinMode(in2_m1, OUTPUT);
  pinMode(in1_m1, OUTPUT);
  pinMode(in2_m1, OUTPUT);

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  println("done.");
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  fwd();
  digitalWrite(LED_BUILTIN, LOW);
  delay(3000 - 100);
  off();
}

// TODO: debug receiving
void receiveEvent(int byte_amount) {

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
    inbytes[index] = Wire.read();
    print(inbytes[index]);
    if(Wire.available() > 0)
      print(";");
    index++;
  }
  println("]");

  switch (command) {
    case SET_DIR_PWM:
      println("SET_DIR_PWM");
      check_length(index, 2);

      update_mot(inbytes[0], inbytes[1]);

      break;

    case SET_DIR_PWM_STEER:
      println("SET_DIR_PWM_STEER");
      check_length(index, 3);

      update_mot(inbytes[0], inbytes[1]);
      steer(inbytes[2]);

      break;

    case SET_STEER:
      // TODO: calculate angle from x bytes received by i2c

      println("SET_STEER");
      check_length(index, 1);

      steer(inbytes[0]);

      break;

    case SET_STATE:
      println("SET_STEER");
      check_length(index, 1);

      if (inbytes[0] == OFF) {
        off();
      } else if (inbytes[0] == OFF_BRAKE) {
        off(true);
      }

      break;

    case GET_STATE:
      println("getstate requested.");

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
  digitalWrite(in1_m1, LOW);
  digitalWrite(in1_m2, LOW);
  digitalWrite(in2_m1, HIGH);
  digitalWrite(in2_m2, HIGH);
  analogWrite(d2_m1, speed);
  analogWrite(d2_m2, speed);
}

void bwd(int speed) {
  digitalWrite(in1_m1, HIGH);
  digitalWrite(in1_m2, HIGH);
  digitalWrite(in2_m1, LOW);
  digitalWrite(in2_m2, LOW);
  analogWrite(d2_m1, speed);
  analogWrite(d2_m2, speed);
}

void off(bool brake) {
  digitalWrite(in1_m1, LOW);
  digitalWrite(in1_m2, LOW);
  digitalWrite(in2_m1, LOW);
  digitalWrite(in2_m2, LOW);
  analogWrite(d2_m1, 0);
  analogWrite(d2_m2, 0);

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

void steer(float angle) {
  // TODO: calculate pwm from angle
  int servo_angle = angle;


  steering_servo.write(servo_angle);
}
