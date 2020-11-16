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

// Motor variables

const int motor_pin = 6;
const int max_speed = 35;

int mot_dir = MOTOR_FORWARD, mot_speed = 0, mot_state = OFF;

// Steering servro variables

const int STEER_PWM_LEFT = 52;
const int STEER_PWM_RIGHT = 115;

Servo steering_servo;
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
  steer((STEER_PWM_LEFT + STEER_PWM_RIGHT) / 2);

  // init led
  println("init pinouts..");
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize pins for motor driver
  pinMode(motor_pin, OUTPUT);

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
  fwd(255);
  delay(2000);
  off();
  digitalWrite(LED_BUILTIN, LOW);
  delay(3000 - 2000);

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
  Wire.write(mot_state);
}


// Motor functions
// TODO: PID controller
// TODO: inverse directions
void fwd(int speed) {
  if(speed > max_speed) speed = max_speed;  // dont drive too fast -> crash
  analogWrite(motor_pin, speed);
  mot_state = ON;
  mot_dir = MOTOR_FORWARD;
}

void bwd(int speed) {
  if(speed > max_speed) speed = max_speed;  // dont drive too fast -> crash
  analogWrite(motor_pin, speed);
  mot_state = ON;
  mot_dir = MOTOR_BACKWARD;
}

void off(bool brake) {
  analogWrite(motor_pin, 0);
  mot_speed = 0;
  mot_state = OFF;
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

void steer(int angle) {
  // TODO: calculate pwm from angle
  angle -= 100;
  print("angle: ");
  println(angle);
  int servo_pwm = 0.851 * angle + 83.5;
  print("pwm: ");
  println(servo_pwm);
  if(servo_pwm < STEER_PWM_LEFT) {
    servo_pwm = STEER_PWM_LEFT;
  }
  else if(servo_pwm > STEER_PWM_RIGHT) {
    servo_pwm = STEER_PWM_RIGHT;
  }
  print("mapped pwm: ");
  println(servo_pwm);
  steering_servo.write(servo_pwm);
}
