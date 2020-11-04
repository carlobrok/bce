#ifndef KAMELI2C_H
#define KAMELI2C_H

#include <cstdint>       	// int8_t, uint8_t, uint16_t, ...
#include <string>


int ebcI2Copen(int devId);

int init_arduino(int address);

namespace mot {

  // === Motoren ===========

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
    FORWARD,
    BACKWARD
  };

  // states returned by get_state.
  enum states_read {
    ON,             // read only
    OFF,
    OFF_BRAKE,
    OFF_NO_GROUND,  // read only
    OFF_OBSTACLE    // read only
  };

  int servo(int angle);

  int set_dir_pwm(uint8_t direction, uint8_t pwm);
  int set_dir_pwm_steer(uint8_t direction, uint8_t pwm, int angle);  

  int set_state(uint8_t state);
  int get_state();


}	// namespace mot

// === Sensoren ===========

namespace sen {

  enum commands {
    GET_ULTRASONIC,
    GET_INFRARED
  };

  enum us_sensors {
    LEFT,
    MID_LEFT,
    MID_RIGHT,
    RIGHT
  };

  const std::string us_names[4] = {
  	"left", "mid left",
  	"mid right", "right"
  };

  bool get_bit(uint8_t byte, uint8_t bit_number);

  int update_us_data();
  bool get_us(int sensor, bool update_data = false);

  bool get_ir();

}	// namespace sen

#endif
