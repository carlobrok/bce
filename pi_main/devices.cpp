#include "devices.hpp"

extern "C" {
    #include <linux/i2c.h>
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}
#include <sys/ioctl.h>				// ioctl
#include <fcntl.h>						// fcntl, O_RDWR
#include <cmath>						// fabs(), fmod()
#include <cstdint>						// int8_t, uint8_t, uint16_t, ...
#include <cerrno>							// errno
#include <cstring>						// strerror
#include <string>							// string
#include <array>
#include <cctype>							// isdigit
#include <mutex>							// mutex
#include <chrono>							// timing
#include <iostream>           // cout


// I2C define
const int I2C_MOTOR_REFRESH_TIME = 100;     // Zeit in ms nach der die Daten über I2C beim aufrufen der Funktion zwingend aktualisiert werden


// returns a file descriptor of the opened I2C device

int ebcI2Copen(int devId) {
	const char *device;
	int fd;

	//device = "/dev/i2c-0";	// Older Raspberry Pi models
	device = "/dev/i2c-1";	// Raspberry Pi 3B+ / 4B

	if((fd = open(device, O_RDWR)) < 0)	{			// open "/dev/i2c-1"
		std::cout << "Unable to open " << device << ": " << strerror(errno) << std::endl;
		return -1;
	}

	if (ioctl (fd, I2C_SLAVE, devId) < 0) {			// set device address of fd to devId
		std::cout << "Unable to open device " << devId << ": " << strerror(errno) << std::endl;
		return -1;
	}

	return fd;
}


int arduino_fd = 0;

int init_arduino(int address) {
	arduino_fd = ebcI2Copen(address);
	return (arduino_fd == -1 ? -1 : 0);
}


// ================== Motoren ==================


/* struct für die Daten des vergangenen Sendens
	 - last_time:		time_point des letzten Mals senden
	 - last_data:		die bytes/Daten des letzten Mals senden
*/
template <size_t N>
struct last_values {
  std::chrono::high_resolution_clock::time_point last_time;
  uint8_t last_data[N];
};


/* prüft, ob es erforderlich ist die Daten zu senden,
	 - true:  wenn das letzte Mal senden zu lange her ist oder die neuen Daten nicht die alten sind
	 - false: wenn die neuen Daten gleich den alten Daten sind
*/
template <size_t N>
bool send_req(uint8_t (&data)[N], last_values<N> &l_data) {
  if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - l_data.last_time).count() > I2C_MOTOR_REFRESH_TIME) {				// prüft ob das letzte Mal senden länger als 100ms her ist
    std::copy(std::begin(data), std::end(data), std::begin(l_data.last_data));				// kopiert data in last_data.last_data
		l_data.last_time = std::chrono::high_resolution_clock::now();							// aktualisiert den time_point des letzten Sendens
		return true;
  }
  else if (!std::equal(std::begin(data), std::end(data), std::begin(l_data.last_data))) {			// prüft ob die Daten array gleich sind
    std::copy(std::begin(data), std::end(data), std::begin(l_data.last_data));
		l_data.last_time = std::chrono::high_resolution_clock::now();							// aktualisiert den time_point des letzten Sendens
		return true;
  }
  return false;
}

last_values<2> last_dir_pwm;
int mot::set_dir_pwm(uint8_t direction, uint8_t pwm) {
  uint8_t data[2] = {direction, pwm};
	if(send_req(data, last_dir_pwm))
		return i2c_smbus_write_block_data(arduino_fd, SET_DIR_PWM, 2, data);
	else
		return 0;
}

last_values<3> last_dir_pwm_steer;
int mot::set_dir_pwm_steer(uint8_t direction, uint8_t pwm, int angle) {
  // normalize angle
  if(angle < -100) {
    angle = -100;
  } else if(angle > 100) {
    angle = 100;
  }
  // shift range from [-100 - 100]  to  [0 - 200]
  angle += 100;

  uint8_t data[3] = {direction, pwm, (uint8_t) angle};

  if(send_req(data, last_dir_pwm_steer))
		return i2c_smbus_write_block_data(arduino_fd, SET_DIR_PWM_STEER, 3, data);
	else
		return 0;
}

// sets the motor to the given state by sending the values over I2C to the arduino

last_values<1> last_state;
int mot::set_state(uint8_t state) {
  uint8_t data[1] = {state};
	if(send_req(data, last_state))
		return i2c_smbus_write_byte_data(arduino_fd, SET_STATE, data[0]);
	else
		return 0;
}

int mot::get_state() {
  return i2c_smbus_read_byte_data(arduino_fd, GET_STATE);
}

last_values<1> last_servo;
int mot::servo(int angle) {

  // normalize angle
  if(angle < -100) {
    angle = -100;
  } else if(angle > 100) {
    angle = 100;
  }
  // shift range from [-100 - 100]  to  [0 - 200]
  angle += 100;

  uint8_t data[1] = {(uint8_t)angle};
	if(send_req(data, last_state))
		return i2c_smbus_write_byte_data(arduino_fd, SET_STATE, data[0]);
	else
		return 0;
}

// ================== Sensoren ==================

/* Returns a specific bit from a single byte.
 * The bit_index has to be between 0 and 7
 * credit of this function: https://stackoverflow.com/questions/4854207/get-a-specific-bit-from-byte
 */

bool sen::get_bit(uint8_t byte, uint8_t bit_index) {
	return (byte & (1 << (bit_index - 1))) != 0;
}

bool m_us_data[4] = {0,0,0,0};

int sen::update_us_data() {
    int in_data = i2c_smbus_read_byte_data(arduino_fd, GET_ULTRASONIC);

    if(in_data < 0) {
      std::cout << "Error reading ultrasonic sensor data: " << strerror(errno) << std::endl;
      return in_data;
    }

    for (size_t i = 0; i < 4; i++) {
      m_us_data[i] = get_bit(in_data, i);
    }
    return 0;
}

bool sen::get_us(int sensor, bool update_data) {
  if(update_data)
    update_us_data();

  return m_us_data[sensor];
}

bool sen::get_ir() {
  int in_data = i2c_smbus_read_byte_data(arduino_fd, GET_INFRARED);

  if(in_data < 0) {
    std::cout << "Error reading infrared sensor data: " << strerror(errno) << std::endl;
    return in_data;
  }

  return (in_data > 0);
}
