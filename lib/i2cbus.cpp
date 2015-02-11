#include "i2cbus.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "common/mathutil.h" //for CtoK, etc
#include "common/logging.h"

namespace drv {

	I2CBus::I2CBus() {}
	}

	I2CBus::~I2CBus() {
	//	close(fd);
	}

	void I2CBus::busSet(const char * deviceName) {
		fd = open(deviceName, O_RDWR);
		if (fd == -1) {
			LOGE("Failed to open %s (did you remember to run as root?)\n", deviceName );
		}
	}

	void I2CBus::addressSet(uint8_t address) {
		int result = ioctl(fd, I2C_SLAVE, address);
		if (result == -1) {
			LOGE("Failed to set I2C address 0x%02X\n", address );
		}
	}

	void I2CBus::writeByte(uint8_t command, uint8_t data) {
		int result = i2c_smbus_write_byte_data(fd, command, data);
		if (result == -1) {
			LOGE("Failed to write byte to I2C.\n");
		}
	}

	uint8_t I2CBus::readByte(uint8_t command) {
		int result = i2c_smbus_read_byte_data(fd, command);
		if (result == -1) {
			LOGE("Failed to read byte from I2C.\n");
		}
		return result;
	}

	uint16_t I2CBus::readShort(uint8_t command) {
		int result = i2c_smbus_read_word_data(fd, command);
		if (result == -1) {
			LOGE("Failed to read short from I2C.\n");
	        result = 0;
		}
		return result;
	}


	int I2CBus::tryReadByte(uint8_t command) {
		return i2c_smbus_read_byte_data(fd, command);
	}

	void I2CBus::readBlock(uint8_t command, uint8_t size, uint8_t * data) {
		int result = i2c_smbus_read_i2c_block_data(fd, command, size, data);
		if (result != size) {
			LOGE("Failed to read block from I2C.\n");
		}
	}

}
