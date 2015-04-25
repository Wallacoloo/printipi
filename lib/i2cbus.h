
#ifndef _I2CBus_h
#define _I2CBus_h
#include <stdint.h>

namespace drv {
class I2CBus
{

	public:
		I2CBus();
		~I2CBus();
		void busSet(const char * deviceName);
		void addressSet(uint8_t address);
		void writeByte(uint8_t command, uint8_t data);
		uint8_t readByte(uint8_t command);
		uint16_t readShort(uint8_t command);
		int tryReadByte(uint8_t command);
		void readBlock(uint8_t command, uint8_t size, uint8_t * data);
		int tryReadByte(uint8_t address, uint8_t command) {
			addressSet(address);
			return tryReadByte(command);
		}

	private:
		int fd;

	};

}

#endif
