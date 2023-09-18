// Serial.h
// Serial.h

#ifndef SERIAL_H_
#define SERIAL_H_

#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>

class Serial {
public:
    Serial(std::string path, int baudrate);
    virtual ~Serial();
    void writeData(const uint8_t* data, size_t size);

private:
    int fd;
    struct termios2 ntio;
};

#endif /* SERIAL_H_ */