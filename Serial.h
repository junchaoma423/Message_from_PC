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
#include <libserialport.h>

class Serial {
public:
    Serial(std::string path, int baudrate);
    virtual ~Serial();
    ssize_t writeData(const uint8_t* buffer, size_t length);
    // ssize_t readData(uint8_t* buffer, size_t size);
    ssize_t readDataWithTimeout(uint8_t* buffer, size_t size, int timeoutMs);
    void decodeMessage(const uint8_t* response, float &tauEst, float &speed, float &position);

private:
    int fd;
    struct termios2 ntio;
    struct sp_port *port;
};

#endif /* SERIAL_H_ */
