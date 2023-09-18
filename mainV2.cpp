#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include "Serial.h"

// Print the message
void printMessage(const uint8_t* message, size_t size) {
    for (size_t i = 0; i < size; i++) {
        std::cout << "0x";
        if (message[i] < 0x10) {
            std::cout << "0";
        }
        std::cout << std::hex << static_cast<int>(message[i]) << "  ";
    }
    std::cout << std::endl;
}


// Function to calculate CRC32 checkbits
uint32_t crc32_core(uint32_t* ptr, uint32_t len){
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i ++){
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++){
            if (CRC32 & 0x80000000){
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

// Generate the desired message function
// Set fixed bytes
void generateMessage(int motorID, int operationMode, float torque, float position, float speed, float kp, float kd, uint8_t* message){
   // Set fixed bytes
   message[0] = 0xFE;
   message[1] = 0xEE;
   message[2] = static_cast<uint8_t>(motorID);
   message[3] = 0x00;
   message[4] = static_cast<uint8_t>(operationMode);
   message[5] = 0xFF;
   message[6] = 0x00;
   message[7] = 0x00;
   message[8] = 0x00;
   message[9] = 0x00;
   message[10] = 0x00;
   message[11] = 0x00; 

   // Fill in the torque (byte 13 and 14) // FLIPPED
   int16_t tff = static_cast<int16_t>(torque * 256) / 9.1;
   message[12] = tff & 0xFF;
   message[13] = tff >> 8;

   // Fill in the speed (bytes 15 and 16) // FLIPPED
   int16_t wdes = static_cast<int16_t>(speed * 128) * 9.1;
   message[14] = wdes & 0xFF;
   message[15] = wdes >> 8;

   // Fill in the position (bytes 17 to 20) // FLIPPED
   int32_t pdes = static_cast<int32_t>(position * (16384 / (2 * 3.1415926))) * 9.1;
   message[16] = pdes & 0xFF;
   message[17] = (pdes >> 8) & 0xFF;
   message[18] = (pdes >> 16) & 0xFF;
   message[19] = pdes >> 24;

   // Fill in the kp (bytes 21 and 22) // FLIPPED
   uint16_t kpScaled = static_cast<uint16_t>(kp * 2048);
   message[20] = kpScaled & 0xFF;
   message[21] = kpScaled >> 8;

   // Fill in the kd (bytes 23 and 24) // FLIPPED
   uint16_t kdScaled = static_cast<uint16_t>(kd * 1024);
   message[22] = kdScaled & 0xFF;
   message[23] = kdScaled >> 8;

   message[24] = 0x00;
   message[25] = 0x00;
   message[26] = 0x00;
   message[27] = 0x00;
   message[28] = 0x00;
   message[29] = 0x00;

   // Calculate CRC32 check bits (bytes 31 to 34)
   uint32_t crcData[7];
   std::memcpy(crcData, message, 30);
   uint32_t crcValue = crc32_core(crcData, 7);
   message[30] = crcValue & 0xFF;
   message[31] = (crcValue >> 8) & 0xFF;
   message[32] = (crcValue >> 16) & 0xFF;
   message[33] = (crcValue >> 24) & 0xFF;
}

Serial::Serial(std::string path, int baudrate) {
    /* Open modem device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C. */
    fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL);
    if (fd <0) {
        exit(-1);
    }

    ioctl(fd, TCGETS2, &ntio);
    ntio.c_cflag &= ~CBAUD;
    ntio.c_cflag |= BOTHER | CREAD;
    ntio.c_ispeed = baudrate;
    ntio.c_ospeed = baudrate;
    ioctl(fd, TCSETS2, &ntio);
}

Serial::~Serial() {
    // TODO: Close the fd if open
}

void Serial::writeData(const uint8_t* data, size_t size) {
    write(fd, data, size);
}

int main(){
    uint8_t message[34];
    int motorid = 2;
    int operation_mode = 5;
    float torque = 0.0;
    float position = 0.0;
    float velocity = 6.28;
    float kp = 0.0;
    float kd = 0.1;

    generateMessage(motorid, operation_mode, torque, position, velocity, kp, kd, message);
    printMessage(message, 34);

    while(true){

        try {
            Serial serial("/dev/ttyUSB0", 4800000);
            serial.writeData(message, sizeof(message));
            std::cout << "Message sent successfully!" << std::endl;
        } catch(const std::exception& e) {
            std::cerr << "An error occurred: " << e.what() << std::endl; 
        }

        usleep(1000);
    }

    return 0;
}