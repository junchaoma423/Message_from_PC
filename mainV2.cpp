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
    enum sp_return result = sp_get_port_by_name(path.c_str(), &port);
    if (result != SP_OK) {
        perror("Error finding the specified serial port");
        exit(EXIT_FAILURE);
    }

    result = sp_open(port, SP_MODE_READ_WRITE);
    if (result != SP_OK) {
        perror("Error opening the specified serial port");
        exit(EXIT_FAILURE);
    }

    sp_set_baudrate(port, baudrate);
    sp_set_bits(port, 8);
    sp_set_parity(port, SP_PARITY_NONE);
    sp_set_stopbits(port, 1);
    sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE);
}

Serial::~Serial() {
    // TODO: Close the fd if open
    sp_close(port);
    sp_free_port(port);
}

ssize_t Serial::writeData(const uint8_t* buffer, size_t length) {
    ssize_t bytesWritten = sp_blocking_write(port, buffer, length, 0);
    if (bytesWritten < 0) {
        throw std::runtime_error("Failed to write to the serial port.");
    }
    return bytesWritten;
}



ssize_t Serial::readDataWithTimeout(uint8_t* buffer, size_t size, int timeoutMs) {
    size_t bytesRead = 0;
    unsigned long startTime = time(NULL);

    while (bytesRead < size && (time(NULL) - startTime) < (timeoutMs / 1000)) {
        ssize_t result = sp_nonblocking_read(port, buffer + bytesRead, size - bytesRead);

        if (result > 0) {
            bytesRead += result;
        } else if (result < 0) {
            return -1;  // Error
        }

        usleep(1000);  // Sleep for a millisecond before the next attempt
    }

    return bytesRead;  // Return the number of bytes actually read
}



void Serial::decodeMessage(const uint8_t* response, float &tauEst, float &speed, float &position){
    int16_t motorTorque = (response[13] << 8 | response[12]);
    if (motorTorque > 250){
        motorTorque = 0;
    }
    int16_t motorSpeed = (response[15] << 8) | response[14];
    int32_t motorPosition = (response[33] << 24) | (response[32] << 16) | (response[31] << 8) | response[30];

    // Convert the scaled values back to the original values
    tauEst = static_cast<float>((motorTorque) / 256.0) * 9.1;
    speed = static_cast<float>(motorSpeed) / 128.0 / 9.1;
    position = static_cast<float>(motorPosition) / (16384.0 / (2.0 * 3.1415926)) / 9.1;

}


int main(){
    // while(true){
    uint8_t message[34];
    uint8_t response[78];
    int motorid = 2;
    int operation_mode = 10;
    float torque = 0.0;
    float position = 0.0;
    float velocity = 6.28;
    float kp = 0.0;
    float kd = 0.1;
    float tauEst;
    float position_actual;
    float velocity_actual;

    generateMessage(motorid, operation_mode, torque, position, velocity, kp, kd, message);
    // printMessage(message, 34);

    Serial serial("/dev/ttyUSB0", 4800000);

    while(true){

        try {
            serial.writeData(message, sizeof(message));
            // std::cout << "Message sent successfully!" << std::endl;
        } catch(const std::exception& e) {
            std::cerr << "An error occurred: " << e.what() << std::endl; 
        }

        ssize_t bytesRead = serial.readDataWithTimeout(response, sizeof(response), 1000);
        // std::cout << "bytesRead is " << bytesRead << std::endl;

        if (bytesRead == sizeof(response)) {
                // std::cout << "Full Response received from the motor:" << std::endl;
                // printMessage(response, bytesRead);
                serial.decodeMessage(response, tauEst, velocity_actual, position_actual);
        }

        std::cout << "position is " << position_actual << std::endl;



        usleep(200);
    }

    return 0;
}