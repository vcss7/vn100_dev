#ifndef _VN100_H  
#define _VN100_H  

#include <Arduino.h>
#define IMU_BUFFERSIZE 100
#define IMU_BAUD 115200

class VN100
{
  private:
    HardwareSerial *Serial_VN100;
    void read_imu_data(void);
    void check_sync_byte(void);
    unsigned short calculate_imu_crc(byte data[], unsigned int length);

    //Attitude YPR
    union {float f; byte b[4];} Yaw;
    union {float f; byte b[4];} Pitch;
    union {float f; byte b[4];} Roll;
    //Attitude Quaternion
    union {float f; byte b[4];} Q_x;
    union {float f; byte b[4];} Q_y;
    union {float f; byte b[4];} Q_z;
    union {float f; byte b[4];} Q_s;
    // MagNed
    union {float f; byte b[4];} mn_x;
    union {float f; byte b[4];} mn_y;
    union {float f; byte b[4];} mn_z;
    // Acceleration Ned
    union {float f; byte b[4];} a_x;
    union {float f; byte b[4];} a_y;
    union {float f; byte b[4];} a_z;
    // Linear Acceleration Ned
    union {float f; byte b[4];} lan_x;
    union {float f; byte b[4];} lan_y;
    union {float f; byte b[4];} lan_z;
    // Checksum
    union {unsigned short s; byte b[2];} checksum;
    // Parameters
    bool imu_sync_detected = false;  // check if the sync byte (0xFA) is detected
    byte in[IMU_BUFFERSIZE];  // array to save data send from the IMU

    public:
      VN100();
      void begin(HardwareSerial *serial1Pointer, int BAUD_RATE=IMU_BAUD);

      bool init();
      void read();
      void printToSerial(HardwareSerial *OutputSerial);

      float GetYaw();
      float GetPitch();
      float GetRoll();
      
      float GetQ1();
      float GetQ2();
      float GetQ3();
      float GetQ4();

      float GetMN_X();
      float GetMN_Y();
      float GetMN_Z();

      float GetACC_X();
      float GetACC_Y();
      float GetACC_Z();

      float GetLACC_X();
      float GetLACC_Y();
      float GetLACC_Z();
};

#endif