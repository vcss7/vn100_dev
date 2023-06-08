#include <Arduino.h>
#include <VN100.h>

VN100::VN100() {
}

void VN100::begin(HardwareSerial *serial1Pointer, int BAUD_RATE=IMU_BAUD){
    Serial_VN100=serial1Pointer;
    if(!Serial_VN100->available())
      {
      Serial_VN100->begin(BAUD_RATE);
      }
    Serial_VN100->print("$VNASY,1*XX\r\n");//start async
    Serial_VN100->print("$VNWRG,06,0*XX\r\n");//stop ascii outputs / only binary
    Serial_VN100->print("$VNWRG,75,2,16,10,00B6*XX\r\n"); //YPR/Quat/mag/acc/linAcc  16 bit divisor
}

bool VN100::init()
  {
  imu_sync_detected = false;
  if (Serial_VN100->available() > 4) check_sync_byte();
  return imu_sync_detected;
  }

void VN100::read() 
  {
  imu_sync_detected = false; //is reset to true in check_sync_byte()
  // Check if new IMU data is available
  if (Serial_VN100->available() > 4) check_sync_byte();
  // If sync byte is detected, read the rest of the data
  if (imu_sync_detected) read_imu_data();
  }

void VN100::check_sync_byte(void) // Check for the sync byte (0xFA)
  {
    for (int i = 0; i < 6; i++) {
      Serial_VN100->readBytes(in, 1);
      if (in[0] == 0xFA) {
        imu_sync_detected = true;
        break;
      }
    } 
  }


// Read the IMU bytes
void VN100::read_imu_data(void) {
  Serial_VN100->readBytes(in, 69);

  checksum.b[0] = in[68];
  checksum.b[1] = in[67];

  if (calculate_imu_crc(in, 67) == checksum.s) {
    for (int i = 0; i < 4; i++) {
      Yaw.b[i]   = in[3 + i];
      Pitch.b[i] = in[7 + i];
      Roll.b[i]  = in[11 + i];

      Q_x.b[i] = in[15 + i];
      Q_y.b[i] = in[19 + i];
      Q_z.b[i] = in[23 + i];
      Q_s.b[i] = in[27 + i];

      mn_x.b[i] = in[31 + i];
      mn_y.b[i] = in[35 + i];
      mn_z.b[i] = in[39 + i];

      a_x.b[i] = in[43 + i];
      a_y.b[i] = in[47 + i];
      a_z.b[i] = in[51 + i];

      lan_x.b[i] = in[55 + i];
      lan_y.b[i] = in[59 + i];
      lan_z.b[i] = in[63 + i];
    }    
  }
}


// Calculate the 16-bit CRC for the given ASCII or binary message.
unsigned short VN100::calculate_imu_crc(byte data[], unsigned int length)
{
  unsigned int i;
  unsigned short crc = 0;
  for(i=0; i<length; i++){
    crc = (byte)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (byte)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
}
float VN100::GetYaw(){return Yaw.f;}
float VN100::GetPitch(){return Pitch.f;}
float VN100::GetRoll(){return Roll.f;}

float VN100::GetQ1(){return Q_x.f;}
float VN100::GetQ2(){return Q_y.f;}
float VN100::GetQ3(){return Q_z.f;}
float VN100::GetQ4(){return Q_s.f;}

float VN100::GetMN_X(){return mn_x.f;}
float VN100::GetMN_Y(){return mn_y.f;}
float VN100::GetMN_Z(){return mn_z.f;}

float VN100::GetACC_X(){return a_x.f;}
float VN100::GetACC_Y(){return a_y.f;}
float VN100::GetACC_Z(){return a_z.f;}

float VN100::GetLACC_X(){return lan_x.f;}
float VN100::GetLACC_Y(){return lan_x.f;}
float VN100::GetLACC_Z(){return lan_x.f;}

void VN100::printToSerial(HardwareSerial *OutputSerial)
{
    OutputSerial->println("YPR: "+String(Yaw.f) + "," + String(Pitch.f) + "," + String(Roll.f));
    OutputSerial->println("QN: "+String(Q_x.f) + "," + String(Q_y.f) + "," + String(Q_z.f)+ "," + String(Q_s.f));
    OutputSerial->println("MN: "+String(mn_x.f) + "," + String(mn_y.f) + "," + String(mn_z.f));
    OutputSerial->println("A: "+String(a_x.f) + "," + String(a_y.f) + "," + String(a_z.f));
    OutputSerial->println("LaN: "+String(lan_x.f) + "," + String(lan_y.f) + "," + String(lan_z.f));
}