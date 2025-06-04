#ifndef IMU_H
#define IMU_H

#include <TeensyThreads.h>


/////////////////
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);
/////////////////


#define TRUE 1
#define FALSE 0

#define MAX_YEI_DATA_PACKET 255

#define DEGTORAD 0.017453f
#define RADTODEG 57.295779f

#define NO_SLOT 255
#define READ_TARED_ORIENTATION_AS_QUATERNION 0
#define READ_TARED_ORIENTATION_AS_EULER_ANGLES 1
#define READ_UNTARED_ORIENTATION_AS_QUATERNION 6
#define READ_UNTARED_ORIENTATION_AS_EULER_ANGLES 7
#define READ_NORMALIZED_ACCELEROMETER_VECTOR 34
#define READ_CORRECTED_GYROSCOPE_VECTOR 38
#define READ_CORRECTED_ACCELEROMETER_VECTOR 39
#define READ_CORRECTED_LINEAR_ACCELERATION 41
#define READ_RAW_GYROSCOPE_VECTOR 65
#define READ_RAW_ACCELEROMETER_VECTOR 66
#define READ_RAW_COMPASS_VECTOR 67

#define CMD_SET_BASE_OFFSET_WITH_CURRENT_ORIENTATION 22
#define CMD_SET_STREAMING_SLOT 80
#define CMD_SET_STREAMING_TIMING 82
#define CMD_GET_STREAMING_BATCH 84
#define CMD_START_STREAMING 85
#define CMD_STOP_STREAMING 86
#define CMD_TARE_WITH_CURRENT_ORIENTATION 96
#define CMD_SET_REFERENCE_VECTOR_MODE 105
#define CMD_SET_COMPASS_ENABLE 109
#define CMD_RESET_FILTER 120
#define CMD_SET_ACCELEROMETER_RANGE 121
#define CMD_SET_FILTER_MODE 123
#define CMD_SET_GYROSCOPE_RANGE 125
#define CMD_SET_COMPASS_RANGE 126

#define CMD_GET_COMPASS_ENABLED_STATE 142
#define CMD_GET_ACCELEROMETER_RANGE 148
#define CMD_GET_FILTER_MODE 152
#define CMD_GET_GYROSCOPE_RANGE 154
#define CMD_GET_COMPASS_RANGE 155
#define CMD_BEGIN_GYROSCOPE_AUTOCALIBRATION 165
#define CMD_SET_CALIBRATION_MODE 169
#define CMD_RESPONSE_HEADER_BITFIELD 221
#define CMD_SET_UART_BAUD_RATE 231
#define CMD_GET_UART_BAUD_RATE 232
#define CMD_SET_LED_COLOR 238
#define START_SPI_DATA_TRANSFER 0xF6
#define START_NO_RESP_HEADER 0xF7
#define START_RESP_HEADER 0xF9

#define SPI_IDLE_STATE 0x00
#define SPI_READY_STATE 0x01
#define SPI_BUSY_STATE 0x02
#define SPI_ACC_STATE 0x04
#define DELAY_SPI_YEI 5          //[us]
#define DELAY_WAIT_SERIAL_YEI 1  //[us]
#define DELAY_SERIAL_YEI 10      //[us]

#define REFERENCE_VECTOR_SINGLE_STATIC_MODE 0
#define REFERENCE_VECTOR_SINGLE_AUTO_MODE 1
#define REFERENCE_VECTOR_SINGLE_AUTO_CONTINUOUS_MODE 2
#define REFERENCE_VECTOR_MULTI_REFERENCE_MODE 3

#define ACCELEROMETER_RANGE_6G 0  // 2/6g
#define ACCELEROMETER_RANGE_12G 1  // 4/12g
#define ACCELEROMETER_RANGE_24G 2  // 8/24g for HH model 

#define GYROSCOPE_RANGE_250 0
#define GYROSCOPE_RANGE_500 1
#define GYROSCOPE_RANGE_2000 2

#define COMPASS_RANGE_0_8 0
#define COMPASS_RANGE_1_3 1
#define COMPASS_RANGE_1_9 2
#define COMPASS_RANGE_2_5 3
#define COMPASS_RANGE_4_0 4
#define COMPASS_RANGE_4_7 5
#define COMPASS_RANGE_5_6 6
#define COMPASS_RANGE_8_1 7

#define FILTER_IMU 0
#define FILTER_KALMAN 1
#define FILTER_KALMAN_ALTERNATING 2
#define FILTER_COMPLEMENTARY 3
#define FILTER_QUATERNION_GRADIENT_DESCEND 4
#define FILTER_MAGNETORESISTIVE_QUATERNION_GRADIENT_DESCEND 5

#define CALIBRATION_MODE_BIAS 0
#define CALIBRATION_MODE_BIAS_SCALE 1
#define CALIBRATION_MODE_ORTHO 2

#define CMD_SET_AXIS_DIRECTIONS 116
#define AXIS_XR_YF_ZU 0x01  // Right-handed system
#define AXIS_XU_YR_ZF 0x02
#define AXIS_XF_YU_ZR 0x05

#define CMD_GET_FIRMWARE_VERSION 0xDF
#define CMD_GET_HARDWARE_VERSION 0xE6

#define YEI_DELAY_AFTER_COMMAND 100  //[ms]

#define BYTE_USB_PACKET 128
#define BYTE_DATA_PACKET 128
#define BYTE_PUREDATA_PACKET 64       //91 60
#define BYTE_WIFI_INCOMING_PACKET 16  //11//7
#define BUFFER_SIZE 5000

uint8_t YEIdataPacket[MAX_YEI_DATA_PACKET];

#define package_Num 300;



//Bread Board
#define IMU1 Serial4



// Curve Fitting Coefficient
const float coef1[] = {4.9623, 4.9715, 4.5513, 4.9613, 5.0471, 4.8370, 4.7539, 4.8434}; // Multiply by 10000
const float coef2[] = {22.869, 28.117, 22.400, 21.504, 26.127, 22.159, 25.630, 22.922}; // Multiply by 1000
const uint16_t segmentpoint = 581;


// AFO
const float pi = 3.14159265359f;

const uint8_t AFO_m = 6; // oscillator numbers
float AFO_phi[] = {pi / 2, pi / 2, pi / 2, pi / 2, pi / 2, pi / 2};
float AFO_theta = 0;
float AFO_omega = pi;
float AFO_a[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float AFO_dt = 0.0f;
float AFO_t[] = {0.0f, 0.0f};
const float AFO_v = 0.25;
const float AFO_eps = 2.6;
float AFO_corrected = 0;
float AFO_phi_error = 0.0f;
float AFO_phi_error_dot[] = {0.0f, 0.0f};
float AFO_phi_error_Kp = 0.5;
float AFOc_simple = 0;

// Gait Event Detector
const uint16_t threshold_pressure_sum = 500;
const uint16_t threshold_pressure_state = 300; //50;
const float    rho = 0.5; // The gain rho tunes the acceptance threshold of stride-period variance
uint8_t  Gait_State[] = {0, 0, 1, 1};
float    t_FC[] = {0, 0}; // Times for first contact (k-1 and k stride)
float    t_LC[] = {0, 0}; // Times for last  contact
float Phase_Error = 0;
float Phase_Error_Hat[] = {0, 0};


// Velocity Estimator
const float g_NJ = 9.802;
struct structVelocityEstimator
{
  float ax[BUFFER_SIZE];
  float ay[BUFFER_SIZE];
  float az[BUFFER_SIZE];
  float  t[BUFFER_SIZE];
  float aM[BUFFER_SIZE]; // magnitude

  uint16_t iter = 0;
  uint8_t  Gait_State[2] = {1, 1};
  float vx[BUFFER_SIZE] = {};
  float vy[BUFFER_SIZE] = {};
  float vz[BUFFER_SIZE] = {};
  float px[BUFFER_SIZE];
  float py[BUFFER_SIZE];
  float pz[BUFFER_SIZE];
  float SV = 0; // estimated stride velocity
  float SL = 0; // estimated stride length
  uint16_t start_idx = 0;
  uint16_t end_idx = 0;
  //
  uint16_t FC_idx[2];
  uint8_t  FC_iter = 0;
  uint16_t LC_idx[2];
  uint8_t  LC_iter = 0;

  uint16_t FF_idx1[2];
  uint16_t FF_iter1 = 0;
  uint16_t FF_idx2[2];
  uint16_t FF_iter2 = 0;
} VelocityEstimator;

struct structStreamingTimingInformation {
  unsigned int interval;
  unsigned int duration;
  unsigned int delay;

} sStreamingTime;


struct structComponentLinearAcceleration {
  //Big Endian
  float az;
  float ay;
  float ax;
};

struct structComponentRawAcceleration {
  //Big Endian
  float r_az;
  float r_ay;
  float r_ax;
};

struct structComponentQuaternion {
  //Big Endian
  float qw;
  float qz;
  float qy;
  float qx;
};


struct structEulerAngles {
  //Big Endian
  float roll;
  float yaw;
  float pitch;
};

struct structComponentRawGyro {
  // Big Endian
  float wz;
  float wy;
  float wx;
};

struct structComponentSensorData {
  //Big Endian
  float mx;
  float my;
  float mz;

  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;
};

struct structStreamingData {
  //Big Endian
  structComponentRawAcceleration acc;
  structComponentRawGyro gyro;
  structComponentLinearAcceleration lAcc;
  //structEulerAngles eulerAngles;
  structComponentQuaternion q;
};

struct structSportSolePacket
{
  uint8_t Start_bytes[3];
  uint8_t val; // 0->reboot; 1->start; 2->broadcast
  uint8_t Odroid_Timestamp[8];
  uint8_t Odroid_Trigger;
  uint8_t Stop_bytes[3];
};

union unionIncomingWifi
{
  uint8_t vData[BYTE_WIFI_INCOMING_PACKET];
  structSportSolePacket Raw;
}
Wifi_Incoming_Data;

union unionStreamingData
{
  structStreamingData sData;
  volatile uint8_t vData[sizeof(structStreamingData)];

} uStreamingDataIMU1, uStreamingDataIMU2;

union unionComponentSensorData
{
  structComponentSensorData sData;
  uint8_t vData[sizeof(structComponentSensorData)];

} uCompSensData;

uint8_t calcCRC256(uint8_t* dataPacket, uint8_t nByte)
{
  uint16_t checksum = 0;
  for (uint8_t i = 1; i < nByte; i++)
  {
    checksum += dataPacket[i];
  }

  return (checksum % 256);
}

void waitByteCountFromSerial(HardwareSerial& serial, unsigned int bytecount)
{
  
  while (serial.available() < bytecount)
  {
    // Serial.println("IMU serial: " + String(serial.available()));
    delayMicroseconds(DELAY_WAIT_SERIAL_YEI);
  }
}

void YEIsettingsHeader(HardwareSerial& serial)
{
  // Settings Header
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = CMD_RESPONSE_HEADER_BITFIELD;
  YEIdataPacket[2] = 0x00;
  YEIdataPacket[3] = 0x00;
  YEIdataPacket[4] = 0x00;
  YEIdataPacket[5] = 0x00;
  YEIdataPacket[6] = calcCRC256(YEIdataPacket, 6);
  serial.write(YEIdataPacket, 7);
  delay(YEI_DELAY_AFTER_COMMAND);
}

void YEIwriteCommandNoDelay(HardwareSerial& serial, uint8_t cmd)
{
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = cmd;
  YEIdataPacket[2] = calcCRC256(YEIdataPacket, 2);
  serial.write(YEIdataPacket, 3);
}

void YEIwriteCommand(HardwareSerial& serial, uint8_t cmd)
{
  YEIwriteCommandNoDelay(serial, cmd);
  delay(YEI_DELAY_AFTER_COMMAND);
}

void YEIwriteCommand(HardwareSerial& serial, uint8_t cmd, uint8_t value)
{
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = cmd;
  YEIdataPacket[2] = value;
  YEIdataPacket[3] = calcCRC256(YEIdataPacket, 3);
  serial.write(YEIdataPacket, 4);
  delay(YEI_DELAY_AFTER_COMMAND);
}

uint8_t YEIgetValue(HardwareSerial& serial, uint8_t cmd)
{
  YEIwriteCommand(serial, cmd);
  return serial.read();
}

void YEIsetStreamingTime(HardwareSerial& serial)
{
  // Set Streaming Time

  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = CMD_SET_STREAMING_TIMING;

  uint8_t *pointer = (uint8_t*)&sStreamingTime.interval;
  YEIdataPacket[2] = pointer[3];
  YEIdataPacket[3] = pointer[2];
  YEIdataPacket[4] = pointer[1];
  YEIdataPacket[5] = pointer[0];

  pointer = (uint8_t*)&sStreamingTime.duration;
  YEIdataPacket[6] = pointer[3];
  YEIdataPacket[7] = pointer[2];
  YEIdataPacket[8] = pointer[1];
  YEIdataPacket[9] = pointer[0];

  pointer = (uint8_t*)&sStreamingTime.delay;
  YEIdataPacket[10] = pointer[3];
  YEIdataPacket[11] = pointer[2];
  YEIdataPacket[12] = pointer[1];
  YEIdataPacket[13] = pointer[0];

  YEIdataPacket[14] = calcCRC256(YEIdataPacket, 14);
  serial.write(YEIdataPacket, 15);

  //delay(YEI_DELAY_AFTER_COMMAND);
}

void YEIsetStreamingMode(HardwareSerial& serial, uint8_t slot1, uint8_t slot2, uint8_t slot3, uint8_t slot4, uint8_t slot5, uint8_t slot6, uint8_t slot7, uint8_t slot8)
{
  // Setting Streaming Mode
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = CMD_SET_STREAMING_SLOT;

  YEIdataPacket[2] = slot1; //1st slot
  YEIdataPacket[3] = slot2; //2nd slot
  YEIdataPacket[4] = slot3; //3rd slot
  YEIdataPacket[5] = slot4; //4th slot
  YEIdataPacket[6] = slot5; //5th slot
  YEIdataPacket[7] = slot6; //6th slot
  YEIdataPacket[8] = slot7; //7th slot
  YEIdataPacket[9] = slot8; //8th slot

  YEIdataPacket[10] = calcCRC256(YEIdataPacket, 10);
  serial.write(YEIdataPacket, 11);

  delay(YEI_DELAY_AFTER_COMMAND);
}

// Polling mode
void YEIgetStreamingBatch(unionStreamingData& uStreamingDataIMU1)
{
  int nPacketStreamingData = sizeof(uStreamingDataIMU1);
  int bIMU = 0;

  YEIwriteCommandNoDelay(IMU1, CMD_GET_STREAMING_BATCH);

  while (IMU1.available() < nPacketStreamingData)
  {
    // Serial.print(nPacketStreamingData);
    // Serial.print("\t");
    // Serial.println(IMU1.available());
    delayMicroseconds(DELAY_WAIT_SERIAL_YEI);
    // threads.delay(DELAY_WAIT_SERIAL_YEI);
  }

  while (!(bIMU >= nPacketStreamingData))
  {
    uStreamingDataIMU1.vData[nPacketStreamingData - bIMU - 1] = IMU1.read();

    bIMU++;
  }
}


void quaternion2euler(float q0, float q1, float q2, float q3, float& phi, float& theta, float& psi)
{
  //quaternConj(q0,q1,q2,q3);

  q1 = -q1;
  q2 = -q2;
  q3 = -q3;

  float R11 = 2 * q0 * q0 - 1 + 2 * q1 * q1;
  float R21 = 2 * (q1 * q2 - q0 * q3);
  float R31 = 2 * (q1 * q3 + q0 * q2);
  float R32 = 2 * (q2 * q3 - q0 * q1);
  float R33 = 2 * q0 * q0 - 1 + 2 * q3 * q3;

  phi = atan2(R32, R33);
  theta = -atan(R31 / sqrt(1 - R31 * R31) );
  psi = atan2(R21, R11 );
}


#endif