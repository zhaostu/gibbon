/*
 Sparkfun 6DOF IMU + 
 
 Reads from Analog Devices Sparkfun 6DOF IMU accelerometer/gyroscope,
 plus the HMC5843 2C 3-axis magnetometer and communicates the data
 to the serial port. 
 
 The circuit:
 analog 0: accl z-axis
 analog 1: accl y-axis
 analog 2: accl x-axis
 analog 3: gyro y-axis
 analog 4: gyro x-axis
 analog 5: gyro z-axis
 Wire    : magnetometer
*/

#include <Wire.h> // Include Wire library for I2C


// Device address
#define HMC5843    (0x1E)

// Register address
#define HMC5843_CRA          (0x00) // 2 bits of measurement mode, 3 bits of output rate from lsb
#define HMC5843_CRB          (0x01) // 3 bits of gain from msb
#define HMC5843_MR           (0x02) // 2 bits of mode select from lsb
#define HMC5843_DX           (0x03) // data X msb

// these constants describe the pins. They won't change:
const int axpin = A2; // x-axis of the accelerometer
const int aypin = A1; // y-axis of the accelerometer
const int azpin = A0; // z-axis of the accelerometer
const int gxpin = A4; // x-axis of the gyroscope
const int gypin = A3; // y-axis of the gyroscope
const int gzpin = A5; // z-axis of the gyroscope

const int RAW_LEN = 9;
const int BUF_LEN = 26;

unsigned long id;
unsigned long t;
unsigned long t_hmc5843;
int raw[RAW_LEN];
byte buffer[BUF_LEN];
byte mode;

float scale_hmc5843[3] = {1.0, 1.0, 1.0};

void setup()
{
  // initialize the serial communications:
  Serial.begin(57600);
  
  // initialize hmc5843
  Wire.begin();
  initHMC5843();
  
  // Set the analog reference to external reference (3.3v)
  analogReference(EXTERNAL);
  
  // Sending 'ready' to the serial port.
  Serial.write('R');
  
  // Wait for serial port for the "mode" parameter.
  while(Serial.available() <= 0);
  mode = Serial.read();
}
void writeHMC5843(byte reg, byte value){
  // Write value to reg on HMC5843
  Wire.beginTransmission(HMC5843);
  Wire.send(reg); // The register.
  Wire.send(value); // The value.
  Wire.endTransmission();
}

byte readByteHMC5843(){
  if(Wire.available()){
    return Wire.receive();
  }
}

void readHMC5843(){
  // Send the desired address.
  Wire.beginTransmission(HMC5843);
  Wire.send(HMC5843_DX);
  Wire.endTransmission();
  
  Wire.beginTransmission(HMC5843);
  Wire.requestFrom(HMC5843, 6);
  raw[6] = readByteHMC5843() << 8;
  raw[6] |= readByteHMC5843();
  raw[7] = readByteHMC5843() << 8;
  raw[7] |= readByteHMC5843();
  raw[8] = readByteHMC5843() << 8;
  raw[8] |= readByteHMC5843();
  Wire.endTransmission();
}

void calibrateHMC5843(){
  // Set calibrate mode.
  writeHMC5843(HMC5843_CRA, 0x11);
  // Set single-conversion mode.
  writeHMC5843(HMC5843_MR, 0x01);
  readHMC5843();
  // Set back to normal mode.
  writeHMC5843(HMC5843_CRA, 0x10);

  scale_hmc5843[0] = 715.0 / raw[6];
  scale_hmc5843[1] = 715.0 / raw[7];
  scale_hmc5843[2] = 715.0 / raw[8];
}

void initHMC5843(){
  // Wait 5ms for hmc5843 as said on datasheet.
  delay(5);
  // Calibrate the HMC5843
  // calibrateHMC5843();
  // Change to continous mode.
  writeHMC5843(HMC5843_MR, 0x00);
}

void waitSignal(){
  while(Serial.available() <= 0);
  Serial.read();
}

void readRaw(){
  t = micros();
  raw[0] = analogRead(axpin);
  raw[1] = analogRead(aypin);
  raw[2] = analogRead(azpin);
  raw[3] = analogRead(gxpin);
  raw[4] = analogRead(gypin);
  raw[5] = analogRead(gzpin);
  
  // Read HMC5843.
  if(t - t_hmc5843 > 100000){
    readHMC5843();
    normalizeHMC5843();
    t_hmc5843 = t;
  }
}

void normalizeHMC5843(){
  raw[6] = raw[6] * scale_hmc5843[0];
  raw[7] = raw[7] * scale_hmc5843[1];
  raw[8] = raw[8] * scale_hmc5843[2];
}

void writeBuffer(){
  unsigned long temp_id, temp_t;
  int idx = 0;
  
  temp_id = id++;
  for(int i = 0; i < 4; i++){
    buffer[idx++] = lowByte(temp_id);
    temp_id >>= 8;
  }
  
  temp_t = t;
  for(int i = 0; i < 4; i++){
    buffer[idx++] = lowByte(temp_t);
    temp_t >>= 8;
  }
  
  for(int i = 0; i < RAW_LEN; i++){
    buffer[idx++] = lowByte(raw[i]);
    buffer[idx++] = highByte(raw[i]);
  }
}

void writeSerial(){
  Serial.write(buffer, BUF_LEN);
}

void loop()
{
  if(mode == 1){
    waitSignal();
  }
  readRaw();
  writeBuffer();
  writeSerial();
}

