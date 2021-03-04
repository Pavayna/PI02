#include "CRC.h"
#include "TMC7300_Register.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#define CRC8_GEN 0x07

///////////////////////////////

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

///////////////////////////////


const int fe=100;

//////////////////////////////
#define EN 14
#define ad0 25
#define ad1 26
#define RX 15
#define TX 4
#define encoderL 32
#define encoderR 33

volatile int count=0;
int totalInterruptCounter;
hw_timer_t* hwtimer=NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//////////////////////////////////////////////////////////////

double posL=0;
double posR=0;
const double R=0.021;
double chrono;
double timer1;
HardwareSerial mySerial(1);



//////////////////////////////////////////////////////////////

Kalman kalmanY;
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroYangle; // Angle calculate using the gyro only
double compAngleY; // Calculated angle using a complementary filter
double kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

double pitch,dt;

double angleerror=0;

double dtPrec;

void setup() {

  delay(1000);
  pinMode(EN,OUTPUT);
  pinMode(ad0,OUTPUT);
  pinMode(ad1,OUTPUT);
  pinMode(encoderL,INPUT);
  pinMode(encoderR,INPUT);
  
  digitalWrite(ad0, HIGH);
  digitalWrite(ad1, LOW);
  attachInterrupt(digitalPinToInterrupt(encoderL), encoderChangeL, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderR), encoderChangeR, RISING);

    //setup timer2
  /*TCCR2A = ((1 << WGM21) | (1 << COM2A0));
  TCCR2B = (1 << CS20);
  TIMSK2 = 0;
  OCR2A = 0;
*/

  
  Serial1.begin(115200, SERIAL_8N1, RX, TX);
  Serial.begin(115200);
  delay(500);
  uartWriteDatagram(0x00, TMC7300_SLAVECONF, 0x00000001); //SLAVEADDR to 1
  tmc7300_initMotorDrivers();
  delay(500);
  
  Wire.begin();
  setupMPU();
  
  timer = micros();
  chrono=0;
  timer1=micros();
  /*hwtimer = timerBegin(0, 80, true);
  timerAttachInterrupt(hwtimer, &asservissement, true);
  timerAlarmWrite(hwtimer, 10000, true);
  timerAlarmEnable(hwtimer);*/
  dtPrec=micros()/1e6;
  
}

/*void onTimer(){
  portENTER_CRITICAL_ISR(&timerMux);
  count++;
  portEXIT_CRITICAL_ISR(&timerMux);
 
}*/

double ev1=0;
double ev2=0;
double tetar1=0;
double tetar2=0;
double posprecR=0;
double posprecL=0;
double vconsigne=0;
double etetaSum=0;

const double Kp=31050;
const double Kd=4843.8;
const double Ki=23000;
void asservissement(){
  
}

void loop() {
  
#if 1
  if (millis()-timer1>10){
    double vmesure,dt,eteta,cmd,ev,tetar;
    
    readValues();
    getAngles();

    dt=(micros()/1e6)-dtPrec;
    //dtPrec=micros()/1e6;
    vmesure=(posL-posprecL)/dt;
    posprecL=posL;

    ev=vconsigne-vmesure;
    //v=v1-2.061e-9*v2+9.686e6*e-1.934e7*e1+9.655e6*e2;
    tetar=-0.0152*ev1+0.01495*ev2+1.937*tetar1-0.937*tetar2;
    
    ev2=ev1;
    ev1=ev;
    tetar2=tetar1;
    tetar1=tetar;
    
    //vi=-v/200000.0;
    //Serial.print(v,8); Serial.print("\t");
    //Serial.print(vi); Serial.print("\t");*/

    eteta=tetar-kalAngleY;
    
    etetaSum+=eteta;

    cmd=Kp*eteta+Ki*etetaSum+Kd*eteta/dt;
    
    dtPrec=micros()/1e6;
    
    Serial.print(cmd); Serial.print("\t");
    
    if (cmd>100)
      cmd=100;
    if (cmd<-100)
      cmd=-100;
    
    Serial.println(cmd);
    timer1=micros();
    runMotors(cmd,cmd);
   }
#endif
  #if 0
    Serial.print(pitch); Serial.print("\t");
    Serial.print(gyroYangle); Serial.print("\t");
    Serial.print(compAngleY); Serial.print("\t");
    Serial.println(kalAngleY); //Serial.print("\t\t");
    //Serial.print(v); Serial.print("\t");
    //Serial.print(posL); Serial.print("\t");
    //Serial.println(posR);
  #endif

  #if 0
    Serial.print(chrono);
    Serial.print("\t");
    Serial.print(count);
    Serial.print("\t");
    Serial.println(posL);
  #endif
}

void encoderChangeL(){
  posL+=(1/(3*150.58));
}
void encoderChangeR(){
  posR+=(1/(3*150.58));
}


void uartWriteDatagram(uint8_t SLAVEADDR, uint8_t registerAddress, 
      unsigned long datagram) {
  //TMC5072 takes 64 bit data: 8 sync + reserved, 8 chip address, 
  //8 register address, 32 data, 8 CRC

  uint8_t CRC = 0;
  int temp;
  unsigned char buf[8];
  CRC = NextCRC(CRC, 0x05, CRC8_GEN);
  CRC = NextCRC(CRC, SLAVEADDR, CRC8_GEN);
  CRC = NextCRC(CRC, registerAddress|0x80, CRC8_GEN);  
  CRC = NextCRC(CRC, (datagram >> 24) & 0xff, CRC8_GEN);
  CRC = NextCRC(CRC, (datagram >> 16) & 0xff, CRC8_GEN);
  CRC = NextCRC(CRC, (datagram >> 8) & 0xff, CRC8_GEN);
  CRC = NextCRC(CRC, datagram & 0xff, CRC8_GEN);
  
  buf[0] = 0x05;
  buf[1] = SLAVEADDR;
  buf[2] = registerAddress|0x80;
  buf[3] = (datagram >> 24) & 0xff;
  buf[4] = (datagram >> 16) & 0xff;
  buf[5] = (datagram >> 8) & 0xff;
  buf[6] = datagram & 0xff; 
  buf[7] = CRC;
  
  temp = Serial1.write(buf, 8); //write datagram
  //Serial1.flush();  //wait until all datas are written
  Serial1.readBytes(buf, 8); //clear input buffer
}

unsigned long uartRead(uint8_t SALVEADDR, uint8_t registerAddress) {

  uint8_t CRC = 0, temp;
  unsigned char buf[8];
  unsigned long dataBytes;

  CRC = NextCRC(CRC, 0x05, CRC8_GEN);
  CRC = NextCRC(CRC, SALVEADDR, CRC8_GEN);
  CRC = NextCRC(CRC, registerAddress, CRC8_GEN);
  buf[0] = 0x05;
  buf[1] = SALVEADDR;
  buf[2] = registerAddress;
  buf[3] = CRC;
  Serial1.write(buf, 4); //write datagram
  //Serial1.flush();  //wait until all datas are written
  Serial1.readBytes(buf, 4); //clear input buffer
  
  Serial1.readBytes(buf, 8);
  temp = buf[2];
  dataBytes = buf[3]; //bit 32...24
  dataBytes <<= 8;
  dataBytes |= buf[4]; //bit 23...16
  dataBytes <<= 8;
  dataBytes |= buf[5]; //bit 15...8
  dataBytes <<= 8;
  dataBytes |= buf[6]; //bit 7...0
  
  CRC = 0;
  for(int i=0;i<7;i++)
  {
    CRC = NextCRC(CRC, buf[i], CRC8_GEN);
  }

  //show received bytes
  Serial.print("Received: 0x");
  for(int i=0;i<8;i++)
  {
    char tmp[16];
    sprintf(tmp, "%.2X", buf[i]);
    Serial.print(tmp);
  }
  Serial.print("\n");
  Serial.print("CRC: "); Serial.print(CRC,HEX);Serial.print(" <-> BUFFER: "); 
  Serial.println(buf[7],HEX);

  return dataBytes;
}

void tmc7300_initMotorDrivers()
{
  digitalWrite(EN, LOW);
  uartWriteDatagram(0x01,0x00,0x01);
  uartWriteDatagram(0x01, 0x10,0x00001F01 ); // 32/32 current limit
  uartWriteDatagram(0x01, 0x6C,0x13008001 ); // CHOPCONF
  uartWriteDatagram(0x01, 0x70,0xC40D1024 ); // PWMCONF
  uartWriteDatagram(0x01,TMC7300_PWM_AB , 0x00000000);
  digitalWrite(EN, HIGH);
}

void runMotors(float pwm_a,float pwm_b){
  int32_t pa=(int)((pwm_a*255/100)+0.5);
  int32_t pb=(int)((pwm_b*255/100)+0.5);

  //Serial.println(String(pb,HEX));

  long int data=((pb<<16)&0xffff0000)|(pa&0x0000ffff);
  //Serial.println(data,HEX);
  
  uartWriteDatagram(0x01, TMC7300_PWM_AB, data);
  
}

void setupMPU(){
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif
  
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  //pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  angleerror= atan2(-accX, accZ) * RAD_TO_DEG+90;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG+90;

  kalmanY.setAngle(pitch);
  gyroYangle = pitch;
  compAngleY = pitch; 
}

void readValues(){
    /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);
}

void getAngles(){
  //pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG+90;
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
  
  dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  //dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  gyroYangle += gyroYrate * dt;
  
  //dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  
  timer = micros();
  // Reset the gyro angle when it has drifted too much
  /*if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;*/

  
}
