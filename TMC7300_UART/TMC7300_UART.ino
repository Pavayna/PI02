#include "CRC.h"
#include "TMC7300_Register.h"

#define CRC8_GEN 0x07


#define EN 2
#define ad0 3
#define ad1 4
#define RX 7


void setup() {

  delay(1000);
  pinMode(EN,OUTPUT);
  pinMode(ad0,OUTPUT);
  pinMode(ad1,OUTPUT);
  pinMode(RX,OUTPUT);
  
  digitalWrite(ad0, HIGH);
  digitalWrite(ad1, LOW);
  
  
    //setup timer2
  /*TCCR2A = ((1 << WGM21) | (1 << COM2A0));
  TCCR2B = (1 << CS20);
  TIMSK2 = 0;
  OCR2A = 0;
*/

  Serial.begin(115200);
  Serial1.begin(115200);
  
  delay(500);
  uartWriteDatagram(0x00, TMC7300_SLAVECONF, 0x00000001); //SLAVEADDR to 1
  tmc7300_initMotorDrivers();
  
  delay(500);

  
  
}



void loop() {
  //runMotors(100,50);
  
  //delay(3000);

  runMotors(0,-100);
  //delay(3000);
  //analogWrite(RX,10);
  //uartWriteDatagram(0x01,TMC7300_PWM_AB , 0x00ff0088);
  delay(500);
  //runMotors(10,50);
  //uartWriteDatagram(0x01,TMC7300_PWM_AB , 0x01ff01ff);

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
  Serial1.flush();  //wait until all datas are written
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
  Serial1.flush();  //wait until all datas are written
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
  int32_t pa=(int)((pwm_a*255/100));
  int32_t pb=(int)((pwm_b*255/100));
  //Serial.println(String(pb,HEX));

  long int data=((pb<<16)&0xffff0000)|(pa&0x0000ffff);
  //Serial.println(data,HEX);
  
  uartWriteDatagram(0x01, TMC7300_PWM_AB, data);
  
}
