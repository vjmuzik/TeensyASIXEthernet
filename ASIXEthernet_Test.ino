#include <USBHost_t36.h>
#include <ASIXEthernet.h>

USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
ASIXEthernet asix1(myusb);

void setup() {
  Serial.begin(115200);

  // Wait 1.5 seconds before turning on USB Host.  If connected USB devices
  // use too much power, Teensy at least completes USB enumeration, which
  // makes isolating the power issue easier.
  delay(1500);
  Serial.println("USB Host InputFunctions example");
  delay(10);
  myusb.begin();
  asix1.setHandleRecieve(handleRecieve);
}

elapsedMillis advertise;

void loop() {
  myusb.Task();
  sendAdvertise(); //Example ethernet message to send
  asix1.read();
}

uint8_t sbuf[2500]; // send buffer
uint16_t sbuflen; // length of data to send
uint8_t mainMac[6] = {0x00, 0xA0, 0x7E, 0xA0, 0x15, 0xB8};
uint8_t mainVersion[10] = {'v','1','.','3','7',0,0,0,0,0};
uint8_t mainName[8] = {'V','e','c','t','o','r','1',' '};
uint8_t mainProduct = 0x03;
uint8_t status1 = 0xE0;
uint16_t packetProduct = 0;
uint16_t advertisePacketCount = 0;

void sendAdvertise() {
  if(advertise >= 5000) {
    advertise = 0;
    sendAdvertisePacket1();
    advertisePacketCount++;
  }
}

void sendAdvertisePacket1() {
  // put your main code here, to run repeatedly:
  sbuflen = 64;

  // Dest Mac Address (Broadcast)
  sbuf[0] = 0xFF;
  sbuf[1] = 0xFF;
  sbuf[2] = 0xFF;
  sbuf[3] = 0xFF;
  sbuf[4] = 0xFF;
  sbuf[5] = 0xFF;

  // Src Mac Address
  sbuf[6] = mainMac[0];
  sbuf[7] = mainMac[1];
  sbuf[8] = mainMac[2];
  sbuf[9] = mainMac[3];
  sbuf[10] = mainMac[4];
  sbuf[11] = mainMac[5];

  // Ethernet frame type
  sbuf[12] = 0x88;
  sbuf[13] = 0x5F;

  // Diginet Number of bytes including itself
  sbuf[14] = 0x00;
  sbuf[15] = 0x32;

  // Diginet product of everything past sbuf[29] up to the number of bytes
  sbuf[16] = 0x00;      //Product of everything past sbuf[29] MSB
  sbuf[17] = 0x00;      //Product of everything past sbuf[29] LSB

  // Diginet probably part of the increasing value
  sbuf[18] = 0x00;
  sbuf[19] = 0x00;

  // Diginet values increase everytime this message is sent
  sbuf[20] = advertisePacketCount >> 8;
  sbuf[21] = advertisePacketCount & 0x00FF;

  // Diginet sends back same value from sbuf[18]-sbuf[21]
  sbuf[22] = 0x00;
  sbuf[23] = 0x00;
  sbuf[24] = 0x00;
  sbuf[25] = 0x00;

  // Diginet Increase for each retry to send the message back to 0 once acknowledge is sent
  sbuf[26] = 0x00;
  sbuf[27] = 0x00;                                

  // Diginet I think it's the status
  sbuf[28] = 0xE0;      //Starts at E0  goes to E1 when connected only for advertise packet   00 when sending messages    a0 when responding to status
  status1 = 0xE0;
  if(hostAddress[0] != 0x00 && hostAddress[1] != 0x00 && hostAddress[2] != 0x00 && hostAddress[3] != 0x00 && hostAddress[4] != 0x00 && hostAddress[5] != 0x00) {
    sbuf[28] = 0xE1;
    status1 = 0xE1;
  }
  // Diginet Number of midi messages that follow
  sbuf[29] = 0x01;

  sbuf[30] = hostAddress[0];
  sbuf[31] = hostAddress[1];
  sbuf[32] = hostAddress[2];
  sbuf[33] = hostAddress[3];
  sbuf[34] = hostAddress[4];
  sbuf[35] = hostAddress[5];

  // Diginet haven't figured it out yet
  sbuf[36] = 0x00;
  sbuf[37] = 0x00;
  sbuf[38] = 0x00;
  sbuf[39] = 0x02;
  sbuf[40] = 0x00;
  sbuf[41] = 0x00;
  sbuf[42] = 0x00;
  sbuf[43] = mainProduct;  //0x03 is Pro Control Main    0x07 is Pro Control Edit Pack   0x05 is Pro Control Fader   0x06 is Control|24    0x08 is D-Control Main    0x09 is D-Control Fader   0x0A is D-Control Main    0x0B is D-Control Fader   0x0C is D-Command Main    0x0D is D-Command Fader   0x0E is D-Control Main    0x0F is D-Command Main    0x10 is C|24

  // Diginet version in ASCII "v1.37"00000 for Pro Control    "v8.0.0.314" for C|24   "v10.0.0.0" for 0x0E D-Control Main
  sbuf[44] = mainVersion[0];
  sbuf[45] = mainVersion[1];
  sbuf[46] = mainVersion[2];
  sbuf[47] = mainVersion[3];
  sbuf[48] = mainVersion[4];
  sbuf[49] = mainVersion[5];
  sbuf[50] = mainVersion[6];
  sbuf[51] = mainVersion[7];
  sbuf[52] = mainVersion[8];
  sbuf[53] = mainVersion[9];

  // Diginet Name in ASCII 8 bytes long, maybe more "PC MAIN "
  sbuf[54] = mainName[0];
  sbuf[55] = mainName[1];
  sbuf[56] = mainName[2];
  sbuf[57] = mainName[3];
  sbuf[58] = mainName[4];
  sbuf[59] = mainName[5];
  sbuf[60] = mainName[6];
  sbuf[61] = mainName[7];

  // Diginet Haven't figured it out yet
  sbuf[62] = 0x00;
  sbuf[63] = 0x00;

  getProduct();

  // sending buffer
  asix1.sendPacket(sbuf, sbuflen);
}

void getProduct() {
  packetProduct = 0;
  for(int d = 30; d < (((sbuf[14] << 8) | sbuf[15]) + 14); d++) {
    packetProduct += sbuf[d];
  }
  sbuf[16] = packetProduct >> 8;
  sbuf[17] = packetProduct & 0x00FF;
}

void handleRecieve(const uint8_t* data, uint16_t length) {
  Serial.println("Message Recieved: ");
  const uint8_t* end = data + length;
  while(data < end){
    if(*data <= 0x0F) Serial.print("0");
    Serial.print(*data++, HEX);
    Serial.print(" ");
  }
  Serial.println();
}
