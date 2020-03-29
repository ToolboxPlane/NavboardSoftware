#include <SPI.h>
#include <Wire.h>

#include <RH_RF95.h>
#include <Adafruit_MPL3115A2.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 434.0

#define BUF_SIZE 255

RH_RF95 rf95(RFM95_CS, RFM95_INT);


Adafruit_MPL3115A2 baro;


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(9600);
  Serial.write("Test\n");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  //rf95.setTxPower(23, false);
  RH_RF95::ModemConfig modemConfig;
  modemConfig.reg_1d = 0b01110010;
  modemConfig.reg_1e = 0b01110100;
  modemConfig.reg_26 = 0b00001000;
  rf95.setModemRegisters(&modemConfig);

  if (!baro.begin()) {
    Serial.println("Couldnt find sensor");
    while (1);
  }
  digitalWrite(13, LOW);
}

// the loop function runs over and over again forever
void loop() {
  static bool led = false;
  
  float altm = baro.getAltitude();
  //Serial.print(altm); Serial.println("m");


  // LoRa
  uint8_t buf[BUF_SIZE];
  uint8_t bufLen = BUF_SIZE;
  
  if (rf95.available()) {
    rf95.recv(buf, &bufLen);
    for (uint16_t c=0; c<bufLen; ++c) {
      Serial.write(buf[c]);
    }
    led = !led;
  }

  bufLen = 0;
  while (Serial.available() && bufLen < BUF_SIZE) {
    buf[bufLen] = Serial.read();
    bufLen += 1;
  }

  if (bufLen > 0) {
    rf95.send(buf, bufLen);
    rf95.waitPacketSent();
    led = !led;
  }


  digitalWrite(13, led);
}
