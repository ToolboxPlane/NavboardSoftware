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

void srf02_start() {
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(112); // transmit to device #112 (0x70)
  // the address specified in the datasheet is 224 (0xE0)
  // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
  Wire.write(byte(0x51));      // command sensor to measure in centimeters (0x51)
  Wire.endTransmission();      // stop transmitting
}

uint16_t srf02_read() {
   // step 3: instruct sensor to return a particular echo reading
  Wire.beginTransmission(112); // transmit to device #112
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting

  // step 4: request reading from sensor
  Wire.requestFrom(112, 2);    // request 2 bytes from slave device #112

  // step 5: receive reading from sensor
  uint16_t reading = 0;
  if (2 <= Wire.available()) { // if two bytes were received
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
  }
  return reading;
}


void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(9600);

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

  // Altimeter
  float altm = baro.getAltitude();
  Serial.print(altm); Serial.println("m");

  // Pitot Tube
  uint16_t adc = analogRead(A0);
  float outputVoltage = adc/1023.0F * 5; // Compensating for the voltage divider
  
  float pressurekPa = outputVoltage - 1;
  Serial.print(outputVoltage); Serial.println("V");
  Serial.print(pressurekPa); Serial.println("kPa");
  float rho_air = 1.2041F;
  float velocity = sqrt(2 * pressurekPa / rho_air);
  Serial.print(velocity); Serial.println("m/s");

  // SRF-02
  srf02_start();
  delay(70);
  uint16_t distance = srf02_read();
  Serial.print(distance); Serial.println("cm");

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
