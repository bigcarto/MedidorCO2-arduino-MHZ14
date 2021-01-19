//Librerías de la pantalla oled
#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//Este código hace referencia a mi pantalla
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED


#include <SoftwareSerial.h>

// El pin 13(T) del sensor al 10
// El pin 14(R) del sensor al 11
// El pin  4(Análogo) del sensor  al A0


const int analogPin = A0;
//const long samplePeriod = 10000L;
const long samplePeriod = 1000L;


SoftwareSerial sensor(10, 11);

const byte requestReading[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
byte result[9];
long lastSampleTime = 0;

void setup() {
  Serial.begin(9600);
  sensor.begin(9600);
  Serial.print("El sensor comienza a calentar");
  //Inicializo la pantalla
  u8g2.begin();
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_6x12_me); // choose a suitable
  u8g2.setCursor (0,14);
  u8g2.print("Inicio calentamiento");
  u8g2.setCursor (0,28);
  u8g2.print("esperar 20'");
  u8g2.sendBuffer();          // transfer internal memory to the display
  delay(20000);
}

void loop() {

  long now = millis();
  if (now > lastSampleTime + samplePeriod) {
    lastSampleTime = now;
    int ppmV = readPPMV();
    int ppmS = readPPMSerial();
    Serial.print(ppmV);
    Serial.print("\t");
    Serial.println(ppmS);

    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_logisoso24_tr); // choose a suitable
    u8g2.setCursor (0,28);
    u8g2.print(String(ppmV)+ "ppm");
    u8g2.sendBuffer();          // transfer internal memory to the display
    delay(2000);
    if (ppmV > 700){
    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_logisoso18_tr); // choose a suitable
    u8g2.setCursor (0,28);
    u8g2.print("VENTILAR");
    u8g2.sendBuffer();          // transfer internal memory to the display
    delay(2000);
    }

    }
}

int readPPMV() {
  float v = analogRead(analogPin) * 5.0 / 1023.0;
  int ppm = int((v - 0.4) * 3125.0);
  return ppm;
}

int readPPMSerial() {
  for (int i = 0; i < 9; i++) {
    sensor.write(requestReading[i]);
  }
  //Serial.println("sent request");
  while (sensor.available() < 9) {}; // wait for response
  for (int i = 0; i < 9; i++) {
    result[i] = sensor.read();
  }
  int high = result[2];
  int low = result[3];
    //Serial.print(high); Serial.print(" ");Serial.println(low);
  return high * 256 + low;
}