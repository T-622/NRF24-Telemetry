#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735


RF24 radio(7, 8); // CE, CSN
const byte address[6] = "000001";


#if defined(ARDUINO_FEATHER_ESP32) // Feather Huzzah32
#define TFT_CS         14
#define TFT_RST        15
#define TFT_DC         32

#elif defined(ESP8266)
#define TFT_CS         4
#define TFT_RST        16
#define TFT_DC         5

#else
// For the breakout board, you can use any 2 or 3 pins.
// These pins will also work for the 1.8" TFT shield.
#define TFT_CS        10
#define TFT_RST        9 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         6
#endif

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

struct dataPackage {
  
int RPM;
float SPD;
float ambTemp;
float escTemp;
float gForce;
float batteryVoltage;

};
dataPackage data;

void setup() {

  Serial.begin(9600);
  radio.begin();
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  tft.fillScreen(0x047E);
  tft.setRotation(3);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.setTextColor(0xFFFF);
  tft.print("V1.12");
  tft.setCursor(10, 40); //(X, Y)
  tft.setTextSize(2);
  tft.setTextColor(0xFD20);
  tft.println("RC Telemetry");
  tft.setCursor(50, 60);
  tft.print("System");
  tft.setTextSize(1);
  tft.setCursor(40, 80);
  tft.setTextColor(0xF800);
  tft.print("By: Tyler Peppy");
  delay(2500);
  //radio.setChannel(115);
  //radio.setDataRate(RF24_250KBPS);

}
void loop() {

  if (radio.available()) {
    radio.read(&data, sizeof(dataPackage));
    tft.setRotation(3);
    tft.fillScreen(0x432E);
    tft.setCursor(0, 0);
    drawData(0xCBC3, 0xFFFF);
    Serial.println("----------------");
    Serial.println(data.ambTemp);
    Serial.println(data.escTemp);
    Serial.println(data.gForce);
    Serial.println(data.batteryVoltage);
    Serial.println("----------------");
    Serial.println("");
//    for (int i = 0; i < 2; i++) {
//    }
    
  }
}

void drawData(uint16_t color, uint16_t color1) {
  tft.setTextColor(color1); // Set draw color

  tft.drawRoundRect(0, 0, 75, 30, 5, color); //drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t R , uint16_t t)
  tft.drawRoundRect(85, 0, 75, 30, 5, color); //drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t R , uint16_t t)
  tft.setCursor(5, 12);
  tft.print("RPM: ");
  tft.print(data.RPM);
  tft.setCursor(90, 12);
  tft.print("KM/H: ");
  tft.print(data.SPD);

  tft.drawRoundRect(0, 49, 75, 30, 5, color); //drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t R , uint16_t t)
  tft.drawRoundRect(85, 49, 75, 30, 5, color); //drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t R , uint16_t t)
  tft.setCursor(5, 61);
  tft.print("AMB: ");
  tft.print(data.ambTemp);
  tft.print((char)247); 
  tft.setCursor(90, 61);
  tft.print("ESC: ");
  tft.print(data.escTemp);
  tft.print((char)247); 

  tft.drawRoundRect(0, 98, 75, 30, 5, color); //drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t R , uint16_t t)
  tft.drawRoundRect(85, 98, 75, 30, 5, color); //drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t R , uint16_t t)
  tft.setCursor(5, 110);
  tft.print("GFR: ");
  tft.print(data.gForce);
  tft.print("G");
  tft.setCursor(90, 110);
  tft.print("VBAT: ");
  tft.print(data.batteryVoltage);
  tft.print("V");

}
