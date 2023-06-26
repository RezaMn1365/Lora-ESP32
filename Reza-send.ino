/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <QMC5883LCompass.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LoRa.h>

int counter = 0;

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
QMC5883LCompass compass;

void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 test"));
  compass.init();
  
  LoRa.begin(434450E3);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setSpreadingFactor(8);
  LoRa.setCodingRate4(8);
  LoRa.setFrequency(434450E3);
  LoRa.setTxPower(14);
  LoRa.enableCrc();
  LoRa.setGain(5);

  Serial.println("LoRa Sender");

  if (!LoRa.begin(434450E3)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);


  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {

  
    int T, P, A;
    Serial.print(F("Temperature = "));
    T = bmp.readTemperature();
    Serial.println(T);    

    Serial.print(F("Pressure = "));
    P = bmp.readPressure();
    Serial.println(P);  

    Serial.print(F("Approx altitude = "));
    A = bmp.readAltitude(1013.25);
    Serial.println(A); /* Adjusted to local forecast! */    

     int x, y, z;
  
  // Read compass values
  compass.read();

  // Return XYZ readings
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
    
  Serial.print("X: ");
  Serial.println(x);
  Serial.print(" Y: ");
  Serial.println(y);
  Serial.print(" Z: ");
  Serial.println(z);
  Serial.println();

  counter++;

  delay(1000);
  display.clearDisplay();   
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font  

  display.setCursor(0, 0);     // Start at top-left corner
  display.println(F("Sending packet: "));
  display.display();        
  display.println(counter);
  display.display();         
  
  delay(1000);
  
    Serial.print("Sending packet:T ");     
   // send packet
  LoRa.beginPacket();
  LoRa.print("Temperature ");
  LoRa.print(T);
  LoRa.endPacket(true);
  
  Serial.print("Sending packet:P ");  
     // send packet
  LoRa.beginPacket();
  LoRa.print("Pressure ");
  LoRa.print(P);
  LoRa.endPacket(true); 

  Serial.print("Sending packet:A ");  
     // send packet
  LoRa.beginPacket();
  LoRa.print("altitude ");
  LoRa.print(A);
  LoRa.endPacket(true);

  Serial.print("Sending packet:X ");  
     // send packet
  LoRa.beginPacket();
  LoRa.print("X: ");
  LoRa.print(x);
  LoRa.endPacket(true);

  Serial.print("Sending packet:Y ");  
     // send packet
  LoRa.beginPacket();
  LoRa.print("Y: ");
  LoRa.print(y);
  LoRa.endPacket(true);

  Serial.println("Sending packet:Z ");  
     // send packet
  LoRa.beginPacket();
  LoRa.print("Z: ");
  LoRa.print(z);
  LoRa.endPacket(true);
}