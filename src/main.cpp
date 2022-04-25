
/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.

  2022/04/24 Added the oled
 ***************************************************************************/
#include <Arduino.h>          // Arduino Framework <> searches the libraries paths
#include <Wire.h>             // TWI/I2C library for Arduino & Wiring
#include <U8g2lib.h>          // For text on the little on-chip OLED

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//#define SEALEVELPRESSURE_HPA (1013.25) //this is the default
#define SEALEVELPRESSURE_HPA (1014.9) // as reported at VNY 2022/04/24 23:00

const String sketchName = "ESP32TwoBME280s";

Adafruit_BME280 bme; // I2C
Adafruit_BME280 bme1; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

// The active board is declared in platformio.ini. The define is 
// declared in pins_arduino.h.

#if defined(ARDUINO_HELTEC_WIFI_LORA_32) //# is evaluated at time of compile
  #define OLED_CLOCK SCL_OLED            // Pins for OLED display
  #define OLED_DATA SDA_OLED
  #define OLED_RESET RST_OLED
//  #define LED_PIN 23                     //Output pin for the WS2812B led strip. Dave recomends pin 5 but it is being used by LoRa on my board
#elif defined(WIFI_LoRa_32_V2) //# is evaluated at time of compile
  #define OLED_CLOCK SCL_OLED            // Pins for OLED display
  #define OLED_DATA SDA_OLED
  #define OLED_RESET RST_OLED
//  #define LED_PIN 23                     //Output pin for the WS2812B led strip. Dave recomends pin 5 but it is being used by LoRa on my board
#elif defined(ARDUINO_LOLIN32)
  #define OLED_CLOCK 4              // Pins for OLED display
  #define OLED_DATA 5
  #define OLED_RESET 16
//  #define LED_PIN 5 //Output pin for the WS2812B led strip.
#else
  // #define OLED_CLOCK 15              // Pins for OLED display
  // #define OLED_DATA 4
  // #define OLED_RESET 16
  // #define LED_PIN 5 //Output pin for the WS2812B led strip.
#endif

//clock and data got swapped around to use hardware I2C instead of software
U8G2_SSD1306_128X64_NONAME_F_SW_I2C g_oled(U8G2_R2, OLED_CLOCK, OLED_DATA, OLED_RESET); // uses Software I2C and results in a framerate of 5 FPS
// The following line that useds I2C hardware does not appear to work with other devices on the I2C
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C g_oled(U8G2_R2, OLED_RESET, OLED_CLOCK, OLED_DATA); // uses Hardware I2C and results in a framerate of 26 FPS 
int g_linehight = 0;


void setup() {
  Serial.begin(115200);
  while(!Serial);    // time to get serial running
  Serial.println();
  Serial.println(sketchName);

  g_oled.begin();
  g_oled.clear(); //sets curser at 0,0. Text draws from the bottom up so you will see nothing.
  g_oled.setFont(u8g2_font_profont15_tf);
  g_linehight = g_oled.getFontAscent() - g_oled.getFontDescent(); // Decent is a negative number so we add it to the total
  g_oled.drawRFrame(0,0,g_oled.getWidth(),g_oled.getHeight(),7);  // Draw a boarder around the display
  g_oled.setCursor(3,g_linehight * 6 + 2);
  g_oled.sendBuffer();
  g_oled.setFont(u8g2_font_inb19_mn);

    unsigned status;
    unsigned status1;
    
    // default settings
    status = bme.begin(0x76);  
    status1 = bme1.begin(0x77);
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor in space One, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        delay(5000);
    }
    
  if (!status1) {
    Serial.println(F("Could not find a valid BME280 sensor in space TWO, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bme1.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    delay(5000);
  }
    Serial.print("Sensor One ID was: 0X"); Serial.print(bme.sensorID(),16); Serial.print(" with a status of ");
    Serial.println(status);
    Serial.println(status1);

        // weather monitoring
    Serial.println("-- Weather Station Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
    Serial.println("filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
    bme1.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
                      
    // suggested rate is 1/60Hz (1m)
    delayTime = 1000; // in milliseconds

    Serial.println();
}

void printValues() {
    Serial.println("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");
    Serial.print(bme1.readTemperature());
    Serial.println(" °C");

    Serial.println("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");
    Serial.print(bme1.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.println("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
    Serial.print(bme1.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.println("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");
    Serial.print(bme1.readHumidity());
    Serial.println(" %");

    Serial.println();
}

void loop() { 
    // Only needed in forced mode! In normal mode, you can remove the next line.
    bme.takeForcedMeasurement(); // has no effect in normal mode
    bme1.takeForcedMeasurement(); // has no effect in normal mode
   printValues();
	g_oled.setCursor(18,g_linehight * 2 + 2);
  g_oled.print(bme.readAltitude(SEALEVELPRESSURE_HPA)*3.28084);
//		g_oled.printf("%05.1lf", amps);       // send the amps to the OLED
		//g_oled.printf("+%05d", diff_0_1);
	g_oled.setCursor(20,g_linehight * 4 + 2);
  g_oled.print(bme1.readAltitude(SEALEVELPRESSURE_HPA)*3.28084);
//		g_oled.printf("%05.1lf", ampsWt);     // send the amps weighted average to the OLED
		g_oled.sendBuffer();                  // Print it out to the OLED
    delay(delayTime);
}