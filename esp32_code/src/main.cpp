
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_FONA.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <driver/adc.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <ArduinoJson.h>

// Define Wifi Object
WiFiClient client;

// Wifi Credentials
const char *ssid = "Coworking-Rosales";
const char *password = "MoniPepe_4327";

// Server connection parameters
const char *server = "192.168.1.13";
const int port = 3030;

//  Altimeter Address 0x77
#define ALTIMETER_I2C_ADDRESS 0X77

// GPS Serial definition
#define gpsSerial Serial1

// GPS pinout definition
const int FONA_RST = 6;
const int FONA_RX = 12;
const int FONA_TX = 14;

// GPS UART Parameters
HardwareSerial *fonaSerial = &Serial1;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Altimeter Parameters
Adafruit_BMP3XX bmp;

// const float seaLevelPressurehPa = 1013.25;

//  Display Address 0x3D
#define DISPLAY_I2C_ADDRESS 0X3D

// Display OLED Parameters
#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);

// Define Audio
#define AUDIO_BUFFER_MAX 2250
uint8_t audioBuffer[AUDIO_BUFFER_MAX];
uint8_t transmitBuffer[AUDIO_BUFFER_MAX];
uint32_t bufferPointer = 0;

// Timer Interrupt Parameters
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Boolean variables for flags
bool transmitNow = false;
bool debug = true;
bool panicMode = false;

// Ldo regulator pin
const int ldoPin = 21;

// Panic Button pin
const int buttonPin = 0;
int buttonState = 0;

// Timestamp data sent to server
const unsigned long interval = 1000; // 5 seconds
unsigned long previousMillis = 0;

void sendAudioData(bool transmitFlag);
void I2CScanner(void);
void adquireGPSData(void);
void adquireAltitudeDataFromPressure(void);
void updateDisplay(void);
void IRAM_ATTR onTimer(void);
void panicButton(void);
void beginLocationGpsAndGsmTriangulation(void);
void adquireGPSData(float &latitude, float &longitude, float &altitude);
void adquirePressureDataforAltitudeCalculation(float &pressure, float &temperature);
void connectWifi(void);
void connectServer(void);

void setup()
{
  // Initialize Debug interface
  if (debug)
    Serial.begin(115200);

  // I2C Initialization
  Wire.begin();

  // Setup Altimeter Parameters
  bmp.begin_I2C(ALTIMETER_I2C_ADDRESS);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Setup GPS Parameters
  beginLocationGpsAndGsmTriangulation();

  // Turn on second LDO regulator for microphone
  pinMode(ldoPin, OUTPUT);
  digitalWrite(ldoPin, HIGH);

  // Configurting Panic Button Interrupt
  pinMode(buttonPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(buttonPin), panicButton, RISING);

  // Configuring ADC Parameters
  adc1_config_width(ADC_WIDTH_BIT_13);
  adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_11); // ADC 1 channel 0 GPIO1

  // Configuring timer interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 125, true);
  timerAlarmEnable(timer);

  // Setup Display
  display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_I2C_ADDRESS);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();

  // Connect to WiFi
  WiFi.begin(ssid, password);
  connectWifi();

  // Connect to Server
  connectServer();

  //// Update display with connection status
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi Connected!");
  display.display();
}

void loop()
{
  if (debug)
  {
    I2CScanner();
  }
  connectWifi();
  connectServer();
  if (panicMode)
  {
    sendAudioData(transmitNow);
    float latitude = 0.0;
    float longitude = 0.0;
    float altitude = 0.0;
    float pressure = bmp.pressure;
    float temperature = bmp.temperature;
    adquireGPSData(latitude, longitude, altitude);
    // adquirePressureDataforAltitudeCalculation(pressure, temperature);
    char data[100];
    sprintf(data, "Latitude: %f, longitude: %f, altitude: %f,temperature: %f,pressure: %f", latitude, longitude, altitude, pressure, temperature);
    Serial.println(data);

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;

      // Create JSON message
      DynamicJsonDocument doc(1024);
      doc["latitude"] = latitude;
      doc["longitude"] = longitude;
      doc["altitiude"] = altitude;
      doc["pressure"] = pressure;
      doc["temperature"] = temperature;
      String message;
      serializeJson(doc, message);

      // Send message to server
      // client.write(message.c_str(), message.length());
      // Send JSON message to 'data' event on the server for a named socket 'data'
      client.printf("emit('data', '%s').to('data')\n", message.c_str());
    }
  }
  else
  {
    if (debug)
    {
      Serial.println("Panic Button is stand by mode");
    }
  }
}

void updateDisplay(void)
{
  display.clearDisplay();
  display.setCursor(0, 0);

  // Print GPS data
  // display.print("Latitude: ");
  // display.println(GPS.latitudeDegrees, 6);
  // display.print("Longitude: ");
  // display.println(GPS.longitudeDegrees, 6);
  // display.print("Altitude: ");
  // display.println(GPS.altitude / 100.0, 2);
  // display.println();
  // display.display();

  delay(2000);

  // Print altitude data
  display.print("Pressure: ");
  display.print(bmp.pressure);
  display.print(" m");
  display.println();
  display.print("Temperature: ");
  display.print(bmp.temperature);
  display.print(" C");
  display.println();

  // Update display
  display.display();
}

void sendAudioData(bool transmitFlag)
{
  if (transmitNow)
  {
    transmitNow = false;
    // client.write(transmitBuffer, sizeof(transmitBuffer));
    client.printf("emit('audio', '%s').to('audio')\n", transmitBuffer);
    client.flush();
  }
}

void I2CScanner(void)
{
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning for I2C devices ...");
  for (address = 0x01; address < 0x7f; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    }
    else if (error != 2)
    {
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found");
  }
}

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);         // to run critical code without being interrupted.
  int adcVal = adc1_get_raw(ADC1_CHANNEL_0); // reads the ADC value

  uint8_t value = map(adcVal, 0, 8192, 0, 255); // // mapping to 8 bits
  audioBuffer[bufferPointer] = value;           // storing the value
  bufferPointer++;

  // Action on buffer filling
  if (bufferPointer == AUDIO_BUFFER_MAX)
  {
    bufferPointer = 0;
    memcpy(transmitBuffer, audioBuffer, AUDIO_BUFFER_MAX); // transfers the buffer
    transmitNow = true;                                    //  flag for buffer transmission
  }
  portEXIT_CRITICAL_ISR(&timerMux); // priority in critical code
}

void panicButton(void)
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200)
  {
    panicMode = !panicMode;
    if (panicMode)
    {
      display.print("Panic Button Mode, sending data");
      if (debug)
      {
        Serial.print("Panic Button Pressed");
      }
    }
  }
  last_interrupt_time = interrupt_time;
}

void beginLocationGpsAndGsmTriangulation(void)
{
  fonaSerial->begin(4800, SERIAL_8N1, FONA_RX, FONA_TX);
  while (!fona.begin(*fonaSerial))
  {
    if (debug)
    {
      Serial.println(F("Couldn't find FONA"));
    }
  }
  if (debug)
  {
    Serial.println(F("FONA is OK"));
  }
}

void adquireGPSData(float &latitude, float &longitude, float &altitude)
{
  float lat, longi, speed_kph, heading, speed_mph, alt;

  // if you ask for an altitude reading, getGPS will return false if there isn't a 3D fix
  bool gps_success = fona.getGPS(&lat, &longi, &speed_kph, &heading, &alt);

  if (gps_success)
  {
    latitude = lat;
    longitude = longi;
    altitude = alt;

    if (debug)
    {
      Serial.print("GPS lat:");
      Serial.println(lat, 6);
      Serial.print("GPS long:");
      Serial.println(longi, 6);
      Serial.print("GPS altitude:");
      Serial.println(alt);
    }
  }
  else
  {
    if (debug)
    {
      Serial.println("Waiting for FONA GPS 3D fix...");
    }
    // Fona 3G doesnt have GPRSlocation
    if ((fona.type() == FONA3G_A) || (fona.type() == FONA3G_E))
      return;
    // Check for network, then GPRS
    if (debug)
    {
      Serial.println(F("Checking for Cell network..."));
    }
    if (fona.getNetworkStatus() == 1)
    {
      // network & GPRS? Great! Print out the GSM location to compare
      bool gsmloc_success = fona.getGSMLoc(&lat, &longi);

      if (gsmloc_success)
      {
        latitude = lat;
        longitude = longi;
        if (debug)
        {
          Serial.print("GSMLoc lat:");
          Serial.println(latitude, 6);
          Serial.print("GSMLoc long:");
          Serial.println(longitude, 6);
        }
      }
      else
      {
        if (debug)
        {
          Serial.println("GSM location failed...");
          Serial.println(F("Disabling GPRS"));
          fona.enableGPRS(false);
          Serial.println(F("Enabling GPRS"));
          if (!fona.enableGPRS(true))
          {
            Serial.println(F("Failed to turn GPRS on"));
          }
        }
      }
    }
  }
}

void adquirePressureDataforAltitudeCalculation(float &pressure, float &temperature)
{
  if (!bmp.performReading())
  {
    if (debug)
    {
      Serial.println("Failed to perform reading :(");
    }
    return;
  }
  temperature = bmp.temperature;
  pressure = bmp.pressure / 100;

  if (debug)
  {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPA");
  }
}

void connectWifi(void)
{
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    if (debug)
    {
      Serial.println("Connecting to WiFi");
    }
  }
  if (debug)
  {
    Serial.println("Connected to WiFi");
  }
}

void connectServer(void)
{
  if (!client.connected())
  {
    while (!client.connect(server, port))
    {
      if (debug)
      {
        Serial.println("Connecting to Server");
      }
    }
  }
  if (debug)
  {
    Serial.println("Connected to server");
  }
}