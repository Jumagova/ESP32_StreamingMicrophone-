// #include <WiFi.h>
// #include <WiFiClientSecure.h>

// const char *ssid = "Coworking-Rosales";
// const char *password = "MoniPepe_4327";
// const char *server = "192.168.1.30";
// const int port = 8080;

// WiFiClient client;

// void setup()
// {
//   Serial.begin(9600);
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED)
//   {
//     delay(1000);
//     Serial.println("Connecting to WiFi...");
//   }
//   Serial.println("Connected to WiFi");
//   while (!client.connect(server, port))
//   {
//     Serial.println("Failed to connect to server");
//   }
//   Serial.println("Connected to server");
// }

// void loop()
// {
//   const int size = 2250;
//   static int16_t buffer[size];
//   for (int i = 0; i < size; i++)
//   {
//     // Sinusoidal Signal
//     //buffer[i] = 32767 * sin(400 * 2 * PI * i / size);
//     // Random Signal
//     buffer[i] = random(0, 32767);
//   }
//   // Send the data to the server
//   client.write((uint8_t *)buffer, size * 2);
//   client.flush();
//   // Wait for a moment before sending the next data
//   delay(500);
// }

#include <Wire.h>
#include <Arduino.h>
void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
}
 
void loop() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);          
}