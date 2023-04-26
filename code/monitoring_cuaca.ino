#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "Ubidots.h"
#include "DHT.h"
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define TOKEN "BBFF-wx4ALX6jzIrLGTgZAglFVKDq0Tq320"     // Your Ubidots TOKEN
#define led D3
#define RX_GPS D5
#define TX_GPS D6  
#define S_cahaya A0
#define S_hujan D0
#define pin_tombol D8
#define DHTPIN D7
#define DHTTYPE DHT11  
#define hujan 1
#define terang 0
#define ditekan 1

const char* DEVICE_TYPE = "monitoring_udara";     // Edit here your device type label
char ssid[50];
char password[50];
float k_Altitude = 1013.25;
float temp,kelembapan,Altitude;
float longtitude,latitude;
boolean gps_valid = false;
int val_cahaya,val_hujan,kondisi_tombol,preassure;
String kalimat,st_cuaca;
boolean kondisi_reset = false;
int waktu_sebelumnya = 0;
unsigned long waktu_sekarang;
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial Gps_Serial(RX_GPS, TX_GPS);
Adafruit_BMP280 bmp;
WiFiManager wifiManager;
Ubidots ubidots(TOKEN, UBI_TCP);
TinyGPSPlus gps;
DHT dht(DHTPIN, DHTTYPE);

void inisial_ota() {
 ArduinoOTA.setHostname("myesp8266");
 ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
    Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


void setup()
{
  Serial.begin(115200);
  Gps_Serial.begin(9600);
  lcd.begin();
  dht.begin();
  pinMode(pin_tombol, INPUT_PULLUP);
  pinMode(S_hujan, INPUT);
  pinMode(S_hujan, INPUT);
  pinMode(led, OUTPUT);
  WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str());
  String s = WiFi.SSID().c_str();
  String p = WiFi.psk().c_str();
  s.toCharArray(ssid, 50);
  p.toCharArray(password, 50);
  inisial_ota();
  kondisi_tombol = digitalRead(pin_tombol);
  if(kondisi_tombol == ditekan && kondisi_reset == false)
  {
    kondisi_reset = true;
    delay(200);
  }
    while(kondisi_reset == true)
    {
      ArduinoOTA.handle();
      digitalWrite(led,LOW);
      delay(500);
      digitalWrite(led,HIGH);
      waktu_sebelumnya = waktu_sekarang;
      lcd.setCursor(0,0);
      lcd.print("MODE PROGRAM");
    Serial.println("MODE PROGRAM");
  }
  if (!wifiManager.autoConnect("Monitoring Cuaca", "12345678")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(3000);
  }
  ubidots.wifiConnect(ssid, password);
  ubidots.setDeviceType(DEVICE_TYPE);
  if (!bmp.begin(0x76))
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

}

void loop()
{
  waktu_sekarang = millis();
  ArduinoOTA.handle();
  bacasensor();
  bacagps();
  tampillcd();
  tampilserial();
  digitalWrite(led,LOW);
  if (waktu_sekarang - waktu_sebelumnya >= 10000) 
  {
    digitalWrite(led,HIGH);
    kirimdata();
    waktu_sebelumnya = waktu_sekarang;
  }
  if(digitalRead(pin_tombol) == ditekan && kondisi_reset == false)
  {
    ESP.reset();
  }
}

void bacasensor()
{
  temp = bmp.readTemperature();
  preassure = bmp.readPressure()/100;
  Altitude = bmp.readAltitude(k_Altitude);
  kelembapan = dht.readHumidity();
  int c  = analogRead(S_cahaya)/1023;
  val_hujan = digitalRead(S_hujan);
  if((c > 1000 && val_cahaya < 1000) || (c < 1000 && val_cahaya > 1000)){
    Serial.println("clear");
    lcd.clear();
  }
  val_cahaya = c;
  Serial.println(c);
  
}

void bacagps()
{
 while (Gps_Serial.available() > 0)
  {
    if (gps.encode(Gps_Serial.read()))
    {
       if (gps.location.isValid())
       {
        gps_valid = true;
         latitude = gps.location.lat(),8;
         longtitude = gps.location.lng(),8;
       }else{ gps_valid = false;}
    }
  }
}
void tampillcd()
{
  lcd.setCursor(0,0);
  lcd.print("T=");
  lcd.print(temp);
  lcd.print((char) 223);
  lcd.print("C");

  lcd.setCursor(10,0);
  lcd.print("K=");
  lcd.print(kelembapan);

  lcd.setCursor(0,1);
  lcd.print("P=");
  lcd.print(preassure,0);
  lcd.print("hPa");

  lcd.setCursor(10,1);
  lcd.print("C=");
  lcd.print(val_cahaya);
}


void tampilserial()
{
  kalimat = "GPS VALID = ";
  kalimat += gps_valid;
  kalimat += " || ";
  kalimat += "latitude = ";
  kalimat += latitude,6;
  kalimat += " || ";
  kalimat += "longtitude = ";
  kalimat += longtitude,6;
  kalimat += " || ";
  kalimat += "Temp = ";
  kalimat += temp;
  kalimat += "c";
  kalimat += " || ";
  kalimat += "kelembapan = ";
  kalimat += kelembapan;
  kalimat += " || ";
  kalimat += "Altitude = ";
  kalimat += Altitude;
  kalimat += " || ";
  kalimat += "preassure = ";
  kalimat += preassure;
  kalimat += "hPa";
  kalimat += " || ";
  kalimat += "cahaya = ";
  kalimat += val_cahaya;
  kalimat += " || ";
  kalimat += "sensor hujan = ";
  kalimat += val_hujan;
  Serial.println(kalimat);
}

void kirimdata()
{
  char* str_lat = (char*)malloc(sizeof(char) * 10);
  char* str_lng = (char*)malloc(sizeof(char) * 10);
  sprintf(str_lat, "%f", latitude);
  sprintf(str_lng, "%f", longtitude);
    char* context = (char*)malloc(sizeof(char) * 30);
 if( gps_valid == true )
 {
  ubidots.addContext("lat", str_lat);
  ubidots.addContext("lng", str_lng);
  ubidots.getContext(context);
  ubidots.add("mapss", 11, context);  // Change for your variable name
 }
  ubidots.add("Sensor Cahaya", val_cahaya);
  ubidots.add("Sensor Hujan", val_hujan);
  ubidots.add("temperature", temp);
  ubidots.add("kelembapan", kelembapan);
  ubidots.add("Altitude", Altitude);
  ubidots.add("preassure", preassure);
  bool bufferSent = false;
  bufferSent = ubidots.send();  // Will send data to a device label that matches the device ID
  if (bufferSent) {
    // Do something if values were sent properly
    Serial.println("Values sent by the device");
  }
    free(context);

}
