
//Libraries
#include <DHT.h>       //Temperatur und Luftfeuchtigkeit
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <FastCRC.h>

//Konstanten und Variablen

  //ID des Arduinos
  const int arduinoId = 400;
  

  //DHT22
  #define DHTPIN 2     // what pin we're connected to
  #define DHTTYPE DHT22   // DHT 22  (AM2302)
  DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
  float humidity;
  float temperatur;
    
  //BPM 180
  Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
  int pressure;

  // NRF24L01
  RF24 radio(9,10);
  // Example below using pipe5 for writing
  const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0x7365727631LL };
   
  char receivePayload[32];
  uint8_t counter=0;

  //Checksum

  FastCRC16 CRC16;

  //
  struct sensorData{
    int id;
    float value;
    int unit;
    int crc;
  };

  struct crcData{
    int id;
    float value;
    int unit;
  }; 

void setup() {
  Serial.begin(9600);  
  
  //DHT22
  dht.begin();

  //BMP 180
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    Serial.println("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
  }

  //nRF24L01
  radio.begin();
  radio.setPayloadSize(sizeof(sensorData));  //Groesse der gesendeten Daten
  radio.setAutoAck(1); 
  radio.setDataRate(RF24_250KBPS); //250kbs
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(70);
  radio.setRetries(15,15);
  radio.setCRCLength(RF24_CRC_8);
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);

}

void loop() {
   getTemperatureHumidty();
   getPressure();
   getLuminosity();  
   sendOverRadio();
   serielleAusgabe();
   delay(10000);
}

void getTemperatureHumidty(){  
  humidity = dht.readHumidity(); 
  temperatur = dht.readTemperature();  
}

void getPressure(){
  sensors_event_t event;
  bmp.getEvent(&event);
  pressure = event.pressure; 
}

void getLuminosity(){

}

void sendData(sensorData t_sensorData) {
    
  for(int retry = 0; retry <= 50; retry++){
   if (radio.write( &t_sensorData, sizeof(t_sensorData) )){
       Serial.print("Break:");
       Serial.println(t_sensorData.unit);
       break; 
   }
   Serial.println(F("failed.")); 
}

}

void sendOverRadio(){
 radio.powerUp(); 
 
  sensorData t_sensorData;
  t_sensorData.id = arduinoId;
  
  if(!isnan(temperatur)){
    t_sensorData.value = temperatur;
    t_sensorData.unit = 1;
    t_sensorData.crc = calcCRC(t_sensorData);
    sendData(t_sensorData);
  }
  delay(8);
  if(!isnan(humidity)){
    t_sensorData.value = humidity;
    t_sensorData.unit = 2;
    t_sensorData.crc = calcCRC(t_sensorData);
    sendData(t_sensorData);
  }
  delay(8);
  if(!isnan(pressure)){
    t_sensorData.value = pressure;
    t_sensorData.unit = 3;
    t_sensorData.crc = calcCRC(t_sensorData);
    sendData(t_sensorData);
  }
  radio.powerDown();
}



void serielleAusgabe(){
    Serial.print("Luftfeuchte: ");
    Serial.print(humidity);
    Serial.print(" %\t");
    Serial.print("Temperatur: ");
    Serial.print(temperatur);
    Serial.print(" C\t");
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPa");
}

int calcCRC(sensorData t_sensorData){
  crcData t_crcData;

  t_crcData.id = t_sensorData.id;
  t_crcData.value = t_sensorData.value;
  t_crcData.unit = t_sensorData.unit;

  return CRC16.ccitt((uint8_t)t_crcData, sizeof(t_crcData));
}









