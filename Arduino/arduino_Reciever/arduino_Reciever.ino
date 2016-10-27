



//Libraries
#include <DHT.h>       //Temperatur und Luftfeuchtigkeit
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <AESLib.h>

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
  
  //Verschluesselung AES
  uint8_t key[] = {'A',1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};


  //
  struct sensorData{
    int id;
    float value;
    int unit;
    unsigned long timeId;
  };
  
  struct secureMessage{
    byte message [16];
  };


void setup() {
  Serial.begin(9600);  
  
  //DHT22
  dht.begin();

  //BMP 180
  // Initialise the sensor 
  if(!bmp.begin())
  {
    Serial.println("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
  }

  //nRF24L01
  radio.begin();
  radio.setPayloadSize(sizeof(secureMessage));  //Groesse der gesendeten Daten
  radio.setAutoAck(1); 
  radio.setDataRate(RF24_250KBPS); //250kbs
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(90);
  radio.setRetries(15,15);
  radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(0,pipes[0]);

  Serial.println(radio.getChannel());

  radio.startListening();

}

void loop() {
    
    sensorData t_sensorData;
    secureMessage t_message;
    
    if( radio.available()){
                                                                    // Variable for the received timestamp
      while (radio.available()) {                                   // While there is data ready
        radio.read( &t_message, sizeof(t_message) ); 
        t_sensorData = entschluessleData(t_message);
        Serial.print("ID: ");
        Serial.print(t_sensorData.id);
        Serial.print(" \t");
        Serial.print("Value: ");
        Serial.print(t_sensorData.value);
        Serial.print(" \t");
        Serial.print("Unit: ");
        Serial.print(t_sensorData.unit);// Get the payload
        Serial.print(" \t");
        Serial.print("TimeId: ");
        Serial.println(t_sensorData.timeId);// Get the payload 
      }
      Serial.println("------------------------------------");
    }
    
    
}


sensorData entschluessleData(secureMessage  message){
  sensorData data;
  aes128_dec_single(key, message.message);
  memcpy(&data, &message.message, sizeof(data)); 
  return data;
}











