
//Libraries
#include <DHT.h>       //Temperatur und Luftfeuchtigkeit
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LowPower.h>
#include <EEPROM.h>
#include <AES.h>

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
  unsigned long timeId;
  long addressTimeId;

  //Verschluesselung AES
  AES aes;
  byte *key = (unsigned char*)"0123456789010123";
  unsigned long long int my_iv = 36753562;
  byte plain[] = "Add NodeAdd NodeAdd NodeAdd NodeAdd Node";
  byte cipher [48] ;
  
  
  //
  struct sensorData{
    int id;
    float value;
    int unit;
    unsigned long timeId;
  };

  struct secureMessage{
    byte message [sizeof(sensorData) + (N_BLOCK - (sizeof(sensorData) % 16)) - 1];
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

  Serial.println("init Radio");
  //nRF24L01
  radio.begin();
  radio.setPayloadSize(sizeof(secureMessage));  //Groesse der gesendeten Daten
  radio.setAutoAck(1); 
  radio.setDataRate(RF24_250KBPS); //250kbs
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(90);
  radio.setRetries(15,15);
  radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);

  Serial.println(radio.getChannel());

  addressTimeId = 1;
  timeId = EEPROMReadlong(addressTimeId);
}

void loop() {

  long start;
  long dauer;


   start = millis();
   getTemperatureHumidty();
   getPressure();
   getLuminosity();  
   sendOverRadio();

   dauer = millis() - start;
   Serial.print("Dauer: ");
   Serial.print(dauer);
   Serial.println(" ms");
   serielleAusgabe();
   delay(100);

   for(int i = 0; i < 8; i++)
   {
     LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
   }
   
   
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
  secureMessage message;
  message = verschluessleData(t_sensorData);

  int attempts  = 0;
  for(int retry = 0; retry <= 60; retry++){
   if (radio.write( &message, sizeof(message) )){
       Serial.print("Break:");
       Serial.print(t_sensorData.unit);
       break; 
   }
   delayMicroseconds(130);
   attempts++;
}
  Serial.print(" \tAttempts: ");
  Serial.println(attempts);
  
}

void sendOverRadio(){
 radio.powerUp(); 
  
  
  sensorData t_sensorData;
  timeId++;
  EEPROMWritelong(addressTimeId, timeId);
  t_sensorData.timeId = timeId;
  t_sensorData.id = arduinoId;
  
  if(!isnan(temperatur)){
    t_sensorData.value = temperatur;
    t_sensorData.unit = 1;
    sendData(t_sensorData);
  }
  /*
  delay(3);
  if(!isnan(humidity)){
    t_sensorData.value = humidity;
    t_sensorData.unit = 2;
    sendData(t_sensorData);
  }
  delay(3);
  if(!isnan(pressure)){
    t_sensorData.value = pressure;
    t_sensorData.unit = 3;
    sendData(t_sensorData);
  }*/
  radio.powerDown();
}

void EEPROMWritelong(int address, long value){
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
}

long EEPROMReadlong(long address)
{
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

secureMessage verschluessleData(sensorData  data){
  secureMessage message;
  byte iv [N_BLOCK] ;
  byte plain[sizeof(data)];
  byte cipher[sizeof(data) + (N_BLOCK - (sizeof(plain) % 16)) - 1]; 
  aes.set_IV(my_iv);
  aes.get_IV(iv);
  memcpy(&plain, &data, sizeof(data));  
  
  Serial.print("Plain: ");
  for(int i = 0; i < sizeof(plain);i++){
    Serial.print(plain[i],HEX);
  }
  Serial.println();
  aes.do_aes_encrypt(plain ,sizeof(plain),message.message,key,128,iv);  
  Serial.print("Message.message: ");
  for(int i = 0; i < sizeof(message.message);i++){
    Serial.print(message.message[i],HEX);
  }
  Serial.println();
  return message;
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
    Serial.print(" hPa\t");
    Serial.print(" TimeId: ");
    Serial.println(timeId);
    
}


void ausgabeSensorData(sensorData t_sensorData){
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












