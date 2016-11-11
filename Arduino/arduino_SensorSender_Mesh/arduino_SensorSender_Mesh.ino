
//Libraries
#include <DHT.h>       //Temperatur und Luftfeuchtigkeit
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
#include <LowPower.h>
#include <EEPROM.h>
#include <AESLib.h>
#include <math.h>

//Konstanten und Variablen

  #define nodeID 1
  //ID des Arduinos
  const int arduinoId = 400;
  

  //DHT22
  #define DHTPIN 4     // what pin we're connected to
  #define DHTTYPE DHT22   // DHT 22  (AM2302)
  DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
  float humidity;
  float temperatur;
    
  //BPM 180
  Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
  int pressure;

  // NRF24L01
  RF24 radio(9,10);
  RF24Network network(radio);
  RF24Mesh mesh(radio, network);
   
  char receivePayload[32];
  unsigned long timeId;
  long addressTimeId;

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
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    Serial.println("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
  }

  Serial.println("init Radio");
  //nRF24L01
  mesh.setNodeID(nodeID);
  mesh.begin();
//  radio.setDataRate(RF24_250KBPS); //250kbs
//  radio.setPALevel(RF24_PA_MAX);
//  radio.setChannel(90);
//  radio.setRetries(15,15);
//  radio.setCRCLength(RF24_CRC_16);

  Serial.println("init Radio");
  //Timestamp
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
   delay(2000);

//   radio.powerDown();
//   
//   for(int i = 0; i < 2; i++)
//   {
//     LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
//   }
//   radio.powerUp(); 
   
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
  Serial.println("send Data");
    // Send an 'M' type message containing the current millis()
    if (!mesh.write(&message, 'M', sizeof(message))) {

      // If a write fails, check connectivity to the mesh network
      if ( ! mesh.checkConnection() ) {
        //refresh the network address
        Serial.println("Renewing Address");
        mesh.renewAddress();
      } else {
        Serial.println("Send fail, Test OK");
      }
    } else {
      Serial.print("Send OK: ");
    }  
}

void sendOverRadio(){
  Serial.println("vor Mesh update");
  mesh.update();
  
  
  sensorData t_sensorData;
  timeId++;
  EEPROMWritelong(addressTimeId, timeId);
  t_sensorData.timeId = timeId;
  t_sensorData.id = arduinoId;
  
  if(!isnan(temperatur)){
    Serial.println("Temperatur is not null");
    t_sensorData.value = temperatur;
    t_sensorData.unit = 1;
    sendData(t_sensorData);
    temperatur = NAN;
  }
  delay(3);
  if(!isnan(humidity)){
    t_sensorData.value = humidity;
    t_sensorData.unit = 2;
    sendData(t_sensorData);
    humidity = NAN;
  }
  delay(3);
  if(!isnan(pressure)){
    t_sensorData.value = pressure;
    t_sensorData.unit = 3;
    sendData(t_sensorData);
    pressure = NAN;
  }
  
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
  memcpy(&message.message, &data, sizeof(data));  
  aes128_enc_single(key, message.message);
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












