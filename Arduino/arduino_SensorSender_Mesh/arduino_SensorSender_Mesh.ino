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
#include <AESLib.h>
#include <math.h>
#include <Base64.h>

//Konstanten und Variablen

  //ID des Arduinos
  const unsigned int arduinoId = 600;
  

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
  // Example below using pipe5 for writing
  const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0x7365727631LL };
   
  char receivePayload[32];
  unsigned long timeId;
  long addressTimeId;

  unsigned int messageId;
  int addressMessageId;

  const int SIZEBLACKLIST = 50;
  unsigned long blacklist[SIZEBLACKLIST];
  int listPointer = 0;
  

  //Sensor Daten
  struct sensorData{
    unsigned  int id;
    float value;
    int unit;
    unsigned long timeId;
  };

  //Mesh Daten Paket
  struct dataPacket{
    unsigned int destinationAddr;
    unsigned int originAddr;
    unsigned int messageId;
    sensorData data; 
  };
  
void setup() {  
  Serial.begin(9600);  
  
  //DHT22
  dht.begin();


  Serial.println("init Radio");
  //nRF24L01
  radio.begin();
  radio.setPayloadSize(sizeof(dataPacket));  //Groesse der gesendeten Daten
  radio.setAutoAck(true); 
  radio.setDataRate(RF24_250KBPS); //250kbs
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(90);
  radio.setRetries(15,15);
  radio.setCRCLength(RF24_CRC_16);
  
  //EEPROM komplett loeschen
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
   }


  Serial.print(radio.getChannel());

  addressTimeId = 1;
  timeId = EEPROMReadlong(addressTimeId);

  Serial.print("timeId ID: \t");
  Serial.println(timeId);
  addressMessageId = 1;
  addressMessageId += sizeof(addressMessageId);
  messageId = (int) EEPROMReadInt(addressMessageId);
  Serial.print("Message ID: \t");
  Serial.println(messageId);
  
  getTemperatureHumidty();
  sendDataPacket(createSensorDataPacket(temperatur));
  radio.startListening();
    
}

void loop() {
    dataPacket t_DataPacket;


    while( radio.available()){
      radio.read( &t_DataPacket, sizeof(t_DataPacket) );
      Serial.print("Paket erhalten: \t");
      ausgabeSensorData(t_DataPacket.data);
      processData(t_DataPacket);     
   }

   
}

void getTemperatureHumidty(){  
  humidity = dht.readHumidity(); 
  temperatur = dht.readTemperature();  
}

dataPacket createSensorDataPacket(float value){

  //Sensor Daten uebernehmen
  sensorData t_sensorData;
  timeId++;
  EEPROMWritelong(addressTimeId, timeId);
  t_sensorData.timeId = timeId;
  t_sensorData.id = arduinoId;
  t_sensorData.value = value;

  //Packet erstellen
  dataPacket t_dataPacket;
  t_dataPacket.destinationAddr = 1;
  t_dataPacket.originAddr = arduinoId;
  messageId++;
  EEPROMWriteInt(addressMessageId, messageId);
  t_dataPacket.messageId = messageId;
  t_dataPacket.data = t_sensorData; 
  
  return t_dataPacket;
}

void sendDataPacket(dataPacket t_dataPacket){
  radio.stopListening();
  delay(500);
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);

  Serial.print("Paket senden: \t");
  ausgabeSensorData(t_dataPacket.data);
  
  for(int retry = 0; retry <= 20; retry++){
   if (radio.write( &t_dataPacket, sizeof(t_dataPacket) )){
    //break wenn alles Richtig gelaufen ist ggf.
   }
   delayMicroseconds(130);
  }  
  
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1,pipes[0]);
  delay(500);
  radio.startListening();
}

void processData(dataPacket t_dataPacket){
  if(!(t_dataPacket.destinationAddr == arduinoId)){

     unsigned long uniqueMessageId = (unsigned long) ((unsigned long) t_dataPacket.originAddr << 16) +  (unsigned long) t_dataPacket.messageId;
     
     if(!compareToList(uniqueMessageId)){
         insertInList(uniqueMessageId);
         sendDataPacket(t_dataPacket);
     }else{
        //Message verwefen
        Serial.println("Message verworfen");
     }    
  }else{
    //Arduino ist Empfaenger
    Serial.print("Arduino ist Empfaenger: \t");
    ausgabeSensorData(t_dataPacket.data);
  }
}

void decrementPoiter() {
    if(listPointer < SIZEBLACKLIST )
        listPointer++;
    else
        listPointer=0;       
}

boolean compareToList(unsigned long uniqueMessageId){
   for (int i=0; i<  (SIZEBLACKLIST - 1); i++) {
        if (uniqueMessageId==blacklist[i])
            return true;
    }
  return false;
  
}
void insertInList(unsigned long uniqueMessageId){
    blacklist[listPointer]= uniqueMessageId;
    decrementPoiter();
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
void EEPROMWriteInt(int address, int value){
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte two = (value & 0xFF);
      byte one = ((value >> 8) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, two);
      EEPROM.write(address + 1, one);
}

int EEPROMReadInt(int address)
{
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF);
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












