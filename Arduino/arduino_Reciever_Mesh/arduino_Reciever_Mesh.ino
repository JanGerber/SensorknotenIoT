//Libraries
#include <SPI.h>             
#include <RF24.h>            //Allgemeine Libary für NRF24L01+
#include <AESLib.h>          //Verschlüsselungsalgorithmen
#include "RF24Network.h"
#include "RF24Mesh.h"
//Include eeprom.h for AVR (Uno, Nano) etc. except ATTiny
#include <EEPROM.h>

//Konstanten und Variablen

  // NRF24L01+ Pinbelegung
  RF24 radio(9,10);
  RF24Network network(radio);
  RF24Mesh mesh(radio,network);
  
  //Verschlüsselungskey für AES
  uint8_t key[] = {'A',1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};


  //Übermittelte Daten eines Sensors
  struct sensorData{
    int id;
    float value;
    int unit;
    unsigned long timeId;
  };

  //Verschlüsselte Nachricht
  struct secureMessage{
    byte message [16];
  };

  uint32_t displayTimer = 0;

  
void setup() {
  Serial.begin(9600);  
  
  Serial.println("Reciever");

  //nRF24L01

    mesh.setNodeID(0);
    mesh.begin();
//    radio.setDataRate(RF24_250KBPS);              //250kbs
//    radio.setPALevel(RF24_PA_MAX);
//    radio.setChannel(90);
//    radio.setRetries(15,15);
//    radio.setCRCLength(RF24_CRC_16); //Cyclic redundancy check

}

void loop() {
    
    sensorData t_sensorData;
    secureMessage t_message;

    mesh.update();
    mesh.DHCP();

    if(network.available()){
      
    RF24NetworkHeader header;
    network.peek(header);
    
    switch(header.type){
      case 'M':
          network.read(header,&t_message,sizeof(t_message));
          t_sensorData = entschluessleData(t_message);
          ausgabeData(t_sensorData);
          break;
      default:
          network.read(header,0,0);
          Serial.println(header.type);
          break;
    }
}
    if(millis() - displayTimer > 5000){
    displayTimer = millis();
    Serial.println(" ");
    Serial.println(F("********Assigned Addresses********"));
     for(int i=0; i<mesh.addrListTop; i++){
       Serial.print("NodeID: ");
       Serial.print(mesh.addrList[i].nodeID);
       Serial.print(" RF24Network Address: 0");
       Serial.println(mesh.addrList[i].address,OCT);
     }
    Serial.println(F("**********************************"));
}
    
}


sensorData entschluessleData(secureMessage  message){
  sensorData data;
  aes128_dec_single(key, message.message);
  memcpy(&data, &message.message, sizeof(data)); 
  return data;
}

void ausgabeData(sensorData t_sensorData){
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










