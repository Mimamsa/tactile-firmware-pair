/**
 * @file tactile_espnow_single_sender.ino
 *
 * @mainpage Adafruit ESP32 feather tactile data sender through ESP-now protocol
 * 
 * @section description Description
 * Adafruit ESP32 feather tactile data sender through ESP-now protocol.
 *
 * @section circuit Circuit
 * - MIT driver board with Adafruit ESP32 feather (2021 schematic)
 * - Could also be applied to MIT driver board with Adafruit ESP32 feather + conversion board.
 *
 * @section libraries Libraries
 * - ESP32 library for Arduino IDE
 *
 * @section note Note
 * - Should be paired with tactile_espnow_single_receiver.
 * - Check this tutorial to install ESP32 library for Arduino IDE : https://microcontrollerslab.com/install-esp32-arduino-ide/
 *
 * @section todo TODO
 * - None 
 *
 * @section author Author
 * - Created by Mike Foshey (mfoshey@mit.edu)
 * - Modified by Chao Liu (cliu21@mit.edu) on 09/10/2022
 * - Modified by Yu-Hsien Chen (illusion.dark@gmail.com) on 10/06/2022
 */
#include <esp_now.h>
#include <WiFi.h>
#include <driver/adc.h>

#define CHANNEL 1
#define START_X 0
#define END_X 23
#define START_Y 0
#define END_Y 7

// REPLACE WITH YOUR RECEIVER MAC Address
//uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0x5F, 0x37, 0x18};  // left
uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0x6B, 0x68, 0xD8};  // right

typedef struct struct_message {
  int8_t PacketIndex;
  uint16_t PressureArray[END_X-START_X+1];
} struct_message;
struct_message tactile_data;

/* For MIT driver boards with ESP32 feather */
//const int selectRowPins[5] = {A0, A1, A5, 12, 13};
/* For MIT driver boards with ESP32 feather + conversion board*/
const int selectRowPins[5] = {A0, A1, A5, 21, 12};
  
const int selectReadPins[5] = {27, 33, 15, 32, 14};
// const int ledPin = 13;
uint16_t a0val = 0;

/**
 * Standard Arduino setup function 
 */
void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);
  
  // ESP-Now setup
  WiFi.mode(WIFI_STA);
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = CHANNEL;  
  peerInfo.encrypt = 0;

  // Add peer        
  esp_err_t addStatus = esp_now_add_peer(&peerInfo);
  if (addStatus == ESP_OK) {
    // Pair success
    Serial.println("Pair success");
  } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
    Serial.println("Peer list full");
  } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("Out of memory");
  } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
    Serial.println("Peer Exists");
  } else {
    Serial.println("Not sure what happened");
  }

  for (int i = 0; i < 5; i++) {
    pinMode(selectRowPins[i], OUTPUT);
    pinMode(selectReadPins[i], OUTPUT);
    digitalWrite(selectRowPins[i], LOW);
    digitalWrite(selectReadPins[i], LOW);
  }
  
  // pinMode(ledPin, OUTPUT);
  // pinMode(zInput, INPUT);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
}

/**
 * Standard Arduino loop function 
 */
void loop()
{
  //digitalWrite(ledPin, HIGH);
  scanArray();
  //delay(5);
}

/**
 * Scan tactile array
 */
void scanArray()
{
  for (int y = START_Y; y <= END_Y; y++) {
    tactile_data.PacketIndex = y;
    selectMuxPin(y);   
    for(int x = START_X; x <= END_X; x++) {
      selectReadSwitch(x);
      a0val = adc1_get_raw(ADC1_CHANNEL_6);
      //Serial.println(a0val);
      tactile_data.PressureArray[x] = a0val;
    }
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &tactile_data, sizeof(tactile_data));
//    if (result == ESP_OK) {
//      // Serial.println("Sent with success");
//      Serial.write(49);
//      Serial.write('\n');
//    }
//    else {
//      // Serial.println("Error sending the data");
//      Serial.write(50);
//      Serial.write('\n');
//    }
    delay(1);
  }  
}

/**
 * Control multiplexer by digital pins to read analog value. 
 * Digital pins: D (digital pins 0 to 7)
 *
 * @param pin   A 5-bits value presents 32 possible values.
 */
void selectMuxPin(byte pin)
{
  if (pin > 31) return; // Exit if pin is out of scope
      
  for (int i=0; i<5; i++)
  {
    if (pin & (1<<i))
      digitalWrite(selectRowPins[i], HIGH);
    else
      digitalWrite(selectRowPins[i], LOW);
  }
}

/**
 * Controls SPDT switches by digital pins to read analog value.
 * Digital pins: B (digital pin 8 to 13)
 *
 * @param pin   A 5-bits value presents 32 possible values.
 */
void selectReadSwitch(byte pin)
{
  if (pin > 31) return;  // Exit if pin is out of scope

  for (int i=0; i<5; i++)
  {
    if (pin & (1<<i))
      digitalWrite(selectReadPins[i], HIGH);
    else
      digitalWrite(selectReadPins[i], LOW);
  }
}