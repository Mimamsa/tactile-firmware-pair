/**
 * @file tactile_espnow_single_receiver.ino
 *
 * @mainpage Adafruit ESP32 feather tactile data receiver through ESP-now protocol
 * 
 * @section description Description
 * Adafruit ESP32 feather tactile data receiver through ESP-now protocol.
 *
 * @section circuit Circuit
 * - No circuit, simply connected to laptop through micro USB interface.
 *
 * @section libraries Libraries
 * - Rosserial Arduino library (ros-noetic-rosserial, ros-noetic-rosserial-arduino)
 * - ESP32 library for Arduino IDE
 *
 * @section note Note
 * - Should be paired with tactile_espnow_single_sender.
 * - Check this tutorial to install Rosserial Arduino library: https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino IDE Setup/
 * - Check this tutorial to install ESP32 library for Arduino IDE : https://microcontrollerslab.com/install-esp32-arduino-ide/
 *
 * @section todo TODO
 * - None 
 *
 * @section author Author
 * - Created by Chao Liu (cliu21@mit.edu) on 09/10/2022
 * - Modified by Yu-Hsien Chen (illusion.dark@gmail.com) on 10/06/2022
 */
#include <esp_now.h>
#include <WiFi.h>

#include "ros_m.h"
#include <mocap_msgs/TactileArray.h>

//#define DEBUG
#define START_X 0
#define END_X 23
#define START_Y 0
#define END_Y 7
#define LEN_X END_X-START_X+1
#define LEN_Y END_Y-START_Y+1
#define SERIAL_RATE 2000000
#define ROS_TOPIC "tactile/right"

typedef struct struct_message {
  int8_t PacketIndex;
  uint16_t PressureArray[LEN_X];
} struct_message;
struct_message tactile_data;

// ROS serial communication
ros::NodeHandle nh;
String tactile_topic = ROS_TOPIC;
mocap_msgs::TactileArray tactile_data_ros;
ros::Publisher tactile_transmit(tactile_topic.c_str(), &tactile_data_ros);

#ifdef DEBUG
  void print_tactile_data(struct_message *td)
  {
    for(int i=0; i<LEN_X; i++) {
      Serial.print(td->PressureArray[i]);
      Serial.print("\n");
    }
  }

  void print_tactile_data_ros(mocap_msgs::TactileArray *td)
  {
    int pressed = 0;
    for(int i=0; i<LEN_X; i++) {
      //Serial.print(td->PressureArray[i]);
      //Serial.print("\n");
      if(td->PressureArray[i]>3400)
        pressed++;
    }
    if(pressed>3) {
      Serial.print("Pressed ");
      Serial.println(pressed);
    }
    //tactile_data.PacketIndex
  }

  struct_message* get_dummy_data(uint8_t PacketIndex)
  {
    struct_message *td;
    td = (struct_message*) malloc(sizeof(struct_message));
    td->PacketIndex = PacketIndex;
    for(int i=0; i<LEN_X; i++)
      td->PressureArray[i] = 3300;
    return td;
  }
#endif


/**
 * Callback function that will be executed when data is received
 *
 * @param mac            MAC address. e.g. {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
 * @param incomingData   Incoming data.
 * @param len            Length of incoming data.
 */
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  tactile_data_ros.stamp = nh.now();
  memcpy(&tactile_data, incomingData, sizeof(tactile_data));
  // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &tactile_data, sizeof(tactile_data));
  tactile_data_ros.PacketIndex = tactile_data.PacketIndex;
  memcpy(tactile_data_ros.PressureArray, &tactile_data.PressureArray[0], 32*sizeof(*tactile_data_ros.PressureArray));

#ifdef DEBUG
  // If DEBUG is defined, publish dummy data
  //struct_message *td = get_dummy_data(tactile_data.PacketIndex);
  //memcpy(tactile_data_ros.PressureArray, &td->PressureArray[0], 32*sizeof(*tactile_data_ros.PressureArray));
  
  // If DEBUG is defined, print tactile data
  //print_tactile_data(&tactile_data);
  //print_tactile_data(td);
  //print_tactile_data_ros(&tactile_data_ros);
#endif

  tactile_transmit.publish(&tactile_data_ros);
  nh.spinOnce();
}

/**
 * Standard Arduino setup function 
 */
void setup()
{
  // rosserial set up
  nh.getHardware()->setBaud(SERIAL_RATE);
  // Note: If `ESP32` is defined, nh.getHardware() would be class `ArduinoTcpHardware`.
  // An `ArduinoTcpHardware` means it communicates through WiFi instead of UART, so
  // that it doesn't have member function setBaud() (see ros.h).
  // 
  // A workaround is modify ros.h to prevent this happen.

  nh.initNode();
  nh.advertise(tactile_transmit);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() == ESP_OK) {
    // Serial.println("ESPNow Init Success");
    nh.loginfo("ESPNow Init Success");
  }
  else {
    // Serial.println("ESPNow Init Failed");
    nh.logwarn("ESPNow Init Failed. Retrying...");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

/**
 * Standard Arduino loop function 
 */
void loop()
{
}