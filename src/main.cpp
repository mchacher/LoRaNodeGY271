#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <U8x8lib.h>
#include <ArduinoJson.h>
#include <LoRaNode.h>

#define DEBUG_ESP_PORT Serial
#ifdef DEBUG_ESP_PORT
#define DEBUG_MSG(...) DEBUG_ESP_PORT.printf( __VA_ARGS__ )
#else
#define DEBUG_MSG(...)
#endif

// -------------------------------------------------------
// LoRa HARDWARE CONFIGURATION
// -------------------------------------------------------
//define the pins used by the transceiver module
#define SS 18
#define RST 14
#define DIO0 26
// -------------------------------------------------------
// LoRa MODEM SETTINGS
// -------------------------------------------------------
// The sync word assures you don't get LoRa messages from other LoRa transceivers
// ranges from 0-0xFF - make sure that the node is using the same sync word
#define LORA_SYNC_WORD 0xB2
// frequency
// can be changed to 433E6, 915E6
#define LORA_FREQUENCY 866E6
// change the spreading factor of the radio.
// LoRa sends chirp signals, that is the signal frequency moves up or down, and the speed moved is roughly 2**spreading factor.
// Each step up in spreading factor doubles the time on air to transmit the same amount of data.
// Higher spreading factors are more resistant to local noise effects and will be read more reliably at the cost of lower data rate and more congestion.
// Supported values are between 7 and 12
#define LORA_SPREADING_FACTOR 7
// LoRa signal bandwidth
// Bandwidth is the frequency range of the chirp signal used to carry the baseband data.
// Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, and 250E3
#define LORA_SIGNAL_BANDWIDTH 125E3
// Coding rate of the radio
// LoRa modulation also adds a forward error correction (FEC) in every data transmission.
// This implementation is done by encoding 4-bit data with redundancies into 5-bit, 6-bit, 7-bit, or even 8-bit.
// Using this redundancy will allow the LoRa signal to endure short interferences.
// The Coding Rate (CR) value need to be adjusted according to conditions of the channel used for data transmission.
// If there are too many interference in the channel, then it’s recommended to increase the value of CR.
// However, the rise in CR value will also increase the duration for the transmission
// Supported values are between 5 and 8, these correspond to coding rates of 4/5 and 4/8. The coding rate numerator is fixed at 4
#define LORA_CODING_RATE_DENOMINATOR 5
// -------------------------------------------------------
// LoRa DATA MODEL CONFIGURATION
// -------------------------------------------------------
const char* L2M_NODE_NAME = "node";
#define LORA_MSG_MAX_SIZE 255

// the OLED used
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

// White LED management
#define LED_WHITE 25

// sampling management
long lastSendTime = 0;    // last send time
long lastProcessTime = 0; // last processing time


/**
* Set Node in Rx Mode with active invert IQ
* LoraWan principle to avoid node talking to each other
* This way a Gateway only reads messages from Nodes and never reads messages from other Gateway, and Node never reads messages from other Node.
*/
void LoRa_rxMode()
{
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

/**
* Set Node in Tx Mode with active invert IQ
* LoraWan principle to avoid node talking to each other
* This way a Gateway only reads messages from Nodes and never reads messages from other Gateway, and Node never reads messages from other Node.
*/
void LoRa_txMode()
{
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}



/**
* initialize LoRa communication with #define settings (pins, SD, bandwidth, coding rate, frequency, sync word)
* CRC is enabled
* set in Rx Mode by default
*/
void LoRa_initialize()
{
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE_DENOMINATOR);
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();

  while (!LoRa.begin(LORA_FREQUENCY)) {
    DEBUG_MSG(".\n");
    delay(500);
  }
  // set in rx mode.
  LoRa_rxMode();
}



void setup()
{
  //initialize Serial Monitor
  #ifdef DEBUG_ESP_PORT
  Serial.begin(115200);
  while (!Serial);
  #endif

  // initilize screen library
  SPI.begin(5, 19, 27, 18);
  u8x8.begin();
  //u8x8.setFont(u8x8_font_artossans8_r);
  u8x8.setFont(u8x8_font_5x7_f);
  u8x8.println(Node.GetNodeName());
  // initialize LoRa
  LoRa_initialize();

  pinMode(LED_WHITE, OUTPUT);

  // call node specific configuration (end user)
  Node.AppSetup();
}

uint16_t crc16_ccitt(char* data, unsigned int data_len) {
    uint16_t crc = 0xFFFF;

    if (data_len == 0)
        return 0;

    for (unsigned int i = 0; i < data_len; ++i) {
        uint16_t dbyte = data[i];
        crc ^= dbyte << 8;
        
        for (unsigned char j = 0; j < 8; ++j) {
            uint16_t mix = crc & 0x8000;
            crc = (crc << 1);
            if (mix)
                crc = crc ^ 0x1021;
        }
    }

    return crc;
}


/**
* [sendToLora2MQTTGateway description]
*/
void sendToLora2MQTTGateway()
{
  digitalWrite(LED_WHITE, HIGH);
  StaticJsonDocument<255> payload;
  // preparing JSON payload
  payload[L2M_NODE_NAME] = Node.GetNodeName();
  Node.AddJSON_TxPayload(payload);
  LoRa_txMode();
  LoRa.beginPacket();
  char TXBuffer[LORA_MSG_MAX_SIZE];
  unsigned int crc16;
  serializeJson(payload, TXBuffer);
  DEBUG_MSG("sendToLora2MQTTGateway: payload = %s\n", TXBuffer);
  crc16 = crc16_ccitt(TXBuffer, strlen(TXBuffer));
  // serialize json payload
  //serializeJson(payload, LoRa);
  LoRa.print(TXBuffer);
  // add crc after the json payload
  LoRa.write((uint8_t)(crc16 & 0xff));
  LoRa.write((uint8_t)((crc16 >> 8) & 0xff));
  DEBUG_MSG("sendToLora2MQTTGateway: CRC = %x\n", crc16);
  LoRa.endPacket();
  LoRa_rxMode();
  // increment TxCounter
  Node.TxCounter++;
  digitalWrite(LED_WHITE, LOW);
}

/**
* [receiveLoraMessage description]
*/
void receiveLoraMessage()
{
  //try to parse packet
  int packetSize = LoRa.parsePacket();
  // if any packet available
  if (packetSize)
  {
    // received a packet
    DEBUG_MSG("Packet received: %d\n", packetSize);

    // activate LED to show incoming message
    digitalWrite(LED_WHITE, HIGH);

    // parse JSON message
    StaticJsonDocument<255> payload;
    DeserializationError error = deserializeJson(payload, LoRa);
    // deserializeJson error
    if (error || (payload[L2M_NODE_NAME].isNull() == true))
    {
      DEBUG_MSG("deserializeJson error\n");
      //u8x8.drawString(0, 2, "Rx Error");
      while(LoRa.read() != -1){}; // flush Rx Buffer
      return;
    }
    // no error we can process the message
    else
    {
      String nodeInvoked = payload[L2M_NODE_NAME];
      // Am I the node invoked for this messages
      if (nodeInvoked.compareTo(Node.GetNodeName()) == 0)
      {
        // I am the one!
        DEBUG_MSG("-tonode %s\n", nodeInvoked.c_str());
        Node.ParseJSON_RxPayload(payload);
      }
    }
    //}
    digitalWrite(LED_WHITE, LOW);
  }
}

void refreshDisplay()
{
  // display line by line
  for (int i =2; i < 8; i++)
  {
    u8x8.clearLine(i);
    u8x8.drawString(0, i, Node.GetLineToDisplay(i-1));
  }
}

/**
* Main loop of the LoRa Node
* Constantly try to receive JSON LoRa message
* Every transmissionTimeInterval send JSON LoRa messages
*/
void loop() {
  if ( (millis() - lastProcessTime) > Node.GetProcessingTimeInterval() )
  {
    Node.AppProcessing();
    lastProcessTime = millis();
  }
  if ( (millis() - lastSendTime) > Node.GetTransmissionTimeInterval() )
  {
    sendToLora2MQTTGateway();
    lastSendTime = millis();            // timestamp the message
  }
  receiveLoraMessage();
  if (Node.NeedDisplayUpdate())
  {
    refreshDisplay();
  }
}
