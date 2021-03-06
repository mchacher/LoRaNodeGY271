#ifndef LORANODE_H
#define LORANODE_H

#include <ArduinoJson.h>
#include <Arduino.h>


class LoRaNode
{
  public:
    LoRaNode();
    void AppSetup();
    void AppProcessing();
    void AddJSON_TxPayload(JsonDocument payload);
    void ParseJSON_RxPayload(JsonDocument payload);
    char* GetNodeName();
    char* GetLineToDisplay(byte lineNumber);
    int GetTransmissionTimeInterval();
    int GetProcessingTimeInterval();
    bool NeedDisplayUpdate();
  public:
    int TxCounter = 0;

  private:
    String lastHeading;
    bool calibrating = false;

};

extern LoRaNode Node;

#endif
