#include <LoRaNode.h>
#include <Wire.h>
#include <QMC5883L.h>


#define DEBUG_ESP_PORT Serial
#ifdef DEBUG_ESP_PORT
#define DEBUG_MSG(...) DEBUG_ESP_PORT.printf( __VA_ARGS__ )
#else
#define DEBUG_MSG(...)
#endif

volatile bool displayNeedRefresh = false;

// -------------------------------------------------------
// NODE USER CONFIGURATION
// -------------------------------------------------------
// Node name displayed on the screen
const char* LORA_NODE_NAME = "NODE_01";
// node data transmission interval in ms
const int transmissionTimeInterval = 3000;
// node processing time interval
const int processingTimeInterval = 1000;


// -------------------------------------------------------
// NODE SPECIFIC USER CONFIGURATION
// -------------------------------------------------------

QMC5883L compass;
// reed swich management
const int reedSwitchPin = 13;
unsigned long lastChangeTime = 0;
unsigned long debounceDelay = 50;
bool reedSwitchState = false;
// mail avaialble ?
bool mail = false;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

/**
* LoRaNode Constructor. Empty
*/
LoRaNode::LoRaNode()
{

}

/**
* Get transmission time interval
* @return transmission time interval in ms. User defined parameter.
*/
int LoRaNode::GetTransmissionTimeInterval()
{
  return (transmissionTimeInterval);
}

/**
* Get processing time interval
* @return processing time interval in ms. User defined paramater.
*/
int LoRaNode::GetProcessingTimeInterval()
{
  return (processingTimeInterval);
}

/**
* Get Node Name
* @return the node name. User defined parameter
*/
char* LoRaNode::GetNodeName()
{
  return (char*)LORA_NODE_NAME;
}

/**
* Method invoke by the node to get the information to display.
* Lines 1, 2 and 3 are used upon Tx
* Lines 4, 5 and used upon Rx
* @param  lineNumber the line number where the messge will be displayed
* @return            the messsage to be displayed
*/
char* LoRaNode::GetLineToDisplay(byte lineNumber)
{
  String msg;
  switch(lineNumber)
  {
    // Tx Line 1
    case 1:
      msg = "*Heading ";
      msg += lastHeading;
    break;
    case 2:
      msg = "*Reed switch ";
      msg += (reedSwitchState == true) ? "ON " : "OFF";
    break;
    case 3:
      // user custom message or by default the number of message sent
      msg = "*TxCounter ";
      msg += TxCounter;
    break;
    case 4 :
      break;
    case 5:
      if (calibrating)
      {
        msg = "calibrating";
      }
      else
      {
        msg = "";
      }
    break;
    case 6:
      break;
    default:
      break;
  }
  return (char*) msg.c_str();
}

void IRAM_ATTR __ISR_reedSwitch()
{
  const boolean readState = !digitalRead(reedSwitchPin);
  // debounce interrupt
  if (readState != reedSwitchState)
  {
    if (millis() > lastChangeTime + debounceDelay)
    {
      portENTER_CRITICAL(&mux);
      reedSwitchState = readState;
      portEXIT_CRITICAL_ISR(&mux);
      //changeFlag = true;
      if (reedSwitchState)
      {
        DEBUG_MSG("Reed Active\n");
        reedSwitchState = true;
        // reed becomes active ... means that letters box has been open / close
        mail = true;
      }
      else
      {
        DEBUG_MSG("Reed Not Active\n");
        reedSwitchState = false;
      }
      displayNeedRefresh = true;
    }
    lastChangeTime = millis();
  }
}

/**
* Function invoked by the node right after its own setup (as per Arduino Setup function)
* To be used for applicative setup
*/
void LoRaNode::AppSetup()
{
  Wire.begin();
  compass.init();
  compass.setSamplingRate(50);
  pinMode(reedSwitchPin, INPUT_PULLUP);
  reedSwitchState = !digitalRead(reedSwitchPin);
  attachInterrupt(digitalPinToInterrupt(reedSwitchPin), __ISR_reedSwitch, CHANGE);
}

/**
* App processing of the node.
* Invoke every loop of the nodes before Rx and Tx
* One should benefit from using processingTimeInterval to avoid overloading the node
*/
void LoRaNode::AppProcessing()
{
  static int calibCounter = 0;

  int16_t x,y,z,t;
  compass.readRaw(&x,&y,&z,&t);
  DEBUG_MSG("x: %i",x);
  DEBUG_MSG("    y: %i",y);
  DEBUG_MSG("    z: %i",z);
  int heading = 0;
  if (calibrating)
  {
    DEBUG_MSG(" calibrating ... ");
    calibCounter++;
    heading = compass.readHeadingAndCalibrate();
    // if more than 20 calibration cycles, stop calibration
    if (calibCounter > 20)
    {
      calibCounter = 0;
      calibrating = false;
      compass.saveCalibrationSettings();
    }
  }
  else
  {
    DEBUG_MSG(" ... ");
    heading = compass.readHeading();
  }
  if ((heading >=340)or (heading <= 23)) { DEBUG_MSG (" * North\n");lastHeading="N"; }
  if ((heading >=24) and (heading <= 68)) { DEBUG_MSG (" * North-East\n");lastHeading="NE"; }
  if ((heading >=69) and (heading <= 113)) { DEBUG_MSG (" * East\n");lastHeading="E"; }
  if ((heading >=114)and (heading <= 158)) { DEBUG_MSG (" * South-East\n");lastHeading="SE"; }
  if ((heading >=159)and (heading <= 203)) { DEBUG_MSG (" * South\n");lastHeading="S"; }
  if ((heading >=204)and (heading <= 248)) { DEBUG_MSG (" * South-West\n");lastHeading="SW"; }
  if ((heading >=249)and (heading <= 293)) { DEBUG_MSG (" * West\n");lastHeading="W"; }
  if ((heading >=294)and (heading <= 339)) { DEBUG_MSG (" * North-West\n");lastHeading="NW"; }
  displayNeedRefresh = true;
}

bool LoRaNode::NeedDisplayUpdate()
{
  if (displayNeedRefresh)
  {
    displayNeedRefresh = false;
    return true;
  }
  return false;
}

/**
* Add JSON Tx payload messages
* @param payload the JSON payload to be completed as per application needs
*/
void LoRaNode::AddJSON_TxPayload(JsonDocument payload)
{
  payload["heading"] = lastHeading;
  portENTER_CRITICAL(&mux);
  payload["mail"] = mail;
  if (true == mail) { mail = false;} // mail notification sent. We cancel it.
  portEXIT_CRITICAL(&mux);
}

/**
* Parse JSON Rx payload
* One should avoid any long processing in this routine. LoraNode::AppProcessing is the one to be used for this purpose
* Limit the processing to parsing the payload and retrieving the expected attributes
* @param payload the JSON payload received by the node
*/
void LoRaNode::ParseJSON_RxPayload(JsonDocument payload)
{
  calibrating = payload["calibration"];
  compass.resetCalibration();
  return;
}


LoRaNode Node;
