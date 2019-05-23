#include <LoRaNode.h>
#include <Wire.h>
#include <QMC5883L.h>

#define DEBUG_ESP_PORT Serial
#ifdef DEBUG_ESP_PORT
#define DEBUG_MSG(...) DEBUG_ESP_PORT.printf( __VA_ARGS__ )
#else
#define DEBUG_MSG(...)
#endif


// -------------------------------------------------------
// NODE USER CONFIGURATION
// -------------------------------------------------------
// Node name displayed on the screen
const char* LORA_NODE_NAME = "NODE_01";
// node data transmission interval in ms
const int transmissionTimeInterval = 5000;
// node processing time interval
const int processingTimeInterval = 2000;


// -------------------------------------------------------
// NODE SPECIFIC USER CONFIGURATION
// -------------------------------------------------------

QMC5883L compass;


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
    // Tx Line 2
    case 2:
    break;
    // Tx Line 3
    case 3:
      // user custom message or by default the number of message sent
      msg = "*TxCounter ";
      msg += TxCounter;
    break;
    // Rx Line 1
    case 4 :
      msg = ">To ";
      msg += LORA_NODE_NAME;
      break;
    // Rx Line 2
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
    default:
    break;
  }
  return (char*) msg.c_str();
}

/**
* Function invoked by the node right after its own setup (as per Arduino Setup function)
* To be used for applicative setup
*/
void LoRaNode::AppSetup()
{
  Wire.begin();
  compass.init();
  // compass.resetCalibration();
  compass.setSamplingRate(50);
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
  // int heading = atan2(x, y)/0.0174532925; //Calculate the degree using X and Y parameters with this formulae
  // //Convert result into 0 to 360
  // if(heading < 0)
  //  heading+=360;
  // heading = 360-heading;
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
  if ((heading >=249)and (heading <= 293)) { DEBUG_MSG (" * West\n");lastHeading="O"; }
  if ((heading >=294)and (heading <= 339)) { DEBUG_MSG (" * North-West\n");lastHeading="NW"; }
}

/**
* Add JSON Tx payload messages
* @param payload the JSON payload to be completed as per application needs
*/
void LoRaNode::AddJSON_TxPayload(JsonDocument payload)
{
  payload["pulse_counter"] = TxCounter;
  payload["heading"] = lastHeading;
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
