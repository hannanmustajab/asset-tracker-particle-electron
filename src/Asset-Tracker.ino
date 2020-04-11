/*
 * Project Asset-Tracker
 * Description:
 * Author: Abdul Hannan Mustajab
 * Date: 7th April 2020
 */

// v1.00 - Libraries setup and tested sleeping


const char releaseNumber[6] = "1.00";                                              // Displays the release on the menu

// Include libraries
#include "AssetTracker.h"
#include "adafruit-sht31.h"                                                       //Include SHT-31 Library
#include <Adafruit_GPS.h>


// Initialize modules here
Adafruit_SHT31 sht31 = Adafruit_SHT31();                                           // Initialize sensor object


// Global objects
FuelGauge batteryMonitor;
AssetTracker t = AssetTracker();

SYSTEM_THREAD(ENABLED);


// State Machine Variables
enum State
{
  INITIALIZATION_STATE,
  ONLINE_WAIT_STATE,
  TEMPERATURE_SENSING,
  REPORTING_STATE,
  RESPONSE_WAIT,
  ERROR_STATE,
  NAPPING_STATE
};

// These are the allowed states in the main loop
char stateNames[7][44] = {"Initial State", "Online Waiting", "Temp. Sensing", "Reporting", "Response Wait", "Error Wait", "Napping State"};
State state = INITIALIZATION_STATE;                                               // Initialize the state variable
State oldState = INITIALIZATION_STATE;                                            //Initialize the oldState Variable

// Variables Related To Particle Mobile Application Reporting
char signalString[16];                                                            // Used to communicate Wireless RSSI and Description
char temperatureString[16];                                                       // Temperature string for Reporting
char batteryString[16];                                                           // Battery value for reporting.
char speedString[16];                                                             // Humidity String for reporting.


// Variables releated to the sensors
bool verboseMode = true;                                                         // Variable VerboseMode.
float temperatureInC = 0;                                                         // Current Temp Reading global variable
float voltage;                                                                    // Voltage level of the LiPo battery - 3.6-4.2V range
float TemperatureInC = 0;                                                         // last sent temperature reading
float temperatureThresholdValue=40;                                               // Threshold temperature value
int awake = 0;
bool inTransit;
unsigned long resetStartTimeStamp = 0;                                            // Start the clock on Reset
const int resetDelayTime = 30000;                                                 // How long to we wait before we reset in the error state

// Variables related to timings 
const unsigned long TIME_PUBLISH_BATTERY_SEC = 4 * 60 * 60;                       // every 4 hours, send a battery update
const uint8_t movementThreshold = 16;
unsigned long stateTime = 0;
const unsigned long WAIT_FOR_GPS_FIX = 4000 * 10 * 10 ; // After publish, wait 4 seconds for data to go out



void setup() {
  
  // Particle Variables
  Particle.variable("Speed",speedString);                                         // Check the humidity from particle console. 
  Particle.variable("Temperature", temperatureString);                            // Setup Particle Variable
  Particle.variable("Release", releaseNumber);                                    // So we can see what release is running from the console
  Particle.variable("Signal", signalString);                                      // Particle variables that enable monitoring using the mobile app
  Particle.variable("Battery", batteryString);                                    // Battery level in V as the Argon does not have a fuel cell

  t.begin();                                                                      // Start the tracker 
  t.gpsOn();
  if (! sht31.begin(0x44)) {                                                      // *** This has to be above takemeasurements() Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
  }
  
  state = ONLINE_WAIT_STATE;                                                    // Set the state to IDLE state 
}

void loop() {

  switch (state)  {                                                                // In the main loop, all code execution must take place in a defined state
    
    case ONLINE_WAIT_STATE:
      
      if (verboseMode && oldState != state) transitionState();
      
      if (Particle.connected()) 
      {
      t.updateGPS();
			state = TEMPERATURE_SENSING;
		  }
		  if (millis() - stateTime > 5000) {
			  stateTime = millis();
        waitUntil(PublishDelayFunction);
        if (verboseMode) Particle.publish("Status","Waiting to come online",PRIVATE);
		  }

    break;

    case TEMPERATURE_SENSING:
      
      if (verboseMode && oldState != state) transitionState(); 
      // t.updateGPS();
      // TemperatureInC = sht31.readTemperature();
      // snprintf(temperatureString,sizeof(temperatureString), "%4.1fC",temperatureInC);
      // if (temperatureInC > temperatureThresholdValue){
      //   waitUntil(PublishDelayFunction);
      //   Particle.publish("Alert","Temperature Above Threshold",PRIVATE);
      //   state = REPORTING_STATE;
      //   break;
      // }

      if (!t.setupLowPowerWakeMode(movementThreshold)) {
      Particle.publish("Alert","accelerometer not found",PRIVATE);
			state = NAPPING_STATE;
			break;
		  }

      state = REPORTING_STATE;

    break;


    case REPORTING_STATE:
      if (verboseMode && oldState != state) transitionState();  
      Particle.publish("reporting state","syncing clock",PRIVATE);
      if (Time.hour() == 12) Particle.syncTime();                                 // SET CLOCK EACH DAY AT 12 NOON. 
      Particle.publish("Getting GPS FIX",String(t.gpsFix()),PRIVATE);
      while (!t.gpsFix()){
        t.gpsFix();
      }
       Particle.publish("GPS STATUS",String(t.gpsFix()),PRIVATE);
      if (t.gpsFix()){
        sendUBIDots();
        state = RESPONSE_WAIT;
      } 
      else {
        waitUntil(PublishDelayFunction);
        Particle.publish("Alert","GPS NOT FIXED",PRIVATE);
        break;
      }
      
     
    break;

    case RESPONSE_WAIT:
      if (verboseMode && oldState != state) transitionState();   
      if (!inTransit) {
        state = NAPPING_STATE;                                                       // This checks for the response from UBIDOTS. 
        if (!verboseMode) {                                                       // Abbreviated messaging for non-verbose mode
          waitUntil(PublishDelayFunction);
          Particle.publish("State", "Data Sent / Response Received", PRIVATE);    // Lets everyone know data was send successfully
        }
      }   
    break;

    case ERROR_STATE:
      if (verboseMode && oldState != state) transitionState();                    // If verboseMode is on and state is changed, Then publish the state transition.
      if (millis() - resetStartTimeStamp >= resetDelayTime) {
        waitUntil(PublishDelayFunction);
        Particle.publish("Error", "Resetting in 30 seconds", PRIVATE);            // Reset the device and hope that fixes it
        delay(2000);                                                              // Get the message out before resetting
        System.reset();
      }
      
    break;

    case NAPPING_STATE:
      
      if (verboseMode && oldState != state) transitionState();     
      System.sleep(WKP, RISING, TIME_PUBLISH_BATTERY_SEC, SLEEP_NETWORK_STANDBY);
		  awake = ((t.clearAccelInterrupt() & LIS3DH_INT1_SRC_IA) != 0);
      waitUntil(PublishDelayFunction);
		  Particle.publish("WokeUP",String(awake),PRIVATE);
      t.updateGPS();
      state = REPORTING_STATE;

    break;

  }

}

// Function to create a delay in the publish time
bool PublishDelayFunction() {
  static unsigned long tstamp = 0;
  if (millis() - tstamp <= 1000)                                                  // Particle limits webhooks and publishes to once every second
    return 0;
  else {
    tstamp = millis();
    return 1;
  }
}

bool SetVerboseMode(String command) {                                             // Function to Toggle VerboseMode.
  if (command == "1" && verboseMode == false)
  {
    verboseMode = true;
    waitUntil(PublishDelayFunction);
    Particle.publish("Mode", "Verbose Mode Started.", PRIVATE);
    return 1;
  }

  if (command == "1" && verboseMode == true)
  {
    waitUntil(PublishDelayFunction);
    Particle.publish("Mode", "Verbose Mode Already ON.", PRIVATE);
    return 0;
  }

  if (command == "0" && verboseMode == true)
  {
    verboseMode = false;
    waitUntil(PublishDelayFunction);
    Particle.publish("Mode", "Verbose Mode Stopped.", PRIVATE);
    return 1;
  }

  if (command == "0" && verboseMode == false)
  {
    waitUntil(PublishDelayFunction);
    Particle.publish("Mode", "Verbose Mode already OFF.", PRIVATE);
    return 0;
  }
  else return 0;
}

void transitionState(void) {                                                      // This function publishes change of state.
  waitUntil(PublishDelayFunction);
  char stateTransitionString[64];                                                 // Declare a String to show state transition.
  snprintf(stateTransitionString, sizeof(stateTransitionString), "Transition: %s to %s", stateNames[oldState], stateNames[state]);
  oldState = state;
  Particle.publish("State", stateTransitionString, PRIVATE);
}


void sendUBIDots()                                                                // Function that sends the JSON payload to Ubidots
{
  t.updateGPS();
  if (t.gpsFix()){
    
    char data[512];
    Particle.publish("Air-Quality-Hook", "Entered Send UbiDots function", PRIVATE);
    snprintf(data, sizeof(data), "{\"position\": {\"value\":1, \"context\":{\"lat\": \"%f\", \"lng\": \"%f\"}}}", t.readLat(), t.readLon());
    Particle.publish("assest-tracker-webhook", data, PRIVATE);
    waitUntil(PublishDelayFunction);                                  // Space out the sends
    inTransit = true;

  }
  
}

void UbidotsHandler(const char *event, const char *data)                          // Looks at the response from Ubidots - Will reset Photon if no successful response
{
  // Response Template: "{{hourly.0.status_code}}"
  if (!data) {                                                                    // First check to see if there is any data
    if (verboseMode) {
      waitUntil(PublishDelayFunction);
      Particle.publish("Ubidots Hook", "No Data", PRIVATE);
    }
    return;
  }
  int responseCode = atoi(data);                                                  // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    if (verboseMode) {
      waitUntil(PublishDelayFunction);
      Particle.publish("State", "Response Received", PRIVATE);
      
    }
    inTransit = false;    
  }
  else if (verboseMode) {
    waitUntil(PublishDelayFunction);      
    Particle.publish("Ubidots Hook", data, PRIVATE);                              // Publish the response code
  }
}