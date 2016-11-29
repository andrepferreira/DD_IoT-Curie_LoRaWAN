
/*******************************************************************************  
 *  Demo cabinet controller based on the Intel Curie with Hope RF95W radio
 *  
 *  Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * Change DEVADDR to a unique address!
 *
 * Do not forget to define the radio type correctly in config.h.
 * 
 * Code is used to read a number of sensor values and send them over a LoRaWAN
 * connection. Designed to run on the Intel Curie based Genuino as it uses
 * the built in gyro to detect shocks.
 * 
 * Needs an external temp sensor, hall sensor (for the door), serial connection to 
 * ups and gps connected to get values.
 * 
 *  1)  Read values from serial registers to get UPS status
 *  2)  Checks the status of the door mag switch
 *  3)  Checks the GPS for a location
 *  4)  Sends all the data via LoRaWAN radio
 *  
 *  Version:           1.0
 *  Design:            Andre Ferreira 
 *  
 *******************************************************************************/
 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// Libraries for GPS and software serial port access
#include <SoftwareSerial.h>
#include <TinyGPS.h>

// Libraries for Curie gyro
#include "CurieIMU.h"

/******** LoRaWAN setup information ********************************************/

// Can use either OTAA or ABP. Comment out the one you don't use. OTAA NOT WORKING CURRENTLY WITH KERLINK
// Below are the ABP settings: 

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the Kerlink gateway and thus our 
// network initially.
static const PROGMEM u1_t NWKSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the Kerlink gateway and thus our
// network initially.
static const u1_t PROGMEM APPSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// LoRaWAN end-device address (DevAddr)
// Needs to be unique for the network. We use 0x7925xxxx
static const u4_t DEVADDR = 0x79250001 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
// COMMENT THESE OUT IF YOU USE OTAA BELOW  
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

/*
// These are the setting for OTAA - NOT WORKING WITH THE KERLINK
//

//From the forums: For me OTAA with LMIC works if I do the following:

//static const u1_t APPEUI[]={reversed 8 bytes of AppEUI registered with ttnctl}
//static const u1_t DEVEUI[]={reversed 8 bytes of DevEUI reistered with ttnctl}
//static const unsigned char** APPKEY**[16] = {<non-reversed 16 bytes of the APPKEY used when registering a device with ttnctl register DevEUI AppKey>}

// application router ID (LSBF)
static const u1_t APPEUI[8]  = { 0x02, 0x00, 0x00, 0x00, 0x00, 0xEE, 0xFF, 0xC0 };

// unique device ID (LSBF)
static const u1_t DEVEUI[8]  = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

// device-specific AES key (derived from device EUI)
static const u1_t DEVKEY[16] = { 0xAB, 0x89, 0xEF, 0xCD, 0x23, 0x01, 0x67, 0x45, 0x54, 0x76, 0x10, 0x32, 0xDC, 0xFE, 0x98, 0xBA };

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    memcpy(buf, DEVKEY, 16);
} 

*/

/* Define a structure to hold all the sensor data
   Usage: sensorData_t contains all sensor data that will be measured.
          Then, use the union type to define loraPacket_t to hold both the sensor data and the byte
          representation of the sensor data. This is useful for when sending later as we need to send a 
          string of bytes rather than a memory hungry string.
          Finally, use the variable payload to refernce the values in the structure as payload.sensor.xxx

          Give this struct to the person decoding the data on the other end.
          NB: Padding is added to ensure even boundries between 8 and 16bit values. If not added system adds padding arbitarly on the Curie
*/
typedef struct sensorData_t {
  uint8_t doorState;  // one byte
  uint8_t shockState;  // one byte
  uint8_t upsfailureState;  //one byte
  uint8_t padding;  //pad the structure to ensure even boundries and no auto padding
  uint16_t upsVolts;  // two bytes
  uint16_t upsFreq;  // two bytes
  uint16_t upsBattchrg;  // two bytes
  uint16_t upsBatttemp;  // two bytes
  uint16_t upsLoad;  // two bytes
  uint16_t shockcount; // two bytes - comment out padding if this is added as it evens the byte count
  //uint16_t padding2;  //More padding to even the boundry
  float cabTemp;  // four bytes
  float gpsLat;  // four bytes
  float gpsLong;  // four bytes
  float gpsAlt;  // four bytes
  float gpsSpeed;  // four bytes
};

#define PACKET_SIZE sizeof(sensorData_t);

// Now define a union so that we can access the byte representation of the sensor metrics via loraPacket[i]
typedef union loraPacket_t {
 sensorData_t sensor;
 byte loraPacket[sizeof(sensorData_t)];
};

// ..and now, a variable of the above type to actually use in the code
loraPacket_t payload;

// Some general variables and the links to call back apps
static osjob_t sendjob;
static osjob_t collect_job;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 20;

// Pin mapping for the Genuino
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

/********** General defines and global variables ********************************/

// Define addresses for reading UPS registers
// These are for the Delta UPS

// Input line freq and voltage
#define linedata "~00P003STO"
// Line failure status
#define alarmstatus "~00P003STA"
// Battery temp and charge
#define batthealth "~00P003STB"

// Store for serial input. String length matches recieved data string as per UPS spec
char ups_linedata[50] = "";
char ups_alarmstatus[20] = "";
char ups_batthealth[50] = "";

// Number of chars to read for each serial result
int linedata_end = 32;
int alarmstatus_end = 9;
int batthealth_end = 41;

// Variable to manipulate result strings
String strbuff[15];
String serialDataIn;
int counter = 0;

// UPS status vaiables to send to AEP
int freq;
int voltage;
int load;
int batt_temp;
int batt_chrg;
int line_state;

// Variable to store the Lora board error condition
int e;

// Packet counter
int16_t pkt = 0;

// Define pin that the door mag switch is connected to and door state flag
const byte door_pin = 3;
boolean door_state;

// Define the temp input analogue pin
const byte temp_pin = 0;

// Define serial port and valiables for the GPS query
SoftwareSerial Serial_gps(8, 5); // RX and TX pins for GPS data connection.
TinyGPS gps;

float flat, flon, faltitude, fspeed;
unsigned long age;

// Define the shock state variable (used by the ISR, so needs to be volatile
volatile boolean shockstate = false;          // initial state of shock
int16_t shock_count = 0;    // Count of shocks received

// Define variable to store the temperature
float temp;


/******** Start of functions *********************************************/

// Function to blink the LED (13) n times
void blink_led (int blinks)
{
  // Setup LED pin for blinking
  pinMode(13, OUTPUT);
  for (int i = 1; i <= blinks; i++) {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);              // wait for a bit
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(200);  
  }
}

/* Interrupt subroutine to trigger on a shock */
static void eventCallback(void)
{
  if (CurieIMU.getInterruptStatus(CURIE_IMU_SHOCK)) {
    if (CurieIMU.shockDetected(X_AXIS, POSITIVE)) {
      shockstate = true;
      Serial.println("Negative shock detected on X-axis");
    }
    if (CurieIMU.shockDetected(X_AXIS, NEGATIVE)) {
      shockstate = true;
      Serial.println("Positive shock detected on X-axis");
    }
    if (CurieIMU.shockDetected(Y_AXIS, POSITIVE)) {
      shockstate = true;
      Serial.println("Negative shock detected on Y-axis");
    }
    if (CurieIMU.shockDetected(Y_AXIS, NEGATIVE)) {
      shockstate = true;
      Serial.println("Positive shock detected on Y-axis");
    }
    if (CurieIMU.shockDetected(Z_AXIS, POSITIVE)) {
      shockstate = true;
      Serial.println("Negative shock detected on Z-axis");
    }
    if (CurieIMU.shockDetected(Z_AXIS, NEGATIVE)) {
      shockstate = true;
      Serial.println("Positive shock detected on Z-axis");
    }
  }
}


// Function to read the temp value
void get_temp () {
  temp = 0; // Reset value first
  for (int i = 1; i <= 10; i++) {
    temp = temp + analogRead(temp_pin)*3.3/1024.0;
  }
  temp = temp /10; // Get average reading
  temp = temp - 0.5;
  temp = temp / 0.01;
};


// Function to read the state of the door mg switch
void get_door_state ()
{
  if (digitalRead (door_pin) == HIGH) 
    door_state = true; //Door is open
  else
    door_state = false;
}

// Function to fetch data from the GPS
void get_gps_data ()
{
  bool newData = false;
  unsigned long chars = 0;

  Serial_gps.begin(9600);  // Start the soft serial port for use below
  
  // For one second we parse GPS data and populate global flat, flon, faltitude and fspeed variables
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial_gps.available())
    {
      char c = Serial_gps.read();
      Serial.print (c);
      ++chars;
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  Serial_gps.end();  // End the softserial port so that it does not mess up other stuff.

  if (newData)
  {
    // Assume we have a string of new data, so extract the values
    gps.f_get_position(&flat, &flon, &age);

    // Manually allocate the alt and speed variables
    faltitude = gps.f_altitude();
    fspeed = gps.f_speed_kmph();

    /* Debug, print to console */
    Serial.print("Lat=");
    Serial.println (flat, 6);
    
    Serial.print("Long=");
    Serial.println(flon, 6);
    
    Serial.print("Alt: ");  
    Serial.println (faltitude);
      
    Serial.print("Speed: "); 
    Serial.println (fspeed); 
    
    
    } else {
      // If no fix, clear the values to show this
      flon = 0; 
      flat = 0; 
      faltitude = 0;
      fspeed = 0;
      Serial.println("No Fix Yet"); 
      }
}



void get_ups_data(void)
{
  // Collect UPS data. Match counter (i) to string length expect from UPS
 
  // Collect UPS Line data and load
  Serial1.begin(2400);  // Reset serial port first. This is NB as it clears the buffer.
  delay(100);
  Serial1.print(linedata);
  delay(500); // Wait for a reply. UPS is slow
  for (int i = 0; i<= linedata_end; i++) {
    ups_linedata[i] = char(Serial1.read());
  }
  // Add ; to mark end of result string
  ups_linedata[linedata_end+1] = ';';
  Serial.println (ups_linedata);

  // Collect UPS Alarm status data
  Serial1.begin(2400); // Reset serial port first. This is NB as it clears the buffer.
  delay(100);
  Serial1.print(alarmstatus);
  delay(500); // Wait for a reply. UPS is slow
  for (int i = 0; i<= alarmstatus_end; i++) {
    ups_alarmstatus[i] = char(Serial1.read());
  }
  // Add ; to mark end of result string
  ups_alarmstatus[alarmstatus_end+1] = ';';
  Serial.println (ups_alarmstatus);

  // Collect UPS Batt health data
  Serial1.begin(2400); // Reset serial port first. This is NB as it clears the buffer.
  delay(100);
  Serial1.print(batthealth);
  delay(500); // Wait for a reply. UPS is slow
  for (int i = 0; i<= batthealth_end; i++) {
   ups_batthealth[i] = char(Serial1.read());
  }
  // Add ; to mark end of result string
  ups_batthealth[batthealth_end+1] = ';';
  Serial.println (ups_batthealth);

  // Extract the uselful values from the input strings

  // First do the line data and extract Voltage, Freq and UPS load
  serialDataIn = "";
  counter = 0;
  for (int i=0;i<=linedata_end+1;i++) {
     if((ups_linedata[i] >= '0') & (ups_linedata[i] <= '9'))
            serialDataIn += ups_linedata[i];
        if (ups_linedata[i] == ';'){  // Handle delimiter
            strbuff[counter] = serialDataIn;
            serialDataIn = "";
            // Print output
            //Serial.print ("Data: ");
            //Serial.println (strbuff[counter]);
            counter = counter + 1;
         }
  }

  // Allocate to variables ready to send to AEP
  freq = strbuff[1].toInt();
  voltage = strbuff[3].toInt();
  load = strbuff[6].toInt();


  // Now do the battery status and extract charge and temp
  serialDataIn = "";
  counter = 0;
  for (int i=0;i<=batthealth_end+1;i++) {
    if((ups_batthealth[i] >= '0') & (ups_batthealth[i] <= '9'))
           serialDataIn += ups_batthealth[i];
       if (ups_batthealth[i] == ';'){  // Handle delimiter
           strbuff[counter] = serialDataIn;
           serialDataIn = "";
           // Print output
            //Serial.print ("Data: ");
            //Serial.println (strbuff[counter]);
           counter = counter + 1;
        }
  }
    
  batt_temp = strbuff[8].toInt();
  batt_chrg = strbuff[9].toInt();
  
  
  // Now do the line status and extract failure state
  serialDataIn = "";
  counter = 0;
  for (int i=0;i<=alarmstatus_end+1;i++) {
    if((ups_alarmstatus[i] >= '0') & (ups_alarmstatus[i] <= '9'))
           serialDataIn += ups_alarmstatus[i];
       if (ups_alarmstatus[i] == ';'){  // Handle delimiter
           strbuff[counter] = serialDataIn;
           serialDataIn = "";
           // Print output
            //Serial.print ("Data: ");
            //Serial.println (strbuff[counter]);
           counter = counter + 1;
        }
  }
  
  line_state = strbuff[1].toInt();
}


// Main function that collects the data and sends it

static void collect_data (osjob_t* j) {

  // Go read the UPS and populate the global ups variables
  //Serial.println ("Getting UPS data");
  get_ups_data();
    
  // Now go read the state of the door
  //Serial.println ("Getting Door State");
  get_door_state ();
   
  // Now go read the temperature
  //Serial.println ("Getting temp value");
  get_temp ();
  
  // Now go read the GPS location (takes 1 second)
  //Serial.println ("Getting GPS data");
  get_gps_data();
     
  // Now, assign the values to the LoRaWAN packet components

  // Always define the padding values as zero
  payload.sensor.padding = 0;
  //payload.sensor.padding2 = 0; // Not needed since 16 bit sets are currenlty even

  // Now assign the rest of the struct values
  // First byte values
  payload.sensor.doorState = door_state;
  payload.sensor.shockState = shockstate;
  payload.sensor.upsfailureState = line_state;

  // Then 16bit ints
  payload.sensor.upsVolts = voltage;
  payload.sensor.upsFreq = freq;
  payload.sensor.upsBattchrg = batt_chrg;
  payload.sensor.upsBatttemp = batt_temp;
  payload.sensor.upsLoad = load;
  payload.sensor.shockcount = shock_count;

  // Finally, send floats as IEEE 754 encoded hex values. C uses 754 by default to encode a float into 4 bytes
  payload.sensor.cabTemp = temp;
  payload.sensor.gpsLong = flon;
  payload.sensor.gpsLat = flat;
  payload.sensor.gpsAlt = faltitude;
  payload.sensor.gpsSpeed = fspeed;


  // Print max lora packet size for reference
  //Serial.print ("Max LoRa payload size allowed: ");
  //Serial.println (MAX_LEN_PAYLOAD);
  
  // Print Payload size
  Serial.print ("Payload size: ");
  Serial.println (sizeof(sensorData_t));

  Serial.print ("Payload (HEX): "); 
  for (int i = 0; i < sizeof(sensorData_t); i++ ) {
    Serial.print (payload.loraPacket[i], HEX);
    Serial.print (" ");
  }
  Serial.println();

  // Increase packet count
  pkt = pkt + 1;

  // ..and increment shock count (if shock state is set, and reset state
  if (shockstate == true) {
    shock_count++;
    shockstate = false;
    Serial.print ("Shock count: ");
    Serial.println (shock_count);
  }
  
  // Schedule next collection of sensor data. ****Not needed here as we call this after a seccessful TX anyway. See onEvent() *****
  //os_setTimedCallback (j, os_getTime()+sec2osticks(60), collect_data);

}


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                
                /* trying to get downlink data to display...
                Serial.println(LMIC.dataBeg);
                Serial.println (LMIC.dataLen);
                for (int i = LMIC.dataBeg; i <= LMIC.dataBeg+(LMIC.dataLen*2); i++) {
                //for (int i = 0; i <= LMIC.dataLen; i++) {
                  Serial.write(LMIC.frame[i]);
                }
                */
                
                Serial.println();
            }

            // Collect sensor data here
            collect_data (&collect_job);
            
            // Schedule next transmission based on fixed interval (could be longer if data rate requires it)
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            // or ...
            // Send as often as possible according to LoRaWAN spec (depends on data rate being used)
            os_setCallback(&sendjob, do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        // Use the loraPacket union reference to send the bytes representation instead of the values.
        // These will need to be converted back at the receiver, so give them the struct to do this
        
        LMIC_setTxData2(1, payload.loraPacket, sizeof(sensorData_t), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup()
{ 
  // Open serial console communications
  Serial.begin(15200);
  //while (!Serial) ; // Wait for serial port to come live (genuino specfic) - REMOVE TO RUN WITHOUT SERIAL CABLE CONNECTED
  //Serial.println ("Starting script.. Setup");

   //Set door state pin to input and pull it high if no input present (Thus, hi == open)
  pinMode(door_pin, INPUT_PULLUP);

  // set the data rate for the UPS port. Use the HW port on pins 0,1
  Serial1.begin(2400);
  //Serial.println ("UPS serial..done");

  // Setup the gps serial port. Using SoftwareSerial. ***** Don't do this here, do it where we use the port to stop corruption *****
  //Serial_gps.begin(9600);
  //Serial.println ("GPS serial..done");

  /* Initialise the IMU and set up the ISR to listen for shocks */
  CurieIMU.begin();
  CurieIMU.attachInterrupt(eventCallback);

  // Enable Shock Detection and set parametres of a shock
  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, 1500); // 1.5g = 1500 mg
  CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 75);   // 50ms
  CurieIMU.interrupts(CURIE_IMU_SHOCK);
  
  // Setup the LoRaWAN radio 
  // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    // Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    
    // This might be needed to allow the Hope RF radio to work with ACK's
    //LMIC.dn2Dr = SF12;
    
    // Set up the channels used by the Kerlinks, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library). Chose ONE of these below:

    // Set data rate for min interpacket gap (also min distance), and max power
    LMIC_setDrTxpow(DR_SF7,14);

    // Set data rate for reasonable interpacket gap (also decent distance), and max power
    //LMIC_setDrTxpow(DR_SF9,14);
    
    // Set data rate for max distance (longest interpacket delay), and max power.
    //LMIC_setDrTxpow(DR_SF12,14);

    // Start job
    do_send(&sendjob);


}


void loop() {
    // Run the never ending loop. All action takes place via call back functions based on 
    // results of network interactions above.
    os_runloop_once();
}
