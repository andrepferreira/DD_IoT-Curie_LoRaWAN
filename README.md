 
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
 * Requires LoRaWAN libraries from https://github.com/matthijskooijman/arduino-lmic
 * 
 *  1)  Read values from serial registers to get UPS status
 *  2)  Checks the status of the door mag switch
 *  3)  Checks the GPS for a location
 *  4)  Sends all the data via LoRaWAN radio
 *  
 * Expects a LoRaWAN Network Server to recieve and decode traffic
 * See https://docs.loraserver.io/loraserver/ for an excellent Open Source NS
 *
 
