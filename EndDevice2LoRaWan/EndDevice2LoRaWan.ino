/* Heltec Automation LoRaWAN communication example
 *
 * Function:
 * 1. Upload node data to the server using the standard LoRaWAN protocol.
 *  
 * Description:
 * 1. Communicate using LoRaWAN protocol.
 * 
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 *
 * End Device 2 LoRaWAN
 * */

#include "LoRaWan_APP.h"

/* OTAA para*/
/* This information was obtained from The Things Network (TTN) */
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x94, 0x92 };      // TTN devEUI
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };      // Not applicable
uint8_t appKey[] = { 0x74, 0xD6, 0x6E, 0x63, 0x45, 0x82, 0x48, 0x27, 0xFE, 0xC5, 0xB7, 0x70, 0xBA, 0x2B, 0x50, 0x45 };      // Not applicable

/* ABP para*/
uint8_t nwkSKey[] = { 0x59, 0x4C, 0x66, 0x73, 0x83, 0xF5, 0x77, 0x89, 0x6F, 0x79, 0x58, 0x9F, 0xC3, 0x25, 0x6B, 0x7B };     // TTN NwkSKey
uint8_t appSKey[] = { 0xBB, 0xF8, 0xFC, 0x3A, 0x1B, 0x45, 0xDB, 0x7A, 0x5E, 0x4F, 0x0B, 0x65, 0x10, 0xD1, 0x9E, 0x75 };     // TTN appSKey
uint32_t devAddr =  ( uint32_t )0x260CBD55;     // TTN Device address

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

/*OTAA or ABP*/
bool overTheAirActivation = false;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

/* Function prototypes */
static void prepareTxFrame(uint8_t port);
void sendSensorData();
uint16_t randomPluviometer();
uint16_t randomTemperature();
uint8_t randomHumidity();
void printPayload();

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port) {
  if (port == 2) {
    sendSensorData();
  }
  printPayload();
}

/* Simulate random pluviómetro value */
uint16_t randomPluviometer() {
  return random(0, 65535);
}

/* Simulate random temperature value */
uint16_t randomTemperature() {
  return random(0, 65535);
}

/* Simulate random humidity value */
uint8_t randomHumidity() {
  return random(0, 255);
}

/* Send sensor data */
/*
Estructura del Paquete de Datos

    Byte 0: Parte alta del valor del pluviómetro.
    Byte 1: Parte baja del valor del pluviómetro.
    Byte 2: Parte alta del valor de la temperatura.
    Byte 3: Parte baja del valor de la temperatura.
    Byte 4: Valor de la humedad.

Ejemplo de Trama

Supongamos que los valores simulados son los siguientes:

    Pluviómetro: 12345 (0x3039 en hexadecimal)
    Temperatura: 6789 (0x1A85 en hexadecimal)
    Humedad: 65 (0x41 en hexadecimal)

La trama de datos (appData) sería:

    Byte 0: 0x30
    Byte 1: 0x39
    Byte 2: 0x1A
    Byte 3: 0x85
    Byte 4: 0x41

En el monitor serie, se vería algo así:

Payload: 30 39 1A 85 41


*/
void sendSensorData() {
  uint16_t pluviometerValue = randomPluviometer();
  uint16_t temperatureValue = randomTemperature();
  uint8_t humidityValue = randomHumidity();

  appDataSize = 5;
  appData[0] = (pluviometerValue >> 8) & 0xFF;  // First byte of the pluviometer value
  appData[1] = pluviometerValue & 0xFF;         // Second byte of the pluviometer value
  appData[2] = (temperatureValue >> 8) & 0xFF;  // First byte of the temperature value
  appData[3] = temperatureValue & 0xFF;         // Second byte of the temperature value
  appData[4] = humidityValue;                   // Byte of the humidity value

  Serial.print("Sending pluviómetro data: ");
  Serial.println(pluviometerValue);
  Serial.print("Sending temperature data: ");
  Serial.println(temperatureValue);
  Serial.print("Sending humidity data: ");
  Serial.println(humidityValue);
}

/* Print the payload */
void printPayload() {
  Serial.print("Payload: ");
  for (int i = 0; i < appDataSize; i++) {
    Serial.print(appData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
}

void loop() {
  switch(deviceState) {
    case DEVICE_STATE_INIT:
    {
#if(LORAWAN_DEVEUI_AUTO)
      LoRaWAN.generateDeveuiByChipID();
#endif
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDefaultDR(3);
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      prepareTxFrame(2); // Send sensor data
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.sleep(loraWanClass);
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}
