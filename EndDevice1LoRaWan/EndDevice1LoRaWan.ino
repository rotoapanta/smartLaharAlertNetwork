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
 * End Device 1 LoRaWAN
 * */

#include "LoRaWan_APP.h" 
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "images.h"

// Definir el objeto de la pantalla OLED (ajusta los parámetros según tu configuración)
SSD1306Wire oledDisplay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

int demoMode = 0;
String receivedData = ""; // Declaración global
uint16_t pluviometerValue = 0; // Variable global para almacenar el valor del pluviómetro

/* OTAA para*/
/* This information was obtained from The Things Network (TTN) */
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x94, 0x7C };      // TTN devEUI
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };      // Not applicable
uint8_t appKey[] = { 0x74, 0xD6, 0x6E, 0x63, 0x45, 0x82, 0x48, 0x27, 0xFE, 0xC5, 0xB7, 0x70, 0xBA, 0x2B, 0x50, 0x45 };      // Not applicable

/* ABP para*/
uint8_t nwkSKey[] = { 0xB9, 0xE8, 0x94, 0xB3, 0xC7, 0x6B, 0x28, 0xA7, 0x6B, 0xEA, 0xAC, 0xBB, 0xAF, 0x6A, 0x68, 0xBE };     // TTN NwkSKey
uint8_t appSKey[] = { 0x6D, 0xCE, 0xC6, 0x5A, 0xA2, 0x43, 0xAE, 0x36, 0x8A, 0x08, 0x5B, 0x0E, 0xD1, 0xDF, 0x7C, 0xEA };     // TTN appSKey
uint32_t devAddr =  ( uint32_t )0x260C14BC;     // TTN Device address

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

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
void sendSensorData(float pluviometerValue);
uint16_t randomPluviometer();
uint16_t randomTemperature();
uint8_t randomHumidity();
void printPayload();
void displaySplashScreen();
void displayInitializingMessage();
void displayProjectInfo();
void displaySystemName();
void displayDevEui(); // Nueva función para mostrar el devEUI
void receiveSerialData(); // Nueva función para recibir datos seriales
void displayLoRaWANDeviceInfo(); // Mostrar el DevEui y DevAddr
void displayRainGaugeValue(int pluviometerValue); // Declaración de la función displayRainGaugeValue

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port) {
  if (port == 2) {
    sendSensorData(pluviometerValue);
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
// Tamaño de los datos de la aplicación
const uint8_t APP_DATA_SIZE = 5;

// Función para enviar los datos del sensor
void sendSensorData(int pluviometerValue) {
    uint8_t appData[APP_DATA_SIZE]; // Array para los datos de la aplicación
    
    // Convertir el valor del pluviómetro a un entero de 16 bits
    uint16_t pluviometerValueInt = static_cast<uint16_t>(pluviometerValue);
    
    // Dividir el valor del pluviómetro en bytes altos y bajos
    appData[0] = (pluviometerValueInt >> 8) & 0xFF;
    appData[1] = pluviometerValueInt & 0xFF;
    
    // Valores de marcador de posición para temperatura y humedad
    appData[2] = 0;  // Placeholder para valor de temperatura
    appData[3] = 0;  // Placeholder para valor de temperatura
    appData[4] = 0;  // Placeholder para valor de humedad

    // Imprimir el valor del pluviómetro en la consola serial
    Serial.print("Rain gauge value: ");
    Serial.print(pluviometerValue);  // Imprimir valor entero
    Serial.println(" [mm]");
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

/* Display splash screen with logo */
void displaySplashScreen() {
    oledDisplay.clear();
    int x = (oledDisplay.width() - Volcano2_Logo_width) / 2;
    int y = 0;  // Mover a la parte superior de la pantalla
    oledDisplay.drawXbm(x, y, Volcano2_Logo_width, Volcano2_Logo_height, Volcano2_Logo_bits);
    oledDisplay.display();
    delay(3000);  // Show splash screen for 3 seconds
}

/* Display system initializing message */
void displayInitializingMessage() {
    String text = "System Initializing...";

    oledDisplay.clear();
    oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);  // Alineación del texto a la izquierda
    oledDisplay.setFont(ArialMT_Plain_10);  // Cambiar el tamaño de la fuente a 10

    int logoX = (oledDisplay.width() - Volcano2_Logo_width) / 2;
    int logoY = 0;  // Comenzar a dibujar el logotipo desde la parte superior
    int textY = Volcano2_Logo_height + 5;  // 5 píxeles debajo del logotipo

    // Calcular el ancho del texto para centrarlo
    int textWidth = oledDisplay.getStringWidth(text);
    int textX = (oledDisplay.width() - textWidth) / 2;  // Centrar el texto horizontalmente

    // Dibujar el logotipo
    oledDisplay.drawXbm(logoX, logoY, Volcano2_Logo_width, Volcano2_Logo_height, Volcano2_Logo_bits);
    oledDisplay.display();

    // Mostrar "System Initializing..." letra por letra en una sola línea
    for (int i = 1; i <= text.length(); i++) {
        oledDisplay.setColor(BLACK);
        oledDisplay.fillRect(0, textY, oledDisplay.width(), 10);  // Limpiar el área donde se dibujará el texto
        oledDisplay.setColor(WHITE);
        oledDisplay.drawString(textX, textY, text.substring(0, i));
        oledDisplay.display();
        delay(200);  // Ajustar el retardo según sea necesario
    }

    delay(3000);  // Mantener la pantalla visible por 3 segundos
}

/* Display project information */
void displayProjectInfo() {
  oledDisplay.clear();
  oledDisplay.setTextAlignment(TEXT_ALIGN_CENTER);
  oledDisplay.setFont(ArialMT_Plain_10);
  oledDisplay.drawString(oledDisplay.width() / 2, oledDisplay.height() / 2 - 20, "Instituto Geofisico");
  oledDisplay.drawString(oledDisplay.width() / 2, oledDisplay.height() / 2 - 10, "Proyecto CEDIA I+D+i 62");
  oledDisplay.drawString(oledDisplay.width() / 2, oledDisplay.height() / 2, "Roberto Toapanta");
  oledDisplay.drawString(oledDisplay.width() / 2, oledDisplay.height() / 2 + 10, "Carlos Macias");
  oledDisplay.display();
  delay(3000);  // Show project info for 3 seconds
}

/* Display system name */
void displaySystemName() {
  oledDisplay.clear();
  oledDisplay.setTextAlignment(TEXT_ALIGN_CENTER);
  oledDisplay.setFont(ArialMT_Plain_10);
  oledDisplay.drawString(oledDisplay.width() / 2, oledDisplay.height() / 2 - 10, "IoT-based Lahar");
  oledDisplay.drawString(oledDisplay.width() / 2, oledDisplay.height() / 2, "Alert System (ILAS)");
  oledDisplay.display();
  delay(3000);  // Show system name for 3 seconds
}

/* Display LoRaWAN Device Info */
void displayLoRaWANDeviceInfo() {
    // Definir las posiciones iniciales de los textos
    int initialPosition = 20;
    int yPosition = initialPosition;

    // Convertir devEui a string
    String devEuiStr = "";
    for (int i = 0; i < 8; i++) {
        if (i > 0) devEuiStr += ":";
        devEuiStr += String(devEui[i], HEX);
    }

    // Convertir devAddr a string y asegurar que esté en mayúsculas
    String devAddrStr = "0x" + String(devAddr, HEX);
    devAddrStr.toUpperCase();

    // Mostrar la información inicialmente y esperar 3 segundos
    oledDisplay.clear();
    oledDisplay.setTextAlignment(TEXT_ALIGN_CENTER);
    oledDisplay.setFont(ArialMT_Plain_10);
    oledDisplay.drawString(oledDisplay.width() / 2, 0, "LoRaWAN Device Info");
    // Dibujar una línea horizontal para separar el título de los datos
    oledDisplay.drawHorizontalLine(0, 15, oledDisplay.width());
    
    oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
    oledDisplay.drawString(0, yPosition, "DevEUI:");
    oledDisplay.drawString(0, yPosition + 10, devEuiStr);
    oledDisplay.drawString(0, yPosition + 25, "DevAddr:");
    oledDisplay.drawString(0, yPosition + 35, devAddrStr);
    oledDisplay.display();
    
    delay(3000); // Esperar 3 segundos antes de iniciar el desplazamiento

    // Bucle para desplazar la información hacia arriba
    for (int offset = 0; offset <= 40; offset++) {
        oledDisplay.clear();
        oledDisplay.setTextAlignment(TEXT_ALIGN_CENTER);
        oledDisplay.setFont(ArialMT_Plain_10);
        oledDisplay.drawString(oledDisplay.width() / 2, 0 - offset, "LoRaWAN Device Info");
        // Dibujar una línea horizontal para separar el título de los datos
        oledDisplay.drawHorizontalLine(0, 15 - offset, oledDisplay.width());
        
        oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
        oledDisplay.drawString(0, yPosition - offset, "DevEUI:");
        oledDisplay.drawString(0, yPosition + 10 - offset, devEuiStr);
        oledDisplay.drawString(0, yPosition + 25 - offset, "DevAddr:");
        oledDisplay.drawString(0, yPosition + 35 - offset, devAddrStr);
        oledDisplay.display();

        // Controlar la velocidad del desplazamiento con un retraso gradual
        delay(100 + (offset * 5)); // Aumentar gradualmente el retraso
    }

    delay(3000); // Pausar la información desplazada por 3 segundos antes de borrar la pantalla
}

/* Función para mostrar el valor del pluviómetro */
void displayRainGaugeValue(int pluviometerValue) {
    oledDisplay.clear();
    oledDisplay.setTextAlignment(TEXT_ALIGN_CENTER);
    oledDisplay.setFont(ArialMT_Plain_10);
    oledDisplay.drawString(oledDisplay.width() / 2, 0, "Real-Time Sensor Data");
    // Dibujar una línea horizontal para separar el título de los datos
    oledDisplay.drawHorizontalLine(0, 15, oledDisplay.width());
    
    oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
    oledDisplay.setFont(ArialMT_Plain_10);  // Tamaño de letra más grande
    oledDisplay.drawString(0, 20, "Rain gauge: " + String(pluviometerValue) + " [mm]");
    oledDisplay.display();
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  // Inicializar la pantalla OLED
  oledDisplay.init();

  // Mostrar mensajes iniciales al arrancar el sistema
  displayInitializingMessage();
  displaySystemName();
  displayProjectInfo();
  displayLoRaWANDeviceInfo();
}

void loop() {
  receiveSerialData(); // Llamada a la función para recibir datos seriales

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
      prepareTxFrame(2); // Enviar datos del sensor
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

  // Actualizar únicamente el valor del pluviómetro en el loop principal
  displayRainGaugeValue(pluviometerValue);
  delay(10000); // Esperar 10 segundos antes de actualizar el valor nuevamente
}

/* Nueva función para recibir datos seriales */
void receiveSerialData() {
    if (Serial.available()) {
        receivedData = Serial.readStringUntil('\n');
        if (receivedData.length() > 0) {
            pluviometerValue = receivedData.toInt(); // Actualizar la variable global
        }
    }
}
