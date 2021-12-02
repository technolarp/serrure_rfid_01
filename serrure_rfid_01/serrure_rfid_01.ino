/*
   ----------------------------------------------------------------------------
   TECHNOLARP - https://technolarp.github.io/
   SERRURE RFID 01 - https://github.com/technolarp/serrure_rfid_01
   version 1.0 - 11/2021
   ----------------------------------------------------------------------------
*/

/*
   ----------------------------------------------------------------------------
   Pour ce montage, vous avez besoin de 
   1 lecteur RFID i2c PN532
   1 ou + led neopixel
   1 buzzer piezo
   ----------------------------------------------------------------------------
*/

/*
   ----------------------------------------------------------------------------
   PINOUT
   D0     NEOPIXEL
   D1     I2C SCL => SCL PN532
   D2     I2C SDA => SDA PN532
   D8     BUZZER
   ----------------------------------------------------------------------------
*/

/* TODO
 renommer nbTagStock / tagOK2
 move i2c reset dan lib
 statut object universel
 move anim led et blink dans fastled
 check ajout new uid
 check taille uid variable
 check bug taille donnees static/dynamic json doc
 print memory size
 cleanup code
 pn532 static
 */

#include <Arduino.h>

// WIFI
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

// WEBSOCKET
AsyncWebSocket ws("/ws");

char bufferWebsocket[300];
bool flagBufferWebsocket = false;

// FASTLED
#include <technolarp_fastled.h>
M_fastled aFastled;

// RFID
#include <technolarp_pn532.h>
M_pn532* aPn532;

// BUZZER
#define PIN_BUZZER D8
#include <technolarp_buzzer.h>
M_buzzer buzzer(PIN_BUZZER);

// CONFIG
#include "config.h"
M_config aConfig;

char bufferToSend[500];
char bufferUid[12];

// CODE ACTUEL DE LA SERRURE
//clean char codeSerrureActuel[8] = {'0', '0', '0', '0', '0', '0', '0', '0'};

// STATUTS DE LA SERRURE
enum {
  SERRURE_OUVERTE = 0,
  SERRURE_FERMEE = 1,
  SERRURE_BLOQUEE = 2,
  SERRURE_ERREUR = 3,
  SERRURE_RECONFIG = 4,
  SERRURE_BLINK = 5
};

// PARAM RECONFIG
//clean 
/*
enum {
  RECONFIG_CODE = 0,
  RECONFIG_ERREUR = 1,
  RECONFIG_DELAI = 2,
  RECONFIG_TAILLE_CODE = 3
};
*/

// DIVERS
bool uneFois = true;

//clean char bufferReconfig[8] = {'0','0','0','0','0','0','0','0'};
//clean uint8_t modeReconfig = 0;

// BLINK
uint32_t startMillisBlink;
uint32_t previousMillisBlink;
uint32_t intervalBlink;
bool blinkFlag = true;
uint8_t cptBlink = 0;

// HEARTBEAT
unsigned long int previousMillisHB;
unsigned long int intervalHB;

/*
   ----------------------------------------------------------------------------
   SETUP
   ----------------------------------------------------------------------------
*/
void setup()
{
  Serial.begin(115200);

  // VERSION
  delay(500);
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("----------------------------------------------------------------------------"));
  Serial.println(F("TECHNOLARP - https://technolarp.github.io/"));
  Serial.println(F("SERRURE RFID 01 - https://github.com/technolarp/serrure_rfid_01"));
  Serial.println(F("version 1.0 - 11/2021"));
  Serial.println(F("----------------------------------------------------------------------------"));

  // I2C RESET
  int rtn = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()
  if (rtn != 0)
  {
    Serial.println(F("I2C bus error. Could not clear"));
    if (rtn == 1) 
    {
      Serial.println(F("SCL clock line held low"));
    }
    else if (rtn == 2)
    {
      Serial.println(F("SCL clock line held low by slave clock stretch"));
    }
    else if (rtn == 3) 
    {
      Serial.println(F("SDA data line held low"));
    }
  }
  else
  { 
    // bus clear
    Serial.println(F("I2C bus clear"));
  }
  Serial.println("setup finished");
  
  // CONFIG OBJET
  Serial.println(F(""));
  Serial.println(F(""));
  aConfig.mountFS();
  aConfig.listDir("/");
  aConfig.listDir("/config");
  aConfig.listDir("/www");
  
  aConfig.printJsonFile("/config/objectconfig.txt");
  aConfig.readObjectConfig("/config/objectconfig.txt");

  //aConfig.printJsonFile("/config/networkconfig.txt");
  aConfig.readNetworkConfig("/config/networkconfig.txt");

  // FASTLED
  // animation led de depart
  for (int i = 0; i < aConfig.objectConfig.activeLeds * 2; i++)
  {
    aFastled.ledOn(i, CRGB::Blue, true);
    delay(50);
    aFastled.ledOff(i, true);
  }
  aFastled.allLedOff();
  
  // RFID
  aPn532 = new M_pn532;

  // WIFI
  WiFi.disconnect(true);
  
  
  // AP MODE
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(aConfig.networkConfig.apIP, aConfig.networkConfig.apIP, aConfig.networkConfig.apNetMsk);
  WiFi.softAP(aConfig.networkConfig.apName, aConfig.networkConfig.apPassword);
  
  /*
  // CLIENT MODE POUR DEBUG
  const char* ssid = "MYDEBUG";
  const char* password = "ppppppp";
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  */

  if (WiFi.waitForConnectResult() != WL_CONNECTED) 
  {
    Serial.printf("WiFi Failed!\n");
  }
  
  // WEB SERVER
  // Print ESP Local IP Address
  Serial.print(F("localIP: "));
  Serial.println(WiFi.localIP());
  Serial.print(F("softAPIP: "));
  Serial.println(WiFi.softAPIP());

  // Route for root / web page
  server.serveStatic("/", LittleFS, "/www/").setDefaultFile("config.html");
  server.serveStatic("/config", LittleFS, "/config/");
  server.onNotFound(notFound);

  // WEBSOCKET
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // Start server
  server.begin();

  // BUZZER
  buzzer.doubleBeep();

  // HEARTBEAT
  previousMillisHB = millis();
  intervalHB = 5000;
  
  // SERIAL
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("START !!!"));
}
/*
   ----------------------------------------------------------------------------
   FIN DU SETUP
   ----------------------------------------------------------------------------
*/




/*
   ----------------------------------------------------------------------------
   LOOP
   ----------------------------------------------------------------------------
*/
void loop()
{  
  // avoid watchdog reset
  yield();
  
  // WEBSOCKET
  ws.cleanupClients();

  // BUZZER
  buzzer.update();

  // RFID PN5323
  aPn532->updateRFID();

  // gerer le statut de la serrure
  switch (aConfig.objectConfig.statutActuel)
  {
    case SERRURE_FERMEE:
      // la serrure est fermee
      serrureFermee();
      break;

    case SERRURE_OUVERTE:
      // la serrure est ouverte
      serrureOuverte();
      break;

    case SERRURE_BLOQUEE:
      // la serrure est bloquee
      serrureBloquee();
      break;

    case SERRURE_ERREUR:
      // un code incorrect a ete entrer
      serrureErreur();
      break;

    case SERRURE_BLINK:
      // blink led pour identification
      serrureBlink();
      break;
      
    default:
      // nothing
      break;
  }

  // traite le buffer du websocket
  if (flagBufferWebsocket)
  {
    flagBufferWebsocket = false;
    handleWebsocketBuffer();
  }
  
  // HEARTBEAT
  if(millis() - previousMillisHB > intervalHB)
  {
    previousMillisHB = millis();
    
    // send new value to html
    sendUptime();
  }
}
/*
   ----------------------------------------------------------------------------
   FIN DU LOOP
   ----------------------------------------------------------------------------
*/





/*
   ----------------------------------------------------------------------------
   FONCTIONS ADDITIONNELLES
   ----------------------------------------------------------------------------
*/

void serrureFermee()
{
  if (uneFois)
  {
    uneFois = false;

    // on allume les led en rouge
    aFastled.allLedOn(aConfig.objectConfig.couleurs[0], true);

    Serial.print(F("SERRURE FERMEE"));
    Serial.println();
  }
  
  // on check si un tag rfid est present
  checkRfidTag();
}

void serrureOuverte()
{
  if (uneFois)
  {
    uneFois = false;

    // on allume les led en vert
    aFastled.allLedOn(aConfig.objectConfig.couleurs[1], true);

    Serial.print(F("SERRURE OUVERTE"));
    Serial.println();
  }
 
  // on check si un tag rfid est present
  checkRfidTag();
}

void serrureErreur()
{
  if (uneFois)
  {
    uneFois = false;
    Serial.println(F("SERRURE ERREUR"));

    previousMillisBlink = millis();
    startMillisBlink = previousMillisBlink;
    intervalBlink = 100;

    blinkFlag = true;
  }

  if(millis() - previousMillisBlink > intervalBlink)
  {
    previousMillisBlink = millis();
    blinkFlag = !blinkFlag;

    if (blinkFlag)
    {
      aFastled.allLedOn(aConfig.objectConfig.couleurs[0], true);
    }
    else
    {
      aFastled.allLedOff();
    }
  }

  // fin de l'animation erreur 
  if(millis() - startMillisBlink > 2000) 
  {
    uneFois = true;
    aFastled.allLedOn(aConfig.objectConfig.couleurs[0], true);

    // si il y a eu trop de faux codes
    if (aConfig.objectConfig.nbErreurCode >= aConfig.objectConfig.nbErreurCodeMax)
    {
      // on bloque la serrure
      aConfig.objectConfig.statutActuel = SERRURE_BLOQUEE;
    }
    else
    {
      aConfig.objectConfig.statutActuel = aConfig.objectConfig.statutPrecedent;
    }
    
    writeObjectConfig();
    sendObjectConfig();
  }
}

void serrureBloquee()
{
  if (uneFois)
  {
    uneFois = false;
    Serial.println(F("SERRURE BLOQUEE"));

    previousMillisBlink = millis();
    startMillisBlink = previousMillisBlink;
    intervalBlink = 500;

    blinkFlag = true;
  }

  if(millis() - previousMillisBlink > intervalBlink)
  {
    previousMillisBlink = millis();
    blinkFlag = !blinkFlag;

    if (blinkFlag)
    {
      aFastled.allLedOn(aConfig.objectConfig.couleurs[1], true);
    }
    else
    {
      aFastled.allLedOn(aConfig.objectConfig.couleurs[0], true);
    }
  }

  // fin de l'animation blocage
  if( (millis() - startMillisBlink) > (aConfig.objectConfig.delaiBlocage*1000) ) 
  {
    Serial.println(F("END BLOCAGE "));
    
    uneFois = true;

    aConfig.objectConfig.statutActuel = aConfig.objectConfig.statutPrecedent;
    aConfig.objectConfig.nbErreurCode = 0;

    writeObjectConfig();
    sendObjectConfig();
  }
}

void serrureBlink()
{
  if (uneFois)
  {
    uneFois = false;
    Serial.println(F("SERRURE BLINK"));

    previousMillisBlink = millis();
    startMillisBlink = previousMillisBlink;
    intervalBlink = 200;

    blinkFlag = true;
  }

  if(millis() - previousMillisBlink > intervalBlink)
  {
    previousMillisBlink = millis();
    blinkFlag = !blinkFlag;

    if (blinkFlag)
    {
      aFastled.allLedOn(CRGB::Blue, true);
    }
    else
    {
      aFastled.allLedOff();
    }
  }

  // fin de l'animation blocage
  if(millis() - startMillisBlink > 2000) 
  {
    uneFois = true;

    aConfig.objectConfig.statutActuel = aConfig.objectConfig.statutPrecedent;

    writeObjectConfig();
    sendObjectConfig();

    Serial.println(F("END BLINK "));
  }
}

// check rfid tag
void checkRfidTag()
{
  if (aPn532->readUID(10))
  {
    // print new tag
    Serial.print(F("nouveau TAG: "));
    Serial.print(aPn532->uidLength);
    Serial.print(F("  "));
    
    printTagUid();

    // check with store uid
    bool tagOK = false;
    
    for (uint8_t i=0;i<aConfig.objectConfig.nbTagActuel;i++)
    {
      bool tagOK2 = true;
      for (uint8_t j=1;j<4;j++)
      {        
        if ( (aPn532->uidLength>=4) && (aPn532->uid[j] != aConfig.objectConfig.tagUid[i][j]) )
        {
          tagOK2 = false;
        }
      }

      if (tagOK2)
      {
        tagOK = true;
      }
    }

    // change lock status if needed
    if (tagOK)
    {
      buzzer.shortBeep();

      aConfig.objectConfig.statutActuel = !aConfig.objectConfig.statutActuel;
      aConfig.objectConfig.nbErreurCode = 0;
      uneFois = true;
      Serial.println(F("tag OK, change lock status"));
    }
    else
    {
      // le tag n'est pas le bon
      Serial.println(F("mauvais tag"));
      buzzer.longBeep();

      // on augmente le compteur de code faux
      aConfig.objectConfig.nbErreurCode += 1;

      aConfig.objectConfig.statutPrecedent = aConfig.objectConfig.statutActuel;

      // on demarre l'anim faux code
      aConfig.objectConfig.statutActuel = SERRURE_ERREUR;

      uneFois = true;
    }

    // send uid 
    sendTagUid();

    // ecrire la config sur littleFS
    writeObjectConfig();

    // resend config object
    sendObjectConfig();
  }
}


void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) 
{
  switch (type) 
   {
      case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        // send config value to html
        sendObjectConfig();
        sendNetworkConfig();
        
        // send volatile info
        sendUptime();
        sendMaxLed();        
        break;
        
      case WS_EVT_DISCONNECT:
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
        break;
        
      case WS_EVT_DATA:
        handleWebSocketMessage(arg, data, len);
        break;
        
      case WS_EVT_PONG:
      case WS_EVT_ERROR:
        break;
  }
}


void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) 
{
  
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) 
  {
    data[len] = 0;
    sprintf(bufferWebsocket,"%s\n", (char*)data);
    Serial.print(len);
    Serial.print(bufferWebsocket);
    flagBufferWebsocket = true;
  }
}

void handleWebsocketBuffer()
{    
    DynamicJsonDocument doc(JSONBUFFERSIZE);
    
    DeserializationError error = deserializeJson(doc, bufferWebsocket);
    if (error)
    {
      Serial.println(F("Failed to deserialize buffer in websocket"));
    }
    else
    {
      // write config or not
      bool writeObjectConfigFlag = false;
      bool sendObjectConfigFlag = false;
      bool writeNetworkConfigFlag = false;
      bool sendNetworkConfigFlag = false;
      
      // modif object config
      if (doc.containsKey("new_objectName"))
      {
        strlcpy(  aConfig.objectConfig.objectName,
                  doc["new_objectName"],
                  sizeof(aConfig.objectConfig.objectName));

        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }
      
      if (doc.containsKey("new_objectId")) 
      {
        uint16_t tmpValeur = doc["new_objectId"];
        aConfig.objectConfig.objectId = checkValeur(tmpValeur,1,1000);

        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if (doc.containsKey("new_groupId")) 
      {
        uint16_t tmpValeur = doc["new_groupId"];
        aConfig.objectConfig.groupId = checkValeur(tmpValeur,1,1000);
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if (doc.containsKey("new_activeLeds")) 
      {
        aFastled.allLedOff();
        
        uint16_t tmpValeur = doc["new_activeLeds"];
        aConfig.objectConfig.activeLeds = checkValeur(tmpValeur,1,NB_LEDS_MAX);
        
        uneFois = true;

        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if (doc.containsKey("new_brightness"))
      {
        uint16_t tmpValeur = doc["new_brightness"];
        aConfig.objectConfig.brightness = checkValeur(tmpValeur,0,255);
        aFastled.setBrightness(aConfig.objectConfig.brightness);
        uneFois=true;
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if (doc.containsKey("new_nbErreurCodeMax")) 
      {
        uint16_t tmpValeur = doc["new_nbErreurCodeMax"];
        aConfig.objectConfig.nbErreurCodeMax = checkValeur(tmpValeur,1,50);
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }
      
      if (doc.containsKey("new_delaiBlocage")) 
      {
        uint16_t tmpValeur = doc["new_delaiBlocage"];
        aConfig.objectConfig.delaiBlocage = checkValeur(tmpValeur,5,300);
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }
      
      if (doc.containsKey("new_statutActuel"))
      {
        uint16_t tmpValeur = doc["new_statutActuel"];
        aConfig.objectConfig.statutPrecedent=aConfig.objectConfig.statutActuel;
        aConfig.objectConfig.statutActuel=tmpValeur;
    
        uneFois=true;
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if (doc.containsKey("new_nbTagStock")) 
      {
        uint16_t tmpValeur = doc["new_nbTagStock"];
        aConfig.objectConfig.nbTagStock = checkValeur(tmpValeur,1,5);
        aConfig.objectConfig.nbTagActuel=min<uint8_t>(aConfig.objectConfig.nbTagActuel,aConfig.objectConfig.nbTagStock);
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }
    
      if ( doc.containsKey("new_removeUid") && doc["new_removeUid"]>=0 && doc["new_removeUid"]<MAX_NB_TAG )
      {
        uint16_t uidToRemove = doc["new_removeUid"];
        Serial.print(F("Remove tag UID: "));
        Serial.println(uidToRemove);
    
        removeUid(uidToRemove);
    
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if ( doc.containsKey("new_addLastUid") && doc["new_addLastUid"]==1 )
      {
        if (aConfig.objectConfig.nbTagActuel<MAX_NB_TAG)
        {
          for (uint8_t  i=0;i<SIZE_UID;i++)
          {
            aConfig.objectConfig.tagUid[aConfig.objectConfig.nbTagActuel][i]=aPn532->uid[i];
          }
          aConfig.objectConfig.nbTagActuel++;          

          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }
        else
        {
          Serial.println(F("too much tags"));
        }
      }

      if (doc.containsKey("new_newTagUid")) 
      {
        if (aConfig.objectConfig.nbTagActuel<MAX_NB_TAG)
        {
          JsonArray newUidValue = doc["new_newTagUid"];
            
          for (uint8_t  i=0;i<SIZE_UID;i++)
          {
            const char* tagUid_0 = newUidValue[i];
            uint8_t hexValue = (uint8_t) strtol( &tagUid_0[0], NULL, 16);
            
            aConfig.objectConfig.tagUid[aConfig.objectConfig.nbTagActuel][i]=hexValue;
            aPn532->uid[i]=hexValue;
          }
          aConfig.objectConfig.nbTagActuel++;          

          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }
        else
        {
          Serial.println(F("too much tags"));
        }
      }

      if ( doc.containsKey("new_resetErreur") && doc["new_resetErreur"]==1 )
      {
        Serial.println(F("Reset erreurs"));
        aConfig.objectConfig.nbErreurCode = 0;

        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if (doc.containsKey("new_couleurs")) 
        {
          JsonArray newCouleur = doc["new_couleurs"];

          uint8_t i = newCouleur[0];
          String newColorStr = newCouleur[1];
          
          uint8_t r;
          uint8_t g;
          uint8_t b;
          
          convertStrToRGB(newColorStr,&r, &g, &b);
          aConfig.objectConfig.couleurs[i].red=r;
          aConfig.objectConfig.couleurs[i].green=g;
          aConfig.objectConfig.couleurs[i].blue=b;
          //convertStrToRGB(newColorStr,&aConfig.objectConfig.couleurs[i].red, &aConfig.objectConfig.couleurs[i].green, &aConfig.objectConfig.couleurs[i].blue);
          uneFois=true;
          
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }
        
      // **********************************************
      // modif network config
      // **********************************************
      if (doc.containsKey("new_apName")) 
      {
        strlcpy(  aConfig.networkConfig.apName,
                  doc["new_apName"],
                  sizeof(aConfig.networkConfig.apName));

        // check for unsupported char
        checkCharacter(aConfig.networkConfig.apName, "ABCDEFGHIJKLMNOPQRSTUVWYZ", 'A');
        
        writeNetworkConfigFlag = true;
        sendNetworkConfigFlag = true;
      }

      if (doc.containsKey("new_apPassword")) 
      {
        strlcpy(  aConfig.networkConfig.apPassword,
                  doc["new_apPassword"],
                  sizeof(aConfig.networkConfig.apPassword));

        writeNetworkConfigFlag = true;
      }

      if (doc.containsKey("new_apIP")) 
      {
        char newIPchar[16] = "";

        strlcpy(  newIPchar,
                  doc["new_apIP"],
                  sizeof(newIPchar));

        IPAddress newIP;
        if (newIP.fromString(newIPchar)) 
        {
          Serial.println("valid IP");
          aConfig.networkConfig.apIP = newIP;

          writeNetworkConfigFlag = true;
        }
        
        sendNetworkConfigFlag = true;
      }

      if (doc.containsKey("new_apNetMsk")) 
      {
        char newNMchar[16] = "";

        strlcpy(  newNMchar,
                  doc["new_apNetMsk"],
                  sizeof(newNMchar));

        IPAddress newNM;
        if (newNM.fromString(newNMchar)) 
        {
          Serial.println("valid netmask");
          aConfig.networkConfig.apNetMsk = newNM;

          writeNetworkConfigFlag = true;
        }

        sendNetworkConfigFlag = true;
      }

      if ( doc.containsKey("new_defaultObjectConfig") && doc["new_defaultObjectConfig"]==1 )
      {
        Serial.println(F("reset to default object config"));
        aConfig.writeDefaultObjectConfig("/config/objectconfig.txt");
        
        sendObjectConfigFlag = true;
        uneFois = true;
      }

      if ( doc.containsKey("new_defaultNetworkConfig") && doc["new_defaultNetworkConfig"]==1 )
      {
        Serial.println(F("reset to default network config"));
        aConfig.writeDefaultNetworkConfig("/config/networkconfig.txt");
        
        sendNetworkConfigFlag = true;
      }
      
      // actions sur le esp8266
      if ( doc.containsKey("new_restart") && doc["new_restart"]==1 )
      {
        Serial.println(F("RESTART RESTART RESTART"));
        ESP.restart();
      }

      if ( doc.containsKey("new_refresh") && doc["new_refresh"]==1 )
      {
        Serial.println(F("REFRESH"));
        sendObjectConfigFlag = true;
        sendNetworkConfigFlag = true;
      }

      // modif config
      // write object config
      if (writeObjectConfigFlag)
      {
        writeObjectConfig();
        
        // update statut
        uneFois = true;
      }

      // resend object config
      if (sendObjectConfigFlag)
      {
        sendObjectConfig();
      }

      // write network config
      if (writeNetworkConfigFlag)
      {
        writeObjectConfig();
      }

      // resend network config
      if (sendNetworkConfigFlag)
      {
        sendNetworkConfig();
      }
    }
 
    // clear json buffer
    doc.clear();
}

void notFound(AsyncWebServerRequest *request)
{
    request->send(404, "text/plain", "Not found");
}

void checkCharacter(char* toCheck, char* allowed, char replaceChar)
{
  //char *allowed = "0123456789ABCD*";

  for (int i = 0; i < strlen(toCheck); i++)
  {
    if (!strchr(allowed, toCheck[i]))
    {
      toCheck[i]=replaceChar;
    }
    Serial.print(toCheck[i]);
  }
  Serial.println("");
}

uint16_t checkValeur(uint16_t valeur, uint16_t minValeur, uint16_t maxValeur)
{
  return(min(max(valeur,minValeur), maxValeur));
}


/**
 * This routine turns off the I2C bus and clears it
 * on return SCA and SCL pins are tri-state inputs.
 * You need to call Wire.begin() after this to re-enable I2C
 * This routine does NOT use the Wire library at all.
 *
 * returns 0 if bus cleared
 *         1 if SCL held low.
 *         2 if SDA held low by slave clock stretch for > 2sec
 *         3 if SDA held low after 20 clocks.
 *         
 * from: 
 * https://github.com/esp8266/Arduino/issues/1025
 * http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html
 */
int I2C_ClearBus()
{
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif
  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW)
  { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) 
  { //  vii. If SDA is Low,
    clockCount--;
    // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) 
    {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW)
    { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW)
  { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}

void sendObjectConfig()
{
  aConfig.stringJsonFile("/config/objectconfig.txt", bufferToSend, 500);
  ws.textAll(bufferToSend);
}

void writeObjectConfig()
{
  aConfig.writeObjectConfig("/config/objectconfig.txt");
}

void sendNetworkConfig()
{
  aConfig.stringJsonFile("/config/networkconfig.txt", bufferToSend, 500);
  ws.textAll(bufferToSend);
}

void writeNetworkConfig()
{
  aConfig.writeNetworkConfig("/config/networkconfig.txt");
}

void sendUptime()
{
  uint32_t now = millis() / 1000;
  uint16_t days = now / 86400;
  uint16_t hours = (now%86400) / 3600;
  uint16_t minutes = (now%3600) / 60;
  uint16_t seconds = now % 60;
    
  char toSend[100];
  snprintf(toSend, 100, "{\"uptime\":\"%id %ih %im %is\"}", days, hours, minutes, seconds);

  ws.textAll(toSend);
  //Serial.println(toSend);
}

void sendStatut()
{
  char toSend[100];
  snprintf(toSend, 100, "{\"statutActuel\":%i}", aConfig.objectConfig.statutActuel); 

  ws.textAll(toSend);
}

void sendMaxLed()
{
  char toSend[20];
  snprintf(toSend, 20, "{\"maxLed\":%i}", NB_LEDS_MAX);
  
  ws.textAll(toSend);
}

void sendTagUid()
{
  char toSend[100];
  snprintf(toSend, 100, "{\"lastTagUid\":[\"%02X\",\"%02X\",\"%02X\",\"%02X\"]}", aPn532->uid[0], aPn532->uid[1], aPn532->uid[2], aPn532->uid[3]);

  ws.textAll(toSend);
}

void printTagUid()
{
  stringTagUid(bufferUid);
  Serial.println(bufferUid);
}

void stringTagUid(char * target)
{
  snprintf(target, 12, "%02x:%02x:%02x:%02x", aPn532->uid[0], aPn532->uid[1], aPn532->uid[2], aPn532->uid[3]);
}

void stringTagUid(char * target, uint8_t uidToStr)
{
  snprintf(target, 12, "%02x:%02x:%02x:%02x", 
            aConfig.objectConfig.tagUid[uidToStr][0], 
            aConfig.objectConfig.tagUid[uidToStr][1], 
            aConfig.objectConfig.tagUid[uidToStr][2], 
            aConfig.objectConfig.tagUid[uidToStr][3]);
}

void removeUid(uint8_t uidToRemove)
{
  uint8_t toRemove = min<uint8_t>(MAX_NB_TAG-1, uidToRemove);
  
  if (aConfig.objectConfig.nbTagActuel>0)
  {
    for (uint8_t i=toRemove;i<aConfig.objectConfig.nbTagActuel-1;i++)
    {
      for (uint8_t j=0;j<SIZE_UID;j++)
      {
        aConfig.objectConfig.tagUid[i][j]=aConfig.objectConfig.tagUid[i+1][j];
      }
    }

    for (uint8_t i=aConfig.objectConfig.nbTagActuel-1;i<MAX_NB_TAG;i++)
    {
      for (uint8_t j=0;j<SIZE_UID;j++)
      {
        aConfig.objectConfig.tagUid[i][j]=0;
      }
    }
    
    aConfig.objectConfig.nbTagActuel--;
  
    Serial.println("remove done");
  }
}

void convertStrToRGB(String source, uint8_t* r, uint8_t* g, uint8_t* b)
{
  uint32_t  number = (uint32_t) strtol( &source[1], NULL, 16);
  
  // Split them up into r, g, b values
  *r = number >> 16;
  *g = number >> 8 & 0xFF;
  *b = number & 0xFF;
}
