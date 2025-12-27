/*
   ----------------------------------------------------------------------------
   TECHNOLARP - https://technolarp.github.io/
   SERRURE RFID 01 - https://github.com/technolarp/serrure_rfid_01
   version 1.3.0 - 11/2025
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
 ajouter option pour moitié led verte / rouge
 check ajout new uid from html
 print memory size
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

// MDNS
#include <ESP8266mDNS.h>

// CONFIG
#include "config.h"
M_config aConfig;

#define BUFFERSENDSIZE 1024
char bufferToSend[BUFFERSENDSIZE];
char bufferUid[12];

// STATUTS DE L OBJET
enum {
  OBJET_OUVERT = 0,
  OBJET_FERME = 1,
  OBJET_BLOQUE = 2,
  OBJET_ERREUR = 3,
  OBJET_RECONFIG = 4,
  OBJET_BLINK = 5
};

// DIVERS
bool uneFois = true;

// HEARTBEAT
uint32_t previousMillisHB;
uint32_t intervalHB;

// FUNCTION DECLARATIONS
uint16_t checkValeur(uint16_t valeur, uint16_t minValeur, uint16_t maxValeur);
void serrureFermee();
void serrureOuverte();
void serrureErreur();
void serrureBloquee();
void serrureBlink();
void checkRfidTag();
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len); 
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len); 
void handleWebsocketBuffer();
void notFound(AsyncWebServerRequest *request);
void checkCharacter(char *toCheck, const char *allowed, char replaceChar);
void sendObjectConfig();
void writeObjectConfig();
void sendNetworkConfig();
void writeNetworkConfig();
void sendUptime();
void sendStatut();
void sendMaxLed();
void sendTagUid();
void printTagUid();
void stringTagUid(char * target);
void stringTagUid(char * target, uint8_t uidToStr);
void removeUid(uint8_t uidToRemove);
void convertStrToRGB(const char * source, uint8_t* r, uint8_t* g, uint8_t* b);


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
  Serial.println(F("version 1.3.0 - 11/2025"));
  Serial.println(F("----------------------------------------------------------------------------"));

  // I2C RESET
  aConfig.i2cReset();
  
  // CONFIG OBJET
  Serial.println(F(""));
  Serial.println(F(""));
  aConfig.mountFS();
  aConfig.listDir("/");
  aConfig.listDir("/config");
  aConfig.listDir("/www");
  
  aConfig.printJsonFile("/config/objectconfig.json");
  aConfig.readObjectConfig("/config/objectconfig.json");

  aConfig.printJsonFile("/config/networkconfig.json");
  aConfig.readNetworkConfig("/config/networkconfig.json");
  
  // RFID
  aPn532 = new M_pn532;

  // FASTLED
  aFastled.setNbLed(aConfig.objectConfig.activeLeds);
  aFastled.setControlBrightness(aConfig.objectConfig.scintillementOnOff);
  aFastled.setIntervalControlBrightness(aConfig.objectConfig.intervalScintillement);
  
  // animation led de depart
  aFastled.animationDepart(50, aFastled.getNbLed() * 2, CRGB::Blue);

  // LOOP TO WIFI CLIENT
  Serial.println(F(""));
  Serial.println(F("connecting to wifi as client"));

  for (uint8_t i = 0; i < WIFI_CLIENTS; i++)
  {
    if (aConfig.networkConfig.active[i] && strlen(aConfig.networkConfig.ssid[i])>0)
    {
      if (WiFi.status() != WL_CONNECTED)
      {
        Serial.print(F("ssid: "));
        Serial.print(aConfig.networkConfig.ssid[i]);
        Serial.print(F(" - delay: "));
        Serial.print(aConfig.networkConfig.wifiConnectDelay);
        Serial.println(F(" seconds"));

        bool ledState=true;
        bool wifiFlag=true;
        
        WiFi.disconnect(true);
        delay(500);
        WiFi.begin(aConfig.networkConfig.ssid[i], aConfig.networkConfig.password[i]);

        FastLED.clear();
        // Loop continuously while WiFi is not connected
        while ( (WiFi.status() != WL_CONNECTED) && (wifiFlag) )
        {
          delay(100);
          Serial.print("/");
          
          if (ledState)
          {
            aFastled.ledOn(i%aConfig.objectConfig.activeLeds, CRGB::Blue, false);
          }
          else
          {
            aFastled.ledOn(i%aConfig.objectConfig.activeLeds, CRGB::Black, false);
          }

          aFastled.ledShow();
          ledState = !ledState;

          if (millis() - previousMillisHB > (aConfig.networkConfig.wifiConnectDelay*1000) )
          {
            previousMillisHB = millis();
            wifiFlag = false;
          }
        }
      }

      Serial.println(F(" "));
      if (WiFi.status() == WL_CONNECTED)
      {
        Serial.print(F("connected to "));
        Serial.println(aConfig.networkConfig.ssid[i]);
        Serial.print(F("IP address: "));
        Serial.println(WiFi.localIP());

        // MDNS
        if (!MDNS.begin(aConfig.objectConfig.objectName))
        {
          Serial.println("Error setting up MDNS responder!");
        }
        Serial.println("mDNS responder started");
        Serial.print(F("connect on webUI admin page : http://"));
        Serial.print(aConfig.objectConfig.objectName);
        Serial.println(F(".local"));        
      }
      else
      {
        if (aConfig.networkConfig.disableSsid)
        {
          Serial.print(F("disable this ssid: "));
          Serial.println(aConfig.networkConfig.ssid[i]);
          aConfig.networkConfig.active[i]=false;
          writeNetworkConfig();
        }
        if (aConfig.networkConfig.rebootEsp)
        {
          Serial.println(F("reboot"));
          delay(1000);
          ESP.restart();
        }        
      }
    }    
  }
  Serial.println(" ");
  
  // WIFI AP MODE
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(F("failed to connect to wifi as client, creating a wifi AP: "));
    Serial.println(aConfig.networkConfig.apName);

    // WiFi.mode(WIFI_AP_STA);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(aConfig.networkConfig.apIP,aConfig.networkConfig.apIP,aConfig.networkConfig.apNetMsk);
    bool apRC = WiFi.softAP(aConfig.networkConfig.apName, aConfig.networkConfig.apPassword);

    if (apRC)
    {
      Serial.println(F("AP WiFi OK"));
    }
    else
    {
      Serial.println(F("AP WiFi failed"));
    }

    // Print ESP soptAP IP Address
    Serial.print(F("softAPIP: "));
    Serial.println(WiFi.softAPIP());

    // MDNS
    if (!MDNS.begin("technolarp"))
    {
      Serial.println("Error setting up MDNS responder!");
    }
    Serial.println("mDNS responder started");
    Serial.println(F("connect on webUI admin page : http://technolarp.local"));
  }

  // WEB SERVER
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
  // WEBSOCKET
  ws.cleanupClients();

  // FASTLED
  aFastled.updateAnimation();

  // CONTROL BRIGHTNESS
  aFastled.controlBrightness(aConfig.objectConfig.brightness);
  
  // BUZZER
  buzzer.update();

  // RFID PN5323
  aPn532->updateRFID();

  // gerer le statut de la serrure
  switch (aConfig.objectConfig.statutActuel)
  {
    case OBJET_FERME:
      // la serrure est fermee
      serrureFermee();
      break;

    case OBJET_OUVERT:
      // la serrure est ouverte
      serrureOuverte();
      break;

    case OBJET_BLOQUE:
      // la serrure est bloquee
      serrureBloquee();
      break;

    case OBJET_ERREUR:
      // un code incorrect a ete entrer
      serrureErreur();
      break;

    case OBJET_BLINK:
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

  // MDNS
  MDNS.update();
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

    aFastled.animationBlink02Start(100, 1100, aConfig.objectConfig.couleurs[0], CRGB::Black);
  }

  // fin de l'animation erreur 
  if(!aFastled.isAnimActive()) 
  {
    uneFois = true;
    
    // si il y a eu trop de faux codes
    if (aConfig.objectConfig.nbErreurCode >= aConfig.objectConfig.nbErreurCodeMax)
    {
      // on bloque la serrure
      aConfig.objectConfig.statutActuel = OBJET_BLOQUE;
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

    aFastled.animationBlink02Start(500, aConfig.objectConfig.delaiBlocage*1000, aConfig.objectConfig.couleurs[0], aConfig.objectConfig.couleurs[1]);
  }

  // fin de l'animation blocage
  if(!aFastled.isAnimActive()) 
  {
    uneFois = true;

    aConfig.objectConfig.statutActuel = aConfig.objectConfig.statutPrecedent;
    aConfig.objectConfig.nbErreurCode = 0;

    writeObjectConfig();
    sendObjectConfig();

    Serial.println(F("END BLOCAGE "));
  }
}

void serrureBlink()
{
  if (uneFois)
  {
    uneFois = false;
    Serial.println(F("SERRURE BLINK"));

    aFastled.animationBlink02Start(100, 3000, CRGB::Blue, CRGB::Black);
  }

  // fin de l'animation blink
  if(!aFastled.isAnimActive()) 
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
    
    printTagUid();

    // check with store uid
    bool tagOkFound = false;
    
    for (uint8_t i=0;i<aConfig.objectConfig.nbTagEnMemoireActuel;i++)
    {
      bool isTagOk = true;
      for (uint8_t j=1;j<4;j++)
      {        
        if ( (aPn532->uidLength>=4) && (aPn532->uid[j] != aConfig.objectConfig.tagUid[i][j]) )
        {
          isTagOk = false;
        }
      }

      if (isTagOk)
      {
        tagOkFound = true;
      }
    }

    // changer le statut si le tag est OK
    if (tagOkFound)
    {
      buzzer.shortBeep();

      aConfig.objectConfig.statutActuel = !aConfig.objectConfig.statutActuel;
      aConfig.objectConfig.nbErreurCode = 0;
      uneFois = true;
      Serial.println(F("tag OK, on change le statut de la serrure"));
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
      aConfig.objectConfig.statutActuel = OBJET_ERREUR;

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
        
      case WS_EVT_PING:
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
    JsonDocument doc;
    
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
      if (doc["new_objectName"].is<const char*>())
    {
      strlcpy(aConfig.objectConfig.objectName,
              doc["new_objectName"],
              sizeof(aConfig.objectConfig.objectName));

      // lowercase
      for (uint8_t i = 0; i < sizeof(aConfig.objectConfig.objectName); i++)
      {
        aConfig.objectConfig.objectName[i] = tolower(aConfig.objectConfig.objectName[i]);
      }
      
      // check character
      char const * listeCheck = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-";
      checkCharacter(aConfig.objectConfig.objectName, listeCheck, '-');

        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }
      
      if (doc["new_objectId"].is<unsigned short>()) 
      {
        uint16_t tmpValeur = doc["new_objectId"];
        aConfig.objectConfig.objectId = checkValeur(tmpValeur,1,1000);

        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if (doc["new_groupId"].is<unsigned short>()) 
      {
        uint16_t tmpValeur = doc["new_groupId"];
        aConfig.objectConfig.groupId = checkValeur(tmpValeur,1,1000);
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if (doc["new_activeLeds"].is<unsigned short>()) 
      {
        aFastled.allLedOff();
        
        uint16_t tmpValeur = doc["new_activeLeds"];
        
        aConfig.objectConfig.activeLeds = checkValeur(tmpValeur,1,NB_LEDS_MAX);
        aFastled.setNbLed(aConfig.objectConfig.activeLeds);
        uneFois = true;

        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if (doc["new_brightness"].is<unsigned short>())
      {
        uint16_t tmpValeur = doc["new_brightness"];
        aConfig.objectConfig.brightness = checkValeur(tmpValeur,0,255);
        aFastled.setBrightness(aConfig.objectConfig.brightness);
        uneFois=true;
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if (doc["new_intervalScintillement"].is<unsigned short>())
      {
        uint16_t tmpValeur = doc["new_intervalScintillement"];
        aConfig.objectConfig.intervalScintillement = checkValeur(tmpValeur,0,1000);
        aFastled.setIntervalControlBrightness(aConfig.objectConfig.intervalScintillement);
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }
      
      if (doc["new_scintillementOnOff"].is<unsigned short>())
      {
        uint16_t tmpValeur = doc["new_scintillementOnOff"];
        aConfig.objectConfig.scintillementOnOff = checkValeur(tmpValeur,0,1);
        aFastled.setControlBrightness(aConfig.objectConfig.scintillementOnOff);
        
        if (aConfig.objectConfig.scintillementOnOff == 0)
        {
          FastLED.setBrightness(aConfig.objectConfig.brightness);
        }
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if (doc["new_nbErreurCodeMax"].is<unsigned short>()) 
      {
        uint16_t tmpValeur = doc["new_nbErreurCodeMax"];
        aConfig.objectConfig.nbErreurCodeMax = checkValeur(tmpValeur,1,50);
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }
      
      if (doc["new_delaiBlocage"].is<unsigned short>()) 
      {
        uint16_t tmpValeur = doc["new_delaiBlocage"];
        aConfig.objectConfig.delaiBlocage = checkValeur(tmpValeur,5,300);
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }
      
      if (doc["new_statutActuel"].is<unsigned short>())
      {
        uint16_t tmpValeur = doc["new_statutActuel"];
        aConfig.objectConfig.statutPrecedent=aConfig.objectConfig.statutActuel;
        aConfig.objectConfig.statutActuel=tmpValeur;

        aFastled.setAnimation(0);
    
        uneFois=true;
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if (doc["new_nbTagEnMemoireMax"].is<unsigned short>()) 
      {
        uint16_t tmpValeur = doc["new_nbTagEnMemoireMax"];
        aConfig.objectConfig.nbTagEnMemoireMax = checkValeur(tmpValeur,1,MAX_NB_TAG);
        aConfig.objectConfig.nbTagEnMemoireActuel=min<uint8_t>(aConfig.objectConfig.nbTagEnMemoireActuel,aConfig.objectConfig.nbTagEnMemoireMax);
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }
    
      if ( doc["new_removeUid"].is<unsigned short>() && doc["new_removeUid"]>=0 && doc["new_removeUid"]<MAX_NB_TAG )
      {
        uint16_t uidToRemove = doc["new_removeUid"];
        Serial.print(F("Remove tag UID: "));
        Serial.println(uidToRemove);
    
        removeUid(uidToRemove);
    
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if ( doc["new_addLastUid"].is<unsigned short>() && doc["new_addLastUid"]==1 )
      {
        if (aConfig.objectConfig.nbTagEnMemoireActuel<min<uint8_t>(aConfig.objectConfig.nbTagEnMemoireMax, MAX_NB_TAG))
        {
          for (uint8_t  i=0;i<SIZE_UID;i++)
          {
            aConfig.objectConfig.tagUid[aConfig.objectConfig.nbTagEnMemoireActuel][i]=aPn532->uid[i];
          }
          aConfig.objectConfig.nbTagEnMemoireActuel++;          

          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }
        else
        {
          Serial.println(F("too much tags"));
        }
      }

      if (doc["new_newTagUid"].is<JsonVariant>()) 
      {
        if (aConfig.objectConfig.nbTagEnMemoireActuel<min<uint8_t>(aConfig.objectConfig.nbTagEnMemoireMax, MAX_NB_TAG))
        {
          JsonArray newUidValue = doc["new_newTagUid"];
            
          for (uint8_t  i=0;i<SIZE_UID;i++)
          {
            const char* tagUid_0 = newUidValue[i];
            uint8_t hexValue = (uint8_t) strtol( &tagUid_0[0], NULL, 16);
            
            aConfig.objectConfig.tagUid[aConfig.objectConfig.nbTagEnMemoireActuel][i]=hexValue;
            aPn532->uid[i]=hexValue;
          }
          aConfig.objectConfig.nbTagEnMemoireActuel++;          

          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }
        else
        {
          Serial.println(F("too much tags"));
        }
      }

      if ( doc["new_resetErreur"].is<unsigned short>() && doc["new_resetErreur"]==1 )
      {
        Serial.println(F("Reset erreurs"));
        aConfig.objectConfig.nbErreurCode = 0;

        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

      if (doc["new_couleurs"].is<JsonVariant>()) 
      {
        JsonArray newCouleur = doc["new_couleurs"];

        uint8_t i = newCouleur[0];
        char newColorStr[8];
        strncpy(newColorStr, newCouleur[1], 8);
          
        uint8_t r;
        uint8_t g;
        uint8_t b;
          
        convertStrToRGB(newColorStr, &r, &g, &b);
        aConfig.objectConfig.couleurs[i].red=r;
        aConfig.objectConfig.couleurs[i].green=g;
        aConfig.objectConfig.couleurs[i].blue=b;
        
        uneFois=true;
          
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }
        
      // **********************************************
      // modif network config
    if (doc["new_active"].is<JsonVariant>())
    {
      JsonArray newActive = doc["new_active"];

      uint8_t x = checkValeur(newActive[0], 0, WIFI_CLIENTS - 1);
      uint8_t y = checkValeur(newActive[1], 0, 1);

      aConfig.networkConfig.active[x] = y;

      writeNetworkConfigFlag = true;
      sendNetworkConfigFlag = true;

      // update statut
      uneFois = true;
    }

    if (doc["new_ssid"].is<JsonVariant>())
    {
      JsonArray newSsid = doc["new_ssid"];

      uint8_t x = checkValeur(newSsid[0], 0, WIFI_CLIENTS - 1);
      strlcpy(aConfig.networkConfig.ssid[x],
              newSsid[1],
              sizeof(aConfig.networkConfig.ssid[x]));

      writeNetworkConfigFlag = true;
      sendNetworkConfigFlag = true;

      // update statut
      uneFois = true;
    }

    if (doc["new_password"].is<JsonVariant>())
    {
      JsonArray newPassword = doc["new_password"];

      uint8_t x = checkValeur(newPassword[0], 0, WIFI_CLIENTS - 1);
      strlcpy(aConfig.networkConfig.password[x],
              newPassword[1],
              sizeof(aConfig.networkConfig.password[x]));

      writeNetworkConfigFlag = true;
      sendNetworkConfigFlag = true;

      // update statut
      uneFois = true;
    }

    if (doc["new_wifiConnectDelay"].is<unsigned short>())
    {
      uint16_t tmpValeur = doc["new_wifiConnectDelay"];
      aConfig.networkConfig.wifiConnectDelay = checkValeur(tmpValeur, 1, 255);

      writeNetworkConfigFlag = true;
      sendNetworkConfigFlag = true;
    }

    if (doc["new_disableSsid"].is<unsigned short>())
    {
      uint16_t tmpValeur = doc["new_disableSsid"];
      aConfig.networkConfig.disableSsid = checkValeur(tmpValeur, 0, 1);

      writeNetworkConfigFlag = true;
      sendNetworkConfigFlag = true;
    }

    if (doc["new_rebootEsp"].is<unsigned short>())
    {
      uint16_t tmpValeur = doc["new_rebootEsp"];
      aConfig.networkConfig.rebootEsp = checkValeur(tmpValeur, 0, 1);

      writeNetworkConfigFlag = true;
      sendNetworkConfigFlag = true;
    }

    if (doc["new_apName"].is<const char*>())
    {
      strlcpy(aConfig.networkConfig.apName,
              doc["new_apName"],
              sizeof(aConfig.networkConfig.apName));

      // uppercase
      for (uint8_t i = 0; i < sizeof(aConfig.objectConfig.objectName); i++)
      {
        aConfig.networkConfig.apName[i] = toupper(aConfig.networkConfig.apName[i]);
      }
      
      // check for unsupported char
      char const * listeCheck = "ABCDEFGHIJKLMNOPQRSTUVWYXZ0123456789_-";
      checkCharacter(aConfig.networkConfig.apName, listeCheck, 'A');      

      writeNetworkConfigFlag = true;
      sendNetworkConfigFlag = true;
    }

    if (doc["new_apPassword"].is<const char*>()) 
      {
        strlcpy(  aConfig.networkConfig.apPassword,
                  doc["new_apPassword"],
                  SIZE_ARRAY);
      
        writeNetworkConfigFlag = true;
        sendNetworkConfigFlag = true;
      }
      
      if (doc["new_apIP"].is<const char*>()) 
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
      
      if (doc["new_apNetMsk"].is<const char*>()) 
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
      
      // actions sur le esp8266
      if ( doc["new_restart"].is<unsigned char>() && doc["new_restart"]==1 )
      {
        Serial.println(F("RESTART RESTART RESTART"));
        ESP.restart();
      }
      
      if ( doc["new_refresh"].is<unsigned char>() && doc["new_refresh"]==1 )
      {
        Serial.println(F("REFRESH"));
      
        sendObjectConfigFlag = true;
        sendNetworkConfigFlag = true;
      }
      
      if ( doc["new_defaultObjectConfig"].is<unsigned char>() && doc["new_defaultObjectConfig"]==1 )
      {
        aConfig.writeDefaultObjectConfig("/config/objectconfig.json");
        Serial.println(F("reset to default object config"));
      
        aFastled.allLedOff();
        aFastled.setNbLed(aConfig.objectConfig.activeLeds);          
        aFastled.setControlBrightness(aConfig.objectConfig.scintillementOnOff);
        aFastled.setIntervalControlBrightness(aConfig.objectConfig.intervalScintillement);
        
        sendObjectConfigFlag = true;
        uneFois = true;
      }
      
      if ( doc["new_defaultNetworkConfig"].is<unsigned char>() && doc["new_defaultNetworkConfig"]==1 )
      {
        aConfig.writeDefaultNetworkConfig("/config/networkconfig.json");
        Serial.println(F("reset to default network config"));          
        
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
        writeNetworkConfig();
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

void checkCharacter(char *toCheck, const char *allowed, char replaceChar)
{
  //char *allowed = "0123456789ABCD*";

  for (uint8_t i = 0; i < strlen(toCheck); i++)
  {
    Serial.print(toCheck[i]);
    if (strchr(allowed, toCheck[i]) == NULL)
    {
      toCheck[i]=replaceChar;
    }
    Serial.print(toCheck[i]);
  }
  Serial.println(F(""));
}

uint16_t checkValeur(uint16_t valeur, uint16_t minValeur, uint16_t maxValeur)
{
  return(min(max(valeur,minValeur), maxValeur));
}

void sendObjectConfig()
{
  aConfig.stringJsonFile("/config/objectconfig.json", bufferToSend, BUFFERSENDSIZE);
  ws.textAll(bufferToSend);
}

void writeObjectConfig()
{
  aConfig.writeObjectConfig("/config/objectconfig.json");
}

void sendNetworkConfig()
{
  aConfig.stringJsonFile("/config/networkconfig.json", bufferToSend, BUFFERSENDSIZE);
  ws.textAll(bufferToSend);
}

void writeNetworkConfig()
{
  aConfig.writeNetworkConfig("/config/networkconfig.json");
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
  snprintf(target, 12, "%02X:%02X:%02X:%02X", aPn532->uid[0], aPn532->uid[1], aPn532->uid[2], aPn532->uid[3]);
}

void stringTagUid(char * target, uint8_t uidToStr)
{
  snprintf(target, 12, "%02X:%02X:%02X:%02X", 
            aConfig.objectConfig.tagUid[uidToStr][0], 
            aConfig.objectConfig.tagUid[uidToStr][1], 
            aConfig.objectConfig.tagUid[uidToStr][2], 
            aConfig.objectConfig.tagUid[uidToStr][3]);
}

void removeUid(uint8_t uidToRemove)
{
  uint8_t toRemove = min<uint8_t>(MAX_NB_TAG-1, uidToRemove);
  
  if (aConfig.objectConfig.nbTagEnMemoireActuel>0)
  {
    for (uint8_t i=toRemove;i<aConfig.objectConfig.nbTagEnMemoireActuel-1;i++)
    {
      for (uint8_t j=0;j<SIZE_UID;j++)
      {
        aConfig.objectConfig.tagUid[i][j]=aConfig.objectConfig.tagUid[i+1][j];
      }
    }

    for (uint8_t i=aConfig.objectConfig.nbTagEnMemoireActuel-1;i<MAX_NB_TAG;i++)
    {
      for (uint8_t j=0;j<SIZE_UID;j++)
      {
        aConfig.objectConfig.tagUid[i][j]=0;
      }
    }
    
    aConfig.objectConfig.nbTagEnMemoireActuel--;
  
    Serial.println(F("remove done"));
  }
}

void convertStrToRGB(const char * source, uint8_t* r, uint8_t* g, uint8_t* b)
{
  uint32_t  number = (uint32_t) strtol( &source[1], NULL, 16);
  
  // Split them up into r, g, b values
  *r = number >> 16;
  *g = number >> 8 & 0xFF;
  *b = number & 0xFF;
}
