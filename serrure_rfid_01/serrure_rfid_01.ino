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

#include <Arduino.h>

// WIFI
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

// WEBSOCKET
AsyncWebSocket ws("/ws");

// TASK SCHEDULER
#define _TASK_OO_CALLBACKS
#include <TaskScheduler.h>
Scheduler globalScheduler;

// LED RGB
#include <technolarp_fastled.h>
M_fastled* aFastled;

// RFID
#include <technolarp_pn532.h>
M_pn532* aPn532;

// BUZZER
#define PIN_BUZZER D8
#include <technolarp_buzzer.h>
M_buzzer* buzzer;

// CONFIG
#include "config.h"
M_config aConfig;

// CODE ACTUEL DE LA SERRURE
char codeSerrureActuel[8] = {'0', '0', '0', '0', '0', '0', '0', '0'};

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
enum {
  RECONFIG_CODE = 0,
  RECONFIG_ERREUR = 1,
  RECONFIG_DELAI = 2,
  RECONFIG_TAILLE_CODE = 3
};

// DIVERS
bool uneFois = true;

char bufferReconfig[8] = {'0','0','0','0','0','0','0','0'};
uint8_t modeReconfig = 0;


// HEARTBEAT
unsigned long int previousMillisHB;
unsigned long int currentMillisHB;
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
    //Wire.begin();
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

  // LED RGB
  aFastled = new M_fastled(&globalScheduler);
  aFastled->setNbLed(aConfig.objectConfig.activeLeds);
  aFastled->setBrightness(aConfig.objectConfig.brightness);

  // animation led de depart  
  aFastled->allLedOff();
  for (int i = 0; i < aConfig.objectConfig.activeLeds * 2; i++)
  {
    aFastled->ledOn(i % aConfig.objectConfig.activeLeds, CRGB::Blue);
    delay(50);
    aFastled->ledOn(i % aConfig.objectConfig.activeLeds, CRGB::Black);
  }
  aFastled->allLedOff();
  
  // RFID
  aPn532 = new M_pn532;

  // WIFI
  WiFi.disconnect(true);
  
  /*
  // AP MODE
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(aConfig.networkConfig.apIP, aConfig.networkConfig.apIP, aConfig.networkConfig.apNetMsk);
  WiFi.softAP(aConfig.networkConfig.apName, aConfig.networkConfig.apPassword);
  
  */
  // CLIENT MODE POUR DEBUG
  const char* ssid = "MYDEBUG";
  const char* password = "--------";
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

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
  server.serveStatic("/", LittleFS, "/www/").setDefaultFile("config.html").setTemplateProcessor(processor);
  server.serveStatic("/config", LittleFS, "/config/");
  server.onNotFound(notFound);

  // WEBSOCKET
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // Start server
  server.begin();

  // BUZZER
  buzzer = new M_buzzer(PIN_BUZZER, &globalScheduler);
  buzzer->doubleBeep();

  // HEARTBEAT
  currentMillisHB = millis();
  previousMillisHB = currentMillisHB;
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
  
  // manage task scheduler
  globalScheduler.execute();

  // RFID PN5323
  aPn532->updateRFID();

  // gerer le statut de la serrure
  switch (aConfig.objectConfig.statutSerrureActuel)
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

    case SERRURE_RECONFIG:
      // un parametre doit etre modifie
//      serrureReconfig();
      break;

    case SERRURE_BLINK:
      // blink led pour identification
      serrureBlink();
      break;
      
    default:
      // nothing
      break;
  }

  // HEARTBEAT
  currentMillisHB = millis();
  if(currentMillisHB - previousMillisHB > intervalHB)
  {
    previousMillisHB = currentMillisHB;
    
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

    // on allume les led rouge
    aFastled->allLedOff();
    for (int i = 0; i < aConfig.objectConfig.activeLeds; i++)
    {
      aFastled->setLed(i, CRGB::Red);
    }
    aFastled->ledShow();

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

    // on allume les led verte
    aFastled->allLedOff();
    for (int i = 0; i < aConfig.objectConfig.activeLeds; i++)
    {
      aFastled->setLed(i, CRGB::Green);
    }
    aFastled->ledShow();

    Serial.print(F("SERRURE OUVERTE"));
    Serial.println();
  }
 
  // on check si un tag rfid est present
  checkRfidTag();
}

void serrureErreur()
{
  if (!aFastled->isEnabled() && uneFois)
  {
    uneFois = false;
    aFastled->startAnimSerrureErreur(10, 100);
  }

  if (!aFastled->isAnimActive())
  {
    Serial.println(F("END TASK ERREUR"));

    uneFois = true;

    // si il y a eu trop de faux codes
    if (aConfig.objectConfig.nbErreurCode >= aConfig.objectConfig.nbErreurCodeMax)
    {
      // on bloque la serrure
      aConfig.objectConfig.statutSerrureActuel = SERRURE_BLOQUEE;
    }
    else
    {
      aConfig.objectConfig.statutSerrureActuel = aConfig.objectConfig.statutSerrurePrecedent;
    }

    // ecrire la config sur littleFS
    aConfig.writeObjectConfig("/config/objectconfig.txt");

    // resend config object
    ws.textAll(aConfig.stringJsonFile("/config/objectconfig.txt"));
  }
}

void serrureBloquee()
{
  if (!aFastled->isEnabled() && uneFois)
  {
    uneFois = false;
    aFastled->startAnimSerrureBloquee(aConfig.objectConfig.delaiBlocage*2, 500);
  }

  if (!aFastled->isAnimActive())
  {
    Serial.print(F("END TASK BLOCAGE "));
    Serial.println();
    aConfig.objectConfig.statutSerrureActuel = aConfig.objectConfig.statutSerrurePrecedent;
    aConfig.objectConfig.nbErreurCode = 0;

    uneFois = true;

    // ecrire la config sur littleFS
    aConfig.writeObjectConfig("/config/objectconfig.txt");
    
    // resend config object
    ws.textAll(aConfig.stringJsonFile("/config/objectconfig.txt"));
  }
}



void serrureBlink()
{
  if (!aFastled->isEnabled() && uneFois)
  {
    uneFois = false;
    aFastled->startAnimBlink(15, 200, CRGB::Blue, aConfig.objectConfig.activeLeds);
  }

  if (!aFastled->isAnimActive())
  {
    Serial.println(F("END TASK BLINK"));

    uneFois = true;

    // retour au statut precedent
    aConfig.objectConfig.statutSerrureActuel = aConfig.objectConfig.statutSerrurePrecedent;
    
    // ecrire la config sur littleFS
    aConfig.writeObjectConfig("/config/objectconfig.txt");

    // resend config object
    ws.textAll(aConfig.stringJsonFile("/config/objectconfig.txt"));
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
    
    for (uint8_t i=0;i<aConfig.objectConfig.nbTag;i++)
    {
      bool tagOK2 = true;
      //Serial.print("i: ");
      //Serial.println(i);
      for (uint8_t j=1;j<4;j++)
      {
//        Serial.print("j: ");
//        Serial.print(j);
//        Serial.print("  ");
//        Serial.print(aPn532->uid[j]);
//        Serial.print("/");
//        Serial.print(aConfig.objectConfig.tagUid[i][j]);
        
        if ( (aPn532->uidLength>=4) && (aPn532->uid[j] != aConfig.objectConfig.tagUid[i][j]) )
        {
          tagOK2 = false;
          //Serial.print(" result ");
          //Serial.print(tagOK2);
        }
       // Serial.println("");
      }

      if (tagOK2)
      {
        tagOK = true;
      }
    }

    // change lock status if needed
    if (tagOK)
    {
      buzzer->shortBeep();

      aConfig.objectConfig.statutSerrureActuel = !aConfig.objectConfig.statutSerrureActuel;
      aConfig.objectConfig.nbErreurCode = 0;
      uneFois = true;
      Serial.println(F("tag OK, change lock status"));
    }
    else
    {
      Serial.println(F("mauvais tag"));
      buzzer->longBeep();
    }

    // send uid 
    sendTagUid();
  }
}


String processor(const String& var)
{  
  if (var == "MAXLEDS")
  {
    return String(aFastled->getNbMaxLed());
  }

  if (var == "APNAME")
  {
    return String(aConfig.networkConfig.apName);
  }

  if (var == "OBJECTNAME")
  {
    return String(aConfig.objectConfig.objectName);
  }
   
  return String();
}


void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) 
{
   switch (type) 
    {
      case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        // send config value to html
        ws.textAll(aConfig.stringJsonFile("/config/objectconfig.txt"));
        ws.textAll(aConfig.stringJsonFile("/config/networkconfig.txt"));

        // send volatile info
        sendUptime();
        
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
    char buffer[100];
    data[len] = 0;
    sprintf(buffer,"%s\n", (char*)data);
    Serial.print(buffer);
    
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, buffer);
    if (error)
    {
      Serial.println(F("Failed to deserialize buffer"));
    }
    else
    {
      // write config or not
      bool writeObjectConfig = false;
      bool sendObjectConfig = false;
      bool writeNetworkConfig = false;
      bool sendNetworkConfig = false;
      
      // modif object config
      if (doc.containsKey("new_objectName"))
      {
        strlcpy(  aConfig.objectConfig.objectName,
                  doc["new_objectName"],
                  sizeof(aConfig.objectConfig.objectName));

        writeObjectConfig = true;
        sendObjectConfig = true;
      }

      /*
      if (doc.containsKey("new_codeSerrure")) 
      {
        strlcpy(  aConfig.objectConfig.codeSerrure,
                  doc["new_codeSerrure"],
                  sizeof(aConfig.objectConfig.codeSerrure));

        // check for unsupported char
        checkCharacter(aConfig.objectConfig.codeSerrure, "0123456789ABCD*", '0');
        
        writeObjectConfig = true;
        sendObjectConfig = true;
      }
      */

      if (doc.containsKey("new_objectId")) 
      {
        uint16_t tmpValeur = doc["new_objectId"];
        aConfig.objectConfig.objectId = checkValeur(tmpValeur,1,1000);

        writeObjectConfig = true;
        sendObjectConfig = true;
      }

      if (doc.containsKey("new_groupId")) 
      {
        uint16_t tmpValeur = doc["new_groupId"];
        aConfig.objectConfig.groupId = checkValeur(tmpValeur,1,1000);
        
        writeObjectConfig = true;
        sendObjectConfig = true;
      }

      if (doc.containsKey("new_activeLeds")) 
      {
        aFastled->allLedOff();
        
        uint16_t tmpValeur = doc["new_activeLeds"];
        aConfig.objectConfig.activeLeds = checkValeur(tmpValeur,1,aFastled->getNbMaxLed());
        aFastled->setNbLed(aConfig.objectConfig.activeLeds);
        
        writeObjectConfig = true;
        sendObjectConfig = true;
      }

      if (doc.containsKey("new_brightness"))
      {
        uint16_t tmpValeur = doc["new_brightness"];
        aConfig.objectConfig.brightness = checkValeur(tmpValeur,0,255);
        aFastled->setBrightness(aConfig.objectConfig.brightness);
        
        writeObjectConfig = true;
        sendObjectConfig = true;
      }

      /*
      if (doc.containsKey("new_tailleCode")) 
      {
        uint16_t tmpValeur = doc["new_tailleCode"];
        aConfig.objectConfig.tailleCode = checkValeur(tmpValeur,1,8);
        
        writeObjectConfig = true;
        sendObjectConfig = true;
      }
      */

      if (doc.containsKey("new_nbErreurCodeMax")) 
      {
        uint16_t tmpValeur = doc["new_nbErreurCodeMax"];
        aConfig.objectConfig.nbErreurCodeMax = checkValeur(tmpValeur,1,50);
        
        writeObjectConfig = true;
        sendObjectConfig = true;
      }
      
      if (doc.containsKey("new_delaiBlocage")) 
      {
        uint16_t tmpValeur = doc["new_delaiBlocage"];
        aConfig.objectConfig.delaiBlocage = checkValeur(tmpValeur,5,300);
        
        writeObjectConfig = true;
        sendObjectConfig = true;
      }
      
      if (doc.containsKey("new_statutSerrureActuel"))
      {
        aConfig.objectConfig.statutSerrurePrecedent = aConfig.objectConfig.statutSerrureActuel;
        aConfig.objectConfig.statutSerrureActuel = doc["new_statutSerrureActuel"];

        aFastled->disable();
        aFastled->setAnim(0);
        
        writeObjectConfig = true;
        sendObjectConfig = true;
      }

      if ( doc.containsKey("new_removeUid") && doc["new_removeUid"]==1 )
      {
        uint16_t tmpValeur = doc["new_removeUid"];
        Serial.print(F("Remove tag UID: "));
        Serial.println(tmpValeur);
      }

      if ( doc.containsKey("new_addUid") && doc["new_addUid"]==1 )
      {
        if (aConfig.objectConfig.nbTag<9)
        {
          Serial.print(F("Add tag UID: "));
          printTagUid();
          
          for (uint8_t  i=0;i<aPn532->uidLength;i++)
          {
            aConfig.objectConfig.tagUid[aConfig.objectConfig.nbTag][i]=aPn532->uid[i];
          }
          aConfig.objectConfig.nbTag++;          

          writeObjectConfig = true;
          sendObjectConfig = true;
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

        writeObjectConfig = true;
        sendObjectConfig = true;
      }
        
      // modif network config
      if (doc.containsKey("new_apName")) 
      {
        strlcpy(  aConfig.networkConfig.apName,
                  doc["new_apName"],
                  sizeof(aConfig.networkConfig.apName));

        // check for unsupported char
        checkCharacter(aConfig.networkConfig.apName, "ABCDEFGHIJKLMNOPQRSTUVWYZ", 'A');
        
        writeNetworkConfig = true;
        sendNetworkConfig = true;
      }

      if (doc.containsKey("new_apPassword")) 
      {
        strlcpy(  aConfig.networkConfig.apPassword,
                  doc["new_apPassword"],
                  sizeof(aConfig.networkConfig.apPassword));

        writeNetworkConfig = true;
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

          writeNetworkConfig = true;
        }
        
        sendNetworkConfig = true;
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

          writeNetworkConfig = true;
        }

        sendNetworkConfig = true;
      }

      if ( doc.containsKey("new_defaultObjectConfig") && doc["new_defaultObjectConfig"]==1 )
      {
        Serial.println(F("reset to default object config"));
        aConfig.writeDefaultObjectConfig("/config/objectconfig.txt");
        
        sendObjectConfig = true;
        uneFois = true;
      }

      if ( doc.containsKey("new_defaultNetworkConfig") && doc["new_defaultNetworkConfig"]==1 )
      {
        Serial.println(F("reset to default network config"));
        aConfig.writeDefaultNetworkConfig("/config/networkconfig.txt");
        
        sendNetworkConfig = true;
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
        sendObjectConfig = true;
        sendNetworkConfig = true;
      }

      // modif config
      // write object config
      if (writeObjectConfig)
      {
        aConfig.writeObjectConfig("/config/objectconfig.txt");
        //aConfig.printJsonFile("/config/objectconfig.txt");
        
        // update statut
        uneFois = true;
      }

      // resend object config
      if (sendObjectConfig)
      {
        ws.textAll(aConfig.stringJsonFile("/config/objectconfig.txt"));
      }

      // write network config
      if (writeNetworkConfig)
      {
        aConfig.writeNetworkConfig("/config/networkconfig.txt");
        //aConfig.printJsonFile("/config/networkconfig.txt");
      }

      // resend network config
      if (sendNetworkConfig)
      {
        ws.textAll(aConfig.stringJsonFile("/config/networkconfig.txt"));
      }
    }
 
    // clear json buffer
    doc.clear();
  }
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


void sendUptime()
{
  uint32_t now = millis() / 1000;
  uint16_t days = now / 86400;
  uint16_t hours = (now%86400) / 3600;
  uint16_t minutes = (now%3600) / 60;
  uint16_t seconds = now % 60;
    
  String toSend = "{\"uptime\":\"";
  toSend+= String(days) + String("d ") + String(hours) + String("h ") + String(minutes) + String("m ") + String(seconds) + String("s");
  toSend+= "\"}";

  ws.textAll(toSend);
  Serial.println(toSend);
}

void sendTagUid()
{
  String toSend = "{\"lastTagUid\":[";
  for (uint8_t i=0;i<aPn532->uidLength;i++)
  {
    toSend+= "\"" + String(aPn532->uid[i], HEX) + "\"";
    if (i<aPn532->uidLength-1)
    {
      toSend+= ",";
    }
  }
  
  toSend+= "]}";

  ws.textAll(toSend);
}

void printTagUid()
{
  for (int i=0;i<aPn532->uidLength;i++)
  {
    Serial.print(aPn532->uid[i], HEX);
    if (i<aPn532->uidLength-1)
    {
      Serial.print(":");
    }
  }
  Serial.println("");
}
