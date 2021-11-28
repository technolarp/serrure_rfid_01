#include <LittleFS.h>
#include <ArduinoJson.h> // arduino json v6  // https://github.com/bblanchon/ArduinoJson

// to upload config dile : https://github.com/earlephilhower/arduino-esp8266littlefs-plugin/releases
#define SIZE_ARRAY 20
#define MAX_NB_TAG 10
#define SIZE_UID 4

#define JSONBUFFERSIZE 2048

#include <IPAddress.h>
#include <FastLED.h>

class M_config
{
  public:
  // structure stockage
  struct OBJECT_CONFIG_STRUCT
  {
    uint16_t objectId;
  	uint16_t groupId;
      
  	char objectName[SIZE_ARRAY];
  	
  	uint8_t activeLeds;
    uint8_t brightness;
  	uint8_t nbErreurCodeMax;
  	uint8_t nbErreurCode;
  	uint16_t delaiBlocage;
  	uint8_t statutActuel;
  	uint8_t statutPrecedent;

    uint8_t nbTagStock;
    uint8_t nbTagActuel;
    uint8_t tagUid[MAX_NB_TAG][SIZE_UID];

    CRGB couleurs[2];
  };
  
  // creer une structure
  OBJECT_CONFIG_STRUCT objectConfig;

  struct NETWORK_CONFIG_STRUCT
  {
    IPAddress apIP;
    IPAddress apNetMsk;
    char apName[SIZE_ARRAY];
    char apPassword[SIZE_ARRAY];
  };
  
  // creer une structure
  NETWORK_CONFIG_STRUCT networkConfig;
  
  
  M_config()
  {
  }
  
  void readObjectConfig(const char * filename)
  {
    // lire les données depuis le fichier littleFS
    // Open file for reading
    File file = LittleFS.open(filename, "r");
    if (!file) 
    {
      Serial.println(F("Failed to open file for reading"));
      return;
    }
    else
    {
      Serial.println(F("File opened"));
    }
  
    StaticJsonDocument<JSONBUFFERSIZE> doc;
    //DynamicJsonDocument doc(JSONBUFFERSIZE);
    
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, file);
    if (error)
    {
      Serial.println(F("Failed to deserialize file in read object"));
      Serial.println(error.c_str());
    }
    else
    {
      // Copy values from the JsonObject to the Config
      objectConfig.objectId = doc["objectId"];
      objectConfig.groupId = doc["groupId"];
      
      objectConfig.activeLeds = doc["activeLeds"];
      objectConfig.brightness = doc["brightness"];      
      objectConfig.nbErreurCodeMax = doc["nbErreurCodeMax"];
      objectConfig.nbErreurCode = doc["nbErreurCode"];
      objectConfig.delaiBlocage = doc["delaiBlocage"];      
      
      objectConfig.statutActuel = doc["statutActuel"];
      objectConfig.statutPrecedent = doc["statutPrecedent"];
      objectConfig.nbTagStock = doc["nbTagStock"];
      objectConfig.nbTagActuel = doc["nbTagActuel"];

      // read uid array
      if (doc.containsKey("tagUid"))
      {
        //JsonArray uidDeserialize = docDeserialize["uid"];
        JsonArray tagUidArray = doc["tagUid"];
       
        for (uint8_t i=0;i<MAX_NB_TAG;i++)
        {
          JsonArray uidArray=tagUidArray[i];

          for (uint8_t j=0;j<SIZE_UID;j++)
          {
            const char* tagUid_0 = uidArray[j]; 
            uint8_t hexValue = (uint8_t) strtol( &tagUid_0[0], NULL, 16);
            objectConfig.tagUid[i][j] = hexValue;
          }
        }
      }

      if (doc.containsKey("couleurs"))
      {
        JsonArray couleurArray=doc["couleurs"];
        
        for (uint8_t i=0;i<2;i++)
        {
          JsonArray rgbArray=couleurArray[i];

          objectConfig.couleurs[i].red = rgbArray[0];
          objectConfig.couleurs[i].green = rgbArray[1];
          objectConfig.couleurs[i].blue =  rgbArray[2];
        }        
      }

      // read object name
      if (doc.containsKey("objectName"))
      { 
        strlcpy(  objectConfig.objectName,
                  doc["objectName"],
                  SIZE_ARRAY);
                  //sizeof(objectConfig.objectName));
      }
    }
      
    // Close the file (File's destructor doesn't close the file)
    file.close();
  }

  void writeObjectConfig(const char * filename)
  { 
    
    
    // Delete existing file, otherwise the configuration is appended to the file
    LittleFS.remove(filename);
    
    // Open file for writing
    File file = LittleFS.open(filename, "w");
    if (!file) 
    {
      Serial.println(F("Failed to create file"));
      return;
    }
    else
    {
      Serial.println(F("File w opened"));
    }

    // Allocate a temporary JsonDocument
    StaticJsonDocument<JSONBUFFERSIZE> doc;

    doc["objectName"] = objectConfig.objectName;
    
    doc["objectId"] = objectConfig.objectId;
    doc["groupId"] = objectConfig.groupId;

    doc["activeLeds"] = objectConfig.activeLeds;
    doc["brightness"] = objectConfig.brightness;
    doc["nbErreurCodeMax"] = objectConfig.nbErreurCodeMax;
    doc["nbErreurCode"] = objectConfig.nbErreurCode;
    doc["delaiBlocage"] = objectConfig.delaiBlocage;
    
    doc["statutActuel"] = objectConfig.statutActuel;
    doc["statutPrecedent"] = objectConfig.statutPrecedent;

    doc["nbTagStock"] = min<uint8_t>(objectConfig.nbTagStock,MAX_NB_TAG); 
    doc["nbTagActuel"] = objectConfig.nbTagActuel;
    
    JsonArray tagUidArray = doc.createNestedArray("tagUid");

    for (uint8_t i=0;i<MAX_NB_TAG;i++)
    {
      JsonArray uid_x = tagUidArray.createNestedArray();
  
      for (uint8_t j=0;j<SIZE_UID;j++)
      {
        char result[2];
        sprintf(result, "%02X", objectConfig.tagUid[i][j]);
        
        uid_x.add(result);
      }
    }

    JsonArray couleurArray = doc.createNestedArray("couleurs");

    for (uint8_t i=0;i<2;i++)
    {
      JsonArray couleur_x = couleurArray.createNestedArray();
      
      couleur_x.add(objectConfig.couleurs[i].red);
      couleur_x.add(objectConfig.couleurs[i].green);
      couleur_x.add(objectConfig.couleurs[i].blue);
    }

    // Serialize JSON to file
    if (serializeJson(doc, file) == 0) 
    {
      Serial.println(F("Failed to write to file"));
    }

    // Close the file (File's destructor doesn't close the file)
    file.close();
    Serial.println(F("File w closed"));
  }

  void writeDefaultObjectConfig(const char * filename)
  {
    objectConfig.objectId = 1;
    objectConfig.groupId = 1;

    objectConfig.activeLeds = 8;
    objectConfig.brightness = 80;  
    objectConfig.nbErreurCodeMax = 3;
    objectConfig.nbErreurCode = 0;
    objectConfig.delaiBlocage = 5;
  
    objectConfig.statutActuel = 1;
    objectConfig.statutPrecedent = 1;
  
    objectConfig.nbTagStock = 5;
    objectConfig.nbTagActuel = 0;
  
    for (uint8_t i=0;i<MAX_NB_TAG;i++)
    {
      for (uint8_t j=0;j<SIZE_UID;j++)
      {
        objectConfig.tagUid[i][j] = 0;
      }
    }

    objectConfig.couleurs[0].red = 255;
    objectConfig.couleurs[0].green = 0;
    objectConfig.couleurs[0].blue =  0;
    
    objectConfig.couleurs[1].red = 0;
    objectConfig.couleurs[1].green = 255;
    objectConfig.couleurs[1].blue =  0;
    
    strlcpy( objectConfig.objectName,
             "serrure rfid",
             sizeof("serrure rfid"));
    
    writeObjectConfig(filename);
    }
  
  void readNetworkConfig(const char * filename)
  {
    // lire les données depuis le fichier littleFS
    // Open file for reading
    File file = LittleFS.open(filename, "r");
    if (!file) 
    {
      Serial.println(F("Failed to open file for reading"));
      return;
    }
    else
    {
      Serial.println(F("File opened"));
    }
  
    StaticJsonDocument<1024> doc;
    
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, file);
    if (error)
    {
      Serial.println(F("Failed to deserialize file in read network "));
      Serial.println(error.c_str());
    }
    else
    {
      // Copy values from the JsonObject to the Config
      if (doc.containsKey("apIP"))
      { 
        JsonArray apIP = doc["apIP"];
        
        networkConfig.apIP[0] = apIP[0];
        networkConfig.apIP[1] = apIP[1];
        networkConfig.apIP[2] = apIP[2];
        networkConfig.apIP[3] = apIP[3];
      }

      if (doc.containsKey("apNetMsk"))
      { 
        JsonArray apNetMsk = doc["apNetMsk"];
        
        networkConfig.apNetMsk[0] = apNetMsk[0];
        networkConfig.apNetMsk[1] = apNetMsk[1];
        networkConfig.apNetMsk[2] = apNetMsk[2];
        networkConfig.apNetMsk[3] = apNetMsk[3];
      }
          
      if (doc.containsKey("apName"))
      { 
        strlcpy(  networkConfig.apName,
                  doc["apName"],
                  sizeof(networkConfig.apName));
      }

      if (doc.containsKey("apPassword"))
      { 
        strlcpy(  networkConfig.apPassword,
                  doc["apPassword"],
                  sizeof(networkConfig.apPassword));
      }
    }
      
    // Close the file (File's destructor doesn't close the file)
    file.close();
  }
  
  void writeNetworkConfig(const char * filename)
  {
    // Delete existing file, otherwise the configuration is appended to the file
    LittleFS.remove(filename);
  
    // Open file for writing
    File file = LittleFS.open(filename, "w");
    if (!file) 
    {
      Serial.println(F("Failed to create file"));
      return;
    }

    // Allocate a temporary JsonDocument
    StaticJsonDocument<1024> doc;

    doc["apName"] = networkConfig.apName;
    doc["apPassword"] = networkConfig.apPassword;

    StaticJsonDocument<128> docIp;
    JsonArray arrayIp = docIp.to<JsonArray>();
    arrayIp.add(networkConfig.apIP[0]);
    arrayIp.add(networkConfig.apIP[1]);
    arrayIp.add(networkConfig.apIP[2]);
    arrayIp.add(networkConfig.apIP[3]);

    StaticJsonDocument<128> docNetMask;
    JsonArray arrayNetMask = docNetMask.to<JsonArray>();
    arrayNetMask.add(networkConfig.apNetMsk[0]);
    arrayNetMask.add(networkConfig.apNetMsk[1]);
    arrayNetMask.add(networkConfig.apNetMsk[2]);
    arrayNetMask.add(networkConfig.apNetMsk[3]);
    
    doc["apIP"]=arrayIp;
    doc["apNetMsk"]=arrayNetMask;

    // Serialize JSON to file
    if (serializeJson(doc, file) == 0) 
    {
      Serial.println(F("Failed to write to file"));
    }
    
    // Close the file (File's destructor doesn't close the file)
    file.close();
  }
  
  void writeDefaultNetworkConfig(const char * filename)
  {
  strlcpy(  networkConfig.apName,
            "SERRURERFID",
            sizeof("SERRURERFID"));
  
  strlcpy(  networkConfig.apPassword,
            "",
            sizeof(""));

  networkConfig.apIP[0]=192;
  networkConfig.apIP[1]=168;
  networkConfig.apIP[2]=1;
  networkConfig.apIP[3]=1;

  networkConfig.apNetMsk[0]=255;
  networkConfig.apNetMsk[1]=255;
  networkConfig.apNetMsk[2]=255;
  networkConfig.apNetMsk[3]=0;
    
  writeNetworkConfig(filename);
  }

  
  void stringJsonFile(const char * filename, char * target, uint16_t targetReadSize)
  {
    // Open file for reading
    File file = LittleFS.open(filename, "r");
    if (!file) 
    {
      Serial.println(F("Failed to open file for reading"));
    }
    else
    {
      uint16_t cptRead = 0;
      while ( (file.available()) && (cptRead<targetReadSize) )
      {
        target[cptRead] = file.read();
        cptRead++;
      }

      if (cptRead<targetReadSize)
      {
        target[cptRead] = '\0';
      }
      else
      {
        target[targetReadSize] = '\0';
      }
      //Serial.print(F("char lus: "));
      //Serial.println(cptRead);
    }
    
    // Close the file (File's destructor doesn't close the file)
    file.close();
  }

  void mountFS()
  {
    Serial.println(F("Mount LittleFS"));
    if (!LittleFS.begin())
    {
      Serial.println(F("LittleFS mount failed"));
      return;
    }
  }
  
  void printJsonFile(const char * filename)
  {
    // Open file for reading
    File file = LittleFS.open(filename, "r");
    if (!file) 
    {
      Serial.println(F("Failed to open file for reading"));
    }
      
    StaticJsonDocument<JSONBUFFERSIZE> doc;
    //DynamicJsonDocument doc(JSONBUFFERSIZE);
    
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, file);
    if (error)
    {
      Serial.println(F("Failed to deserialize file in print object"));
      Serial.println(error.c_str());
    }
    else
    {
      //serializeJsonPretty(doc, Serial);
      serializeJson(doc, Serial);
      Serial.println();
    }
    
    // Close the file (File's destructor doesn't close the file)
    file.close();
  }
  
  void listDir(const char * dirname)
  {
    Serial.printf("Listing directory: %s", dirname);
    Serial.println();
  
    Dir root = LittleFS.openDir(dirname);
  
    while (root.next())
    {
      File file = root.openFile("r");
      Serial.print(F("  FILE: "));
      Serial.print(root.fileName());
      Serial.print(F("  SIZE: "));
      Serial.print(file.size());
      Serial.println();
      file.close();
    }

    Serial.println();
  }
};
