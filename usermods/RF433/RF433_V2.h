#pragma once

#include "wled.h"
#include "Arduino.h"
#include <RCSwitch.h>

#ifndef RF_PIN
#ifdef ARDUINO_ARCH_ESP32
  #define RF_PIN 23
#else // ESP8266
  #define RF_PIN 12
#endif
#endif

class RF433Usermod : public Usermod
{
private:
  RCSwitch mySwitch = RCSwitch();
  unsigned long lastVaue = 0;
  unsigned long lastTime = 0;

#ifndef SWITCH_DATA
  unsigned long switchdata = 0;
#else
  unsigned long switchdata = SWITCH_DATA;
#endif

  unsigned long rftime = 0;
  bool switchenabled = true;

  byte NotifyUpdateMode = CALL_MODE_NO_NOTIFY;

  static const char _switchname[];
  static const char _switchdata[];
  static const char _switchenabled[];


  static const char _pirname[];
  static const char _pirdata[];
  static const char _pirenabled[];
  static const char _offtime[];

#ifndef PIR_DATA
  unsigned long pirdata = 0;
#else
  unsigned long pirdata = PIR_DATA;
#endif

  bool pirenabled = true;

  #ifndef DELAY_OFF
  unsigned long offtime = 600;
  #else
  unsigned long offtime = DELAY_OFF;
  #endif
  
  bool initDone = false;
  bool statusflag = false;

public:

  void setup()
  {
    if (switchenabled || pirenabled)
    {
      mySwitch.enableReceive(RF_PIN);
    }
    initDone = true;
  }

  /*
   * connected() is called every time the WiFi is (re)connected
   * Use it to initialize network interfaces
   */
  void connected()
  {
    // Double beep on WiFi
  }

  /*
   * loop() is called continuously. Here you can check for events, read sensors, etc.
   */
  void loop()
  {
    if ((!switchenabled && !pirenabled) || strip.isUpdating()) return;
    if (mySwitch.available())
    {
      unsigned long value = mySwitch.getReceivedValue();
      mySwitch.resetAvailable();
      if (lastVaue == value && millis() - lastTime < 1000)
      {
        return;
      }
      lastVaue = value;
      lastTime = millis();
      DEBUG_PRINT(F("RF433 Receive: "));
      DEBUG_PRINTLN(lastVaue);
      if (switchenabled && lastVaue == switchdata)
      {
        statusflag = false;
        toggleOnOff();
        stateUpdated(CALL_MODE_NO_NOTIFY);
        DEBUG_PRINTLN(F("Toggle switch"));
      }
      else if (pirenabled && lastVaue == pirdata)
      {
        statusflag = true;
        rftime = millis() / 1000;
        if (bri == 0)
        {
          bri = briLast;
        }
        stateUpdated(CALL_MODE_NO_NOTIFY);
        DEBUG_PRINTLN(F("Turn on"));
      }
      else
      {
        stateChanged = true; // inform external dvices/UI of change
        stateUpdated(CALL_MODE_DIRECT_CHANGE);
        DEBUG_PRINTLN(F("Do nothing"));
      }
    }
    if (rftime > 0 && millis() / 1000 - rftime > offtime && statusflag && pirenabled)
    {
      briLast = bri;
      bri = 0;
      stateUpdated(CALL_MODE_NO_NOTIFY);
      rftime = 0;
      DEBUG_PRINTLN(F("Delay Off"));
    }
    
  }

  void addToJsonInfo(JsonObject &root)
  {
    // this code adds "u":{"Light":[20," lux"]} to the info object
    if (!initDone)
      return; // prevent crash on boot applyPreset()
    JsonObject user = root["u"];
    if (user.isNull())
      user = root.createNestedObject("u");

    JsonArray switchArr = user.createNestedArray("RF433 Receive"); // name
    switchArr.add(lastVaue);                                // value
  }

  void readFromJsonState(JsonObject &root)
  {
    if (!initDone)
      return; // prevent crash on boot applyPreset()

    JsonObject usermod = root[FPSTR(_switchname)];
    if (!usermod.isNull())
    {
      // expect JSON usermod data in usermod name object: {"ExampleUsermod:{"user0":10}"}
      switchdata = usermod[FPSTR(_switchdata)] | switchdata; // if "user0" key exists in JSON, update, else keep old value
    }

    JsonObject pirusermod = root[FPSTR(_pirname)];
    if (!pirusermod.isNull())
    {
      // expect JSON usermod data in usermod name object: {"ExampleUsermod:{"user0":10}"}
      pirdata = pirusermod[FPSTR(_pirdata)] | pirdata; // if "user0" key exists in JSON, update, else keep old value
      offtime = pirusermod[FPSTR(_offtime)] | offtime;
    }
    // you can as well check WLED state JSON keys
    // if (root["bri"] == 255) Serial.println(F("Don't burn down your garage!"));
  }

  void addToConfig(JsonObject &root)
  {

    // we add JSON object: {"RF433": {"data": 11673316}}
    JsonObject top = root.createNestedObject(FPSTR(_switchname)); // usermodname
    top[FPSTR(_switchenabled)] = switchenabled;
    top[FPSTR(_switchdata)] = switchdata;

    JsonObject pirtop = root.createNestedObject(FPSTR(_pirname)); // usermodname
    pirtop[FPSTR(_pirenabled)] = pirenabled;
    pirtop[FPSTR(_pirdata)] = pirdata;
    pirtop[FPSTR(_offtime)] = offtime;
    DEBUG_PRINTLN(F("config saved."));

  }

  // void appendConfigData()
  // {
  //   oappend(SET_F("addInfo('RF433Usermod:offtime',1,'Second');"));  // 0 is field type, 1 is actual field
  // }

  bool readFromConfig(JsonObject &root)
  {
    // we look for JSON object: {"Autosave": {"switchenabled": true, "autoSaveAfterSec": 10, "autoSavePreset": 250, ...}}
    JsonObject top = root[FPSTR(_switchname)];
    if (top.isNull())
    {
      DEBUG_PRINT(FPSTR(_switchname));
      DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
      return false;
    }
    switchenabled = top[FPSTR(_switchenabled)] | switchenabled;
    switchdata = top[FPSTR(_switchdata)] | switchdata;

    JsonObject pirtop = root[FPSTR(_pirname)];
    if (pirtop.isNull())
    {
      DEBUG_PRINT(FPSTR(_pirname));
      DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
      return false;
    }
    pirenabled = pirtop[FPSTR(_pirenabled)] | pirenabled;
    pirdata = pirtop[FPSTR(_pirdata)] | pirdata;
    offtime = pirtop[FPSTR(_offtime)] | offtime;

    DEBUG_PRINTLN(F(" config (re)loaded."));

    // use "return !top["newestParameter"].isNull();" when updating Usermod with new features
    return true;
  }

  /*
   * getId() allows you to optionally give your V2 usermod an unique ID (please define it in const.h!).
   * This could be used in the future for the system to determine whether your usermod is installed.
   */
  uint16_t getId()
  {
    return USERMOD_ID_RF433;
  }
};



const char RF433Usermod::_switchname[]    PROGMEM = "RF433Switch";
const char RF433Usermod::_switchenabled[] PROGMEM = "enabled";
const char RF433Usermod::_switchdata[]    PROGMEM = "switchdata";

const char RF433Usermod::_pirname[]       PROGMEM = "RF433Pir";
const char RF433Usermod::_pirenabled[]    PROGMEM = "enabled";
const char RF433Usermod::_pirdata[]       PROGMEM = "pirdata";
const char RF433Usermod::_offtime[]       PROGMEM = "offtime(Sec)";
