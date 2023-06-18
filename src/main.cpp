#include "Arduino.h"
#include <ArduinoJson.h>
#include "GBusHelpers.h"
#include "GBusWifiMesh.h"
#include <EEPROM.h>
#include <WiFi.h>
#include "Tasker.h"
#include <Bounce2.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "SH1106Wire.h"
#include "EasyPCF8574.h"

#define FWVERSION "1.43"
#define MODULNAME "GBusPool"
#define LogLevel ESP_LOG_NONE

// Outputs
#define FilterPumpOutput 1
#define SaltSystemPower 2
#define SaltSystemEnableControl 3
#define SaltSystemActivate 4
#define ValvePowerOutput 5
#define ValveOutput 6
#define NUM_BUTTONS 3
#define ONE_WIRE_BUS 16
#define TEMPERATURE_PRECISION 12
#define FilterpumpMaximumOnTime 12 * 3600 * 1000 // 12h
#define SaltSystempowerOffDelay 20 * 60 * 1000   // xmin
#define SaltSystemResetViaPowerCycle 2           // Power Off Salt System every X Cycle
#define ValvePowerOffDelay 35 * 1000

int WaterMAxTemperature = 30;
bool ValveAutomaticMode = true;
uint8_t SaltSystemResetViaPowerCycleCounter = 0;
bool ValvePositionHeat = false;
bool DisplayIsOn = true;
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {15, 4, 2};

void SetOutput(uint8_t Output, bool Value);
String getValue(String data, char separator, int index);
void HandleDisplaypower(int DisplayOn);

Tasker tasker;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
SH1106Wire Display(0x3c, 13, 14);
EasyPCF8574 RelaisCard(0x20, 0xFF);
Bounce *buttons = new Bounce[NUM_BUTTONS];

// Temperature definitions
DeviceAddress WaterThermometer = {0x28, 0x4F, 0x23, 0xEC, 0x50, 0x20, 0x01, 0x46};
DeviceAddress VorlaufThermometer = {0x28, 0x52, 0x04, 0xE8, 0x50, 0x20, 0x01, 0xF0};
DeviceAddress RucklaufThermometer = {0x28, 0x47, 0x53, 0xF2, 0x50, 0x20, 0x01, 0xA7};
DeviceAddress GarageRoofThermometer = {0x28, 0x3A, 0x0D, 0xD6, 0x50, 0x20, 0x01, 0x3C};
float WaterThermometerValue, VorlaufThermometerValue, RucklaufThermometerValue, GarageRoofThermometerValue;
const long durationTemp = 2 * 60 * 1000; // The frequency of temperature measurement
bool NewTemperatures = false;

bool FilterpumpAutomaticOn;
int8_t FilterpumpAutomaticOnTime;

bool SaltSystemAutomaticOn;
int8_t SaltSystemAutomaticOnTime;

int8_t AutomaticStartTime;
bool AutomaticStartActive;

uint8_t MaxDisplayPage = 8;
uint8_t ActualDisplayPage = 1;

uint32_t Minute = 0;
uint32_t Hour = 0;
MeshApp GBusMesh;

bool NewMeshMessage = false;
String LastMeshMessage;
uint8_t LastSrcMac[6];
void LastmeshMessage(String msg, uint8_t SrcMac[6]);

// Prototypes
void meshMessage(String msg, uint8_t SrcMac[6]);
void SentNodeInfo();
void RootNotActiveWatchdog();
void meshConnected();
void SetSaltSystemModeAutomatic(int ModeOn);
void TempSensorStartConversion();
void SaltSystemPowerOff();
void SetAutomaticStartTime(int time);
void UpdateDisplay();
void SetFilterPumpModeAutomatic(int Mode);
void SetFilterpumpAutomaticOnTime(uint8_t Time);
void SetSaltSystemAutomaticOnTime(uint8_t Time);
void ValvePowerOff();
void SetAutomaticStartActive(bool Mode);
void SetValvePosition(int ValveToHeat);
void UpdateMqtt();

uint8_t ModulType = 255;

void setup()
{
  Serial.begin(115200);

  if (!RelaisCard.startI2C(13, 14))
  {
    Serial.println("RelaisCard Not started. Check pin and address.");
  }

  delay(100);

  /**
   * @brief Set the log level for serial port printing.
   */
  esp_log_level_set("*", LogLevel);
  esp_log_level_set(TAG, LogLevel);

  MDF_LOGI("ModuleType: %u\n", ModulType);

  Display.init();
  Display.flipScreenVertically();
  Display.setFont(ArialMT_Plain_10);

  Display.clear();
  Display.drawString(4, 0, "Connecting to GBusMesh"); //, OLED::DOUBLE_SIZE);
  Display.display();

  GBusMesh.onMessage(meshMessage);
  GBusMesh.onConnected(meshConnected);
  GBusMesh.start(false);

  Display.clear();
  Display.drawString(4, 0, "Connected to Wifi"); //, OLED::DOUBLE_SIZE);
  Display.display();

  // DS18S20 Init
  sensors.begin();
  sensors.setResolution(WaterThermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(VorlaufThermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(RucklaufThermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(GarageRoofThermometer, TEMPERATURE_PRECISION);
  sensors.setWaitForConversion(false);

  Display.clear();
  Display.drawString(4, 0, "Init output");
  Display.display();

  
  for (int x = 1; x <= 8; x++)
  {
    SetOutput(x, 0);
  }

  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    buttons[i].attach(BUTTON_PINS[i], INPUT_PULLDOWN); // setup the bounce instance for the current button
    buttons[i].interval(40);                           // interval in ms
  }

  SetFilterpumpAutomaticOnTime(6);
  SetFilterPumpModeAutomatic(false);

  SetSaltSystemAutomaticOnTime(4);
  SaltSystemPowerOff();

  SetAutomaticStartActive(true);
  SetAutomaticStartTime(10);

  HandleDisplaypower(1);

  tasker.setTimeout(RootNotActiveWatchdog, CheckForRootNodeIntervall);

  // Beginn with temperature task
  //TempSensorStartConversion();
  tasker.setInterval(TempSensorStartConversion,durationTemp);
}

void loop()
{
  tasker.loop();
  GBusMesh.Task();

  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    // Update the Bounce instance :
    buttons[i].update();
  }

  // Button 1
  if (buttons[0].rose())
  {
    if (DisplayIsOn)
    {
      if (ActualDisplayPage == 2)
      {
        SetFilterPumpModeAutomatic(!FilterpumpAutomaticOn);
        UpdateDisplay();
      }
      else if (ActualDisplayPage == 4)
      {
        FilterpumpAutomaticOnTime--;
        if (FilterpumpAutomaticOnTime < 0)
        {
          FilterpumpAutomaticOnTime = 23;
        }
        SetFilterpumpAutomaticOnTime(FilterpumpAutomaticOnTime);
        UpdateDisplay();
      }
      else if (ActualDisplayPage == 5)
      {
        SaltSystemAutomaticOnTime--;
        if (SaltSystemAutomaticOnTime < 0)
        {
          SaltSystemAutomaticOnTime = 23;
        }
        SetSaltSystemAutomaticOnTime(SaltSystemAutomaticOnTime);
        UpdateDisplay();
      }
      else if (ActualDisplayPage == 6)
      {
        SetAutomaticStartActive(!AutomaticStartActive);
        UpdateDisplay();
      }
      else if (ActualDisplayPage == 7)
      {
        AutomaticStartTime--;
        if (AutomaticStartTime < 0)
        {
          AutomaticStartTime = 23;
        }
        SetAutomaticStartTime(AutomaticStartTime);
        UpdateDisplay();
      }
      else if (ActualDisplayPage == 8)
      {
        SetValvePosition(!ValvePositionHeat);
        UpdateDisplay();
      }
    }
    HandleDisplaypower(true);
  }
  // Button 2
  if (buttons[1].rose())
  {
    if (DisplayIsOn)
    {
      if (ActualDisplayPage == 2)
      {
        SetSaltSystemModeAutomatic(!SaltSystemAutomaticOn);
        UpdateDisplay();
      }
      else if (ActualDisplayPage == 4)
      {
        FilterpumpAutomaticOnTime++;
        if (FilterpumpAutomaticOnTime > 23)
        {
          FilterpumpAutomaticOnTime = 1;
        }
        SetFilterpumpAutomaticOnTime(FilterpumpAutomaticOnTime);
        UpdateDisplay();
      }
      else if (ActualDisplayPage == 5)
      {
        SaltSystemAutomaticOnTime++;
        if (SaltSystemAutomaticOnTime > 23)
        {
          SaltSystemAutomaticOnTime = 1;
        }
        SetSaltSystemAutomaticOnTime(SaltSystemAutomaticOnTime);
        UpdateDisplay();
      }
      else if (ActualDisplayPage == 7)
      {
        AutomaticStartTime++;
        if (AutomaticStartTime > 23)
        {
          AutomaticStartTime = 0;
        }
        SetAutomaticStartTime(AutomaticStartTime);
        UpdateDisplay();
      }
    }
    HandleDisplaypower(true);
  }
  // Next Screen
  if (buttons[2].rose())
  {
    if (DisplayIsOn)
    {
      ActualDisplayPage++;
      if (ActualDisplayPage > MaxDisplayPage)
      {
        ActualDisplayPage = 1;
      }
      UpdateDisplay();
    }
    HandleDisplaypower(true);
  }

  if (NewTemperatures)
  {
    NewTemperatures = false;

    // Serial.println("Refresh Display");

    if (WaterThermometerValue > WaterMAxTemperature &&
        ValvePositionHeat == 1 &&
        ValveAutomaticMode)
    {
      SetValvePosition(0);
    }

    UpdateMqtt();
    UpdateDisplay();
  }

  if (NewMeshMessage)
  {
    NewMeshMessage = false;
    LastmeshMessage(LastMeshMessage, LastSrcMac);
  }

}

void RootNotActiveWatchdog()
{
  String MsgBack = "MQTT Reboot WatchdogReboot";
  GBusMesh.SendMessage(MsgBack);
  //ESP.restart();
}

void SentNodeInfo()
{
  mesh_addr_t bssid;
  esp_err_t err = esp_mesh_get_parent_bssid(&bssid);

  char MsgBuffer[300];
  sprintf(MsgBuffer, "MQTT Info ModulName:%s,SubType:%u,MAC:%s,WifiStrength:%d,Parent:%s,FW:%s", MODULNAME, ModulType, WiFi.macAddress().c_str(), getWifiStrength(3), hextab_to_string(bssid.addr).c_str(), FWVERSION);
  String Msg = String(MsgBuffer);
  GBusMesh.SendMessage(Msg);
}

void meshConnected()
{
  SentNodeInfo();
}

void meshMessage(String msg, uint8_t SrcMac[6])
{
  LastMeshMessage = msg;
  memcpy(LastSrcMac, SrcMac, sizeof(LastSrcMac));
  NewMeshMessage = true;
}

void LastmeshMessage(String msg, uint8_t SrcMac[6])
{
  MDF_LOGD("Rec msg %u: %s", msg.length(), msg.c_str());

  String Type = getValue(msg, ' ', 0);
  String Number = getValue(msg, ' ', 1);
  String Command = getValue(msg, ' ', 2);
  uint8_t NumberInt = Number.toInt();

  //String MsgBack = "MQTT Get " + Type + " " + Number + " " + Command;
  //mesh.SendMessage(MsgBack);

  if (msg.startsWith("I'm Root!"))
  {
    MDF_LOGI("Gateway hold alive received");
    tasker.cancel(RootNotActiveWatchdog);
    tasker.setTimeout(RootNotActiveWatchdog, CheckForRootNodeIntervall);
  }
  else if (Type == "Config")
  {
    Serial.printf("Config\n");
  }
  else if (Type == "GetNodeInfo")
  {
    SentNodeInfo();
  }
  else if (Type == "Reboot")
  {
    delay(2000);
    ESP.restart();
  }
  else if (Type == "time")
  {
    String ActualTime = getValue(msg, ' ', 1);
    sscanf(ActualTime.c_str(), "%u:%u", &Hour, &Minute);

    //String MsgBack = "MQTT time Time=" + String(Hour) + ":" + String(Minute) + "," + String(AutomaticStartTime) + "," + String(AutomaticStartActive) + "," + String(FilterpumpAutomaticOn);
    //mesh.SendMessage(MsgBack);

    if (Hour == AutomaticStartTime &&
        Minute == 0 &&
        AutomaticStartActive &&
        !FilterpumpAutomaticOn)
    {
      if (WaterThermometerValue < (WaterMAxTemperature - 2) &&
          ValvePositionHeat == 0 &&
          ValveAutomaticMode)
      {
        SetValvePosition(1);
      }

      SetFilterPumpModeAutomatic(!FilterpumpAutomaticOn);
      SetSaltSystemModeAutomatic(!SaltSystemAutomaticOn);
      UpdateDisplay();
      UpdateMqtt();
    }

  }
  else if (Type == "output")
  {
    String OutputString = getValue(msg, ' ', 1);
    uint8_t OutputNumber = OutputString.toInt();

    SetOutput(OutputNumber, getValue(msg, ' ', 2).toInt());
  }
  else if (Type == "FilterPumpModeAutomatic")
  {
    SetFilterPumpModeAutomatic(getValue(msg, ' ', 1).toInt());
  }
  else if (Type == "FilterpumpAutomaticOnTime")
  {
    SetFilterpumpAutomaticOnTime(getValue(msg, ' ', 1).toInt());
  }
  else if (Type == "SaltSystemModeAutomatic")
  {
    SetSaltSystemModeAutomatic(getValue(msg, ' ', 1).toInt());
  }
  else if (Type == "SaltSystemAutomaticOnTime")
  {
    SetSaltSystemAutomaticOnTime(getValue(msg, ' ', 1).toInt());
  }
  else if (Type == "AutomaticStartActive")
  {
    SetAutomaticStartActive(getValue(msg, ' ', 1).toInt());
  }
  else if (Type == "AutomaticStartTime")
  {
    SetAutomaticStartTime(getValue(msg, ' ', 1).toInt());
  }
  else if (Type == "ValveToHeat")
  {
    SetValvePosition(getValue(msg, ' ', 1).toInt());
  }
  else if (Type == "WaterMaxTemperature")
  {
    WaterMAxTemperature = getValue(msg, ' ', 1).toInt();
    //String Msg = "MQTT WaterMaxTemperature " + String(WaterMAxTemperature);
    //mesh.SendMessage(Msg);
    UpdateMqtt();
    // client.publish("gimpire/EspPool/WaterMaxTemperature", String(WaterMAxTemperature).c_str());
  }
  else if (Type == "ValveAutomaticMode")
  {
    ValveAutomaticMode = getValue(msg, ' ', 1).toInt();
    //String Msg = "MQTT ValveAutomaticMode " + String(ValveAutomaticMode);
    //mesh.SendMessage(Msg);
    UpdateMqtt();
    // client.publish("gimpire/EspPool/ValveAutomaticMode", String(ValveAutomaticMode).c_str());
  }
}
void HandleDisplaypower(int DisplayOn)
{
  if (DisplayOn == 1)
  {
    DisplayIsOn = true;
    tasker.cancel(HandleDisplaypower, 0);
    tasker.setTimeout(HandleDisplaypower, 10000, 0);
    UpdateDisplay();
  }
  else
  {
    Display.clear();
    Display.display();
    DisplayIsOn = false;
  }
}
void readSensor()
{
  WaterThermometerValue = sensors.getTempC(WaterThermometer);
  VorlaufThermometerValue = sensors.getTempC(VorlaufThermometer);
  RucklaufThermometerValue = sensors.getTempC(RucklaufThermometer);
  GarageRoofThermometerValue = sensors.getTempC(GarageRoofThermometer);
  NewTemperatures = true;
}
void TempSensorStartConversion()
{
  // start temperature conversion (does not block)
  sensors.requestTemperatures();
  // schedule reading the actual temperature in 750 milliseconds
  tasker.setTimeout(readSensor, 1000);
  
}
void UpdateMqtt()
{
  StaticJsonDocument<1000> PoolJson;

  if (WaterThermometerValue > -127)
  {
    PoolJson["WaterTemp"] = String(WaterThermometerValue);
  }

  PoolJson["VLTemp"] = String(VorlaufThermometerValue);
  PoolJson["RLTemp"] = String(RucklaufThermometerValue);
  PoolJson["TemperatureGarageRoof"] = String(GarageRoofThermometerValue);
  PoolJson["ValveAutomaticMode"] = String(ValveAutomaticMode);
  PoolJson["WaterMaxTemperature"] = String(WaterMAxTemperature);
  PoolJson["AutomaticStartActive"] = String(AutomaticStartActive);
  PoolJson["SaltSystemModeAutomatic"] = String(SaltSystemAutomaticOn);
  PoolJson["SaltSystemAutomaticOnTime"] = String(SaltSystemAutomaticOnTime);
  PoolJson["ValveToHeat"] = String(ValvePositionHeat);
  PoolJson["FilterPumpModeAutomatic"] = String(FilterpumpAutomaticOn);
  PoolJson["AutomaticStartTime"] = String(AutomaticStartTime);
  PoolJson["FilterpumpAutomaticOnTime"] = String(FilterpumpAutomaticOnTime);

  String PoolJsonString;
  serializeJson(PoolJson, PoolJsonString);
  String Msg = "MQTT values " + PoolJsonString;
  GBusMesh.SendMessage(Msg);
}
void UpdateDisplay()
{
  if (DisplayIsOn)
  {

    if (ActualDisplayPage == 1)
    {
      Display.clear();

      String ShowRssi = "RSSI: " + String(WiFi.RSSI()) + " " + String(Hour) + ":" + String(Minute);
      Display.drawString(4, 0, ShowRssi.c_str());

      String DisplayText = "Wasser: " + String(WaterThermometerValue);
      Display.drawString(4, 12, DisplayText.c_str());

      DisplayText = "Vorlauf: " + String(VorlaufThermometerValue);
      Display.drawString(4, 22, DisplayText.c_str());

      DisplayText = "Rücklauf: " + String(RucklaufThermometerValue);
      Display.drawString(4, 32, DisplayText.c_str());

      DisplayText = "Dach: " + String(GarageRoofThermometerValue);
      Display.drawString(4, 42, DisplayText.c_str());

      Display.display();
    }
    else if (ActualDisplayPage == 2)
    {
      Display.clear();

      String ShowRssi = "RSSI: " + String(WiFi.RSSI()) + " " + String(Hour) + ":" + String(Minute);
      Display.drawString(4, 0, ShowRssi.c_str());

      String DisplayText = "Filterpumpe: " + String(FilterpumpAutomaticOn);
      Display.drawString(4, 12, DisplayText.c_str());

      DisplayText = "Salzwasser: " + String(SaltSystemAutomaticOn);
      Display.drawString(4, 22, DisplayText.c_str());

      DisplayText = "Waser Max Temp: " + String(WaterMAxTemperature);
      Display.drawString(4, 32, DisplayText.c_str());

      if (ValvePositionHeat)
      {
        DisplayText = "Valve: solar";
      }
      else
      {
        DisplayText = "Valve: pool";
      }

      Display.drawString(4, 42, DisplayText.c_str());

      String ipString = WiFi.localIP().toString();

      DisplayText = "IP: " + ipString;
      Display.drawString(4, 52, DisplayText.c_str());

      Display.display();
    }
    else if (ActualDisplayPage == 3)
    {
      Display.clear();

      String ShowRssi = "RSSI: " + String(WiFi.RSSI()) + " " + String(Hour) + ":" + String(Minute);
      Display.drawString(4, 0, ShowRssi.c_str());

      String DisplayText = "Filter Zeit: " + String(FilterpumpAutomaticOnTime);
      Display.drawString(4, 12, DisplayText.c_str());

      DisplayText = "Salzwasser Zeit: " + String(SaltSystemAutomaticOnTime);
      Display.drawString(4, 22, DisplayText.c_str());

      DisplayText = "Autostart Zeit: " + String(AutomaticStartTime);
      Display.drawString(4, 32, DisplayText.c_str());

      DisplayText = "Autostart aktiv: " + String(AutomaticStartActive);
      Display.drawString(4, 42, DisplayText.c_str());

      Display.display();
    }
    else if (ActualDisplayPage == 4) // Set Filtertime
    {
      Display.clear();

      String ShowRssi = "RSSI: " + String(WiFi.RSSI()) + " " + String(Hour) + ":" + String(Minute);
      Display.drawString(4, 0, ShowRssi.c_str());

      String DisplayText = "Set Filter Zeit: " + String(FilterpumpAutomaticOnTime);
      Display.drawString(4, 12, DisplayText.c_str());

      Display.display();
    }
    else if (ActualDisplayPage == 5) // Set Saltwater time
    {
      Display.clear();

      String ShowRssi = "RSSI: " + String(WiFi.RSSI()) + " " + String(Hour) + ":" + String(Minute);
      Display.drawString(4, 0, ShowRssi.c_str());

      String DisplayText = "Set Salzwasser Zeit: " + String(SaltSystemAutomaticOnTime);
      Display.drawString(4, 12, DisplayText.c_str());

      Display.display();
    }
    else if (ActualDisplayPage == 6) // Set automatic begin active
    {
      Display.clear();

      String ShowRssi = "RSSI: " + String(WiFi.RSSI()) + " " + String(Hour) + ":" + String(Minute);
      Display.drawString(4, 0, ShowRssi.c_str());

      String DisplayText = "Set Autostart aktiv: " + String(AutomaticStartActive);
      Display.drawString(4, 12, DisplayText.c_str());

      DisplayText = "Stunde jetzt: " + String(Hour);
      Display.drawString(4, 22, DisplayText.c_str());

      DisplayText = "Minute jetzt: " + String(Minute);
      Display.drawString(4, 32, DisplayText.c_str());

      Display.display();
    }
    else if (ActualDisplayPage == 7) // Set automatic start time
    {
      Display.clear();

      String ShowRssi = "RSSI: " + String(WiFi.RSSI()) + " " + String(Hour) + ":" + String(Minute);
      Display.drawString(4, 0, ShowRssi.c_str());

      String DisplayText = "Set Auto start Zeit: " + String(AutomaticStartTime);
      Display.drawString(4, 12, DisplayText.c_str());

      Display.display();
    }
    else if (ActualDisplayPage == 8) // valve position
    {
      Display.clear();

      String ShowRssi = "RSSI: " + String(WiFi.RSSI()) + " " + String(Hour) + ":" + String(Minute);
      Display.drawString(4, 0, ShowRssi.c_str());

      String DisplayText;

      if (ValvePositionHeat)
      {
        DisplayText = "Valve: solar";
      }
      else
      {
        DisplayText = "Valve: pool";
      }
      Display.drawString(4, 12, DisplayText.c_str());

      Display.display();
    }
  }
}
void SetOutput(uint8_t Output, bool Value)
{
  Serial.println("SetOutput: " + String(Output) + " " + String(Value));

  RelaisCard.WriteBit(!Value, Output - 1);
  // String PublishString = "gimpire/EspPool/output/" + String(Output);

  String Msg = "MQTT output/" + String(Output) + " " + String(Value).c_str();
  GBusMesh.SendMessage(Msg);

  // client.publish(PublishString.c_str(), String(Value).c_str());
}
void SetFilterPumpModeAutomatic(int Mode)
{
  Serial.println("Set FilterPumpModeAutomatic to: " + String(Mode));
  SetOutput(FilterPumpOutput, Mode);
  FilterpumpAutomaticOn = Mode;

  if (Mode)
  {
    tasker.setTimeout(SetFilterPumpModeAutomatic, (unsigned long)FilterpumpAutomaticOnTime * 3600 * 1000, 0);
    // tasker.setTimeout(SetFilterPumpModeAutomatic, (unsigned long)FilterpumpAutomaticOnTime * 1000, 0);
  }
  else
  {
    tasker.cancel(SetFilterPumpModeAutomatic, 0);
    if (SaltSystemAutomaticOn == 1)
    {
      SetSaltSystemModeAutomatic(0);
    }
  }

  UpdateMqtt();
  //String Msg = "MQTT FilterPumpModeAutomatic " + String(FilterpumpAutomaticOn);
  //mesh.SendMessage(Msg);

  // client.publish("gimpire/EspPool/FilterPumpModeAutomatic", String(FilterpumpAutomaticOn).c_str());
}
void SetAutomaticStartActive(bool Mode)
{
  //Serial.println("Set AutomaticStartActive to: " + String(Mode));
  AutomaticStartActive = Mode;

  UpdateMqtt();

  //String Msg = "MQTT AutomaticStartActive " + String(AutomaticStartActive);
  //mesh.SendMessage(Msg);

  // client.publish("gimpire/EspPool/AutomaticStartActive", String(AutomaticStartActive).c_str());
}
void SetAutomaticStartTime(int time)
{
  Serial.println("Set AutomaticStartTime to: " + String(time));
  AutomaticStartTime = time;

  UpdateMqtt();
  //String Msg = "MQTT AutomaticStartTimeback " + String(AutomaticStartTime);
  //mesh.SendMessage(Msg);
}
void SetSaltSystemModeAutomatic(int ModeOn)
{
  Serial.println("Set SaltSystemModeAutomatic: " + String(ModeOn));

  tasker.cancel(SetSaltSystemModeAutomatic, 0);
  tasker.cancel(SaltSystemPowerOff);

  SaltSystemAutomaticOn = ModeOn;

  if (ModeOn)
  {
    UpdateMqtt();
    //String Msg = "MQTT SaltSystemModeAutomatic " + String(SaltSystemAutomaticOn);
    //mesh.SendMessage(Msg);

    // client.publish("gimpire/EspPool/SaltSystemModeAutomatic", String(SaltSystemAutomaticOn).c_str());

    SetOutput(SaltSystemPower, 1);
    delay(3000);

    // SetOutput(SaltSystemEnableControl, 1);
    // delay(300);
    SetOutput(SaltSystemActivate, 1);
    delay(300);
    SetOutput(SaltSystemActivate, 0);
    // delay(300);
    // SetOutput(SaltSystemEnableControl, 0);

    tasker.setTimeout(SetSaltSystemModeAutomatic, (unsigned long)SaltSystemAutomaticOnTime * 3600 * 1000, 0);
    // tasker.setTimeout(SetSaltSystemModeAutomatic, (unsigned long)SaltSystemAutomaticOnTime * 1000, 0);
  }
  else
  {
    // SetOutput(SaltSystemEnableControl, 1);
    //  delay(200);
    SetOutput(SaltSystemActivate, 1);
    delay(200);
    SetOutput(SaltSystemActivate, 0);
    // SetOutput(SaltSystemEnableControl, 0);

    tasker.setTimeout(SaltSystemPowerOff, SaltSystempowerOffDelay);
  }
}
void SaltSystemPowerOff()
{
  if (SaltSystemResetViaPowerCycleCounter >= SaltSystemResetViaPowerCycle)
  {
    SetOutput(SaltSystemPower, 0);
    SaltSystemResetViaPowerCycleCounter = 0;
  }
  // wieder auskommentieren wenn neue Anlage iengebaut wird
  // SetOutput(SaltSystemPower, 0);

  SaltSystemResetViaPowerCycleCounter++;

  SaltSystemAutomaticOn = false;
  UpdateMqtt();
  // client.publish("gimpire/EspPool/SaltSystemModeAutomatic", String(SaltSystemAutomaticOn).c_str());
  //String Msg = "MQTT SaltSystemModeAutomatic " + String(SaltSystemAutomaticOn);
  //mesh.SendMessage(Msg);
}
void SetSaltSystemAutomaticOnTime(uint8_t Time)
{
  SaltSystemAutomaticOnTime = Time;
  UpdateMqtt();
  String Msg = "MQTT SetSetSaltSystemAutomaticOnTime " + String(SaltSystemAutomaticOnTime);
  GBusMesh.SendMessage(Msg);
}
void SetFilterpumpAutomaticOnTime(uint8_t Time)
{
  FilterpumpAutomaticOnTime = Time;
  UpdateMqtt();
  // client.publish("gimpire/EspPool/FilterpumpAutomaticOnTime", String(FilterpumpAutomaticOnTime).c_str());
  //String Msg = "MQTT FilterpumpAutomaticOnTime " + String(FilterpumpAutomaticOnTime);
  //mesh.SendMessage(Msg);

  //Serial.println("FilterpumpAutomaticOnTime: " + String(FilterpumpAutomaticOnTime));
}
void SetValvePosition(int ValveToHeat)
{
  tasker.cancel(ValvePowerOff);
  SetOutput(ValvePowerOutput, 0);
  delay(100);
  ValvePositionHeat = ValveToHeat;

  if (ValveToHeat)
  {
    SetOutput(ValveOutput, 1);
  }
  else
  {
    SetOutput(ValveOutput, 0);
  }

  delay(100);
  SetOutput(ValvePowerOutput, 1);
  tasker.setTimeout(ValvePowerOff, ValvePowerOffDelay);
  UpdateMqtt();
  //String Msg = "MQTT ValveToHeat " + String(ValvePositionHeat);
  //mesh.SendMessage(Msg);
  // client.publish("gimpire/EspPool/ValveToHeat", String(ValvePositionHeat).c_str());
}
void ValvePowerOff()
{
  SetOutput(ValvePowerOutput, 0);
  delay(200);
  SetOutput(ValveOutput, 0);
}
