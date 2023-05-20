#ifndef TemperatureSensor_H
#define TemperatureSensor_H

#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#define ONE_WIRE_BUS 16
#define TEMPERATURE_PRECISION 12

//Temperature definitions
DeviceAddress WaterThermometer = {0x28, 0x4F, 0x23, 0xEC, 0x50, 0x20, 0x01, 0x46};
DeviceAddress VorlaufThermometer = {0x28, 0x52, 0x04, 0xE8, 0x50, 0x20, 0x01, 0xF0};
DeviceAddress RucklaufThermometer = {0x28, 0x47, 0x53, 0xF2, 0x50, 0x20, 0x01, 0xA7};
DeviceAddress GarageRoofThermometer = {0x28, 0x3A, 0x0D, 0xD6, 0x50, 0x20, 0x01, 0x3C};
float WaterThermometerValue, VorlaufThermometerValue, RucklaufThermometerValue, GarageRoofThermometerValue;
const long durationTemp = 1 * 60 * 1000; //The frequency of temperature measurement
bool NewTemperatures = false;

void readSensor();
void startConversion();

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

#endif