
#include "TemperatureSensor.h"

void readSensor()
{
    WaterThermometerValue = sensors.getTempC(WaterThermometer);
    VorlaufThermometerValue = sensors.getTempC(VorlaufThermometer);
    RucklaufThermometerValue = sensors.getTempC(RucklaufThermometer);
    GarageRoofThermometerValue = sensors.getTempC(GarageRoofThermometer);
    NewTemperatures = true;
}

void startConversion()
{
    // start temperature conversion (does not block)
    sensors.requestTemperatures();
    // schedule reading the actual temperature in 750 milliseconds
    tasker.setTimeout(readSensor, 750);
    tasker.setTimeout(startConversion, durationTemp);
}