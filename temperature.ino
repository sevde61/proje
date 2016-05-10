#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

////////////////

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
 double tempF=0.0;


//audioo/////////////////
 const int sampleWindow = 1000;

 unsigned int sample;

 int barLength = 0;
 const int barLengthTotal = 120;
 char barGraph[barLengthTotal] = "";
/////////////

void setup(void)
{
  // start serial port
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin();

////////////////light////////////////
Serial.println("Light Sensor Test"); Serial.println("");

/* Initialise the sensor */
if(!tsl.begin())
{
  /* There was a problem detecting the ADXL345 ... check your connections */
  Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
  while(1);
}

/* Display some basic information on this sensor */
displaySensorDetails();

/* Setup the sensor gain and integration time */
configureSensor();

/* We're ready to go! */
Serial.println("");
////////////////////////////////////

}

void loop(void)
{
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  sensors.requestTemperatures();

  Serial.print("Temperature is: ");
  // You can have more than one IC on the same bus.
  // 0 refers to the first IC on the wire

  tempF=sensors.getTempCByIndex(0);
  Serial.println(tempF);



  /////audioo///////////////////////7
  unsigned long startMillis= millis();  // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 4096;

  // collect data for sampleWindow mS
  while (millis() - startMillis < sampleWindow)
  {
      sample = analogRead(0);
      if (sample < 4096)  // toss out spurious readings
      {
          if (sample > signalMax)
          {
              signalMax = sample;  // save just the max levels
          }
          else if (sample < signalMin)
          {
              signalMin = sample;  // save just the min levels
          }
      }
  }

  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  // convert to millivolts
  int millivolts = int(1000 * (3.3 * peakToPeak) / 4096);

  Serial.println(millivolts);

  barLength = int(millivolts/10);
  for (int i = 0; i < barLengthTotal; i++) {
      if (i <= barLength) {
          barGraph[i] = '*';
      } else {
          barGraph[i] = ' ';
      }
  }
  Serial.println(barGraph);
  ////////////////////////////////////

 //////////////////light/////////////
 /* Get a new sensor event */
 sensors_event_t event;
 tsl.getEvent(&event);

 /* Display the results (light is measured in lux) */
 if (event.light)
 {
   Serial.print(event.light); Serial.println(" lux");
 }
 else
 {
   /* If event.light = 0 lux the sensor is probably saturated
      and no reliable data could be generated! */
   Serial.println("Sensor overload");
 }
 delay(250);

 //////////////////////////////


}

////////////////light////////////////////
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

// Configures the gain and integration time for the TSL2561
void configureSensor(void)
{
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */

  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}
