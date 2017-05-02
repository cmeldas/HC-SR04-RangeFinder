/*
This program is for read distance from ultrasonic sensor HC-sr04 for arduino and send it to arducopter (PX4, APM) on serial. 
Serial comunication is same as LightWare SF10, so on arducopter use same settings  -  http://ardupilot.org/copter/docs/common-lightware-sf10-lidar.html
Only RNGFND_MAX_CM se to "300" it is in centimetres

I wrote only small part of it, the compute part for mode, median etc i found on internet.
*/
#define LED_PIN 13                            // Standard LED PIN - LED is on when reading is out of range
#define MIN_RANGE 1
#define MAX_RANGE 320                         

#define SPIKE_EFFECT 4.0                      // used in spike filter - effectively this is the size of teh equivalent moving average filter
#define ARRAY_SIZE 5                          // size of median and mode sampling arrays. Needs to be an odd number

#define STBY_MODE 0                           // Startup Mode 
#define NORM_MODE 1                           // Normal Mode: Return latest raw sonar value on request 
#define MODE_MODE 2                           // Mode filter mode: Return the latest value from running 'mode' filter
#define MEDN_MODE 3                           // Median filter mode: Return the latest value from running 'median' filter
#define SPIK_MODE 4                           // Spike filter mode: Return value from running 'spike' filter

unsigned long lastT = 0;
const unsigned long updateTime = 30;          // time between sonar reads

int val = 0;
int filtMode = STBY_MODE;
// For Mode / median filter variables needed to store values
// array to store the raw samples
int rawValues[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
// array to store the sorted samples
int srtValues[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
// Index for latest position in array
int idx = 0;

#include <NewPing.h>      // use Timer 2

#define TRIGGER_PIN  3  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     2  // Arduino pin tied to echo pin on the ultrasonic sensor.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_RANGE); // NewPing setup of pins and maximum distance.


void setup() {

  Serial.begin(19200);
  pinMode(LED_PIN, OUTPUT);                   // Setup LED pin
  digitalWrite(LED_PIN, HIGH);                // set the LED on
  filtMode = MODE_MODE;                       // Start in modal filter mode
}


int getCm() {

  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS)
  unsigned int cm = uS / US_ROUNDTRIP_CM;
  if (uS == 0) {                    // 0 means out of range or sometimes extremely close
    digitalWrite(13, HIGH);
    cm = MAX_RANGE;
  }
  else if (cm > 300) {
    digitalWrite(13, HIGH);
  }
  else digitalWrite(13, LOW);

  return cm;
}


void loop()
{
  unsigned long nowT = millis();

  if (lastT + updateTime < nowT) {
    int tmp = 0;
    tmp = getCm();
    if (tmp > MAX_RANGE) tmp = MAX_RANGE;
    if (tmp < MIN_RANGE) tmp = 0;

    // Process the Sonar data
    switch (filtMode) {
      case NORM_MODE:
        val = tmp;                                // Set the value (val) to the latest data
        break;
      case SPIK_MODE:
        val = val * (1.0 - (1.0 / SPIKE_EFFECT)) + tmp * (1.0 / SPIKE_EFFECT);    // implement a slow ramp / spike filter somewhat equiv to x-sample running average
        break;
      case MODE_MODE:
        //Do MODE stuff here
        rawValues[idx] = tmp;
        idx++;
        if (idx >= ARRAY_SIZE)
          idx = 0;
        for (int i = 0; i < ARRAY_SIZE; i++)      // copy raw data to sorted data array
          srtValues[i] = rawValues[i];
        isort(srtValues, ARRAY_SIZE);             // sort the sorted data array
        val = mode(srtValues, ARRAY_SIZE);        // find the modal value and use it
        break;
      case MEDN_MODE:
        //Do MEDN stuff here
        rawValues[idx] = tmp;
        idx++;
        if (idx >= ARRAY_SIZE)
          idx = 0;
        for (int i = 0; i < ARRAY_SIZE; i++)      // copy raw data to sorted data array
          srtValues[i] = rawValues[i];
        isort(srtValues, ARRAY_SIZE);             // sort the sorted data array
        val = median(srtValues, ARRAY_SIZE);      // find the median value and use it
        break;
      default:
        break;
        lastT = nowT;
    }


    if (Serial.available() > 0) {
      // read the incoming byte:
      int incomingByte = Serial.read();       // when something is recieved send sonar data
      float floatm = val;                    // Send value should be in format "123.45\r\n"   same as Lightware SF10
      floatm = floatm * 0.01f ;
      Serial.print(floatm, 2);
      Serial.print("\r\n");
    }
  }
}


//---------------------------------------
//Sorting function
// sort function (Author: Bill Gentles, Nov. 12, 2010)
void isort(int *a, int n) {
  for (int i = 1; i < n; ++i)
  {
    int j = a[i];
    int k;
    for (k = i - 1; (k >= 0) && (j < a[k]); k--)
    {
      a[k + 1] = a[k];
    }
    a[k + 1] = j;
  }

}

//---------------------------------------
//Mode function, returning the modal value (or median value if mode does not exist).
int mode(int *x, int n) {

  int i = 0;
  int count = 0;
  int maxCount = 0;
  int mode = 0;
  int bimodal;
  int prevCount = 0;

  while (i < (n - 1)) {

    prevCount = count;
    count = 0;

    while (x[i] == x[i + 1]) {

      count++;
      i++;
    }

    if (count > prevCount & count > maxCount) {
      mode = x[i];
      maxCount = count;
      bimodal = 0;
    }
    if (count == 0) {
      i++;
    }
    if (count == maxCount) {                  //If the dataset has 2 or more modes.
      bimodal = 1;
    }
    if (mode == 0 || bimodal == 1) {          //Return the median if there is no mode.
      mode = x[(n / 2)];
    }
    return mode;
  }

}

//---------------------------------------
//Mode function, returning the median.
int median(int *x, int n) {

  return (x[(n / 2)]);
}



