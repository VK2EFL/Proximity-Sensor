/* 
No ring buffer filtering
This code is for an ultrasonic proximity warning unit.
The code has been implemented on an Arduino Uno. An Arduino Nano or an Arduino Pro Mini are also viable
Pin 4 is connected to a vibration motor (haptic feedback).

The formulae for distance calculations:
cm     = (duration / 2) / 29.1;
inches = (duration / 2) / 74  ;

Watch Dog Timer (WDT) register bits and usage
WDTCSR register bits and usage
Bit   7    6    5    4    3    2    1    0 
Name WDIF WDIE WDP3 WDCE WDE  WDP2 WDP1 WDP0

WDP3 WDP2 WDP1 WDP0 Time-out
 3     2    1    0    (ms)
 0     0    0    0     16
 0     0    0    1     32
 0     0    1    0     64
 0     0    1    1    125
 0     1    0    0    250
 0     1    0    1    500
 0     1    1    0   1000
 0     1    1    1   2000
 1     0    0    0   4000
 1     0    0    1   8000

*** NOTE: WDP3 is bit 5 of the WDTCSR register and is not contiguous with WDP2, WDP1 and WDP0

CLKPR = _BV(CLKPCE);                                                         // write the CLKPCE bit to one and all the other to zero

Clock prescaler values to divide the 16MHz clock by a factor (the prescaler value)
CLKPR = 0x80;                                                                // ready for clock change
CLKPR = 0x00;                                                                // set the clock prescaler to /1 (native clock speed)
CLKPR = 0x01;                                                                // set the clock prescaler to /2
CLKPR = 0x02;                                                                // set the clock prescaler to /4
CLKPR = 0x03;                                                                // set the clock prescaler to /8
CLKPR = 0x04;                                                                // set the clock prescaler to /16
CLKPR = 0x05;                                                                // set the clock prescaler to /32
CLKPR = 0x06;                                                                // set the clock prescaler to /64
CLKPR = 0x07;                                                                // set the clock prescaler to /128
CLKPR = 0x08;                                                                // set the clock prescaler to /256
*/

#include <avr/wdt.h>                                                         // library for default watchdog functions
#include <avr/interrupt.h>                                                   // library for interrupts handling
#include <avr/sleep.h>                                                       // library for sleep
#include <avr/power.h>                                                       // library for power control

//#define DEBUG                                                                // uncomment if you want the leds to flash showing processing status and haptic line output
//#define SERIAL_MON                                                           // uncomment if you want information displayed on the serial monitor
#ifdef DEBUG
  #define activityLED                    12                                  // the pin the led is attached to and indicates 'awake and processing'
  #define hapticLED                      13                                  // the pin the led is attached to that indicates 'haptic' output
#endif
#define hapticLine                        4                                  // connect a haptic motor to this line
#define echoPin                           6                                  // ultrasonic unit echo receive pin
#define trigPin                           7                                  // ultrasonic unit trigger pin
#define numberOfMapSteps                  6                                  // the number of steps that we use when we map the received value to a step number
#define maxRange                       8000                                  // the value that is considered the maximum value (distance limit) that we will consider for this operation
#define timeStoreArrayLength              8                                  // the number of elements in the timeStore array, used to ascertain inactivity

int  newValue                    = maxRange;                                 // the distance value that is recorded
int  deviation                   =        0;                                 // the deviation of a new value from the average
long arrayMap            = numberOfMapSteps;                                 // the number of steps that we apply to the range of readings that are received, used to determine haptic motor activity (urgency)
int  timeStore[timeStoreArrayLength];                                        // an array of values that are used to determine inactivity, if all values are less than a threshold limit then go to sleep
int  devStore[timeStoreArrayLength];                                         // an array used to store the deviations of each received value from the average
int  timeStorePtrStart           =        0;                                 // a pointer into the timeStore and devStore arrays
int  timeStorePtrEnd             =        1;                                 // a pointer into the timeStore and devStore arrays

unsigned long timeStoreSum       = maxRange;                                 // the sum of the values in the array
unsigned long devStoreSum        =        0;                                 // the sum of the values in the array
int timeStoreAvg                 =        0;                                 // the average value of those values stored in the timeStore array
int devStoreAvg                  =        0;                                 // the average value of those values stored in the devStore array
const int devStoreAvgLimit       =      100;                                 // the deviation average, below which we consider the received distances to be stable and so we can enter sleep mode
const int timeStoreInterval      =      500;                                 // the time, in mS, between storing values into the timeStore array
unsigned long timeStoreTimer     = millis();                                 // the timer used to time how long it has been since the last recording into the timeStore array
unsigned long hapticTimer        = millis();                                 // used for timing the haptic motor on and off times, in accordance with the various proximities detected
byte hapticState                 =      LOW;                                 // used to track if we have the haptic motor on or off
unsigned long previousMillis     =        0;                                 // store the last time haptic motor was updated
unsigned long currentMillis      =        0;                                 // a timer used for the haptic motor on/off timing
unsigned long wakeUpFromWDTTimer = millis();                                 // a timer to time how long it has been since we woke from the WDT time out
const int WDTWakeUpTime          =       50;                                 // the time from a WDT wake up before we begin to process echos - to prevent false data immediately after wake up

ISR(WDT_vect) {                                                              // interrupt raised by the watchdog timer firing (it times out). When the watchdog fires, this function will be executed,  remember that interrupts are disabled in ISR functions
  wdt_reset();                                                               // reset the watchdog timer. so we get at least one wdt interval before this interrupt is generated again
}                                                                            // end ISR(WDT_vect)

void configure_wdt(void) {                                                   // function to configure the watchdog
  cli();                                                                     // disable interrupts for changing the registers 
  MCUSR   = 0;                                                               // reset status register flags; put timer in interrupt-only mode
  WDTCSR |= 0b00011000;                                                      // set WDCE bit (5th from left) and WDE (4th from left) in the register WDTSCR to enter config mode
  WDTCSR  =  0b01000000 | 0b00000111;                                        // set WDIE: interrupt enabled (left side of the bar); clr WDE: disable WDT restart and set the WDT delay interval (right side of bar), to 2000 milli seconds
  sei();                                                                     // re-enable interrupts
}

void sleep(void) {                                                           // put the Arduino into a deep sleep. Only an interrupt can wake it up
  digitalWrite(hapticLine, LOW);                                             // turn off the haptic motor
  #ifdef DEBUG
    digitalWrite(hapticLED, LOW);                                            // make sure the led showing haptic activity is off
  #endif                                                                     // DEBUG
  wdt_reset();                                                               // reset the watchdog timer now, so we get one full wdt period of sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);                                       // set sleep to be a total power down. Only external interrupts or the watchdog timer can wake the CPU! 
  sleep_mode();                                                              // enable sleep and enter sleep mode

  // CPU is now asleep and program execution completely halts! It is awoken only by the WDT interrupt
  // Once awake, execution will resume at this point if the watchdog is configured for resume rather than restart. In this sketch it is configured for resume

  sleep_disable();                                                           // when awake, disable sleep mode
}

void hapticOutput(unsigned long hapticOnTime, unsigned long hapticOffTime) {
  currentMillis = millis();                                                  // get the current time
  if((hapticState == HIGH) && (currentMillis - previousMillis > hapticOnTime)) {     // check to see if we have been high for longer than the on time and change the state of the haptic motor to off
    hapticState = LOW;                                                       // set the state to LOW
    previousMillis = currentMillis;                                          // remember the time
    digitalWrite(hapticLine, LOW);                                           // turn off the haptic motor
    #ifdef DEBUG
      digitalWrite(hapticLED, LOW);                                          // turn on the led showing haptic activity
    #endif                                                                   // DEBUG
  }                                                                          // end if
  if ((hapticState == LOW) && (currentMillis - previousMillis > hapticOffTime)) {    // check to see if we have been low longer than the off time and change the state of the haptic motor to on
    hapticState = HIGH;                                                      // set the state to HIGH
    previousMillis = currentMillis;                                          // remember the time
    digitalWrite(hapticLine, HIGH);                                          // turn on the haptic motor
    #ifdef DEBUG
      digitalWrite(hapticLED, HIGH);                                         // turn on the led showing haptic activity
    #endif                                                                   // DEBUG
  }                                                                          // end if
}                                                                            // end hapticOutput

void setup() {
  #ifdef SERIAL_MON
    Serial.begin(9600);                                                      // if we are going to be use the SerialMonitor, initialise it
  #endif
  #ifdef DEBUG
    pinMode(activityLED, OUTPUT);                                            // set the activity led pin to be an output
    digitalWrite(activityLED, LOW);                                          // turn off the activity led
    pinMode(hapticLED, OUTPUT);                                              // set the haptice led pin to be an output
    digitalWrite(hapticLED, LOW);                                            // turn off the haptic led
  #endif
  pinMode(trigPin, OUTPUT);                                                  // set the trigger pin to be an output
  digitalWrite (trigPin, LOW);                                               // and drive it low (off)
  pinMode (hapticLine, OUTPUT);                                              // set the haptic pin to be an output
  digitalWrite (hapticLine, LOW);                                            // and drive it low (off)
  pinMode(echoPin, INPUT);                                                   // set the echo pin to be an intput
  configure_wdt();                                                           // configure the watchdog timer
  power_adc_disable();                                                       // turn off the ADC since we don't use it at all - and save a little power
}                                                                            // end setup()

void loop() {
  if (millis() - wakeUpFromWDTTimer > WDTWakeUpTime) {                       // if we have cleared the WDTWakeUpTime time of inactivity aftr waking up, then get ultrasonic readings and process them
    digitalWrite(trigPin, LOW);                                              // clears the trigPin on the ultrasonic unit
    delayMicroseconds(2);                                                    // settling delay
    digitalWrite(trigPin, HIGH);                                             // sets the trigPin on the ultrasonic unit to a HIGH state
    delayMicroseconds(10);                                                   // holds the HIGH state for 10 microseconds, essentially one half a cycle of a 40kHz audio frequency
    digitalWrite(trigPin, LOW);                                              // clears the trigPin on the ultrasonic unit
    newValue = pulseIn(echoPin, HIGH);                                       // reads in the time it took to receive the return echo pulse
    if (newValue > maxRange || newValue == 0) newValue = maxRange;           // discard values that are considered to be out of range or invalid and only store values up to the maximum range
    if (millis() - timeStoreTimer > timeStoreInterval) {                     // if the timeStoreInterval has passed, save this new value to the timeStore array and calculate the new average, then calculate the new deviation average
      timeStoreTimer = millis();                                             // reset the timer
      timeStore[timeStorePtrStart] = newValue;                               // store the new value in the timeStore array
      timeStoreSum = 0;                                                      // clear the timeStoreSum in preparation for calculating the sum by adding the values stored in the array
      for (int k = 0; k < timeStoreArrayLength; k++) timeStoreSum = timeStoreSum + timeStore[k];  // calculate the new sum of the values stored in timeStore
      timeStoreAvg = timeStoreSum >> 3;                                      // determine the timeStore average by dividing the sum by 8, by shifting right 3 times (faster than using /)
      deviation    = abs(newValue - timeStoreAvg);                           // calculate the deviation of the new value from the average value
      devStore[timeStorePtrStart] = deviation;                               // store the new deviation value in the devStore array
      devStoreSum = 0;                                                       // clear the devStoreSum in preparation for calculating the sum by adding the values stored in the array
      for (int k = 0; k < timeStoreArrayLength; k++) devStoreSum = devStoreSum + devStore[k];  // calculate the new sum of the deviation values stored in devStore
      devStoreAvg  = devStoreSum >> 3;                                       // determine the average deviation by dividing the sum by 8, by shifting right 3 times (faster than using /)
      timeStorePtrStart = (timeStorePtrStart + 1) % timeStoreArrayLength;    // update the pointer to point to the next location in the arrays
      timeStorePtrEnd   = (timeStorePtrEnd   + 1) % timeStoreArrayLength;    // update the pointer to point to the next location in the arrays

      #ifdef SERIAL_MON
        Serial.print(timeStorePtrStart);                                     // print on the serial monitor some of the internal values
        Serial.print(" ");                                                   // 
        Serial.print(timeStorePtrEnd);                                       // 
        Serial.print(" ");                                                   // 
        Serial.print(newValue);                                              // 
        Serial.print(" ");                                                   // 
        Serial.print(timeStoreSum);                                          // 
        Serial.print(" ");                                                   // 
        Serial.print(timeStoreAvg);                                          // 
        Serial.print(" ");                                                   // 
        Serial.print(deviation);                                             // 
        Serial.print(" ");                                                   // 
        Serial.print(devStoreSum);                                           // 
        Serial.print(" ");                                                   // 
        Serial.println(devStoreAvg);                                         // 
      #endif                                                                 // SERIAL_MON
    }                                                                        // end if (millis() - timeStoreTimer > timeStoreInterval)

    if (devStoreAvg < devStoreAvgLimit) {                                    // here is where we determine whether to go to sleep or not - have things stabilised?
      #ifdef DEBUG
        digitalWrite(activityLED, LOW);                                      // since we are going into sleep, turn off the activity led
	    #endif                                                                 // DEBUG
      sleep();                                                               // the recorded values are not changing enough, therefore the average deviation is below the deviastion limit, so sleep in lowest power mode
      wakeUpFromWDTTimer = millis();                                         // start the wakeUpFromWDTTimer
      #ifdef DEBUG
        digitalWrite(activityLED, HIGH);                                     // turn on the led to show we are still processing
	    #endif                                                                 // DEBUG
    } else {                                                                 // we are not going to sleep
      wdt_reset();                                                           // so we reset the watchdog timer so that we don't enter sleep mode
	    #ifdef DEBUG
        digitalWrite(activityLED, HIGH);                                     // turn on the activity led to show we are still processing
	    #endif                                                                 // DEBUG
    }                                                                        // end else

    arrayMap = map(newValue, 0, maxRange, 0, numberOfMapSteps);              // map the new value into one of the number of steps specified
    switch(arrayMap) {
      case 0:   hapticOutput(100,    3); break;                              // closer than  160mm  vibration cycle time  103ms
      case 1:   hapticOutput(100,   10); break;                              // closer than  333mm  vibration cycle time  110ms
      case 2:   hapticOutput(100,   75); break;                              // closer than  500mm  vibration cycle time  175ms
      case 3:   hapticOutput(100,  200); break;                              // closer than  667mm  vibration cycle time  300ms
      case 4:   hapticOutput(100,  400); break;                              // closer than  835mm  vibration cycle time  500ms
      case 5:   hapticOutput(100,  900); break;                              // closer than 1500mm  vibration cycle time 1000ms
      default:  {
        digitalWrite(hapticLine, LOW);                                       // if we don't have a value to consider then turn off the haptic motor
        #ifdef DEBUG
          digitalWrite(hapticLED, LOW);                                      // and trun off the led that indicates haptic activity
	      #endif                                                               // DEBUG
      }
    }                                                                        // end switch
  }                                                                          // end if (millis() - wakeUpFromWDTTimer > WDTWakeUpTime)
  delay(100);                                                                 // after some experimentation, this delay is required to settle things down and avoid false readings
}                                                                            // end loop()
