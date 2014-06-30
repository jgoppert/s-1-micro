// SR1 Arduino Controller
//
// rcarduino.blogspot.com
//
// A simple approach for reading three RC Channels using pin change interrupts
//
// See related posts - 
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// rcarduino.blogspot.com
//

// include the pinchangeint library - see the links in the related topics section above for details
#include <PinChangeInt.h>

#include <Servo.h>

// Assign your channel in pins
#define THROTTLE_IN_PIN 8
#define AILERON_IN_PIN 9
#define ELEVATOR_IN_PIN 10

// Assign your channel out pins
#define LEFT_WING_OUT_PIN 6
#define RIGHT_WING_OUT_PIN 7

// Servo objects generate the signals expected by Electronic Speed Controllers and Servos
// We will use the objects to output the signals we read in
// this example code provides a straight pass through of the signal with no custom processing
Servo servoLeftWing;
Servo servoRightWing;

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define AILERON_FLAG 2
#define ELEVATOR_FLAG 4

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the 
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unAileronInShared;
volatile uint16_t unElevatorInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulAileronStart;
uint32_t ulElevatorStart;

float cycleFrequency = 1;
uint64_t timeStart = 0;
    
void setup()
{
  Serial.begin(9600);
  
  Serial.println("multiChannels");

  // attach servo objects, these will generate the correct 
  // pulses for driving Electronic speed controllers, servos or other devices
  // designed to interface directly with RC Receivers  
  servoLeftWing.attach(LEFT_WING_OUT_PIN);
  servoRightWing.attach(RIGHT_WING_OUT_PIN);

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE); 
  PCintPort::attachInterrupt(AILERON_IN_PIN, calcAileron,CHANGE); 
  PCintPort::attachInterrupt(ELEVATOR_IN_PIN, calcElevator,CHANGE);
  
  timeStart = micros();
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained 
  // between calls to loop.
  static uint16_t unThrottleIn;
  static uint16_t unAileronIn;
  static uint16_t unElevatorIn;
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
    
    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
    
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }
    
    if(bUpdateFlags & AILERON_FLAG)
    {
      unAileronIn = unAileronInShared;
    }
    
    if(bUpdateFlags & ELEVATOR_FLAG)
    {
      unElevatorIn = unElevatorInShared;
    }
     
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
    
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the 
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }
  
  // do any processing from here onwards
  // only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
  // variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by 
  // the interrupt routines and should not be used in loop
  
  // the following code provides simple pass through 
  // this is a good initial test, the Arduino will pass through
  // receiver input as if the Arduino is not there.
  // This should be used to confirm the circuit and power
  // before attempting any custom processing in a project.
  
  // we are checking to see if the channel value has changed, this is indicated  
  // by the flags. For the simple pass through we don't really need this check,
  // but for a more complex project where a new signal requires significant processing
  // this allows us to only calculate new values when we have new inputs, rather than
  // on every cycle.
  if(bUpdateFlags & THROTTLE_FLAG)
  {
    // compute inputs
    float throttle = pwm2Norm(unThrottleIn, 1100, 800);
    float aileron = pwm2Norm(unAileronIn, 1500, 400);
    float elevator = pwm2Norm(unElevatorIn, 1500, 400);
    
    // find cycle frequency
    cycleFrequencyFunction(throttle, cycleFrequency);
    
    // compute time
    uint64_t timeNow = micros();
    uint32_t t_micros = timeNow - timeStart;
    uint32_t period_micros = 1e6/cycleFrequency;
    if (t_micros > period_micros) {
      t_micros -= period_micros*(t_micros/period_micros);
      timeStart = timeNow - t_micros;
    }
    float t = t_micros/1.0e6;

    // compute left/right wing
    float leftWing = 0;
    float rightWing = 0;
    flappingFunction(t, throttle, aileron, elevator, leftWing, rightWing);
    
    // send to motors
    servoLeftWing.writeMicroseconds(norm2Pwm(-leftWing, 1500, 400));
    servoRightWing.writeMicroseconds(norm2Pwm(rightWing, 1500, 400));
  }
  bUpdateFlags = 0;
}

void flappingFunction(float t, float throttle, float aileron, float elevator, float & leftWing, float & rightWing) {
  
  // flapping parameters
  const float servoTravel = 45.0;
  const float wingUp = -25;
  const float wingDown = 30;
  const float wingGlide = -8;
  const float timeDown2Up = 0.1;
  const float timeUp2Glide = 0.2;

  if (throttle < 0.2) {
    leftWing = -elevator + aileron + wingGlide/servoTravel;
    rightWing = -elevator - aileron + wingGlide/servoTravel;
  } else {
    if (t < timeDown2Up) {
      leftWing = -elevator + aileron + wingDown/servoTravel;
      rightWing = -elevator - aileron + wingDown/servoTravel;
    } else if (t < timeUp2Glide) {
      leftWing = -elevator + aileron + wingUp/servoTravel;
      rightWing = -elevator - aileron + wingUp/servoTravel;
    } else {
      leftWing = -elevator + aileron + wingGlide/servoTravel;
      rightWing = -elevator - aileron + wingGlide/servoTravel;
    }
  }
}

void cycleFrequencyFunction(float throttle, float & cycleFrequency) {
  if (throttle < 0) throttle = 0;
  if (throttle > 1) throttle = 1;
  cycleFrequency = 1.66 + 3.33*throttle;
}

float pwm2Norm(uint16_t pwm, uint16_t center, uint16_t width) {
  return float((int32_t)pwm - center)/width;
}

uint32_t norm2Pwm(float norm, uint16_t center, uint16_t width) {
  return norm*width + center;
}


// simple interrupt service routine
void calcThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  { 
    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcAileron()
{
  if(digitalRead(AILERON_IN_PIN) == HIGH)
  { 
    ulAileronStart = micros();
  }
  else
  {
    unAileronInShared = (uint16_t)(micros() - ulAileronStart);
    bUpdateFlagsShared |= AILERON_FLAG;
  }
}

void calcElevator()
{
  if(digitalRead(ELEVATOR_IN_PIN) == HIGH)
  { 
    ulElevatorStart = micros();
  }
  else
  {
    unElevatorInShared = (uint16_t)(micros() - ulElevatorStart);
    bUpdateFlagsShared |= ELEVATOR_FLAG;
  }
}
