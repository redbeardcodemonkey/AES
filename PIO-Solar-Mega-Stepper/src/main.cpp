#include <Arduino.h>
#include <Bounce2.h>
#include <AccelStepper.h>
#include <Ephemeris.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

/*Button and switch configuration*/
//define service button pins
#define LAY_FLAT_BUTTON_PIN 0
#define MANUAL_BUTTON_PIN 0
#define AUTO_BUTTON_PIN 0

//define rotation limit switch pins
#define SLEW_MIN_SWITCH_PIN 0 //At counter-clockwise limit
#define SLEW_MAX_SWITCH_PIN 0 //At clockwise limit
#define ROTATE_MIN_SWITCH_PIN 0 //At counter-clockwise limit
#define ROTATE_MAX_SWITCH_PIN 0 //At clockwise limit
#define NORTH_SWITCH_PIN 0 //north point
#define HORIZON_SWITCH_PIN 0 //horizon point

//Instatiate debounce objects (button class)
Button layFlatButton = Button();
Button manualButton = Button();
Button autoButton = Button();
Button slewMinSwitch = Button();
Button slewMaxSwitch = Button();
Button rotateMinSwitch = Button();
Button rotateMaxSwitch = Button();
Button northSwitch = Button();
Button horizonSwitch = Button();


/*GPS configuration*/
#define GPS_RX_PIN 0
#define GPS_TX_PIN 0

static const uint32_t GPSBaud = 4800;
static const int MAX_SATELLITES = 40;
static const int PAGE_LENGTH = 40;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(GPS_RX_PIN, GPS_TX_PIN);

// Epherium object
Ephemeris eph;

/*LED configuration*/
//define service button LEDS
#define LAY_FLAT_LED_PIN 0
#define MANUAL_LED_PIN 0
#define AUTO_LED_PIN 0

/*Stepper Configuration*/
AccelStepper slewStepperMotor(AccelStepper::FULL4WIRE, 0, 0, 0, 0);
long slewStepperMotorLimits[2] = {0,0};
AccelStepper rotateStepperMotor(AccelStepper::FULL4WIRE, 0, 0, 0, 0);
long rotateStepperMotorLimits[2] = {0,0};


void setup() {
  /*Button and switch setup*/
  layFlatButton.attach( LAY_FLAT_BUTTON_PIN , INPUT_PULLUP );
  layFlatButton.interval(5); // interval in ms
  layFlatButton.setPressedState(LOW);
  
  manualButton.attach( MANUAL_BUTTON_PIN , INPUT_PULLUP );
  manualButton.interval(5); // interval in ms
  manualButton.setPressedState(LOW);

  autoButton.attach( AUTO_BUTTON_PIN , INPUT_PULLUP );
  autoButton.interval(5); // interval in ms
  autoButton.setPressedState(LOW);

  slewMinSwitch.attach( SLEW_MIN_SWITCH_PIN , INPUT_PULLUP );
  slewMinSwitch.interval(5); // interval in ms
  slewMinSwitch.setPressedState(LOW);

  slewMaxSwitch.attach( SLEW_MAX_SWITCH_PIN , INPUT_PULLUP );
  slewMaxSwitch.interval(5); // interval in ms
  slewMaxSwitch.setPressedState(LOW);

  rotateMinSwitch.attach( ROTATE_MIN_SWITCH_PIN , INPUT_PULLUP );
  rotateMinSwitch.interval(5); // interval in ms
  rotateMinSwitch.setPressedState(LOW);

  rotateMaxSwitch.attach( ROTATE_MAX_SWITCH_PIN , INPUT_PULLUP );
  rotateMaxSwitch.interval(5); // interval in ms
  rotateMaxSwitch.setPressedState(LOW);

  northSwitch.attach( NORTH_SWITCH_PIN , INPUT_PULLUP );
  northSwitch.interval(5); // interval in ms
  northSwitch.setPressedState(LOW);

  horizonSwitch.attach( HORIZON_SWITCH_PIN , INPUT_PULLUP );
  horizonSwitch.interval(5); // interval in ms
  horizonSwitch.setPressedState(LOW);

  /*LED setup*/
  pinMode(LAY_FLAT_LED_PIN,OUTPUT);
  pinMode(MANUAL_LED_PIN,OUTPUT);
  pinMode(AUTO_LED_PIN,OUTPUT);

  /*Stepper Setup*/
  //slew motor
  slewStepperMotor.setMaxSpeed(100);
  slewStepperMotor.setAcceleration(50);
  slewStepperMotorLimits[0]=homeStepper(slewStepperMotor,slewMinSwitch,false);
  slewStepperMotorLimits[1]=homeStepper(slewStepperMotor,slewMaxSwitch,true);
  
  //rotation motor
  rotateStepperMotor.setMaxSpeed(100);
  rotateStepperMotor.setAcceleration(50);
  rotateStepperMotorLimits[0] = homeStepper(rotateStepperMotor,rotateMinSwitch,false);
  rotateStepperMotorLimits[1] = homeStepper(rotateStepperMotor,rotateMaxSwitch,true);
  
  /*GPS Setup*/
  ss.begin(GPSBaud);

}

void loop() {
  // put your main code here, to run repeatedly:
  buttonStateMachine();
}

/*Button state functions*/
void buttonStateMachine(){
  if(autoButton.pressed() && !manualButton.pressed() && !layFlatButton.pressed()){
    long northPosition = focusStepper(slewStepperMotor,slewMinSwitch,slewMaxSwitch,layFlatButton,manualButton,northSwitch); 
    long horizonPosition = focusStepper(rotateStepperMotor,rotateMinSwitch,rotateMinSwitch,layFlatButton,manualButton,horizonSwitch);
    autoStepper(slewStepperMotor,rotateStepperMotor);
  }
  if(layFlatButton.pressed() && !manualButton.pressed() && !autoButton.pressed()){
    layFlat(slewStepperMotor,rotateStepperMotor);
  }
  if(manualButton.pressed()&& layFlatButton.pressed()&&!autoButton.pressed()){
    if(manualButton.duration()>3000){ //hold the manual button for 3 seconds to enter jog mode
      //enter jog the slew stepper motor
      jogStepper(slewStepperMotor,slewMinSwitch,slewMaxSwitch,layFlatButton,autoButton);
    }
  }
  if(manualButton.pressed()&& layFlatButton.pressed()&&!autoButton.pressed()){
    if(manualButton.duration()>3000){ //hold the manual button for 3 seconds to enter jog mode
      //enter jog the rotation stepper motor
      jogStepper(rotateStepperMotor,rotateMinSwitch,rotateMaxSwitch,layFlatButton,autoButton);
    }
  }
}
  
  //create other if statements for button combos to jog however you want to jog, jog method is ready to go.

}

/*stepper motor functions*/

//function to move stepper to a limit switch
long homeStepper(AccelStepper stepperMotor, Button limitSwitch, boolean direction){
  long position = stepperMotor.currentPosition();  
  if(direction){ //clockwise
    while(!limitSwitch.pressed()){
      position++;
      stepperMotor.move(position);
      stepperMotor.run();
    }
  }
  else{ //counter-clockwise
    while(!limitSwitch.pressed()){
      position--;
      stepperMotor.move(position);
      stepperMotor.run();
    }
  }
  return position;
}

//function to jog stepper motor
void jogStepper(AccelStepper stepperMotor, Button minimumLimitSwitch,Button maximumLimitSwitch, Button clockWiseButton,Button counterClockWiseButton){
  long position = stepperMotor.currentPosition();
  if(clockWiseButton.pressed()){
    if(!clockWiseButton.released()){
      while(!maximumLimitSwitch.pressed()){
        position++;
        stepperMotor.move(position);
        stepperMotor.run();
      }
    }
  }else if(counterClockWiseButton.pressed()){
    if(!counterClockWiseButton.released()){
      while(!minimumLimitSwitch.pressed()){
        position--;
        stepperMotor.move(position);
        stepperMotor.run();
      }
    }
  }
}

//function to move 
long focusStepper(AccelStepper stepperMotor, Button minimumLimitSwitch,Button maximumLimitSwitch, Button clockWiseButton,Button counterClockWiseButton,Button focusButton){
  long position = stepperMotor.currentPosition();
  if(!focusButton.pressed()){
    if(clockWiseButton.pressed()){
      if(!clockWiseButton.released()){
        while(!maximumLimitSwitch.pressed()){
          position++;
          stepperMotor.move(position);
          stepperMotor.run();
        }
      }
    }else if(counterClockWiseButton.pressed()){
      if(!counterClockWiseButton.released()){
        while(!minimumLimitSwitch.pressed()){
          position--;
          stepperMotor.move(position);
          stepperMotor.run();
        }
      }
    }
    return position;
  }
}

//function to move both steppers to flat position
void layFlat(AccelStepper slew, AccelStepper rotate){
  //calculate center point of slew motor
  long slewCenter = slewStepperMotorLimits[1]-slewStepperMotorLimits[0];
  slew.moveTo(slewCenter);
  slew.run();

  //calculate center point of rotate motor
  long rotateCenter = rotateStepperMotorLimits[1]-rotateStepperMotorLimits[0];
  rotate.moveTo(rotateCenter);
  rotate.run();
  return;
}

//function to track sun automatically
void autoStepper(AccelStepper slew, AccelStepper rotate,long northPosition, long horizonPosition){
  
  while(!manualButton.pressed()){//press manual button to break out of auto mode
    SolarSystemObject sun = calculateSolarPostion();
    if(sun != NULL){ //its daytime, automode
      if(rotateMaxSwitch.changed() || rotateMinSwitch.changed()||slewMaxSwitch.changed()||slewMinSwitch.changed()){
        layFlat(slew,rotateMinSwitch);//hit a limit switch unexpectantly
      }
      else{
        long deltaSlew,deltaRotation;
        deltaSlew = northPosition + angleToSteps(sun.horiCoordinates.azi,1,2); // you will need to input the slew ratio, then the steps per rotation
        deltaRotation = horizonPosition + angleToSteps(sun.horiCoordinates.alt,1,2); // you will need to input the rotation ratio, then the steps per rotation
        slew.moveTo(deltaSlew);
        rotate.moveTo(deltaRotation);
      }
    }
    else{
      layFlat(slew,rotate); //its night time, or unknown error from gps module
    }
  }
}

//function to convert angle to step
long angleToSteps(float angle, float ratio,long stepsPerRotation){
  return(long((angle/ratio)/stepsPerRotation));
}

/*Solar functions*/
//function to find the sun
SolarSystemObject calculateSolarPostion(){
    while (ss.available() > 0)
    if (gps.encode(ss.read())){
      if(gps.location.isValid()){
        float currentTime = eph.hoursMinutesSecondsToFloatingHours((int)gps.time.hour(),(int)gps.time.minute(),(int)gps.time.second());
        float latitude = (float)gps.location.lat();
        float longitude = (float)gps.location.lng();
        unsigned int year = (unsigned int)gps.date.year();
        unsigned int month = (unsigned int)gps.date.month();
        unsigned int day = (unsigned int)gps.date.day();
        unsigned int hour = (unsigned int)gps.time.hour();
        unsigned int minute = (unsigned int)gps.time.minute();
        unsigned int second = (unsigned int)gps.time.second();
        eph.setLocationOnEarth(latitude,longitude);
        SolarSystemObject sun = eph.solarSystemObjectAtDateAndTime(Sun,day,month,year,hour,minute,second);
        if(currentTime >= sun.rise && currentTime <= sun.set){
          return sun; //return sun object
        }
        else{
          return NULL; //night time
        }
      }
      else{
        return NULL; //gps error
      }
    }
    return NULL; //serial error
}