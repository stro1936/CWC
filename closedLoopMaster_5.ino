/* 8 April 2024
 This script incorperates pitching control, estop control (button or serial monitor)
 and rpm readings
 */
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_INA260.h>
#include <util/atomic.h>

Adafruit_INA260 sensor;
Adafruit_MCP4725 dac;

#define ENCA 4  // YELLOW
#define ENCB 3  // WHITE
#define PWM 10
#define IN2 9
#define IN1 8
#define GenHall 2       //Generator Hall Sensor Readings
#define Storage 12        // Control Release of Energy Storage
#define BrakeControl 11  // Send PWM to determin brake position
#define EStopButton 13    // reading from the estop button
#define bbLock 7 //buck-boost enable pin

// Constant definitions
// Pitching
volatile int posi = 0;  //Read encoder position var
int deg = 0;            // Degree Target
float kp = 2;           // Proportional constant
int e = 0;              // error between target and actual position
int u = 0;              // proportionally adjusted error
int pwr = 0;            // motor power
int b = 0;              // encoder read
int target = 0;         // encoder count target
int dir = 0;            // gear motor direction
int pos = 0;            // main loop encoder position

// loop times
float loopStart = 0.0;
float loopEnd = 0.0;
float loopTime = 0.0;

// E-brake parameters
int brake_on = 136;   // Position extended
int brake_off = 130;  // Position retracted
bool buttonState = 1;
bool lastButtonState = 0;
int brakeCondition = 1;  // 1 is unbraked and -1 is braked
int pccTrigger = 0;
int rpmCutOff = 800;
int doNotOverflowDanesEyes = 0;

// Load Parameters
int seconds = 0;
float sensorCurrent = 0.0;
int writeVoltage = 0;
float loadVoltage = 0.0;
float loadCurrent = 0.0;
float loadPower = 0.0;
float sensorPower = 0.0;
float loadResistance = 0.0;
float targetResistance = 12;

// RPM
int peakcount = 0;
int peaks = 0;      // stores number of peaks detected from generator hall sensor
int min_peaks = 7;  // number of peaks from genhall before rpm calculation (7 per rotation)
float start_time = 0.0;
float end_time = 0.0;
float time_passed = 0.0;
float rpm = 0.0;
unsigned long prevtime = 0.0;
int prevstat = 0;

//Control Parameters
int const numData = 100;  // This can make your dreams come true
int binCount = 1;
int iterator = 0;
int powerCounter = 0; //counting between power recordings
int timerPower = 2; //how many loops inbetween power readings
float startLoad = 0; //Set this based on experimental data from 5m/s wind speed
float powJump[9] = {0,0,2.8,6,11,18,27,0,0}; // threshold power for every bin [0] is air -> 5m/s, [1] is first stable power range before pitching in bin 1
//float powJump[9] = {0,0,2.8,9,14,22,29,0,0};
float binLoad[9] = {0,.200,.600,.900,1.150,1.500,2.000,0,.100};
float deltaPowStable = 0.2; //Set this to desired stability percent threshold based on experiment
float loadStepDura = 50; // mA per load increment in durability
float controlPowPitch = 2; //Pitching degree increment when slowing down during control of rated power
float controlPowPitchFine = 1;
float revUpPitch = 5; //Pitching degree increment when trying to reach max pitch
float minVoltage = 3.5; //minimum cut-in voltage before we pitch from buck-boost
float bbShutOff = 3.0;

//data keeping
float powHistOld[numData];
float powHistNew[numData];
float voltHist[numData];
float voltSum = 0.0;
float voltAve = 0.0;
float powSumNew = 0.0; //this looks at each new power reading for 
float powAveNew = 0.0; // this is looking at the last numdata and taking an average
float powSumOld = 0.0; // sum of the numData points before the new set
float powAveOld = 0.0; // the older numData set average

float deltaPow = 0.0; // instantaneus difference between powAveNew and powAveOld
float deltaPowBig = 0.0; // The difference between powAveNew and the top of the last Bin


float RPMHistOld[numData];
float RPMHistNew[numData];
float RPMAveOld = 0.0;

float binPow[9]; //Save the stable power for each bin
float loadHist[9]; // save the load value at each stable power

int brakeHistory = 0;
int count = 0;
int maxPitch = 20;
long delayCounter = 0;
int timerPitch = 1000;  //timer delay for pitching num loops
int timerControlPitch = 2000; //timer delay for control of rated power task
int timerLoad = 1300;  //timer delay for load increment loops
int loadStep = 3;
int serialTimer = 0;
int binTimer = 3000;
int powAveCounter = 0;
long capacitorCounter = 0;
int timerCapacitor = 2000;
long pccStartTimer = 0;
int pccEndTime = 10000;
int releaseCount = 0;
// int chargeCount = 0;
// int chargeNum = 10;

float check = 0.0;

void setup() {
  for (int i = 0; i < numData; i++) {
    powHistOld[i] = 0;
    powHistNew[i] = 0;
    voltHist[i] = 0;
  }

  Serial.begin(9600);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(BrakeControl, OUTPUT);
  pinMode(Storage, OUTPUT);
  digitalWrite(Storage, LOW);
  pinMode(EStopButton, INPUT);

  pinMode(GenHall, INPUT);

  pinMode(bbLock, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(GenHall), RPM_Add, RISING);

 sensor.begin();
  // sensor.setCurrentConversionTime(INA260_TIME_140_us);
  // sensor.setVoltageConversionTime(INA260_TIME_558_us);
  // sensor.setAveragingCount(INA260_COUNT_1);
 dac.begin(0x60);

  loadCurrent = startLoad;
}

void loop() {
  //measure time for overall loop time
  loopStart = micros();

if(voltAve > minVoltage && pccTrigger == 0){ //Set buck-boost enable pin to be on/off
  digitalWrite(bbLock, HIGH);
}else if(rpm > rpmCutOff && pccTrigger == 1){
  digitalWrite(bbLock, HIGH);
}else if (voltAve <= bbShutOff && pccTrigger == 0){
  digitalWrite(bbLock, LOW);
}else if(rpm <= rpmCutOff && pccTrigger == 1){
  digitalWrite(bbLock, LOW);
}

if (Serial.available()) {
    check = Serial.parseFloat();
    Serial.setTimeout(10);
    if (check > 0 && check <1000) {
      //deg = check;
      deg = check;
      binCount = 1;
    }
    else if (check == -1){
      brakeCondition = -brakeCondition;
      pccTrigger = 0;
    }
  }

  // Read and calculate Power
  loadVoltage = sensor.readBusVoltage()*0.001;
  sensorCurrent = sensor.readCurrent()*0.001;
  sensorPower = sensor.readPower()*0.001;
  loadPower = loadVoltage * loadCurrent;
  loadResistance = loadVoltage / sensorCurrent;

  //Average power old and new
  if(powerCounter > timerPower){
  powHistNew[iterator] = loadPower;
  voltHist[iterator] = loadVoltage;
  powerCounter = 0;
  iterator++;
  }
  powerCounter++;

  if(powAveCounter >= numData){ //required to make the running averages meaningful
    
    powAveCounter = 0;
  }
  powAveCounter++;

  if(binCount >= 1){
    deltaPowBig = fabs(powAveNew - binPow[binCount-1]);
  }
  if (iterator >= numData) {
    powAveOld = powAveNew; // archive powAveNew to powAveOld
    for (int i = 0; i < numData; i++){
      if (i==0){
        powSumNew = powHistNew[0]; // start by setting resetting powSumNew to the first data point
        voltSum = voltHist[0];
      }
      else{
        powSumNew += powHistNew[i];
        voltSum += voltHist[i];
      }
    }
    powAveNew = powSumNew / numData;
    deltaPow = fabs(powAveNew - powAveOld);
    voltAve = voltSum / numData;

    iterator = 0;
  }

  // Read RPM
  if (peaks > min_peaks) {

    time_passed = (micros() - start_time) / 1000000.0;
    rpm = (peaks / time_passed) * (60 / min_peaks);
    start_time = micros();
    peaks = 0;
  }
  if (micros() - start_time > 5.0 * 1000000.0) {
    rpm = 0;
  }

  // Check E-brake Button
  buttonState = digitalRead(EStopButton);
  if (buttonState >= 1 && pccTrigger == 0) {
    brakeCondition = 1;
  } else if(buttonState < 1 && pccTrigger == 0){
    brakeCondition = -1;
    rpm = 0;
  } else if(buttonState >= 1 && pccTrigger == 1){
    brakeCondition = -1;
  }

  // Check PCC disconnect
  if(voltAve < 0.01 && rpm > 500 && pccTrigger == 0 && brakeCondition == 1){
    brakeCondition = -1;
    pccTrigger = 1;
    pccStartTimer = millis();
  }

  // Pitching - else where in control we are changing deg to pitch.
  // Convert encoder signal to degree
   target = deg * (16.7);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  e = pos - target;
  u = kp * e;
  pwr = fabs(u);

  if (pwr < 15) {
    pwr = 0;
  }
  if (pwr > 14 && pwr < 30) {
    pwr = 30;
  }
  if (pwr > 255) {
    pwr = 255;
  }

  dir = 0;
  if (u < -15) {
    dir = -1;
  }
  if (u > 15) {
    dir = 1;
  }
  setMotor(dir, pwr, PWM, IN1, IN2);

  writeVoltage = (int)(loadCurrent * (4095.0 / 10));
  dac.setVoltage(writeVoltage, false);

  if (brakeCondition == -1) {
    if(millis()-pccStartTimer > pccEndTime && pccTrigger == 1){
      brakeCondition = 1;
      pccTrigger = 0;
    }
    capacitorCounter = 0;
    if(binCount == 8){
      deg = 0;
    }else{
      deg = 0;  //set pitch to inital pitch
    }
    count++;  // incrament count for not going into the next if on first pass

    // order the actuator to brake if we have already pitched
    if (pwr == 0 && count > 1) {
      analogWrite(BrakeControl, brake_on);
      brakeHistory = 1;
      count = 0;
      loadCurrent = 0;  // Set load for E-brake condition
    }

  } else if (brakeCondition == 1 && brakeHistory == 1) {  // restart condition
    if (releaseCount == 0) {
      digitalWrite(Storage, HIGH);              // discharge capacitors
      analogWrite(BrakeControl, brake_off);  // deactivate brake
      delay(5000);  //wait for rotor to spin up to speed
      releaseCount++;
    }
    // give the system time to spin up from releasing the brake.
    digitalWrite(Storage, LOW);
    if (binCount >= 2 && binCount <= 6) {  // we want to pitch back to low drag and ramp load
      if (rpm > 2000 && rpm < 2500 && deg < maxPitch){
        loadCurrent = 0.500;
      }
      if(rpm > 2500 && deg < maxPitch){
        loadCurrent = 1;
      }
      if (delayCounter > timerPitch && deg < maxPitch && voltAve > minVoltage) { // adjust timer to obtain enough time to stabilize at new pitch
        deg += revUpPitch;
        delayCounter= 0;
      } else if (delayCounter > timerLoad && deg >= maxPitch) {
        loadCurrent = binLoad[binCount-1]; //Set to 6m/s load
        count = 0;
        brakeHistory = 0;
        delayCounter= 0;
        releaseCount = 0;
      }
     
      delayCounter++;

    } else if(binCount == 7){
      if (rpm > 2000 && rpm < 2500 && deg < maxPitch-5){
        loadCurrent = 1.00;
      }
      if(rpm > 2500 && deg < maxPitch-5){
        loadCurrent = binLoad[5];
      }
      if (delayCounter > timerPitch && deg < maxPitch-5 && voltAve > minVoltage) { // adjust timer to obtain enough time to stabilize at new pitch 
        deg = deg + revUpPitch;
        delayCounter= 0;
      } else if (delayCounter > timerLoad && loadCurrent < binLoad[6] && deg >= maxPitch-5){ // adjust timer to obtain enough time to stabilize at new load
        loadCurrent = binLoad[6];
        delayCounter= 0;
      } else if (delayCounter > timerLoad && loadCurrent >= binLoad[6]){
        count = 0;
        brakeHistory = 0;
        delayCounter = 0;
        releaseCount = 0;
      }
      delayCounter++;
    } else if (binCount >= 8) {
      loadCurrent = binLoad[1];
      count = 0;
      brakeHistory = 0;
      delayCounter = 0;
      releaseCount = 0;
    } else {
      loadCurrent = 0;
      count = 0;
      brakeHistory = 0;
      delayCounter = 0;
      releaseCount = 0;
    }

  //all of the loops in which E brake condition is not activated
  } else {
    if (binCount == 0) {  //This is 5m/s start up
      if(rpm > 100){
      loadCurrent += binLoad[0]/loadStep;
            delayCounter = 0;
      }
      if(deltaPow < deltaPowStable*powAveNew && powAveNew > powJump[1] && delayCounter > binTimer){
        binCount++;
        delayCounter = 0;
      }
    } else if (binCount == 1) {  // 6m/s pitching
      if(deltaPow <= deltaPowStable*powAveNew && deg < maxPitch){ //check if our small time power average is "stable" 
        loadCurrent = 0;
        // if(loadVoltage > minVoltage && chargeCount < chargeNum){
        //   chargeCount++;
        // }
        if(delayCounter > timerPitch && voltAve > minVoltage){
          deg += revUpPitch;
          delayCounter = 0;
        }
      } else if(capacitorCounter < timerCapacitor && deg>= maxPitch && loadCurrent == 0){
        capacitorCounter++;
        }else if(deltaPow <= deltaPowStable*powAveNew && deg >= maxPitch && capacitorCounter >= timerCapacitor){ //increase load current incrementally until target resistance is met
          if(delayCounter > timerLoad && loadCurrent < 0.99*binLoad[1]){
            loadCurrent += binLoad[1]/loadStep;
            delayCounter = 0;
          }
      }
      delayCounter++;
      // if(deltaPow < deltaPowStable*powAveNew && deg >= maxPitch && loadResistance <= targetResistance){ //catalog the value of steady state power and commanded load in this bin 
      //   binPow[1] = powAveNew;
      //   loadHist[1] = loadCurrent;
      // }
      if(deltaPow <= deltaPowStable*powAveNew && powAveNew > powJump[2] && delayCounter > binTimer){
        binCount++;
        delayCounter = 0;
      }
      
    } else if (binCount > 1 && binCount <= 6) {  // 7-11 m/s
        if(deltaPow < deltaPowStable*powAveNew){
          if(delayCounter > timerLoad && loadCurrent < 0.99*binLoad[binCount]){
            loadCurrent += (binLoad[binCount] - binLoad[binCount-1])/loadStep;
            delayCounter = 0;
          } 
        }
        delayCounter++;
        // if(deltaPow < deltaPowStable*powAveNew && deg >= maxPitch && loadResistance <= targetResistance){ //catalog the value of steady state power and commanded load in this bin 
        // binPow[6] = powAveNew;
        // loadHist[6] = loadCurrent;
        // }
        if(binCount < 6 && deltaPow < deltaPowStable*powAveNew && powAveNew > powJump[binCount+1] && delayCounter > binTimer){
          binCount++;
          delayCounter = 0;
        }
        if(binCount == 6 && deltaPow < deltaPowStable*powAveNew && delayCounter > binTimer && loadCurrent > 0.99*binLoad[6]){
          binPow[6] = powAveNew;
          binCount++;
        }
       
    } else if (binCount == 7) {  // control of rated power and RPM
        if(deltaPow < deltaPowStable*powAveNew && powAveNew > 1.2*binPow[6]){ //incrementally reduce pitch until power is back where it was in bin 6, rpm should follow linearly
          if(delayCounter > timerControlPitch){
          deg -= controlPowPitch;
          delayCounter = 0;
        }
        } else if(deltaPow < deltaPowStable*powAveNew && powAveNew > 1.05*binPow[6]){ //incrementally reduce pitch until power is back where it was in bin 6, rpm should follow linearly
          if(delayCounter > timerControlPitch){
          deg -= controlPowPitchFine;
          delayCounter = 0;
        }
        } else if(deltaPow < deltaPowStable*powAveNew && powAveNew < 0.95*binPow[6] && deg < maxPitch){ //incrementally reduce pitch until power is back where it was in bin 6, rpm should follow linearly
          if(delayCounter > timerControlPitch){
          deg += controlPowPitchFine;
          delayCounter = 0;
        }
        }
        delayCounter++;
        //Need to find a condition to bump to bin 8 durability
        if(powAveNew < 0.5*binPow[6]){
          binCount = 8;
        }
    } else if (binCount == 8) {  // Durability
        if(rpm < 800){
          loadCurrent = 0.01;
        } else{
          loadCurrent = binLoad[8];
        }
        if(voltAve > minVoltage){
          deg = 0; 
        }
    }
  }


  // E - brake application
  //lastButtonState = buttonState;
  // if (brakeCondition == 1) {
  //   digitalWrite(Storage, HIGH);
  //   analogWrite(BrakeControl, brake_off);
  // } else {
  //   digitalWrite(Storage, LOW);
  //   analogWrite(BrakeControl, brake_on);
  // }

  //Print any desired data during testing
  if (serialTimer > 300){
  Serial.print("Bin = ");
  Serial.print(binCount);
  Serial.print(", ");
  Serial.print("Brake = ");
  Serial.print(brakeCondition);
  Serial.print(", ");
  Serial.print("Power = ");
  Serial.print(powAveNew);
  Serial.print(", ");
  Serial.print("RPM = ");
  Serial.print(rpm);
  Serial.print(", ");
  Serial.print("sCurrent = ");
  Serial.print(sensorCurrent);
  Serial.print(", ");
  Serial.print("Current = ");
  Serial.print(loadCurrent);
  Serial.print(", ");
  Serial.print("Pitch = ");
  Serial.print(deg+40);
  Serial.print(", ");
  // Serial.print("pccTimer = ");
  // Serial.print(pccStartTimer);
  // Serial.print(", ");
  Serial.print("sVolts = ");
  Serial.print(loadVoltage);
  Serial.print(", ");
  Serial.print("Volt Ave = ");
  Serial.print(voltAve);
  Serial.print(", ");
  Serial.print("binPow[6] = ");
  Serial.print(binPow[6]);
  Serial.print(", ");
  Serial.print("PCC = ");
  Serial.print(pccTrigger);
  Serial.print(", ");
  // Serial.print("BH = ");
  // Serial.print(brakeHistory);
  // Serial.print(", ");
  // Serial.print("RC = ");
  // Serial.print(releaseCount);
  // Serial.print(", ");
  // Serial.print("chargeCount = ");
  // Serial.print(chargeCount);
  // Serial.print(", ");
  Serial.print("posi = ");
  Serial.print(posi);
  Serial.print(", ");
  Serial.println();
  serialTimer = 0;
  }
  serialTimer++;

  // Calc loop time
  loopEnd = micros();
  loopTime = loopEnd - loopStart;
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  
    b = digitalRead(ENCA);
    //a = digitalRead(ENCB);
    if(b == 0){
      posi--;
    }
    else if(b == 1){
      posi++;
    }
    else {
      //counter++;
    }
  
}

void RPM_Add() {
  peaks++;
}