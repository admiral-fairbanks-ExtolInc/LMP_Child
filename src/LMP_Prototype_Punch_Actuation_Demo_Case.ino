/*
  Started 7/25/17
  LMP Prototype - Code for Low Mass Punch Prototype.
  Last revision on 10/05/2017.
  Prototype code attempts to cover both
  "No punch actuation" and "punch actuation" scenarios.
  Current capabilities: Repeatable cycles with pushbutton-based control
  SD Card Data_Logging
  PID Heater control
*/
#include <FlexiTimer2.h>
#include <Wire.h>
#include <EEPROM.h>
#include "fillWithTemplate.h" // Template function for filling arrays
#include "lmpFunctions.h"

// Begin Variable Definition

// Constant Variables
const uint8_t i2cAddress = 0x05;
const uint8_t heaterPowerPinArray[4] = {6, 9, 10, 11};
const uint8_t heaterIndPinArray[4] = {13, 12, 8, 7};
const uint8_t numHeaters = 1;
const uint8_t pulsePeriod = 49;
const uint8_t statusLED = 5;
const uint16_t maxEnergy = 25600;
const uint16_t meltTempMemAddr = 0;
const uint16_t releaseTempMemAddr = 2;
const uint16_t heaterOnTimeMemAddr = 4;
const uint16_t dwellTimeMemAddr = 6;
const int Safe_Temp = 150, Safe_Hysteresis = 5;

const bool afi = false;
const bool ati = true;
uint16_t sec;
uint16_t j;
uint16_t k;
uint16_t maxFlashCycles = 2;
uint16_t numFaultMessages = 15;
uint16_t releaseHyst = 5;
uint16_t lastTempReading = 0;
uint16_t rtdAmbientAn[numHeaters] = {};
uint16_t flashTime = 1000;
uint16_t cycleCompleteTime;
uint16_t fullStrokeTime[numHeaters] = {};
uint16_t heaterPeakTempTime[numHeaters] = {};
uint16_t heaterRetractedTime[numHeaters] = {};
uint16_t maxOverallCycleTime = 30000;
uint16_t pulloffTime = 2500;
uint16_t meltTempReachedTime[numHeaters] = {};
uint16_t flashCycleReleaseTime;
uint16_t flashCycleStartTime;
uint16_t heaterContactDipTime[numHeaters];
uint16_t heaterReleaseTime;
uint16_t heatersOnTime;
uint16_t pidOutputMax = 255;
uint16_t heaterOnDutyArray[numHeaters] = {};
uint16_t lmpEnergy[numHeaters] = {};
uint16_t releaseTemp;
uint16_t meltTempHyst = 10;
uint16_t heaterHiLim = 1000;
uint16_t openLoopHeaterOnTime;
uint16_t dwellTime;
uint16_t heaterMeltTemp;
uint16_t heaterTemp[numHeaters] = {};

int heaterPeakTemp[numHeaters] = {};
int currentFlashCycle = 1;
int flashCycleHeat;
int flashCycleStep;
int lmpTypes[4] = {0, 0, 0, 0};

unsigned long currentCycleTime;
unsigned long cycleStartTime;
unsigned long currentSnapshot;
unsigned long ledBlinkStart;
unsigned long ledBlinkLength = 1000;

bool All_Safe = true;
bool setpointFlag;
bool setpointCheck = false;
bool resetSystemTimerCntr;
bool resetTimer;
bool atMeltTemp[numHeaters] = {};
bool allAtMeltTemp;
bool coolingAirEnable;
bool cycleActive;
bool cycleComplete;
bool extendPress;
bool faultsActiveArray[15] = {};
bool fltCoolingAirActive;
bool fltsPresent;
bool fltResetSig;
bool heatEnable;
bool lmpActuation = false;
bool heatersRdyForRelease[numHeaters] = {};
bool calibrateRtd;
bool rtdCalibrated;
bool cycleDataLogged;
bool punchFullStrokeSig[numHeaters] = {};
bool processFlags[20] = {};
bool punchExtend[numHeaters] = {};
bool punchesAtFullStroke;
bool allHeatersReadyRelease;
bool systemHomed;
bool flashCycleTargetHeaters[numHeaters] = {};
bool flashSuccess;
bool sendMappingData;
bool inputsFromParent[5];
bool sendDatalog;
bool ledOnOff;
bool cycleStopSignal;
bool pressFullStrokeReached = false;
bool biasCalculated;

double heaterTempAtRelease[numHeaters] = {};
double heaterTempAtShutoff[numHeaters] = {};
double heaterStartTemp[numHeaters];
double heaterContactDipMin[numHeaters];
double cycleCompleteTemp[numHeaters];
double atSetpointTemp[numHeaters];
double biasSoakDuty;
double biasRampTemp;

byte statusByte;
byte startTimeHb;
byte startTimeLb;
byte startTempHb[numHeaters];
byte startTempLb[numHeaters];
byte atSetpointTimeHb[numHeaters];
byte atSetpointTimeLb[numHeaters];
byte atSetpointTempHb[numHeaters];
byte atSetpointTempLb[numHeaters];
byte contactDipTimeHb[numHeaters];
byte contactDipTimeLb[numHeaters];
byte contactDipTempHb[numHeaters];
byte contactDipTempLb[numHeaters];
byte fullStrokeTimeHb[numHeaters];
byte fullStrokeTimeLb[numHeaters];
byte heaterTempAtShutoffHb[numHeaters];
byte heaterTempAtShutoffLb[numHeaters];
byte cycleCompleteTimeHb[numHeaters];
byte cycleCompleteTimeLb[numHeaters];
byte cycleCompleteTempHb[numHeaters];
byte cycleCompleteTempLb[numHeaters];

union ArrayToInteger {
  byte array[2];
 uint16_t integer;
};

// End Variable Definition

// Setup Function
void setup() {
  // Start Timers
  FlexiTimer2::set(50, pidHeaterControl);
  FlexiTimer2::start();
  // End Timers Start

  //fill default RTD ambient analog values
  //equation is Aamb = Afs*Ramb/(Rref+Ramb)
  fillWith(rtdAmbientAn, numHeaters, (uint16_t)301); //default value

  // Start Serial
  Serial.begin(115200);
  Wire.begin(i2cAddress);
  TWAR = (i2cAddress << 1) | 1;  // enable broadcasts to be received
  Wire.onReceive(readData);
  Wire.onRequest(writeData);
  // End of Serial Setup

  if (!lmpActuation) fillWith(punchFullStrokeSig, numHeaters, true); // Force this true if no punch actuation
  for (j = 0; j < 4; j++) {
    pinMode(heaterPowerPinArray[j], OUTPUT);
    pinMode(heaterIndPinArray[j], OUTPUT);
  } // Sets up outputs for Heater Power
  // End of I/O Initialization

  pinMode(statusLED, OUTPUT);
  //pinMode(heaterIndPinArray[1], OUTPUT);
  //digitalWrite(heaterIndPinArray[1], LOW);
  if (maxFlashCycles < 0) {maxFlashCycles = 0;} // Quick check for invalid Max Flash Cycles parameter

  EEPROM.get(meltTempMemAddr, heaterMeltTemp);
  EEPROM.get(releaseTempMemAddr, releaseTemp);
  EEPROM.get(heaterOnTimeMemAddr, openLoopHeaterOnTime);
  EEPROM.get(dwellTimeMemAddr, dwellTime);

  if (heaterMeltTemp == 65535) heaterMeltTemp = 550;
  if (releaseTemp == 65535) releaseTemp = 150;
  if (openLoopHeaterOnTime == 65535) openLoopHeaterOnTime = 30;
  if (dwellTime == 65535) dwellTime = 0;

  Serial.println(heaterMeltTemp);
  Serial.println(releaseTemp);
  Serial.println(openLoopHeaterOnTime);
  Serial.println(dwellTime);


} // End of Setup Loop

// Main Program
void loop() {
  bool startSignal = false;

  systemTimer();

  // Read Inputs
  if (inputsFromParent[0]) startSignal = true;
  else if (!inputsFromParent[0]) startSignal = false;
  if (inputsFromParent[1]) cycleStopSignal = true;
  else if (!inputsFromParent[1]) cycleStopSignal = false;
  if (inputsFromParent[2]) pressFullStrokeReached = true;
  else if (!inputsFromParent[2]) pressFullStrokeReached = false;
  if (inputsFromParent[3]) sendDatalog = true;
  else if (!inputsFromParent[3]) sendDatalog = false;
  if (inputsFromParent[4] && !calibrateRtd) {
    calibrateRtd = true;
    rtdCalibrated = false;
  }
  else if (!inputsFromParent[4] && calibrateRtd) {
    calibrateRtd = false;
  }

  // Reads heater temps and monitors punch full-stroke status

  punchesAtFullStroke = true;
  for (j = 0; j < numHeaters; j++) {
    heaterTemp[j] = rtdTempConversion(analogRead(14 + j), rtdAmbientAn[j]);

    if (heaterTemp[j] > heaterPeakTemp[j]) { // Tracks peak temp and timestamp for each heater
      heaterPeakTemp[j] = heaterTemp[j];
      heaterPeakTempTime[j] = currentCycleTime;
    }
  }

  // End Read Inputs

  // Before entering into loop, check program time. If total cycle time exceeds max allowable, abort cycle and flag fault
  if (heatEnable && (currentCycleTime > maxOverallCycleTime) && cycleActive) {
    fltsPresent = true;
    faultsActiveArray[13] = true;
    cycleActive = false;
    Serial.println("Cycle too long");
    Serial.println(currentCycleTime - cycleStartTime);
  }

  // Cool Modules
  lmpCoolingAir();

  // Fault Handling
  faultHandling(faultsActiveArray);
  // End Fault Handling

  //Calibrate RTD
  if (calibrateRtd && !rtdCalibrated) {
    for (j = 0; j < numHeaters; j++) {
      rtdAmbientAn[j] = analogRead(14 + j);
    }
    Serial.println("RTD Calibrated");
    rtdCalibrated = true;
  }

  /*
  * If the start signal is given, Cycle is not active, and there are no active faults,
  * Turn cycleActive and heatEnable on, begin cycle.
  */
  if (startSignal && !cycleActive && !fltsPresent) {
    // Cycle start also clears out any leftover bits from the previous cycle
    cycleComplete = false;
    cycleDataLogged = false;
    systemHomed = false;
    flashSuccess = false;
    cycleActive = true; // Starts cycle
    heatEnable = true; // Starts heat
    Serial.print("Cycle Start");
  }

  if (heatEnable && !biasCalculated) {
    biasRampTemp = pow((-0.0004167 * (float)heaterMeltTemp), 2) + 1.636*heaterMeltTemp - 123;
    biasSoakDuty = pow((float)heaterMeltTemp, 2)/15923567 + heaterMeltTemp/13850;
    biasCalculated = true;
  }

  /*
  * If Heat is enabled and cycle is active, check if each heater has exceeded
  * the high limit. If it has, flag the appropriate fault, disable heat, end cycle.
  * Additionally, check to see if the total time any heaters have been on exceeds max heater on time.
  * If heatersOnTime exceeds this value, disable heat, flag fault, end cycle.
  * Flag heater start time and monitor for when heaters reach setpoint.
  */
  if (heatEnable) {
    if (!processFlags[1]) {
      heatersOnTime = currentCycleTime; // Captures cycle time at which at least one heater first turns on.
      for (j = 0; j < numHeaters; j++) heaterStartTemp[j] = heaterTemp[j];
      resetSystemTimerCntr = true; // Reset cycle timer
      processFlags[1] = true; // Functions as a one-shot to ensure that Heaters_On-Time is only captured once.
    }
    for (k = 0; k < numHeaters; k++) {
      // When a heater reaches setpoint, flag it and capture time
      if (!fltsPresent && (heaterTemp[k] > heaterMeltTemp) && !atMeltTemp[k]) {
        atMeltTemp[k] = true;
        setpointFlag = true;
        atSetpointTemp[k] = heaterTemp[k];
        meltTempReachedTime[k] = (uint16_t)currentCycleTime; // Captures the first cycle time at which all heaters are within setpoint window.
        if(setpointFlag && k == numHeaters-1) allAtMeltTemp = true;
      }
      else setpointFlag = false;
      if (!atMeltTemp[k]) allAtMeltTemp = false;
    }
    // When all heaters are at setpoint, extend press.
    if (allAtMeltTemp && !processFlags[2]) {
      extendPress = true;
      processFlags[2] = true;
      Serial.print("At Setpoint");
    }
  }
  if (allAtMeltTemp && heatEnable) {
    for (j = 0; j < numHeaters; j++) {
      if (heaterTemp[j] < heaterContactDipMin[j]) {
        heaterContactDipMin[j] = heaterTemp[j];
        heaterContactDipTime[j] = (uint16_t)currentCycleTime;
      }
    }
  }

  // For punch actuation, extend punches when at setpoint. Punch retraction handled later.
  if (lmpActuation) {
    for (j = 0; j < numHeaters; j++) {
      if (atMeltTemp[j] && pressFullStrokeReached) punchExtend[j] = true;
    }
  }

  /*
   When we receive the full stroke signal from the press, check each heater to see if at
   setpoint and if punch is at full stroke. If it is, flag time and heater temp and shut off its heat.
 */
  for (j = 0; j < numHeaters; j++) {
    if ((pressFullStrokeReached && punchFullStrokeSig[j]) ||
    currentCycleTime > openLoopHeaterOnTime) {
      if (atMeltTemp[j] &&
      heatEnable && !fltsPresent && cycleActive && !processFlags[3 + j]) {
        fullStrokeTime[j] = (uint16_t)currentCycleTime;
        heaterTempAtShutoff[j] = heaterTemp[j];
        atMeltTemp[j] = false;
        processFlags[3 + j] = true;
        Serial.print("At Full Stroke");
    }
}
  // If all heaters are at full stroke (press/punch) disable heat and turn on cooling air.
    if (((pressFullStrokeReached && punchFullStrokeSig[j]) ||
    currentCycleTime > openLoopHeaterOnTime) &&
    (currentCycleTime - fullStrokeTime[j] >= dwellTime) && !processFlags[7]) {
      if ((!fltsPresent && cycleActive) || (fltCoolingAirActive)) {
        Serial.print(pressFullStrokeReached);
        heatEnable = false;
        processFlags[7] = true;
        Serial.print("Cooling Air On");
      }
    }
  }

  /*
  * When cooling air is enabled, heaters are off, and full stroke is reached, compare each heater"s
  * temperature vs. the release temp. When each heater reaches release temp, flag it as ready for release.
  * When all heaters are ready for release, if no actuation, tell press to retract.
  * If punch actuation, instead tell all punches to retract.
  */
  if (((processFlags[7] && !fltsPresent && cycleActive) || (fltsPresent && !cycleActive && fltCoolingAirActive))
  && !heatEnable && !processFlags[8]) {
    allHeatersReadyRelease = true;
    for (j = 0; j < numHeaters; j++) {
      if (heaterTemp[j] > releaseTemp) {
        allHeatersReadyRelease = false;
      }
    }
    if (allHeatersReadyRelease) {
      heaterReleaseTime = (uint16_t)currentCycleTime;
      if (!lmpActuation) {extendPress = false;}
      if (lmpActuation) {fillWith(punchExtend, numHeaters, false);}
      processFlags[8] = true;
      Serial.print("At Release");
    }
  }

  // Check to see if punch/press has released correctly.
  if (allHeatersReadyRelease && (currentCycleTime - heaterReleaseTime < pulloffTime)) {
    for (j = 0; j < numHeaters; j++) { // This loop checks for valid heater release. When valid release is obtained, flag time and heater temp.
      if (((lmpActuation && !punchFullStrokeSig[j]) ||
      (!lmpActuation && !pressFullStrokeReached)) && !processFlags[9 + j]) {
        heaterTempAtRelease[j] = heaterTemp[j];
        heaterRetractedTime[j] = (uint16_t)currentCycleTime;
        processFlags[9 + j] = true; // Functions as one-shot to make sure this part of the process is only done once per cycle.
      }
    }
    // If we got a valid release from all heaters, cycle is complete. Flag time and send signal to machine plc
    if (((lmpActuation && !punchesAtFullStroke) || (!lmpActuation && !pressFullStrokeReached)) && !processFlags[13]) {
      cycleCompleteTime = (uint16_t)currentCycleTime;
      for (j = 0; j < numHeaters; j++) {cycleCompleteTemp[j] = heaterTemp[j];}
      cycleComplete = true;
      cycleActive = false;
      extendPress = false;
      processFlags[13] = true;
      Serial.print("Cycle Complete");
    }
  }

  // If we do not obtain a valid release, enter flash cycle
  else if (allHeatersReadyRelease && (currentCycleTime - heaterReleaseTime >= pulloffTime) && cycleActive && maxFlashCycles != 0) {
    flashCycle();
  }
  // If max flash cycles are 0, flash cycle is disabled. If we do not get valid release on first attempt, flag fault and exit cycle.
  else if (allHeatersReadyRelease && ((currentCycleTime - heaterReleaseTime) >= pulloffTime) && !processFlags[13] && maxFlashCycles == 0) {
    fltsPresent = true;  // punches/press, flag fault and abort cycle.
    faultsActiveArray[14] = true;
    cycleActive = false;
    Serial.print("flash cycle failure");
  }

  // Cycle Stop logic
  if (cycleStopSignal) {
    cycleActive = false;
  }

  // Data Logging only happens at end of loop if Cycle is complete or if a fault was flagged on this scan.
  if (!cycleActive && !cycleDataLogged && (cycleComplete || fltsPresent || cycleStopSignal)) {
    logData();
    if (!systemHomed) homeSystem();
  }
  // Output Handling

  for (j = 0; j < numHeaters; j++) {
    analogWrite(heaterPowerPinArray[j], (int)heaterOnDutyArray[j]);
    digitalWrite(heaterIndPinArray[j], heatEnable);
  }

  if (!cycleActive && !fltsPresent) digitalWrite(statusLED, LOW);
  else if (cycleActive && !fltsPresent) digitalWrite(statusLED, HIGH);
  else if (fltsPresent) {
    if (!ledOnOff) {
      ledBlinkStart = currentCycleTime;
      ledOnOff = true;
      digitalWrite(statusLED, HIGH);
    }
    else if (ledOnOff && currentCycleTime - ledBlinkStart > ledBlinkLength) {
      ledOnOff = false;
      digitalWrite(statusLED, LOW);
    }
  }
}
// End of main loop

void systemTimer() {
  if(cycleActive){
    if(resetSystemTimerCntr){
      cycleStartTime = millis();
      resetSystemTimerCntr = false;
    }
    currentSnapshot = millis();
    currentCycleTime = currentSnapshot - cycleStartTime;
  }
  else{
    currentSnapshot = 0;
    cycleStartTime = currentSnapshot;
  }
}

void readData() {
  int l = Wire.available();
  int index;
  char receivedBuffer[l + 1];
  byte sentBuffer[4];
  ArrayToInteger meltTempUnion;
  ArrayToInteger releaseTempUnion;
  ArrayToInteger maxOnTimeUnion;
  ArrayToInteger dwellTimeUnion;
  while (Wire.available()) {
    receivedBuffer[index] = Wire.read();
    ++index;
  }
  if (!Wire.available()) { index = 0; }
  if (l == 1) {
    sendMappingData = true;
  }
  else if (l == 5) {
    for (j = 0; j < l; j++){
      inputsFromParent[j] = (bool)receivedBuffer[j];
    }
  }
  else if (l == 13) {
    for (j = 0; j < 4; j++){
      inputsFromParent[j] = (bool)receivedBuffer[j];
    }
    meltTempUnion.array[1] = receivedBuffer[5];
    meltTempUnion.array[0] = receivedBuffer[6];
    releaseTempUnion.array[1] = receivedBuffer[7];
    releaseTempUnion.array[0] = receivedBuffer[8];
    maxOnTimeUnion.array[1] = receivedBuffer[9];
    maxOnTimeUnion.array[0] = receivedBuffer[10];
    dwellTimeUnion.array[1] = receivedBuffer[11];
    dwellTimeUnion.array[0] = receivedBuffer[12];
    if (heaterMeltTemp != meltTempUnion.integer) {
      heaterMeltTemp = meltTempUnion.integer;
      EEPROM.put(meltTempMemAddr, heaterMeltTemp);
      Serial.print("Melt Temp Updated: ");
      Serial.print(heaterMeltTemp);
      Serial.print(" deg F");
    }
    if (releaseTemp != releaseTempUnion.integer) {
      releaseTemp = releaseTempUnion.integer;
      EEPROM.put(releaseTempMemAddr, releaseTemp);
      Serial.print("Release Temp Updated: ");
      Serial.print(releaseTemp);
      Serial.print(" deg F");
    }
    if (openLoopHeaterOnTime != maxOnTimeUnion.integer) {
      openLoopHeaterOnTime = maxOnTimeUnion.integer*100;
      EEPROM.put(heaterOnTimeMemAddr, openLoopHeaterOnTime);
      Serial.print("Max Heater On Time Updated: ");
      Serial.print(openLoopHeaterOnTime);
      Serial.print(" msec");
    }
    if (dwellTime != dwellTimeUnion.integer) {
      dwellTime = dwellTimeUnion.integer*100;
      EEPROM.put(dwellTimeMemAddr, dwellTime);
      Serial.print("Dwell Time Updated: ");
      Serial.print(dwellTime);
      Serial.print(" msec");
    }
  }
  else {
    //fltsPresent = true;
    //Serial.print(l);
  }
}

void writeData() {
  const bool afi = false;
  bool statusArray[8] = {cycleActive, allAtMeltTemp, coolingAirEnable,
    allHeatersReadyRelease, cycleComplete, fltsPresent, cycleDataLogged, afi};
  if (sendMappingData) {
    byte sentBuffer[4];
    for (j = 0; j < 4; j++) sentBuffer[j] = (byte)lmpTypes[j];
    sendMappingData = false;
    Wire.write(sentBuffer, 4);
  }
  else {
    if (!sendDatalog){
      byte sentBuffer[1 + 2*numHeaters];
      sentBuffer[0] = boolArrayToByte(statusArray);
      for (j = 0; j < numHeaters; j++) {
        sentBuffer[1+2*j] = highByte((int16_t)heaterTemp[j]*10);
        sentBuffer[2+2*j] = lowByte((int16_t)heaterTemp[j]*10);
      }
      Wire.write(sentBuffer, (1 + 2*numHeaters));
    }
    else {
      byte sentBuffer[1 + 32*numHeaters];
      sentBuffer[0] = boolArrayToByte(statusArray);
      for (j = 0; j < numHeaters; j++) {
        sentBuffer[1+2*j] = highByte((int16_t)heaterTemp[j]*10);
        sentBuffer[2+2*j] = lowByte((int16_t)heaterTemp[j]*10);
        sentBuffer[3+20*j] = startTimeHb;
        sentBuffer[4+20*j] = startTimeLb;
        sentBuffer[5+20*j] = startTempHb[j];
        sentBuffer[6+20*j] = startTempLb[j];
        sentBuffer[7+20*j] = atSetpointTimeHb[j];
        sentBuffer[8+20*j] = atSetpointTimeLb[j];
        sentBuffer[9+20*j] = atSetpointTempHb[j];
        sentBuffer[10+20*j] = atSetpointTempLb[j];
        sentBuffer[11+20*j] = contactDipTimeHb[j];
        sentBuffer[12+20*j] = contactDipTimeLb[j];
        sentBuffer[13+20*j] = contactDipTempHb[j];
        sentBuffer[14+20*j] = contactDipTempLb[j];
        sentBuffer[15+20*j] = fullStrokeTimeHb[j];
        sentBuffer[16+20*j] = fullStrokeTimeLb[j];
        sentBuffer[17+20*j] = cycleCompleteTempHb[j];
        sentBuffer[18+20*j] = cycleCompleteTempLb[j];
        sentBuffer[19+20*j] = cycleCompleteTimeHb[j];
        sentBuffer[20+20*j] = cycleCompleteTimeLb[j];
        sentBuffer[21+20*j] = cycleCompleteTempHb[j];
        sentBuffer[22+20*j] = cycleCompleteTempLb[j];
      }
      Wire.write(sentBuffer, (1 + 22*numHeaters));
    }
  }
}

// Convert Bool array to Byte
byte boolArrayToByte(bool boolArray[8]){
  byte result = 0;
  if (boolArray[0]) result = result + 1;
  if (boolArray[1]) result = result + 2;
  if (boolArray[2]) result = result + 4;
  if (boolArray[3]) result = result + 8;
  if (boolArray[4]) result = result + 16;
  if (boolArray[5]) result = result + 32;
  if (boolArray[6]) result = result + 64;
  if (boolArray[7]) result = result + 128;
  return result;
}

// Fault Handling function that reads active faults and sends out the appropriate fault M_2_S_Message.
void faultHandling(bool faultsArray[]) {
  if (cycleStopSignal) { // Resets faults
    fillWith(faultsArray, 15, false);
    fltsPresent = false;
  } // End of fault reset
  if (fltsPresent) { // If faults are present, Heat is disabled, Cycle is aborted.
    heatEnable = false;
    cycleActive = false;
    fillWith(processFlags, 20, false);
    fillWith(atMeltTemp, numHeaters, false);
    fillWith(heatersRdyForRelease, numHeaters, false);
  }
} // End of faultHandling

// This turns on cooling air whenever the LMP is above a safe temperature
void lmpCoolingAir() {
  for (j = 0; j < numHeaters; j++) {
    if (!heatEnable && (heaterTemp[j] > (Safe_Temp + Safe_Hysteresis) || heaterTemp[j] > (releaseTemp + releaseHyst))) {
      coolingAirEnable = true;
    }
    if (heaterTemp[j] > (Safe_Temp - Safe_Hysteresis) || heaterTemp[j] > (releaseTemp - releaseHyst)) All_Safe = false;
  }
  if (heatEnable || All_Safe) coolingAirEnable = false;
} // End lmpCoolingAir

/*
 * Function for control of Heater Duty Cycles. Duty cycles are 250 msec. PID Output varies in 10 msec increments from 0-250 msec.
 * Time is tracked by dutyMsec from 0-250 msec, resetting when it exceeds this value. The PID produces an output value in msec.
 * The function compares the current time (as represented by dutyMsec) in the duty cycle to the PID output value. If the current time
 * is LESS than the PID output, Heaters are turned on and kept on. If MORE than the PID output, Heaters are turned off.
 */
void pidHeaterControl() {

  if (flashCycleHeat == 0) {
    if (heatEnable && !fltsPresent && cycleActive) { // Only performs PID calculation if heat is enabled and no faults are present.
      for (j = 0; j < numHeaters; j++) {
        if (heaterTemp[j] < biasRampTemp) heaterOnDutyArray[j] = 255;
        else {
          heaterOnDutyArray[j] = ((heaterMeltTemp - heaterTemp[j])*.005 + biasSoakDuty) * 256;
          if (heaterOnDutyArray[j] > 255) heaterOnDutyArray[j] = 255;
          else if (heaterOnDutyArray[j] < 0) heaterOnDutyArray[j] = 0;
        }
        lmpEnergy[j] = heaterOnDutyArray[j] + lmpEnergy[j];
        if (lmpEnergy[j] >= maxEnergy) {
          fltsPresent = true;
          heatEnable = false;
          cycleActive = false;
          Serial.println("exceeded max allowable energy");
        }
      }
    }
  }

  else if (flashCycleHeat == 1) {
    for (j = 0; j < numHeaters; j++) {
      if (flashCycleTargetHeaters[j]) heaterOnDutyArray[j] = pidOutputMax;
      else {
        heaterOnDutyArray[j] = 0.0;
        heatEnable = false;
      }
    }
  }
  else {
    for (j = 0; j < numHeaters; j++) {
      heaterOnDutyArray[j] = 0;
    }
    heatEnable = false;
  }

  if (heatEnable) {
    for (k = 0; k < numHeaters; k++) {
      if (heaterTemp[k] > heaterHiLim) { // If any heater exceeds hi-lim, exit cycle and trigger fault.
        heatEnable = false;
        fltsPresent = true;
        faultsActiveArray[numHeaters + k] = true;
        cycleActive = false;
        Serial.print("too hot");
      }
    }
    // When all heaters are at setpoint, extend press.
    if (allAtMeltTemp && !processFlags[2]) {
      extendPress = true;
      processFlags[2] = true;
      Serial.print("At Setpoint");
    }
  }
  if (allAtMeltTemp && heatEnable) {
    for (j = 0; j < numHeaters; j++) {
      if (heaterTemp[j] < heaterContactDipMin[j]) {
        heaterContactDipMin[j] = heaterTemp[j];
        heaterContactDipTime[j] = (uint16_t)currentCycleTime;
      }
    }
  }
} // End of pidHeaterControl

/*
 * The following function is for the Flash Cycle. It is only entered into if the press/punch fail to release from the
 * welded plastic. There is a pre-defined number of flash cycles the system will cycle through before entering into a faulted state.
 * Procedure is as follows: First, enable all heaters and extend punches/press (depending on type of LMP Unit) for one second,
 * then disable heat and attempt to retract punches or press. After a set delay, if the punches/press fail to release, increment the cycle
 * counter and repeat. If the flash cycle is successful in allowing the punches/press to retract, enter into a normal "cycle complete"
 * state and exit Flash Cycle. If the maximum number of flash cycles is exceeded without successfully detaching punches/press, a fault
 * is flagged and the cycle is aborted.
 */
void flashCycle() {
  if (currentFlashCycle <= maxFlashCycles) { // Current flash cycle has to be within the limit
    if (flashCycleStep == 0) { // First step
      flashCycleStartTime = currentCycleTime; // Flag start time of flash heating
      flashCycleHeat = 1; // flashCycleHeat = 1 means heat is forced on
      flashCycleStep = 1; // Move to second step
      if (!lmpActuation) { // If no punch actuation, turn on all heaters and extend press
        fillWith(flashCycleTargetHeaters, numHeaters, true);
        extendPress = true; // Extend press if no punch actuation
      }
      else if (lmpActuation) { // If punch actuation, turn only stuck heaters on and extend punches
        for (j = 0; j < numHeaters; j++) {
          if (punchFullStrokeSig[j]) {
            flashCycleTargetHeaters[j] = true;// Turn on heater
            punchExtend[j] = true; // Extend punch if punch actuation
          }
        }
      }
    }
    // If second step is active and flash time has been exceeded, proceed.
    else if (flashCycleStep == 1 && (currentCycleTime >= flashCycleStartTime + flashTime)) {
      flashCycleHeat = 2; // flashCycleHeat = 2 means heat is forced off
      fillWith(flashCycleTargetHeaters, numHeaters, false); // Turn off heater
      fillWith(punchExtend, numHeaters, false); // Retract punch if punch actuation
      if (!lmpActuation) {
        extendPress = false; // Retract press if no punch actuation
      }
      flashCycleReleaseTime = (uint16_t)currentCycleTime; // Flag start time of flash release
      flashCycleStep = 2; // Move to third step
    }
    // If third step is active and pulloff time has not been exceeded, check for
    // successful release.
    else if (flashCycleStep == 2 && !flashSuccess && ((uint16_t)currentCycleTime <= flashCycleReleaseTime + pulloffTime)) {
      if (lmpActuation) {
        flashSuccess = true;
        for (j = 0; j < numHeaters; j++) {
          if (punchFullStrokeSig[j]) flashSuccess = false;
        }
      }
      else if (!lmpActuation && !pressFullStrokeReached) flashSuccess = true;
    }
    // If third step is active and pulloff time has been exceeded and flash cycle
    // was successfully completed, cycle is complete, end cycle.
    else if (flashCycleStep == 2 && !cycleComplete && ((uint16_t)currentCycleTime > flashCycleReleaseTime + pulloffTime)) {
      if (flashSuccess) {
        flashCycleStep = 0;
        currentFlashCycle = 1;
        cycleActive = false;
        cycleCompleteTime = (uint16_t)currentCycleTime;
        cycleComplete = true;
        extendPress = false;
        fillWith(heatersRdyForRelease, numHeaters, false);
        allHeatersReadyRelease = false;
        processFlags[13] = true;
        flashCycleHeat = 0;
      }
      // If third step is active and pulloff time has been exceeded and flash cycles
      // was unsuccessfully completed, move to next flash cycle.
      else currentFlashCycle++;
    }
  }
  else if (maxFlashCycles != 0 && currentFlashCycle > maxFlashCycles) { // If max number of cycles is exceeded without successfullly removing
    fltsPresent = true;  // punches/press, flag fault and abort cycle.
    Serial.print("max flash cycles exceeded");
    faultsActiveArray[14] = true;
    cycleActive = false;
    currentFlashCycle = 1;
    flashCycleStep = 0;
    flashCycleHeat = 0;
  }
} // End of flashCycle

void homeSystem() {
  // Re-Initialize system for next cycle
  fillWith(punchExtend, numHeaters, false);
  fillWith(atMeltTemp, numHeaters, false);
  fillWith(processFlags, 20, false);
  fillWith(heatersRdyForRelease, numHeaters, false);
  fillWith(flashCycleTargetHeaters, numHeaters, false);
  fillWith(heaterContactDipMin, numHeaters, 0.0);
  for (j = 0; j < numHeaters; j++) {
    heaterContactDipTime[j] = 0;
  }
  allHeatersReadyRelease = false;
  extendPress = false;
  setpointFlag = true;
  //coolingAirEnable = false;
  currentFlashCycle = 1;
  flashCycleHeat = 0;
  flashCycleStep = 0;
  flashCycleStartTime = 0;
  flashCycleReleaseTime = 0;
  heatersOnTime = 0;
  heaterReleaseTime = 0;
  biasCalculated = false;
  biasRampTemp = 0;
  biasSoakDuty = 0;
  systemHomed = true;
} // End of homeSystem

// Logs Cycle data at the end of each cycle to the in-board SD card.
void logData() {
  for (j = 0; j < numHeaters; j++) {
    startTimeHb = 0;
    startTimeLb = 0;
    startTempHb[j] = highByte((int16_t)(heaterStartTemp[j]*10));
    startTempLb[j] = lowByte((int16_t)(heaterStartTemp[j]*10));
    atSetpointTimeHb[j] = highByte(meltTempReachedTime[j]);
    atSetpointTimeLb[j] = lowByte(meltTempReachedTime[j]);
    atSetpointTempHb[j] = highByte((int16_t)(atSetpointTemp[j]*10));
    atSetpointTempLb[j] = lowByte((int16_t)(atSetpointTemp[j]*10));
    contactDipTimeHb[j] = highByte(heaterContactDipTime[j]);
    contactDipTimeLb[j] = lowByte(heaterContactDipTime[j]);
    contactDipTempHb[j] = highByte((int16_t)(heaterContactDipMin[j]*100));
    contactDipTempLb[j] = lowByte((int16_t)(heaterContactDipMin[j]*100));
    fullStrokeTimeHb[j] = highByte(fullStrokeTime[j]);
    fullStrokeTimeLb[j] = lowByte(fullStrokeTime[j]);
    heaterTempAtShutoffHb[j] = highByte((int16_t)(heaterTempAtShutoff[j]*10));
    heaterTempAtShutoffLb[j] = lowByte((int16_t)(heaterTempAtShutoff[j]*10));
    cycleCompleteTimeHb[j] = highByte(cycleCompleteTime);
    cycleCompleteTimeLb[j] = lowByte(cycleCompleteTime);
    cycleCompleteTempHb[j] = highByte((int16_t)(cycleCompleteTemp[j]*10));
    cycleCompleteTempLb[j] = lowByte((int16_t)(cycleCompleteTemp[j]*10));
  }
  cycleDataLogged = true;
} // End of logData
