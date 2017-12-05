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
#include <TimerOne.h>
// #include <utility/Adafruit_MCP23017.h>
#include <Wire.h>
#include "fillWithTemplate.h" // Template function for filling arrays
#include <PID_v1.h> // PID controller
#include "lmpFunctions.h"
// #include "LMP_Class.h"
// #include "LMP_Class_Core.cpp"

// Begin Variable Definition

// Constant Variables
const uint8_t i2cAddress = 0x05;
const uint8_t heaterPowerPinArray[4] = {10, 11, 12, 13};
const uint8_t numHeaters = 1;
const uint8_t punchFullStrokePinArray[4] = {6, 7, 8, 9};
const uint8_t pulsePeriod = 49;
const uint8_t punchExtendPin[4] = {2, 3, 4, 5};
const uint8_t systemTimebaseUs = 5000;

const bool afi = false;
const bool ati = true;
volatile bool beginRead;
volatile bool beginWrite;

// Global Variables

volatile uint16_t dutyPulseValue = 0;

uint16_t dutyMsec;
uint16_t sec;
uint16_t j;
uint16_t k;
uint16_t maxFlashCycles = 2;
uint16_t numFaultMessages = 15;
uint16_t heaterHiLim = 600;
uint16_t releaseHyst = 5;
uint16_t lastTempReading = 0;
uint16_t releaseTemp = 125;
uint16_t meltTempHyst = 10;
uint16_t heaterRtdAnalogSig[numHeaters] = {};
uint16_t rtdAmbientAn[numHeaters] = {};

int heaterPeakTemp[numHeaters] = {};
int currentFlashCycle = 1;
int flashCycleHeat;
int flashCycleStep;
int lmpTypes[4];

unsigned long heartbeatPulseCounter;
unsigned long currentSnapshot;
unsigned long lastSnapshot;

volatile bool resetSystemTimerCntr;

bool resetTimer;
bool atMeltTemp[numHeaters] = {};
bool allAtMeltTemp;
bool coolingAirEnable;
bool cycleActive;
bool cycleComplete;
bool cycleStopSignal;
bool extendPress = true;
bool faultsActiveArray[15] = {};
bool fltCoolingAirActive;
bool fltsPresent;
bool fltResetSig;
bool heatEnable;
bool lmpActuation;
bool heatersRdyForRelease[numHeaters] = {};
bool calibrateRtd;
bool cycleDataLogged;
bool msecOneshot;
bool pressFullStrokeReached;
bool punchFullStrokeSig[numHeaters] = {};
bool processFlags[20] = {};
bool punchExtend[numHeaters] = {};
bool startSignal;
bool punchesAtFullStroke;
bool allHeatersReadyRelease;
bool systemHomed;
bool pidInitialized;
bool flashCycleTargetHeaters[numHeaters] = {};
bool flashSuccess;
bool setpointIncreaseFlag;
bool setpointDecreaseFlag;
bool setpointIncreased;
bool setpointDecreased;
bool sendMappingData;
bool inputsFromParent[5];
bool statusArray[8] = {false, false, cycleActive, allAtMeltTemp,
  allHeatersReadyRelease, fltsPresent, cycleDataLogged};
bool sendTempData;
bool sendDatalog;

double currentCycleTime;
double cycleCompleteTime;
double flashTime = 1.0;
double fullStrokeTime[numHeaters] = {};
double heatersOnTime;
double heaterPeakTempTime[numHeaters] = {};
double heaterRetractedTime[numHeaters] = {};
double heaterRtdReading[numHeaters] = {};
double heaterReleaseTime;
double heaterMeltTemp = 500;
double heaterTempAtRelease[numHeaters] = {};
double heaterTempAtShutoff[numHeaters] = {};
double heaterTemp[numHeaters] = {};
double maxHeaterOnTime = 20;
double maxOverallCycleTime = 30;
double puloffTime = 2.0;
double meltTempReachedTime[numHeaters] = {};
double lastLcdRefreshTime;
double heaterOnDutyArray[numHeaters] = {};
double pidOutputMax = 50;
double pidSampleTime = 50;
double flashCycleReleaseTime;
double flashCycleStartTime;
double heaterStartTemp[numHeaters];
double heaterContactDipMin[numHeaters];
double heaterContactDipTime[numHeaters];
double cycleCompleteTemp[numHeaters];
double atSetpointTemp[numHeaters];

const char *cycleCodes[3]; // Initialized as "CI," or Cycle Inactive

byte statusByte;
byte startTimeHb;
byte startTimeLb;
byte startTempHb[numHeaters];
byte startTempLb[numHeaters];
byte startPosHb[numHeaters];
byte startPosLb[numHeaters];
byte atSetpointTimeHb[numHeaters];
byte atSetpointTimeLb[numHeaters];
byte atSetpointTempHb[numHeaters];
byte atSetpointTempLb[numHeaters];
byte atSetpointPosHb[numHeaters];
byte atSetpointPosLb[numHeaters];
byte contactDipTimeHb[numHeaters];
byte contactDipTimeLb[numHeaters];
byte contactDipTempHb[numHeaters];
byte contactDipTempLb[numHeaters];
byte contactDipPosHb[numHeaters];
byte contactDipPosLb[numHeaters];
byte fullStrokeTimeHb[numHeaters];
byte fullStrokeTimeLb[numHeaters];
byte heaterTempAtShutoffHb[numHeaters];
byte heaterTempAtShutoffLb[numHeaters];
byte heaterPosAtShutoffHb[numHeaters];
byte heaterPosAtShutoffLb[numHeaters];
byte cycleCompleteTimeHb[numHeaters];
byte cycleCompleteTimeLb[numHeaters];
byte cycleCompleteTempHb[numHeaters];
byte cycleCompleteTempLb[numHeaters];
byte cycleCompletePosHb[numHeaters];
byte cycleCompletePosLb[numHeaters];

// Define PID Controllers
PID heater1PID (&heaterTemp[0], &heaterOnDutyArray[0], &heaterMeltTemp, 15, 3, 0, DIRECT);
PID heater2PID (&heaterTemp[1], &heaterOnDutyArray[1], &heaterMeltTemp, 15, 3, 0, DIRECT);
PID heater3PID (&heaterTemp[2], &heaterOnDutyArray[2], &heaterMeltTemp, 15, 3, 0, DIRECT);
PID heater4PID (&heaterTemp[3], &heaterOnDutyArray[3], &heaterMeltTemp, 15, 3, 0, DIRECT);

// End Variable Definition

// Setup Function
void setup() {
  // Start Timers
  Timer1.initialize(500);
  Timer1.attachInterrupt(Heater_Duty); // Run Heater Duty interrupt every msec

  FlexiTimer2::set(50, PID_Heater_Control);
  FlexiTimer2::start();
  // End Timers Start

  //fill default RTD ambient analog values
  //equation is Aamb = Afs*Ramb/(Rref+Ramb)
  Fill_With(rtdAmbientAn, numHeaters, (uint16_t)488); //default value

  // Start Serial
  Serial.begin(115200);
  Wire.begin(i2cAddress);
  TWAR = (i2cAddress << 1) | 1;  // enable broadcasts to be received
  Wire.onReceive(readData);
  Wire.onRequest(writeData);
  // End of Serial Setup

  // End Setup of Ethernet and Web Server

  if (lmpActuation) {
    // If punch actuation, enable this I/O
    for (j = 0; j < numHeaters; j++) {
      pinMode(punchFullStrokePinArray[j], INPUT);
      pinMode(punchExtendPin[j], OUTPUT);
    }
  }

  if (!lmpActuation) Fill_With(punchFullStrokeSig, numHeaters, true); // Force this true if no punch actuation
  for (j = 0; j < numHeaters; j++) {pinMode(heaterPowerPinArray[j], OUTPUT);} // Sets up outputs for Heater Power
  // End of I/O Initialization

  if (maxFlashCycles < 0) {maxFlashCycles = 0;} // Quick check for invalid Max Flash Cycles parameter

  // PID Setup
  heater1PID.SetOutputLimits(0, pidOutputMax), heater1PID.SetMode(AUTOMATIC), heater1PID.SetSampleTime(pidSampleTime);
  heater2PID.SetOutputLimits(0, pidOutputMax), heater2PID.SetMode(AUTOMATIC), heater2PID.SetSampleTime(pidSampleTime);
  heater3PID.SetOutputLimits(0, pidOutputMax), heater3PID.SetMode(AUTOMATIC), heater3PID.SetSampleTime(pidSampleTime);
  heater4PID.SetOutputLimits(0, pidOutputMax), heater4PID.SetMode(AUTOMATIC), heater4PID.SetSampleTime(pidSampleTime);
  // End of PID Setup
} // End of Setup Loop

// Main Program
void loop() {
  // Update system time
  systemTimer();

  // Read Inputs
  inputsFromParent[0] = startSignal;
  inputsFromParent[1] = cycleStopSignal;
  inputsFromParent[2] = pressFullStrokeReached;
  inputsFromParent[3] = sendDatalog;

  // Reads heater temps and monitors punch full-stroke status
  punchesAtFullStroke = true;
  for (j = 0; j < numHeaters; j++) {
    heaterRtdAnalogSig[j] = analogRead(54 + j);
    heaterTemp[j] = RTD_Temp_Conversion(heaterRtdAnalogSig[j], rtdAmbientAn[j]);

    if (heaterTemp[j] > heaterPeakTemp[j]) { // Tracks peak temp and timestamp for each heater
      heaterPeakTemp[j] = heaterTemp[j];
      heaterPeakTempTime[j] = currentCycleTime;
    }

    if (lmpActuation) { // Tracks punch full stroke signals if punch actuation
      punchFullStrokeSig[j] = digitalRead(punchFullStrokePinArray[j]);
      if (!punchFullStrokeSig[j]) punchesAtFullStroke = false;
    }
  }
  // End Read Inputs

  // Cool Modules
  lmpCoolingAir();

  // Fault Handling
  faultHandling(faultsActiveArray);
  // End Fault Handling

  //Calibrate RTD
  if (calibrateRtd) {
    for (j = 0; j < numHeaters; j++) {
      rtdAmbientAn[j] = heaterRtdAnalogSig[j];
    }
  }

  // Before entering into loop, check program time. If total cycle time exceeds max allowable, abort cycle and flag fault

  if ((currentCycleTime > maxOverallCycleTime) && cycleActive) {
    fltsPresent = true;
    faultsActiveArray[13] = true;
    cycleActive = false;
  }

  // If cycle is inactive and no faults are present and no cycle has been completed,
  // tell LCD "CI" or Cycle Inactive.
  if (!cycleActive && !fltsPresent && !cycleComplete) cycleCodes[0] = "CI";

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

    resetSystemTimerCntr = true; // Reset cycle timer
    cycleActive = true; // Starts cycle
    heatEnable = true; // Starts heat
    cycleCodes[0] = "CS";
    Serial.print("Cycle Start");
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
      processFlags[1] = true; // Functions as a one-shot to ensure that Heaters_On-Time is only captured once.
    }

    allAtMeltTemp = true;

    for (k = 0; k < numHeaters; k++) {
      if (heaterTemp[k] > heaterHiLim) { // If any heater exceeds hi-lim, exit cycle and trigger fault.
        heatEnable = false;
        fltsPresent = true;
        faultsActiveArray[numHeaters + k] = true;
        cycleActive = false;
      }

      // When a heater reaches setpoint, flag it and capture time
      else if (!fltsPresent && (heaterTemp[k] > heaterMeltTemp - meltTempHyst)/* &&
      (heaterTemp[k] < heaterMeltTemp + meltTempHyst)*/ && !atMeltTemp[k]) {
        atMeltTemp[k] = true;
        atSetpointTemp[k] = heaterTemp[k];
        meltTempReachedTime[k] = currentCycleTime; // Captures the first cycle time at which all heaters are within setpoint window.
      }

      if (!atMeltTemp[k]) allAtMeltTemp = false;
    }

    // If any heater is on for too long, exit cycle and trigger fault
    if ((currentCycleTime - heatersOnTime) > maxHeaterOnTime) {
      heatEnable = false;
      fltsPresent = true;
      faultsActiveArray[numHeaters*3] = true;
      cycleActive = false;
    }
    // When all heaters are at setpoint, extend press.
    if (allAtMeltTemp && !processFlags[2]) {
      cycleCodes[0] = "AS"; // "At Setpoint"
      Serial.print('At Setpoint');
      extendPress = true;
      processFlags[2] = true;
    }
  }

  if (allAtMeltTemp && heatEnable) {
    for (j = 0; j < numHeaters; j++) {
      if (heaterTemp[j] < heaterContactDipMin[j]) {
        heaterContactDipMin[j] = heaterTemp[j];
        heaterContactDipTime[j] = currentCycleTime;
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
    if (pressFullStrokeReached && punchFullStrokeSig[j] && atMeltTemp[j] &&
    heatEnable && !fltsPresent && cycleActive && !processFlags[3 + j]) {
      fullStrokeTime[j] = currentCycleTime;
      heaterTempAtShutoff[j] = heaterTemp[j];
      atMeltTemp[j] = false;
      processFlags[3 + j] = true;
      cycleCodes[0] = "FS";
      Serial.print('At Full Stroke');
    }
  // If all heaters are at full stroke (press/punch) disable heat and turn on cooling air.
    if (pressFullStrokeReached && punchFullStrokeSig[j] && !processFlags[7]) {
      if ((!fltsPresent && cycleActive) || (fltCoolingAirActive)) {
        heatEnable = false;
        //coolingAirEnable = true;
        processFlags[7] = true;
        cycleCodes[0] = "CA";
        Serial.print('Cooling Air On');
      }
    }
  }

  /*
  * When cooling air is enabled, heaters are off, and full stroke is reached, compare each heater's
  * temperature vs. the release temp. When each heater reaches release temp, flag it as ready for release.
  * When all heaters are ready for release, if no actuation, tell press to retract.
  * If punch actuation, instead tell all punches to retract.
  */
  if (((coolingAirEnable && pressFullStrokeReached && punchesAtFullStroke && !fltsPresent && cycleActive) ||
  (fltsPresent && !cycleActive && fltCoolingAirActive)) && !heatEnable && !processFlags[8]) {
    allHeatersReadyRelease = true;
    for (j = 0; j < numHeaters; j++) {
      if (heaterTemp[j] > (releaseTemp + releaseHyst)) {
        allHeatersReadyRelease = false;
      }
    }

    if (allHeatersReadyRelease) {
      heaterReleaseTime = currentCycleTime;
      //coolingAirEnable = false;
      if (!lmpActuation) {extendPress = false;}
      if (lmpActuation) {Fill_With(punchExtend, numHeaters, false);}
      processFlags[8] = true;
      cycleCodes[0] = "AR";
      Serial.print('At Release');
    }
  }

  // Check to see if punch/press has released correctly.
  if (allHeatersReadyRelease && (currentCycleTime - heaterReleaseTime < puloffTime)) {
    for (j = 0; j < numHeaters; j++) { // This loop checks for valid heater release. When valid release is obtained, flag time and heater temp.
      if (((lmpActuation && !punchFullStrokeSig[j]) ||
      (!lmpActuation && !pressFullStrokeReached)) && !processFlags[9 + j]) {
        heaterTempAtRelease[j] = heaterTemp[j];
        heaterRetractedTime[j] = currentCycleTime;
        processFlags[9 + j] = true; // Functions as one-shot to make sure this part of the process is only done once per cycle.
      }
    }
    // If we got a valid release from all heaters, cycle is complete. Flag time and send signal to machine plc
    if (((lmpActuation && !punchesAtFullStroke) || (!lmpActuation && !pressFullStrokeReached)) && !processFlags[13]) {
      cycleCompleteTime = currentCycleTime;
      for (j = 0; j < numHeaters; j++) {cycleCompleteTemp[j] = heaterTemp[j];}
      cycleComplete = true;
      cycleActive = false;
      extendPress = false;
      processFlags[13] = true;
      cycleCodes[0] = "CC";
      Serial.print('Cycle Complete');
    }
  }

  // If we do not obtain a valid release, enter flash cycle
  else if (allHeatersReadyRelease && (currentCycleTime - heaterReleaseTime >= puloffTime) && cycleActive && maxFlashCycles != 0) {
    flashCycle();
  }
  // If max flash cycles are 0, flash cycle is disabled. If we do not get valid release on first attempt, flag fault and exit cycle.
  else if (allHeatersReadyRelease && (currentCycleTime - heaterReleaseTime >= puloffTime) && !processFlags[13] && maxFlashCycles == 0) {
    fltsPresent = true;  // punches/press, flag fault and abort cycle.
    faultsActiveArray[14] = true;
    cycleActive = false;
  }

  // Cycle Stop logic
  if (cycleStopSignal) {
    cycleActive = false;
  }

  // Data Logging only happens at end of loop if Cycle is complete or if a fault was flagged on this scan.
  if (!cycleActive && !cycleDataLogged/* && (cycleComplete || fltsPresent || cycleStopSignal)*/) {
    logData();
    if (!systemHomed) homeSystem();
  }
  // Output Handling

  // Punch actuation
  if (lmpActuation) {
    for (j = 0; j < numHeaters; j++) {
      digitalWrite(punchExtendPin[j], punchExtend[j]);
    }
  }
}
// End of main loop

// Keep track of Cycle time
void systemTimer() {
  if(cycleActive){
    if(resetSystemTimerCntr){
      lastSnapshot = millis();
      resetSystemTimerCntr = false;
    }

    currentSnapshot = millis();
    currentCycleTime = (currentSnapshot - lastSnapshot)/1000;
    lastSnapshot = currentSnapshot;
  }
  else{
    currentSnapshot = 0;
    lastSnapshot = currentSnapshot;
  }

}

void triggerRead() {
  beginRead = true;
}

void triggerWrite() {
  beginWrite = true;
}

int index;

void readData() {
  Serial.print("msg rcvd");
  int l = Wire.available();
  char receivedBuffer[l + 1];
  byte sentBuffer[4];
  while (Wire.available()) {
    receivedBuffer[index] = Wire.read();
    ++index;
  }
  if (!Wire.available()) { index = 0; }
  if (l == 4) {
    for (j = 0; j < l; j++){
      inputsFromParent[j] = (bool)receivedBuffer[j];
    }
  }
  else if (l == 1) {
    Serial.print("mapping request rcvd");
    sendMappingData = true;
  }
  else {
    fltsPresent = true;
  }
}

void writeData() {
  Serial.print("sending data");
  if (sendMappingData) {
    byte sentBuffer[4] = {1, 1, 1, 1};
    char msg[5] = "hello";
    //for (j = 0; j < 4; j++) {sentBuffer[j] = lmpTypes[j];}
    sendMappingData = false;
    Wire.write(msg, 5);
    Serial.print("mapping request sent");
  }
  else {
    if (!sendDatalog){
      byte sentBuffer[1 + 2*numHeaters];
      sentBuffer[0] = boolArrayToByte(statusArray);
      for (j = 0; j < numHeaters; j++) {
        sentBuffer[1+2*j] = highByte((int16_t)heaterTemp[j]);
        sentBuffer[2+2*j] = lowByte((int16_t)heaterTemp[j]);
      }
      Wire.write(sentBuffer, (1 + 2*numHeaters));
    }
    else {
      byte sentBuffer[1 + 32*numHeaters];
      sentBuffer[0] = boolArrayToByte(statusArray);
      for (j = 0; j < numHeaters; j++) {
        sentBuffer[1+2*j] = highByte((int16_t)heaterTemp[j]);
        sentBuffer[2+2*j] = lowByte((int16_t)heaterTemp[j]);
        sentBuffer[9+30*j] = startTimeHb;
        sentBuffer[10+30*j] = startTimeLb;
        sentBuffer[11+30*j] = startTempHb[j];
        sentBuffer[12+30*j] = startTempLb[j];
        sentBuffer[13+30*j] = startPosHb[j];
        sentBuffer[14+30*j] = startPosLb[j];
        sentBuffer[15+30*j] = atSetpointTimeHb[j];
        sentBuffer[16+30*j] = atSetpointTimeLb[j];
        sentBuffer[17+30*j] = atSetpointTempHb[j];
        sentBuffer[18+30*j] = atSetpointTempLb[j];
        sentBuffer[19+30*j] = atSetpointPosHb[j];
        sentBuffer[20+30*j] = atSetpointPosLb[j];
        sentBuffer[21+30*j] = contactDipTimeHb[j];
        sentBuffer[22+30*j] = contactDipTimeLb[j];
        sentBuffer[23+30*j] = contactDipTempHb[j];
        sentBuffer[24+30*j] = contactDipTempLb[j];
        sentBuffer[25+30*j] = contactDipPosHb[j];
        sentBuffer[26+30*j] = contactDipPosLb[j];
        sentBuffer[27+30*j] = fullStrokeTimeHb[j];
        sentBuffer[28+30*j] = fullStrokeTimeLb[j];
        sentBuffer[29+30*j] = cycleCompleteTempHb[j];
        sentBuffer[30+30*j] = cycleCompleteTempLb[j];
        sentBuffer[31+30*j] = cycleCompletePosHb[j];
        sentBuffer[32+30*j] = cycleCompletePosLb[j];
        sentBuffer[33+30*j] = cycleCompleteTimeHb[j];
        sentBuffer[34+30*j] = cycleCompleteTimeLb[j];
        sentBuffer[35+30*j] = cycleCompleteTempHb[j];
        sentBuffer[36+30*j] = cycleCompleteTempLb[j];
        sentBuffer[37+30*j] = cycleCompletePosHb[j];
        sentBuffer[38+30*j] = cycleCompletePosLb[j];
      }
      Wire.write(sentBuffer, (9 + 30*numHeaters));
    }
  }
}

// Convert Bool array to Byte
byte boolArrayToByte(bool boolArray[8]){
  byte result = 0;
  for(int i = 0; i < 8; i++)
  {
    if(boolArray[i])
    {
      result = result | (1 << i);
    }
  }
  return result;
}

// Fault Handling function that reads active faults and sends out the appropriate fault M_2_S_Message.
void faultHandling(bool faultsActiveArray[]) {
  if (fltResetSig) { // Resets faults
    Fill_With(faultsActiveArray, 15, false);
    fltsPresent = false;
  } // End of fault reset
  char* Fault_Messages [] = {
    "Fault01: Heater 1 Start Failure",
    "Fault02: Heater 2 Start Failure",
    "Fault03: Heater 3 Start Failure",
    "Fault04: Heater 4 Start Failure",
    "Fault05: Heater 1 Exceeded Max Temp",
    "Fault06: Heater 2 Exceeded Max Temp",
    "Fault07: Heater 3 Exceeded Max Temp",
    "Fault08: Heater 4 Exceeded Max Temp",
    "Fault09: Heater 1 Off Failure",
    "Fault10: Heater 2 Off Failure",
    "Fault11: Heater 3 Off Failure",
    "Fault12: Heater 4 Off Failure",
    "Fault13: Heaters On Too Long.",
    "Fault14: Cycle Exceeded Max Allowable Time",
    "Fault15: Flash Cycle Failed. Punch/Press Not Releasing."
  }; //
  if (fltsPresent) { // If faults are present, Heat is disabled, Cycle is aborted.
    cycleCodes[0] = "FA";
    Serial.print('Faults Active');
    heatEnable = false;
    cycleActive = false;
    Fill_With(processFlags, 20, false);
    Fill_With(atMeltTemp, numHeaters, false);
    Fill_With(heatersRdyForRelease, numHeaters, false);
    for (j = 0; j < numHeaters; j++) {
      digitalWrite(heaterPowerPinArray[j], LOW); // Physically removes power to heaters as safeguard.
    }
  }
} // End of faultHandling



/*
  This interrupt serves as the cycle timing clock. It is reset using a PB on the LCD.
  The interrupt happens every five msec.
 */

void Heater_Duty(){
  if (dutyPulseValue < pulsePeriod){
    ++dutyPulseValue;
  }
  else{
    dutyPulseValue = 0;
    for (j = 0; j < numHeaters; j++){
      digitalWrite(heaterPowerPinArray[j], HIGH);
    }
  }
  for (j = 0; j < numHeaters; j++){
    if (dutyPulseValue >= heaterOnDutyArray[j]){
      digitalWrite(heaterPowerPinArray[j], LOW);
    }
  }
} // End of Heater_Duty

// This turns on cooling air whenever the LMP is above a safe temperature
void lmpCoolingAir() {
  const int Safe_Temp = 150, Safe_Hysteresis = 5;
  bool All_Safe = true;
  for (j = 0; j < numHeaters; j++) {
    if (!heatEnable && (heaterTemp[j] > (Safe_Temp + Safe_Hysteresis) || heaterTemp[j] > (releaseTemp + releaseHyst))) coolingAirEnable = true;
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
void PID_Heater_Control() {
  if (flashCycleHeat == 0) {
    if (heatEnable && !fltsPresent && cycleActive/* &&
    (!lmpActuation || (lmpActuation && !punchFullStrokeSig[j]))*/) { // Only performs PID calculation if heat is enabled and no faults are present.
      heater1PID.Compute();
      heater2PID.Compute();
      heater3PID.Compute();
      heater4PID.Compute();
    }
    else {
      Fill_With(heaterOnDutyArray, numHeaters, 0.0);
      heatEnable = false;
    }
  }
  else if (flashCycleHeat == 1) {
    for (j = 0; j < numHeaters; j++) {
      if (flashCycleTargetHeaters[j]) {heaterOnDutyArray[j] = pidOutputMax;}
      else {
        heaterOnDutyArray[j] = 0.0;
        heatEnable = false;
      }
    }
  }
  else if (flashCycleHeat == 2) {
    Fill_With(heaterOnDutyArray, numHeaters, 0.0);
    heatEnable = false;
  }
} // End of PID_Heater_Control

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
        Fill_With(flashCycleTargetHeaters, numHeaters, true);
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
      Fill_With(flashCycleTargetHeaters, numHeaters, false); // Turn off heater
      Fill_With(punchExtend, numHeaters, false); // Retract punch if punch actuation
      if (!lmpActuation) {
        extendPress = false; // Retract press if no punch actuation
      }
      flashCycleReleaseTime = currentCycleTime; // Flag start time of flash release
      flashCycleStep = 2; // Move to third step
    }
    // If third step is active and pulloff time has not been exceeded, check for
    // successful release.
    else if (flashCycleStep == 2 && !flashSuccess && (currentCycleTime <= flashCycleReleaseTime + puloffTime)) {
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
    else if (flashCycleStep == 2 && !cycleComplete && (currentCycleTime > flashCycleReleaseTime + puloffTime)) {
      if (flashSuccess) {
        flashCycleStep = 0;
        currentFlashCycle = 1;
        cycleActive = false;
        cycleCompleteTime = currentCycleTime;
        cycleComplete = true;
        cycleCodes[0] = "CC";
        extendPress = false;
        Fill_With(heatersRdyForRelease, numHeaters, false);
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
    faultsActiveArray[14] = true;
    cycleActive = false;
    currentFlashCycle = 1;
    flashCycleStep = 0;
    flashCycleHeat = 0;
  }
} // End of flashCycle

void homeSystem() {
  // Re-Initialize system for next cycle
  Fill_With(punchExtend, numHeaters, false);
  Fill_With(atMeltTemp, numHeaters, false);
  Fill_With(processFlags, 20, false);
  Fill_With(heatersRdyForRelease, numHeaters, false);
  Fill_With(flashCycleTargetHeaters, numHeaters, false);
  Fill_With(heaterContactDipMin, numHeaters, 0.0);
  Fill_With(heaterContactDipTime, numHeaters, 0.0);
  allHeatersReadyRelease = false;
  extendPress = false;
  //coolingAirEnable = false;
  currentFlashCycle = 1;
  flashCycleHeat = 0;
  flashCycleStep = 0;
  flashCycleStartTime = 0;
  flashCycleReleaseTime = 0;
  heatersOnTime = 0;
  heaterReleaseTime = 0;
  systemHomed = true;
} // End of homeSystem

// Logs Cycle data at the end of each cycle to the in-board SD card.
void logData() {
  for (j = 0; j < numHeaters; j++) {
    startTimeHb = highByte((int16_t)(heatersOnTime*100));
    startTimeLb = lowByte((int16_t)(heatersOnTime*100));
    startTempHb[j] = highByte((int16_t)(heaterStartTemp[j]*10));
    startTempLb[j] = lowByte((int16_t)(heaterStartTemp[j]*10));
    startPosHb[j] = 0;
    startPosLb[j] = 0;
    atSetpointTimeHb[j] = highByte((int16_t)(meltTempReachedTime[j]*100));
    atSetpointTimeLb[j] = lowByte((int16_t)(meltTempReachedTime[j]*100));
    atSetpointTempHb[j] = highByte((int16_t)(atSetpointTemp[j]*10));
    atSetpointTempLb[j] = lowByte((int16_t)(atSetpointTemp[j]*10));
    atSetpointPosHb[j] = 0;
    atSetpointPosLb[j] = 0;
    contactDipTimeHb[j] = highByte((int16_t)(heaterContactDipTime[j]*100));
    contactDipTimeLb[j] = lowByte((int16_t)(heaterContactDipTime[j]*100));
    contactDipTempHb[j] = highByte((int16_t)(heaterContactDipMin[j]*100));
    contactDipTempLb[j] = lowByte((int16_t)(heaterContactDipMin[j]*100));
    contactDipPosHb[j] = 0;
    contactDipPosLb[j] = 0;
    fullStrokeTimeHb[j] = highByte((int16_t)(fullStrokeTime[j]*100));
    fullStrokeTimeLb[j] = lowByte((int16_t)(fullStrokeTime[j]*100));
    heaterTempAtShutoffHb[j] = highByte((int16_t)(heaterTempAtShutoff[j]*10));
    heaterTempAtShutoffLb[j] = lowByte((int16_t)(heaterTempAtShutoff[j]*10));
    heaterPosAtShutoffHb[j] = 0;
    heaterPosAtShutoffLb[j] = 0;
    cycleCompleteTimeHb[j] = highByte((int16_t)(cycleCompleteTime*100));
    cycleCompleteTimeLb[j] = lowByte((int16_t)(cycleCompleteTime*100));
    cycleCompleteTempHb[j] = highByte((int16_t)(cycleCompleteTemp[j]*10));
    cycleCompleteTempLb[j] = lowByte((int16_t)(cycleCompleteTemp[j]*10));
    cycleCompletePosHb[j] = 0;
    cycleCompletePosLb[j] = 0;
  }
  cycleDataLogged = true;
} // End of logData
