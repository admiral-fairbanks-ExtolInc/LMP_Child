/*
 * The following function is for the Flash Cycle. It is only entered into if the press/punch fail to release from the
 * welded plastic. There is a pre-defined number of flash cycles the system will cycle through before entering into a faulted state.
 * Procedure is as follows: First, enable all heaters and extend punches/press (depending on type of LMP Unit) for one second,
 * then disable heat and attempt to retract punches or press. After a set delay, if the punches/press fail to release, increment the cycle
 * counter and repeat. If the flash cycle is successful in allowing the punches/press to retract, enter into a normal "cycle complete"
 * state and exit Flash Cycle. If the maximum number of flash cycles is exceeded without successfully detaching punches/press, a fault
 * is flagged and the cycle is aborted.
 */
void Flash_Cycle(){
  if (Current_Flash_Cycle <= Max_Flash_Cycles){ // Current flash cycle has to be within the limit
    if (Flash_Cycle_Step == 0){ // First step
      Flash_Cycle_Start_Time = Current_Cycle_Time; // Flag start time of flash heating
      Flash_Cycle_Heat = 1; // Flash_Cycle_Heat = 1 means heat is forced on
      Flash_Cycle_Step = 1; // Move to second step
      if(LMP_Type == 0){ // If no punch actuation, turn on all heaters and extend press
        Fill_With(Flash_Cycle_Target_Heaters, Num_Heaters, true);
        Extend_Press = true; // Extend press if no punch actuation
      }
      else if(LMP_Type == 1){ // If punch actuation, turn only stuck heaters on and extend punches
        for(j = 0; j < Num_Heaters; j++){
          if(Punch_Full_Stroke_Sig[j] == true){
            Flash_Cycle_Target_Heaters[j] = true;// Turn on heater
            Punch_Power_Sig[j] = true; // Extend punch if punch actuation
          }
        }
      }
    }

    // If second step is active and flash time has been exceeded, proceed.
    else if (Flash_Cycle_Step == 1 && (Current_Cycle_Time >= Flash_Cycle_Start_Time + Flash_Time)){
      Flash_Cycle_Heat = 2; // Flash_Cycle_Heat = 2 means heat is forced off
      Fill_With(Flash_Cycle_Target_Heaters, Num_Heaters, false); // Turn off heater
      Fill_With(Punch_Power_Sig, Num_Heaters, false); // Retract punch if punch actuation
      if(LMP_Type == 0){
        Extend_Press = false; // Retract press if no punch actuation
      }
      Flash_Cycle_Release_Time = Current_Cycle_Time; // Flag start time of flash release
      Flash_Cycle_Step = 2; // Move to third step
    }
    // If third step is active and pulloff time has not been exceeded, check for
    // successful release.
    else if (Flash_Cycle_Step == 2 && Flash_Success == false &&
    (Current_Cycle_Time <= Flash_Cycle_Release_Time + Pulloff_Time)){
      if (LMP_Type == 1){
        Flash_Success = true;
        for(j = 0; j < Num_Heaters; j++){
          if (Punch_Full_Stroke_Sig[j] == true){
            Flash_Success = false;
          }
        }
      }
      else if (LMP_Type == 0 && Press_Full_Stroke_Reached == false){
        Flash_Success = true;
      }
    }
    // If third step is active and pulloff time has been exceeded and flash cycle
    // was successfully completed, cycle is complete, end cycle.
    else if (Flash_Cycle_Step == 2 && Cycle_Complete == false &&
    (Current_Cycle_Time > Flash_Cycle_Release_Time + Pulloff_Time)){
      if (Flash_Success == true){
        Flash_Cycle_Step = 0;
        Current_Flash_Cycle = 1;
        Cycle_Active = false;
        Cycle_Complete_Time = Current_Cycle_Time;
        Cycle_Complete = true;
        Cycle_Codes[i] = "CC";
        Extend_Press = false;
        Fill_With(Heaters_Ready_for_Release, Num_Heaters, false);
        All_Heaters_Ready_Release = false;
        Process_Flags[13] = true;
        Flash_Cycle_Heat = 0;
      }
      // If third step is active and pulloff time has been exceeded and flash cycles
      // was unsuccessfully completed, move to next flash cycle.
      else if (Flash_Success == false){
        Current_Flash_Cycle++;
      }
    }
  }
  else if (Max_Flash_Cycles != 0 && Current_Flash_Cycle > Max_Flash_Cycles){ // If max number of cycles is exceeded without successfullly removing
    Faults_Present = true;  // punches/press, flag fault and abort cycle.
    Faults_Active_Array[14] = true;
    Cycle_Active = false;
    Current_Flash_Cycle = 1;
    Flash_Cycle_Step = 0;
    Flash_Cycle_Heat = 0;
  }
} // End of Flash_Cycle
