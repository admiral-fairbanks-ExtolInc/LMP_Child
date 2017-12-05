#ifndef LMP_Class_h
#define LMP_Class_h

#include "Arduino.h"
#include <PID_v1.h>

class LMP{
  public:
    int LMP_Type, Heater_Peak_Temp, RTD_Analog_Sig, RTD_Ambient_AN;

    double Temp, Setpoint, Full_Stroke_Time, Peak_Temp_Time, Retracted_Time,
    RTD_Reading, Temp_At_Release, Temp_at_Shutoff, MeltTemp_Reached_Time,
    On_Duty_Array, Start_Temp, Contact_Dip_Min, Contact_Dip_Time, Cycle_Complete_Time;

    bool At_MeltTemp, Ready_for_Release, Punch_Full_Stroke_Sig, Punch_Extend,
    Flash_Cycle_Active;

    byte Start_Temp_HB, Start_Temp_LB, Start_Pos_HB, Start_Pos_LB, At_Setpoint_Time_HB,
    At_Setpoint_Time_LB, At_Setpoint_Temp_HB, At_Setpoint_Temp_LB, At_Setpoint_Pos_HB,
    At_Setpoint_Pos_LB, Contact_Dip_Time_HB, Contact_Dip_Time_LB, Contact_Dip_Temp_HB,
    Contact_Dip_Temp_LB, Contact_Dip_Pos_HB, Contact_Dip_Pos_LB, Cycle_Complete_Time_HB,
    Cycle_Complete_Time_LB, Cycle_Complete_Temp_HB, Cycle_Complete_Temp_LB,
    Cycle_Complete_Pos_HB, Cycle_Complete_Pos_LB;

    void Initialize (void);

    //PID PID_Cntrl;

  private:

};

#endif
