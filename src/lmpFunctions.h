// Converts 0-5V RTD Reading from analog pin to Temp value for simple voltage divider circuit (RTD on bottom leg, Ref_Res on top leg)
float RTD_Temp_Conversion(uint16_t A_in, uint16_t A_in_Ambient) {
  const float LMP_Amb_Temp = 75, RTD_Temp_Factor = 480, Ref_Res = 100, AnalogFullScale = 1023;
  float RTD_Res, RTD_Res_Ambient;

  RTD_Res = Ref_Res/(AnalogFullScale/A_in - 1.0);
  RTD_Res_Ambient = Ref_Res/(AnalogFullScale/A_in_Ambient - 1.0);
  return (RTD_Temp_Factor*(RTD_Res/RTD_Res_Ambient - 1.0) + LMP_Amb_Temp);
} // End of RTD_Temp_Conversion
