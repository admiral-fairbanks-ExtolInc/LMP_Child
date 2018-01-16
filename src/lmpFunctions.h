// Converts 0-5V RTD Reading from analog pin to Temp value for simple voltage divider circuit (RTD on bottom leg, refRes on top leg)
double rtdTempConversion(uint16_t aIn, uint16_t aInAmbient) {
  const double lmpAmbTemp = 75, rtdTempFactor = 530, refRes = 220, analogFullScale = 1023;
  double rtdRes, rtdResAmbient;
  if (aIn >= 1023) aIn = 1022;
  if (aIn <= 0) aIn = 1;

  rtdRes = refRes/(analogFullScale/aIn - 1.0);
  rtdResAmbient = refRes/(analogFullScale/aInAmbient - 1.0);
  if ((rtdTempFactor*(rtdRes/rtdResAmbient - 1.0) + lmpAmbTemp) < 0) {
    return 0;
  }
  else return (rtdTempFactor*(rtdRes/rtdResAmbient - 1.0) + lmpAmbTemp);
} // End of RTD_Temp_Conversion
