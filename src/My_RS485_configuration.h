#ifdef  IsIncluded_My_RS485_configuration_h
#define IsIncluded_My_RS485_configuration_h

#define HardwareSerial_h  // required!! This disables the inclusion of HardwareSerial.h
extern void serialEventRun(void) __attribute__((weak));
#include "Stream.h"
// Define the configuration of the Serial1, Serial2, etc. devices (See HardwaSerialRS485.h). Adapt these to your needs

//
#define RS485configuration_HardwareSerialRS485_1 \
        HardwareSerialRS485< USART1, 6, 7, ' ', RS485serial< TRxControl< 'B', 2, 3 > >, MFP< '{', '}' > >

// defines the HardwareSerialRS485 class for the Serial2 device associated with USART2
#define RS485configuration_HardwareSerialRS485_2 \
        HardwareSerialRS485< USART2, 6, 7, ' ', RS485serial< TRxControl< 'C', 6, 6 >, Transaction_dummy > >

// defines the HardwareSerialRS485 class for the Serial3 device associated with USART3
#define RS485configuration_HardwareSerialRS485_3 \
        HardwareSerialRS485< USART3, 6, 7, ' ' >

#endif   // ifdef  IsIncluded_My_RS485_configuration_h
//eof My_RS485_configuration.h
