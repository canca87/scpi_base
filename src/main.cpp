#include <Arduino.h>
#include "scpi/scpi.h"
#include "scpi-def.h"

// Macros and functions for testing SCPI comms:
#define TEST_SCPI_INPUT(cmd) result = SCPI_Input(&scpi_context, cmd, strlen(cmd))
void test_scpi(void);
void serial_scpi(void);

//redirect printf functions to serial port:
extern "C" {
int _write(int file, char *ptr, int len) {
  int todo;

  for (todo = 0; todo < len; todo++) {
      Serial.print(ptr[todo]);
    }
    return len;
  }
}

void setup() {
  init_hardware(); //run the hardware initalisation routine

  Serial.begin(9600);
  
  //initalise the SCPI interface: scpi-def.h has all the commands...
  SCPI_Init(&scpi_context,
          scpi_commands,
          &scpi_interface,
          scpi_units_def,
          SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
          scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH,
          scpi_error_queue_data, SCPI_ERROR_QUEUE_SIZE);
}

void loop() {
  test_scpi();
  delay(1000);
  //serial_scpi();
}

void serial_scpi(void) {
  int result;
  (void)result; //this is to silence the unused variable warning from the compiler. Its used by the macro below:
  static String cmd = "";
  static String scpi_cmd = "";
  if (Serial.available()) //if ther is data avaliable
  {
    //digitalWriteFast(LED_BUILTIN,!digitalReadFast(LED_BUILTIN));
    char ch = Serial.read(); //read the character
    cmd = ch;
    scpi_cmd += ch;
    //Serial.println(ch); //reprint the number/letter to the screen
    if(ch >= '\n') // is it a new line?
    {
      char _cmd[sizeof(cmd)];
      cmd.toCharArray(_cmd,sizeof(_cmd));
      //Serial.print(_cmd);
      
      TEST_SCPI_INPUT(_cmd);
      cmd = "";

      //reset the watchdog timer on a new line character
      if (watchdog_timeout_microseconds != 0) {
        watchdogTimer.begin(watchdog_timeout,watchdog_timeout_microseconds);
      }
      else {
        watchdogTimer.end();
      }
      
    }
    if(ch == '\n'){
      scpi_cmd = "";
    }
  }
}

void test_scpi(void) {
  int result; //used by the TEST_SCPI_INPUT macro...
  (void)result; //this is to silence the unused variable warning from the compiler. Its used by the macro below:
  //test the *IDN? input. Should print itentity to the serial port:
  TEST_SCPI_INPUT("*IDN?\r\n");
  //test the *TST? input. Should print 0 to the serial port:
  TEST_SCPI_INPUT("*TST?\r\n");
  //clear all queue errors:
  TEST_SCPI_INPUT("*CLS\r\n");
  //
  //TEST_SCPI_INPUT("TEST:CHANnellist 3.5,(@1!1,2!1,6!1,1!8,3)\r\n");
  //
  //TEST_SCPI_INPUT("TEST:CHOice? EXT\r\n");
  //
  TEST_SCPI_INPUT("DRIV:STAT?\r\n"); //get the CSV list of available drivers/channels
  //
  TEST_SCPI_INPUT("DRIV:STAT? (@2)\r\n"); //gets the availability of driver/channel 2
  //
  //TEST_SCPI_INPUT("DRIV:STAT 201026.1,(@2)\r\n"); //This writes to the EEPROM: Use with caution.
  //
  TEST_SCPI_INPUT("DRIV:TEMP?\r\n"); // get the temperatures of all channels
  //
  TEST_SCPI_INPUT("DRIV:TEMP? (@2)\r\n"); //gets the temeprature of channel 2
  //
  TEST_SCPI_INPUT("DRIV:MOD? (@2!1)\r\n"); //gets the driver mode of channel 2 IO 1 (should be DAC)
  //
  TEST_SCPI_INPUT("DRIV:MOD? (@2!2,6)\r\n"); //gets the driver mode of channel 2.2 and 6.1-6.8
  //
  TEST_SCPI_INPUT("DRIV:MOD GPI,(@2!4)\r\n"); //Change channel 2 driver 4 to input (returns GPI)
  //
  TEST_SCPI_INPUT("DRIV:READ? (@2!2,6!2)\r\n"); // Read the ADC channels (io 2 of channels by default)
  //
  TEST_SCPI_INPUT("DRIV:WRITE 1,(@2!3,6!3)\r\n");
  TEST_SCPI_INPUT("DRIV:WRITE 1,(@2!4,6!4)\r\n"); //IO 2!4 is an input
  TEST_SCPI_INPUT("DRIV:WRITE 0,(@2!3,6!3)\r\n");
  TEST_SCPI_INPUT("DRIV:WRITE 0,(@2!4,6!4)\r\n"); //IO 2!4 is an input
  TEST_SCPI_INPUT("DRIV:WRITE 2.5,(@2!1)\r\n"); //IO 2!1 set it to 2.5v output (DAC)
  TEST_SCPI_INPUT("DRIV:READ? (@2!1:2!2)\r\n"); // read DAC and ADC values (short IO-0 and IO-1 to see if they match)

}