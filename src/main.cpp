#include <Arduino.h>
#include "scpi/scpi.h"
#include "scpi-def.h"

// Macros and functions for testing SCPI comms:
#define TEST_SCPI_INPUT(cmd) result = SCPI_Input(&scpi_context, cmd, strlen(cmd))
void test_scpi(void);

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
  TEST_SCPI_INPUT("DRIV:STAT?\r\n");
  //
  TEST_SCPI_INPUT("DRIV:STAT? (@2)\r\n");
  //
  //TEST_SCPI_INPUT("DRIV:STAT 201026.1,(@2)\r\n");
}