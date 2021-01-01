#ifndef __HARDWARE_DEF_H_
#define __HARDWARE_DEF_H_

#include <Arduino.h>
#include <SPI.h>
#include "ad5592.h"

// Define any IO mappings here
#define pin0 = 0

extern uint32_t watchdog_timeout_microseconds;

// hardware related functions should go here:
void run_stuff(void); //general dummy function for testing stuff.
void init_hardware(void); // initialisation routine for hardware modules and IO.

float read_ad5592_temperature(byte module);

int read_ad5592_mode(byte module, byte io);

int write_ad5592_mode(byte module, byte io, int mode);

float read_ad5592_data(byte module, byte io);

float write_ad5592_data(byte module, byte io, float val);

byte read_eeprom_byte(byte module, uint16_t address);

void write_eeprom_byte(byte module, uint16_t address, byte value);

int save_laser_module_eeprom(byte module);

int save_system_eeprom();

int save_parameters_to_eeprom(void);

int recall_laser_module_eeprom(byte module);

int recall_system_eeprom();

int recall_parameters_from_eeprom(void);

byte check_eeprom_presence(byte module);

double get_eeprom_version(byte module);

int set_eeprom_version(byte module, double val);


#endif /* __HARDWARE_DEF_H_ */