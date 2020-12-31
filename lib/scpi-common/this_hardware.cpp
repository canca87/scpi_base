#include "this_hardware.h"

// Local function definitions

byte module_EEPROM_pin[6] = {6,9,15,20,23,26};
byte module_DIGIPOT_pin[6] = {8,14,17,22,25,28};
byte module_AD5592_pin[6] = {10,10,10,10,10,10};
byte module_TTL_pin[6] = {0,1,2,3,4,5};


/* init_hardware
 *
 * initalises the hardware componenets of the microcontroller.
 * Pin direction (input/output/pullup)
 * Set starting state (default safe)
 * Starts any communication hardware (WIRE, SPI, ETC)
 * Starts any device drivers as needed.
 * Initalises timers/interrupts as needed.
 * 
 */
void init_hardware(void){
  // set SPI cs pins to output and high level
  for (byte i = 0; i < 6; i++){
    pinMode(module_EEPROM_pin[i],OUTPUT);
    digitalWriteFast(module_EEPROM_pin[i],HIGH);
    pinMode(module_AD5592_pin[i],OUTPUT);
    digitalWriteFast(module_AD5592_pin[i],HIGH);
    pinMode(module_DIGIPOT_pin[i],OUTPUT);
    digitalWriteFast(module_DIGIPOT_pin[i],HIGH);
    pinMode(module_TTL_pin[i],OUTPUT);
    digitalWriteFast(module_TTL_pin[i],LOW); // TTL is active high.
  }
  SPI.begin();
}

void run_stuff(void) {
   
}


byte read_eeprom_byte(byte module, uint16_t address) {
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    byte return_value = 0xFF;
    digitalWriteFast(module_EEPROM_pin[module-1],LOW);
    SPI.transfer(0b00000011); //read instruction
    SPI.transfer(0xFF & (byte)(address>>8)); //data address to read
    SPI.transfer(0xFF & (byte)address); //data address to read
    return_value = SPI.transfer(0);
    digitalWriteFast(module_EEPROM_pin[module-1],HIGH);
    SPI.endTransaction();
    return(return_value);
}

void write_eeprom_byte(byte module, uint16_t address, byte value) {
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    // Setup to allow writing to the EEPROM:
    digitalWriteFast(module_EEPROM_pin[module-1],LOW);
    SPI.transfer(0b00000110); //write enable instruction
    digitalWriteFast(module_EEPROM_pin[module-1],HIGH);
    delay(1);
    // Write to the EEPROM a single byte:
    digitalWriteFast(module_EEPROM_pin[module-1],LOW);
    SPI.transfer(0b00000010); //write instruction
    SPI.transfer(0xFF & (byte)(address>>8)); //data address to wrute
    SPI.transfer(0xFF & (byte)address); //data address to write
    SPI.transfer(value);
    digitalWriteFast(module_EEPROM_pin[module-1],HIGH);
    delay(1);
    // Wait for the write to complete:
    digitalWriteFast(module_EEPROM_pin[module-1],LOW);
    SPI.transfer(0b00000101); //read status register
    byte status = 1;
    byte timeout = 100; //100ms timeout for writes
    while (status && (timeout > 0)) {
        status = SPI.transfer(0); // get the value from the status register
        status = status & 1;
        delay(1);
        timeout --;
    }
    digitalWriteFast(module_EEPROM_pin[module-1],HIGH);
    SPI.endTransaction();
    return;
}

int save_module_eeprom(byte module) {
    // EEPROM map is found in the electronics bringup folder
    // First set the inital byte to 0xAA to indicate the EEPROM is initalised and functional
    if (read_eeprom_byte(module, 0) != 0xAA) {
      write_eeprom_byte(module, 0, 0xAA);
    }
    // Check that the first byte is correct (and there is actually a working EEPROM!)
    if (read_eeprom_byte(module, 0) == 0xAA) {
        // set the current sensor I2C address from the device
        return 0;
    }
    else return -1;
}

int save_system_eeprom(){
    /* Save to the teensy EEPROM:
    * address 0 is reserved.
    */
    unsigned int eeAddress = 1;
    uint16_t count = 0;
    // add any system level eeprom material here
    return 0;
}

int save_parameters_to_eeprom(void){
    /* Saves all the eeproms and returns the count of missed EEPROMs
    */
    int ret_val = 0;
    ret_val += save_system_eeprom();
    for (int i = 1; i<=6; i++){
        ret_val += save_laser_module_eeprom(i);
    }
    //done.
    return ret_val;
}

int recall_module_eeprom(byte module) {
    // EEPROM map is found in the electronics bringup folder
    // First byte is a check byte, it must be 0xAA for a valid initalised module to function:
    if (read_eeprom_byte(module,0) == 0xAA){
        return 0;
    }
    return -1;
}

int recall_system_eeprom(){
    /* Recall from the teensy EEPROM:
    * address 0 is reserved.
    */
    unsigned int eeAddress = 1;
    uint16_t count = 0;
    return 0;
}

int recall_parameters_from_eeprom(void){
    /* Recall from module eeproms
    */
   recall_system_eeprom();
    
    for (int i = 1; i<=6; i++){
        recall_module_eeprom(i);
    }

    //done.
    return 0;
}

byte check_eeprom_presence(byte module){
  // return 1 if eeprom is present
  // zero if not:
    if (read_eeprom_byte(module, 0) != 0xAA) {
      write_eeprom_byte(module, 0, 0xAA);
    }
    // Check that the first byte is correct (and there is actually a working EEPROM!)
    if (read_eeprom_byte(module, 0) == 0xAA) {
        // the EEPROM is present and functional
        return 1;
    }
    return 0;
}

double get_eeprom_version(byte module){
  //two-part number: Pre decimal point is 17-18 and post is 19.
  if (read_eeprom_byte(module, 0) != 0xAA) {
    write_eeprom_byte(module, 0, 0xAA);
  }
  // Check that the first byte is correct (and there is actually a working EEPROM!)
  if (read_eeprom_byte(module, 0) == 0xAA) {
    // the EEPROM is present and functional:
    int high_byte = (uint32_t(read_eeprom_byte(module,17)) << 16) & 0xFF0000;
    int middle_byte = (uint32_t(read_eeprom_byte(module,18)) << 8) & 0x00FF00;
    int low_byte = (uint32_t(read_eeprom_byte(module,19))) & 0x0000FF;
    double version = (double(read_eeprom_byte(module,20))/10.0) + double(high_byte) + double(middle_byte) + double(low_byte);
    return version;
  }
  return 0.0;
}

int set_eeprom_version(byte module, double val){
  //two-part number: Pre decimal point is 17-18 and post is 19.
  if (read_eeprom_byte(module, 0) != 0xAA) {
    write_eeprom_byte(module, 0, 0xAA);
  }
  // Check that the first byte is correct (and there is actually a working EEPROM!)
  if (read_eeprom_byte(module, 0) == 0xAA) {
    // the EEPROM is present and functional:
    byte revision = byte(floor((val - floor(val))*10));
    byte low_byte = uint32_t(floor(val)) & 0x0000FF;
    byte middle_byte = (uint32_t(floor(val)) & 0x00FF00) >> 8;
    byte high_byte = (uint32_t(floor(val)) & 0xFF0000) >> 16;
    write_eeprom_byte(module,17,high_byte);
    write_eeprom_byte(module,18,middle_byte);
    write_eeprom_byte(module,19,low_byte);
    write_eeprom_byte(module,20,revision);
    return 1;
  }
  return 0;

}
