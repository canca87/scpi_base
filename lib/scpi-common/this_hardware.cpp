#include "this_hardware.h"

// Hardware IO pin mappings. From schematic 200312 LD driver M controller.
byte module_EEPROM_pin[6] = {6,9,15,20,23,26};
byte module_DIGIPOT_pin[6] = {8,14,17,22,25,28};
byte module_AD5592_pin[6] = {7,10,16,21,24,27};
byte module_TTL_pin[6] = {0,1,2,3,4,5};
ad5592 driver_modules[6];

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
  for (byte i = 0; i< 6; i++) {
    if (check_eeprom_presence(i+1) == 1) {
      driver_modules[i].begin(module_AD5592_pin[i]);
      Serial.begin(9600);
      delay(1000);
      Serial.print("Driver module ");
      Serial.print(i);
      Serial.print(" started with pin ");
      Serial.println(module_AD5592_pin[i]);
      // Set some default pin modes for the driver:
      // IO-0 is DAC
      // IO-1 is ADC
      // IO-2 is GPO_DRIVE
      // IO-3 is GPO_DRIVE
      // IO-4 is GPI
      // IO-5 is GPI
      // IO-6 is OFF
      // IO-7 is OFF
      driver_modules[i].set_state(0,ad5592_state::dac_mode);
      driver_modules[i].write_state(0,0); // DAC output to zero volts
      driver_modules[i].set_state(1,ad5592_state::adc_mode);
      driver_modules[i].set_state(2,ad5592_state::gpo_mode);
      driver_modules[i].set_gpo_mode(2,ad5592_gio_mode::push_pull);
      driver_modules[i].write_state(2,0); // GPO set to zero volts
      driver_modules[i].set_state(3,ad5592_state::gpo_mode);
      driver_modules[i].set_gpo_mode(3,ad5592_gio_mode::push_pull);
      driver_modules[i].write_state(3,0); // GPO set to zero volts
      driver_modules[i].set_state(4,ad5592_state::gpi_mode);
      driver_modules[i].set_state(5,ad5592_state::gpi_mode);
      driver_modules[i].set_state(6,ad5592_state::off_mode);
      driver_modules[i].set_state(7,ad5592_state::off_mode);
    }
  }
}

void run_stuff(void) {
   
}

float read_ad5592_temperature(byte module) {
  if (check_eeprom_presence(module) == 1) {
    return driver_modules[module-1].read_temperature();
  }
  else {
    return 0;
  }
}

int read_ad5592_mode(byte module, byte io){
  if (check_eeprom_presence(module) == 1) {
    // its a bit messy converting between ad5592 modes and SCPI modes.
    /* SCPI modes:
     *    {"UNKNOWN", 0},
     *    {"ADC", 1},
     *    {"DAC", 2},
     *    {"GPO_DRIVE", 3},
     *    {"GPO_SINK", 4},
     *    {"GPI", 5},
     *    {"Z", 6},
     *    {"OFF", 7},
     *    {"UNAVAILABLE", 8},
     * 
     * AD5592 modes:
     *    enum ad5592_state
     *    {
     *        adc_mode,
     *        dac_mode,
     *        gpi_mode,
     *        gpo_mode,
     *        z_mode,
     *        off_mode, //off is the equivalent of pull-down mode.
     *        unknown //there is one other state: when nothing else is set. Not sure what happens then.
     *    };
     *    enum ad5592_gio_mode
     *    {
     *        push_pull,
     *        open_drain
     *    };
     */
    ad5592_state mode_enum = driver_modules[module-1].get_state(io);
    if (mode_enum == ad5592_state::adc_mode) {
      return 1;
    }
    else if (mode_enum == ad5592_state::dac_mode) {
      return 2;
    }
    else if(mode_enum == ad5592_state::gpo_mode) {
      //gpo has two sub-modes
      ad5592_gio_mode gio_enum = driver_modules[module-1].get_mode(io);
      if (gio_enum == ad5592_gio_mode::push_pull) {
        return 3;
      }
      else {
        return 4;
      }
    }
    else if (mode_enum == ad5592_state::gpi_mode) {
      return 5;
    }
    else if (mode_enum == ad5592_state::z_mode) {
      return 6;
    }
    else if (mode_enum == ad5592_state::off_mode) {
      return 7;
    }
    else {
      return 0;
    }
  }
  else {
    return 8; //return unknown mode
  }
}

int write_ad5592_mode(byte module, byte io, int mode){
  if (check_eeprom_presence(module) == 1) {
    // its a bit messy converting between ad5592 modes and SCPI modes.
    /* SCPI modes:
     *    {"UNKNOWN", 0},
     *    {"ADC", 1},
     *    {"DAC", 2},
     *    {"GPO_DRIVE", 3},
     *    {"GPO_SINK", 4},
     *    {"GPI", 5},
     *    {"Z", 6},
     *    {"OFF", 7},
     *    {"UNAVAILABLE", 8},
     * 
     * AD5592 modes:
     *    enum ad5592_state
     *    {
     *        adc_mode,
     *        dac_mode,
     *        gpi_mode,
     *        gpo_mode,
     *        z_mode,
     *        off_mode, //off is the equivalent of pull-down mode.
     *        unknown //there is one other state: when nothing else is set. Not sure what happens then.
     *    };
     *    enum ad5592_gio_mode
     *    {
     *        push_pull,
     *        open_drain
     *    };
     */
    if (mode == 1) {
      // adc mode
      driver_modules[module-1].set_state(io,ad5592_state::adc_mode);
      return 1;
    }
    else if (mode == 2) {
      // dac mode
      driver_modules[module-1].set_state(io,ad5592_state::dac_mode);
      return 2;
    }
    else if (mode == 3) {
      // gpo drive mode: must set both gpo and drive modes
      driver_modules[module-1].set_state(io,ad5592_state::gpo_mode);
      driver_modules[module-1].set_gpo_mode(io,ad5592_gio_mode::push_pull);
      return 3;
    }
    else if (mode == 4) {
      // gpo sink mode: must set both gpo and drive modes
      driver_modules[module-1].set_state(io,ad5592_state::gpo_mode);
      driver_modules[module-1].set_gpo_mode(io,ad5592_gio_mode::open_drain);
      return 4;
    }
    else if (mode == 5) {
      // gpi mode
      driver_modules[module-1].set_state(io,ad5592_state::gpi_mode);
      return 5;
    }
    else if (mode == 7) {
      // high z mode
      driver_modules[module-1].set_state(io,ad5592_state::z_mode);
      return 6;
    }
    else if (mode == 7) {
      // off mode (weak pull-down):
      driver_modules[module-1].set_state(io,ad5592_state::off_mode);
      return 7;
    }
    else {
      // unknown mode
      driver_modules[module-1].set_state(io,ad5592_state::unknown);
      return 0;
    }
  }
  else {
    return 8; //return unknown mode
  }
}

float read_ad5592_data(byte module, byte io){
  if (check_eeprom_presence(module) == 1) {
    return driver_modules[module-1].read_state(io);
  }
  else {
    return 0;
  }
}

float write_ad5592_data(byte module, byte io, float val){
  if (check_eeprom_presence(module) == 1) {
    return driver_modules[module-1].write_state(io,(float)val);
  }
  else {
    return 0;
  }
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
