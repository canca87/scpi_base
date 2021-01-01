#include "ad5592.h"
#include <Arduino.h>

uint16_t ad5592::tx_packet_generator(uint16_t command, uint16_t data){
    //merge the two data/command elements using OR
    data = data & 0x07FFF;
    return command | data;
}

void ad5592::write_spi_command(uint16_t data)
{
    SPI.beginTransaction(write_spi_settings);
    digitalWriteFast(_sync_pin,LOW);
    SPI.transfer16(data);
    digitalWriteFast(_sync_pin,HIGH);
    delayMicroseconds(1); //should be 20ns
    SPI.endTransaction();
}

uint16_t ad5592::rw_spi_command(uint16_t data,int delay)
{
    write_spi_command(data);
    if (delay > 0) {
        delayMicroseconds(delay);
    }
    return read_spi_command();
}

uint16_t ad5592::read_spi_command(void)
{
    SPI.beginTransaction(read_spi_settings);
    digitalWriteFast(_sync_pin,LOW);
    uint16_t data = SPI.transfer16(_nop);
    digitalWriteFast(_sync_pin,HIGH);
    delayMicroseconds(1); //should be 20ns
    SPI.endTransaction();
    return data;
}

uint16_t ad5592::readback_register(uint16_t register_id)
{
    uint16_t data = 0;
    data = data | (register_id >> 9); // the low to bits are LDAC, and should be left as 00.
    data = bitSet(data,6); //set this to one to enable the register readback.
    data = tx_packet_generator(_readback_ldac,data);
    uint16_t returned_data = rw_spi_command(data,1);
    return returned_data;
}

float ad5592::convert_dac_to_analog(uint16_t val)
{
    // check if Vref is defined (internally enabled)
    uint16_t _power_ctrl_reg_state = readback_register(_power_control);
    float v_ref = 2.5; //default to the 2.5v Vref interally
    if (bitRead(_power_ctrl_reg_state,9) == 0)
    {
        //the V-ref internal is not enabled: use the set v-ref
        v_ref = _external_vref;
    }
    // check if the gain value is set (either x1 or x2 of Vref)
    // Determine the current ADC gain by reading the _gpio_control register bit 5:
    uint16_t _gpio_ctrl_reg_state = readback_register(_gpio_control);
    float gain = 1;
    if (bitRead(_gpio_ctrl_reg_state,5))
    {
        //gain is set to x2:
        gain = 2;
    }
    // return the calculated voltage:
    float adc_code = val & 0x0FFF; //get the 12-bit number.
    return v_ref * (adc_code/4095.0) * gain;
}

uint16_t ad5592::convert_analog_to_dac(float val)
{
    // check if Vref is defined (internally enabled)
    uint16_t _power_ctrl_reg_state = readback_register(_power_control);
    float v_ref = 2.5; //default to the 2.5v Vref interally
    if (bitRead(_power_ctrl_reg_state,9) == 0)
    {
        //the V-ref internal is not enabled: use the set v-ref
        v_ref = _external_vref;
    }
    // check if the gain value is set (either x1 or x2 of Vref)
    // Determine the current ADC gain by reading the _gpio_control register bit 5:
    uint16_t _gpio_ctrl_reg_state = readback_register(_gpio_control);
    float gain = 1;
    if (bitRead(_gpio_ctrl_reg_state,5))
    {
        //gain is set to x2:
        gain = 2;
    }
    // return the calculated voltage:
    uint16_t adc_code = (uint16_t)((val/(v_ref * gain))*4095.0) & 0x0FFF; //get the 12-bit number.
    return adc_code;
}

void ad5592::set_pin_mode(uint16_t address, byte io, byte state)
{
    //get the current register details:
    uint16_t data = readback_register(address);
    data = bitWrite(data,io,state);
    write_spi_command(tx_packet_generator(address,data));
    return;
}

void ad5592::begin(int sync_pin)
{
    _sync_pin = sync_pin; //store the sync_pin value
    pinMode(_sync_pin,OUTPUT);
    digitalWriteFast(_sync_pin,HIGH);
    // Assume SPI.begin() has already been called.
    // Reset the IC to ensure register consistency in soft-reset condition (ie reprogramming)
    write_spi_command(_reset);
    delayMicroseconds(250); // Delay recommended by datasheet, page 40.
    // By default, all pins are now in off_mode (pull down):
    for (int i = 0; i<8; i++)
    {
        _gpio_state[i] = ad5592_state::off_mode;
    }
    // Set the LDAC mode to update immediately
    write_spi_command(tx_packet_generator(_readback_ldac,0)); //zero is update LDAC immediately
    delayMicroseconds(1); // Delay needed is actually only 20ns.
    //activate internal reference -> all channels online
    uint16_t data = 0;
    data = bitSet(data,9);
    write_spi_command(tx_packet_generator(_power_control,data));
    //set reference gain to x2
    data = 0;
    data = bitSet(data,9); //ADC buffer pre-charge enabled
    data = bitSet(data,5); //ADC gain to x2
    data = bitSet(data,4); //DAC gain to x2
    write_spi_command(tx_packet_generator(_gpio_control,data));
}

float ad5592::read_temperature(void)
{
    //Datasheet page 33 covers reading the temperature. 
    //Use _adc_sequence with bit 8 set to read temperature.
    uint16_t data = 0;
    data = bitSet(data,8);
    uint16_t returned_data = rw_spi_command(tx_packet_generator(_adc_sequence,data),25);
    returned_data = read_spi_command(); //ADC takes one extra read to get out
    //From the datasheet, the address of the temperature reading is 0b1000
    int returned_io = (returned_data >> 12) & 0x000F;
    if (0b1000 != returned_io) {
        Serial.print("Returned DAC address error. Expected ");
        Serial.print(8);
        Serial.print(" but received ");
        Serial.print(returned_io);
        Serial.println(".");
        return -1;
    }
    float adc_code = returned_data & 0x0FFF; //as per page 32 of the datasheet
    // The datasheet mentions the equation is different depending on the ADC gain.
    // Determine the current ADC gain by reading the _gpio_control register bit 5:
    uint16_t _gpio_ctrl_reg_state = readback_register(_gpio_control);
    float adc_code_delta = 819;
    if (bitRead(_gpio_ctrl_reg_state,5))
    {
        adc_code_delta = 409.5;
    }
    // Ensure the ADC reference is set to internal as well (else Vin is unknown) using _power_control
    uint16_t _power_ctrl_reg_state = readback_register(_power_control);
    if (bitRead(_power_ctrl_reg_state,9) == 0)
    {
        //the V-ref internal is not enabled: Not sure what the value of V-ref is...
        //return the raw ADC value and let the user work out what to do:
        return adc_code;
    }
    return 25.0 + ((adc_code - adc_code_delta) / (1.327)); //as per page 25 of the datasheet
}

ad5592_state ad5592::get_state(int io)
{
    return _gpio_state[io];
}

float ad5592::read_state(int io)
{
    // Reading an IO depends on which mode it is in:
    if (_gpio_state[io] == ad5592_state::dac_mode) {
        // DAC mode reads a special register set:
        uint16_t data = 0;
        data = bitSet(data,5);
        data = bitSet(data,4);
        data = data | (io & 0x0007);
        uint16_t returned_data = rw_spi_command(tx_packet_generator(_dac_readback,data),20);
        //check the return data address is correct (bit 14-12 should match IO)
        int returned_io = (returned_data >> 12) & 0x0007;
        if (io != returned_io) {
            Serial.print("Returned DAC address error. Expected ");
            Serial.print(io);
            Serial.print(" but received ");
            Serial.print(returned_io);
            Serial.println(".");
            return returned_data; //return everything.
        }
        return convert_dac_to_analog(returned_data);
    }
    else if (_gpio_state[io] == ad5592_state::adc_mode) {
        // ADC mode
        uint16_t data = 0;
        data = bitSet(data,io); //select the IO to readback mode
        uint16_t returned_data = rw_spi_command(tx_packet_generator(_adc_sequence,data),25);
        returned_data = read_spi_command(); //ADC takes one extra read to get out
        //Check the returned register is the correct one 
        int returned_io = (returned_data >> 12) & 0x000F;
        if (io != returned_io) {
            Serial.print("Returned ADC address error. Expected ");
            Serial.print(io);
            Serial.print(" but received ");
            Serial.print(returned_io);
            Serial.println(".");
            return returned_data; //return everything: let the user decide what it means.
        }
        return convert_dac_to_analog(returned_data);
    }
    else if (_gpio_state[io] == ad5592_state::gpo_mode) {
        // GPO mode: read output state
        // read the gpi_write register and return the bit.
        uint16_t returned_data = readback_register(_gpo_write);
        return bitRead(returned_data,io);
    }
    else if (_gpio_state[io] == ad5592_state::gpi_mode) {
        // GIO mode: read as input
        // enable GPIO readback mode and set the IO bit to readback
        uint16_t data = 0;
        data = bitSet(data,10); //select readback mode: page 37 of datasheet
        data = bitSet(data,io); //select the IO to readback mode
        uint16_t returned_data = rw_spi_command(tx_packet_generator(_gpio_read,data),1);
        return bitRead(returned_data,io);
    }
    else if (_gpio_state[io] == ad5592_state::z_mode) {
        // z mode: read as a GPI? - its not clear what to do.
        // enable GPIO readback mode and set the IO bit to readback
        uint16_t data = 0;
        data = bitSet(data,10); //select readback mode: page 37 of datasheet
        data = bitSet(data,io); //select the IO to readback mode
        uint16_t returned_data = rw_spi_command(tx_packet_generator(_gpio_read,data),1);
        return bitRead(returned_data,io);
    }
    else if (_gpio_state[io] == ad5592_state::off_mode) {
        // OFF mode
        return 0;
    }
    else if (_gpio_state[io] == ad5592_state::unknown) {
        // Unknown mode
        return -1;
    }
    return -88;
}

float ad5592::write_state(int io, float val)
{
    // Set an IO depends on which mode it is in:
    if (_gpio_state[io] == ad5592_state::dac_mode) {
        // convert float input voltage to a 12-bit number:
        // DAC mode writes a special register set:
        uint16_t data = convert_analog_to_dac(val);
        data = data | ((io << 12) & 0x7000); //the address is bit 12-14: attach it do the data
        uint16_t returned_data = rw_spi_command(tx_packet_generator(_dac_write,data),20);
        //check the return data address is correct (bit 14-12 should match IO)
        int returned_io = (returned_data >> 12) & 0x0007;
        if (io != returned_io) {
            Serial.print("Returned DAC address error. Expected ");
            Serial.print(io);
            Serial.print(" but received ");
            Serial.print(returned_io);
            Serial.println(".");
            return returned_data; //return everything.
        }
        return read_state(io); //read the actual register info and return it.
    }
    else if (_gpio_state[io] == ad5592_state::adc_mode) {
        // ADC mode
        // cannot write to an ADC, so just return the result.
        return read_state(io);
    }
    else if (_gpio_state[io] == ad5592_state::gpo_mode) {
        // write to the IO... before you do that make sure you read the other IO first
        // so as to not change the state of the other output pins
        // read which pins were GPO:
        uint16_t gpo_config = readback_register(_gpo_config);
        // read the state of the GPO pins:
        uint16_t gpo_state = readback_register(_gpo_write);
        // mask GPO state with GPO config to get the current configuration pattern
        uint16_t data = gpo_state & gpo_config;
        // set the IO that we need in the data:
        bitWrite(data,io,(int)val);
        // send the data
        write_spi_command(tx_packet_generator(_gpo_write,data));
        // return the IO state
        return read_state(io);
    }
    else if (_gpio_state[io] == ad5592_state::gpi_mode) {
        // GIO mode:
        // cannot write to an input, so just return the result.
        return read_state(io);
    }
    else if (_gpio_state[io] == ad5592_state::z_mode) {
        // cannot write to an tri-state, so just return the result.
        return read_state(io);
    }
    else if (_gpio_state[io] == ad5592_state::off_mode) {
        // OFF mode
        return 0;
    }
    else if (_gpio_state[io] == ad5592_state::unknown) {
        // Unknown mode
        return -1;
    }
    return -88;
}

void ad5592::set_state(int io, ad5592_state state)
{
    // Possible states are listed in the ad5592 struct.
    // Clear the current IO state (ie, unknown mode):
    if (_gpio_state[io] == ad5592_state::adc_mode) 
    {
        //Clear the state:
        set_pin_mode(_adc_config,io,0);
    }
    else if (_gpio_state[io] == ad5592_state::dac_mode) 
    {
        //Clear the state:
        set_pin_mode(_dac_config,io,0);
    }
    else if (_gpio_state[io] == ad5592_state::gpi_mode) 
    {
        //Clear the state:
        set_pin_mode(_gpio_read,io,0);
    }
    else if (_gpio_state[io] == ad5592_state::gpo_mode) 
    {
        //Clear the state:
        set_pin_mode(_gpo_config,io,0);
    }
    else if (_gpio_state[io] == ad5592_state::z_mode) 
    {
        //Clear the state:
        set_pin_mode(_gpio_high_z,io,0);
    }
    else if (_gpio_state[io] == ad5592_state::off_mode) 
    {
        //Clear the state:
        set_pin_mode(_pulldown_config,io,0);
    }
    _gpio_state[io] = ad5592_state::unknown;
    // Set the new state:
    if (state == ad5592_state::adc_mode) 
    {
        //Set the state:
        set_pin_mode(_adc_config,io,1);
        _gpio_state[io] = ad5592_state::adc_mode;
    }
    else if (state == ad5592_state::dac_mode) 
    {
        //Set the state:
        set_pin_mode(_dac_config,io,1);
        _gpio_state[io] = ad5592_state::dac_mode;
    }
    else if (state == ad5592_state::gpi_mode) 
    {
        //Set the state:
        set_pin_mode(_gpio_read,io,1);
        _gpio_state[io] = ad5592_state::gpi_mode;
    }
    else if (state == ad5592_state::gpo_mode) 
    {
        //Set the state:
        set_pin_mode(_gpo_config,io,1);
        _gpio_state[io] = ad5592_state::gpo_mode;
    }
    else if (state == ad5592_state::z_mode) 
    {
        //Set the state:
        set_pin_mode(_gpio_high_z,io,1);
        _gpio_state[io] = ad5592_state::z_mode;
    }
    // don't allow states to get to the unknown mode: stop them here
    else
    {
        //Set the state:
        set_pin_mode(_pulldown_config,io,1);
        _gpio_state[io] = ad5592_state::off_mode;
    }
    return;
}

int ad5592::set_gpo_mode(int io, ad5592_gio_mode mode)
{
    // only do this if the IO mode is set to GPO:
    if (_gpio_state[io] == ad5592_state::gpo_mode)
    {
        // change the mode:
        if (mode == ad5592_gio_mode::open_drain)
        {
            set_pin_mode(_gpo_open_drain,io,1);
        }
        else 
        {
            set_pin_mode(_gpo_open_drain,io,0);
        }
    }
    return -88;
}

ad5592_gio_mode ad5592::get_mode(int io){
    int data = readback_register(_gpo_open_drain);
    if (bitRead(data,io) == 1) {
        return ad5592_gio_mode::open_drain;
    }
    else {
        return ad5592_gio_mode::push_pull;
    }
}

float ad5592::get_external_vref_value(void)
{
    return _external_vref;
}

void ad5592::set_external_vref_value(float val)
{
    _external_vref = val;
}
