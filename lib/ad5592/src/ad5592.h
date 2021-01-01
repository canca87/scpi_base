/* AD5592.h - Library to manage communications with an AD5592 IC
 * Created by Adam Chrimes, December 6 2020.
 * 
 * Assumptions so far:
 * - Teensy 3.2 microcontroller
 * - SPI1 with default pins (11,12,13) used for comms
 * - Any other pin for CS/SYNC
 * - 16-bit SPI transactions
 */

#ifndef ad5592_h
#define ad5592_h

#include <Arduino.h>
#include <SPI.h>

// defining register addresses here
#define _nop              0b0000000000000000
#define _dac_readback     0b0000100000000000
#define _adc_sequence     0b0001000000000000
#define _gpio_control     0b0001100000000000
#define _adc_config       0b0010000000000000
#define _dac_config       0b0010100000000000
#define _pulldown_config  0b0011000000000000
#define _readback_ldac    0b0011100000000000
#define _gpo_config       0b0100000000000000
#define _gpo_write        0b0100100000000000
#define _gpio_read        0b0101000000000000
#define _power_control    0b0101100000000000
#define _gpo_open_drain   0b0110000000000000
#define _gpio_high_z      0b0110100000000000
#define _reset            0b0111110110101100
#define _dac_write        0b1000000000000000

enum ad5592_state
{
    adc_mode,
    dac_mode,
    gpi_mode,
    gpo_mode,
    z_mode,
    off_mode, //off is the equivalent of pull-down mode.
    unknown //there is one other state: when nothing else is set. Not sure what happens then.
};
enum ad5592_gio_mode
{
    push_pull,
    open_drain
};

class ad5592
{
    private:
        //datasheet gives read timing of 50ns (20 MHz)
        SPISettings read_spi_settings = SPISettings(20000000,MSBFIRST,SPI_MODE1);
        //datasheet gives read timing of 20ns (50 MHz)
        SPISettings write_spi_settings = SPISettings(50000000,MSBFIRST,SPI_MODE1);
        // sync pin can be any IO (equivalent to CS)
        int _sync_pin;
        // gpio state is important to keep track of (ie DAC/ADC/GPIO/OFF)
        ad5592_state _gpio_state[8];
        // external Vref:
        float _external_vref = 0;
        // function descriptors:
        uint16_t tx_packet_generator(uint16_t command, uint16_t data);
        void write_spi_command(uint16_t data);
        uint16_t rw_spi_command(uint16_t data,int delay = -1);
        uint16_t read_spi_command(void);
        uint16_t readback_register(uint16_t register_id);
        float convert_dac_to_analog(uint16_t val);
        uint16_t convert_analog_to_dac(float val);
        void set_pin_mode(uint16_t address, byte io, byte state);
    public:
        void begin(int sync_pin);
        float read_temperature(void);
        ad5592_state get_state(int io);
        float read_state(int io);
        float write_state(int io, float val);
        void set_state(int io, ad5592_state state);
        int set_gpo_mode(int io, ad5592_gio_mode mode);
        ad5592_gio_mode get_mode(int io);
        float get_external_vref_value(void);
        void set_external_vref_value(float val);
};

#endif