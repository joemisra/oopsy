#include "DAC7554.h"
#include "daisy.h"
#include "../daisy_dpt.h"

// Driver for DAC7554 based on code from Making Sound Machines 
// Based on Code from Westlicht Performer   - https://westlicht.github.io/performer/
// Port for Daisy by Making Sound Machines  - https://github.com/makingsoundmachines

// Pins on Plinky Expander -
// 16   RX       TX     15
// 14   GND      CS     13
// 12   GND      +12V   11
// 10   MISO     CLK    9
// 8    MOSI     -12V   7
// 6    GND      +5V    5
// 4    DM       DP     3
// 2    +3v3     +3v3   1

// Pins on Daisy Seed used in this configuration:
// D7   CS
// D8   SCLK
// D9   MISO
// D10  MOSI

// Pins on Daisy Patch SM used in this configuration:
// D1   CS
// D10   SCLK
// D9   MISO
// D8  MOSI

// Usage:
// Dac7554 dac;
// dac.Init();
// dac.Set(1, [0-4095]);
// dac.Set(2, [0-4095]);
// dac.Set(3, [0-4095]);
// dac.Set(4, [0-4095]);

// dac.Write();

using namespace daisy;
using namespace dpt;
using namespace std;

#define MAX_DAC7554_BUF_SIZE 256
uint8_t DMA_BUFFER_MEM_SECTION dac7554buf[MAX_DAC7554_BUF_SIZE];
uint8_t DMA_BUFFER_MEM_SECTION dac7554buf_count = 0;

typedef struct
{
    uint8_t Initialized;
} Dac7554_t;

static SpiHandle         h_spi;
static GPIO              pin_sync;
static Dac7554_t         Dac7554_;
static SpiHandle::Config spi_config;

// DMA completion callback for chaining transfers
// In v4.0.0, we can only use the completion callback (no start callback)
void TxCpltCallback(void* context, daisy::SpiHandle::Result result) {
    // If we need to send more data, chain the next transfer here
    // For now, just reset the counter - the main transfer sends all 8 bytes at once
    dac7554buf_count = 0;
}

void Dac7554::Init()
{
    // Initialize PIO - possibly not needed?
    pin_sync.Init(DPT::D1, GPIO::Mode::OUTPUT);

    // Initialize SPI

    spi_config.periph         = SpiHandle::Config::Peripheral::SPI_2;
    spi_config.mode           = SpiHandle::Config::Mode::MASTER;
    spi_config.direction      = SpiHandle::Config::Direction::TWO_LINES_TX_ONLY;
    spi_config.datasize       = 8;
    spi_config.clock_polarity = SpiHandle::Config::ClockPolarity::HIGH;
    spi_config.clock_phase    = SpiHandle::Config::ClockPhase::ONE_EDGE;
    spi_config.nss            = SpiHandle::Config::NSS::HARD_OUTPUT;
    spi_config.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_2;

    spi_config.pin_config.sclk = DPT::D10;
    spi_config.pin_config.mosi = DPT::D9;
    spi_config.pin_config.miso = DPT::D8;
    spi_config.pin_config.nss  = DPT::D1;

    h_spi.Init(spi_config);

    Dac7554_.Initialized = 1;
}

void Dac7554::Write(uint16_t gogo[4])
{
    //uint16_t cmd = (2 << 14) | (address << 12) | data;

    for(int i = 0; i < 4; i++)
        _values[i] = gogo[i];
    dac_ready = true;
}

void Dac7554::WriteDac7554()
{
    // Build buffer with all 4 channels (8 bytes total: 2 bytes per channel)
    for(int i=0; i<4; i++) {
        uint8_t chan = i;
        uint16_t cmd = (2 << 14) | (chan << 12) | _values[chan];

        dac7554buf[2*i] = (cmd >> 8) & 0xff;
        dac7554buf[2*i+1] = cmd & 0xff;
    }
    
    // libDaisy v4.0.0 API: DmaTransmit(buff, size, callback, context)
    // Send all 8 bytes (4 channels) in a single DMA transfer
    // The callback can be used to chain additional transfers if needed
    h_spi.DmaTransmit(dac7554buf, 8, TxCpltCallback, nullptr);

    // Alternative: If you need to send channels separately for timing reasons,
    // you can chain them in the callback, but sending all at once is more efficient
    // h_spi.DmaTransmit(dac7554buf, 8, nullptr, nullptr);  // No callback if not needed
}

void Dac7554::Clear(void* context, int result) {
    for(int i = 0; i < 4; i++)
        _values[i] = 0;

}

void Dac7554::Reset()
{
}

void Dac7554::SetInternalRef(bool enabled)
{
}

void Dac7554::SetClearCode(ClearCode code)
{
}