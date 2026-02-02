#include "daisy_dpt.h"

#include "dev/DAC7554.h"

#include "util/hal_map.h"
#include "sys/system.h"
#include "per/gpio.h"
#include "per/tim.h"

#include <vector>

#define DSY_MIN(in, mn) (in < mn ? in : mn)
#define DSY_MAX(in, mx) (in > mx ? in : mx)
#define DSY_CLAMP(in, mn, mx) (DSY_MIN(DSY_MAX(in, mn), mx))

// #define EXTERNAL_SDRAM_SECTION __attribute__((section(".sdram_bss")))
// uint8_t EXTERNAL_SDRAM_SECTION buff[1024];

namespace daisy
{
    namespace dpt
    {
        /** Const definitions */
        static constexpr Pin DUMMYPIN = Pin(PORTX, 0);
        static constexpr Pin PIN_ADC_CTRL_1 = Pin(PORTA, 3);
        static constexpr Pin PIN_ADC_CTRL_2 = Pin(PORTA, 6);
        static constexpr Pin PIN_ADC_CTRL_3 = Pin(PORTA, 2);
        static constexpr Pin PIN_ADC_CTRL_4 = Pin(PORTA, 7);
        static constexpr Pin PIN_ADC_CTRL_5 = Pin(PORTB, 1);
        static constexpr Pin PIN_ADC_CTRL_6 = Pin(PORTC, 4);
        static constexpr Pin PIN_ADC_CTRL_7 = Pin(PORTC, 0);
        static constexpr Pin PIN_ADC_CTRL_8 = Pin(PORTC, 1);
        static constexpr Pin PIN_ADC_CTRL_9 = Pin(PORTA, 1);
        static constexpr Pin PIN_ADC_CTRL_10 = Pin(PORTA, 0);
        static constexpr Pin PIN_ADC_CTRL_11 = Pin(PORTC, 3);
        static constexpr Pin PIN_ADC_CTRL_12 = Pin(PORTC, 2);
        static constexpr Pin PIN_USER_LED = Pin(PORTC, 7);

        const Pin kPinMap[4][10] = {
            /** Header Bank A */
            {
                DUMMYPIN,       /**< A1  - -12V Power Input */
                Pin(PORTA, 1),  /**< A2  - UART1 Rx */
                Pin(PORTA, 0),  /**< A3  - UART1 Tx */
                DUMMYPIN,       /**< A4  - GND */
                DUMMYPIN,       /**< A5  - +12V Power Input */
                DUMMYPIN,       /**< A6  - +5V Power Output */
                DUMMYPIN,       /**< A7  - GND */
                Pin(PORTB, 14), /**< A8  - USB DM */
                Pin(PORTB, 15), /**< A9  - USB DP */
                DUMMYPIN,       /**< A10 - +3V3 Power Output */
            },
            /** Header Bank B */
            {
                DUMMYPIN,       /**< B1  - Audio Out Right */
                DUMMYPIN,       /**< B2  - Audio Out Left*/
                DUMMYPIN,       /**< B3  - Audio In Right */
                DUMMYPIN,       /**< B4  - Audio In Left */
                Pin(PORTC, 13), /**< B5  - GATE OUT 1 */
                Pin(PORTC, 14), /**< B6  - GATE OUT 2 */
                Pin(PORTB, 8),  /**< B7  - I2C1 SCL */
                Pin(PORTB, 9),  /**< B8  - I2C1 SDA */
                Pin(PORTG, 14), /**< B9  - GATE IN 2 */
                Pin(PORTG, 13), /**< B10 - GATE IN 1 */
            },
            /** Header Bank C */
            {
                Pin(PORTA, 5),  /**< C1  - CV Out 2 */
                PIN_ADC_CTRL_4, /**< C2  - CV In 4 */
                PIN_ADC_CTRL_3, /**< C3  - CV In 3 */
                PIN_ADC_CTRL_2, /**< C4  - CV In 2 */
                PIN_ADC_CTRL_1, /**< C5  - CV In 1 */
                PIN_ADC_CTRL_5, /**< C6  - CV In 5 */
                PIN_ADC_CTRL_6, /**< C7  - CV In 6 */
                PIN_ADC_CTRL_7, /**< C8  - CV In 7 */
                PIN_ADC_CTRL_8, /**< C9  - CV In 8 */
                Pin(PORTA, 4),  /**< C10 - CV Out 1 */
            },
            /** Header Bank D */
            {
                Pin(PORTB, 4),  /**< D1  - SPI2 CS */
                Pin(PORTC, 11), /**< D2  - SDMMC D3 */
                Pin(PORTC, 10), /**< D3  - SDMMC D2*/
                Pin(PORTC, 9),  /**< D4  - SDMMC D1*/
                Pin(PORTC, 8),  /**< D5  - SDMMC D0 */
                Pin(PORTC, 12), /**< D6  - SDMMC CK */
                Pin(PORTD, 2),  /**< D7  - SDMMC CMD */
                Pin(PORTC, 2),  /**< D8  - SPI2 MISO */
                Pin(PORTC, 3),  /**< D9  - SPI2 MOSI */
                Pin(PORTD, 3),  /**< D10 - SPI2 SCK  */
            },
        };

        // constexpr static member definitions (required for ODR-use in C++14)
        constexpr Pin DPT::A1;
        constexpr Pin DPT::A2;
        constexpr Pin DPT::A3;
        constexpr Pin DPT::A4;
        constexpr Pin DPT::A5;
        constexpr Pin DPT::A6;
        constexpr Pin DPT::A7;
        constexpr Pin DPT::A8;
        constexpr Pin DPT::A9;
        constexpr Pin DPT::A10;
        constexpr Pin DPT::B1;
        constexpr Pin DPT::B2;
        constexpr Pin DPT::B3;
        constexpr Pin DPT::B4;
        constexpr Pin DPT::B5;
        constexpr Pin DPT::B6;
        constexpr Pin DPT::B7;
        constexpr Pin DPT::B8;
        constexpr Pin DPT::B9;
        constexpr Pin DPT::B10;
        constexpr Pin DPT::C1;
        constexpr Pin DPT::C2;
        constexpr Pin DPT::C3;
        constexpr Pin DPT::C4;
        constexpr Pin DPT::C5;
        constexpr Pin DPT::C6;
        constexpr Pin DPT::C7;
        constexpr Pin DPT::C8;
        constexpr Pin DPT::C9;
        constexpr Pin DPT::C10;
        constexpr Pin DPT::D1;
        constexpr Pin DPT::D2;
        constexpr Pin DPT::D3;
        constexpr Pin DPT::D4;
        constexpr Pin DPT::D5;
        constexpr Pin DPT::D6;
        constexpr Pin DPT::D7;
        constexpr Pin DPT::D8;
        constexpr Pin DPT::D9;
        constexpr Pin DPT::D10;

        /** outside of class static buffer(s) for DMA access */
        uint16_t DMA_BUFFER_MEM_SECTION dsy_patch_sm_dac_buffer[2][48];

        class DPT::Impl
        {
        public:
            Impl()
            {
                dac_running_ = false;
                dac_buffer_size_ = 48;
                dac_output_[0] = 0;
                dac_output_[1] = 0;
                internal_dac_buffer_[0] = dsy_patch_sm_dac_buffer[0];
                internal_dac_buffer_[1] = dsy_patch_sm_dac_buffer[1];
            }

            void InitDac();

            void StartDac(DacHandle::DacCallback callback);

            void StopDac();

            static void InternalDacCallback(uint16_t **output, size_t size);

            /** Based on a -7V to +7V bipolar output with a 0-4095 12-bit DAC (inverted) */
            static inline uint16_t VoltageToCode(float input)
            {
                float pre = abs(((input + 7.f) / 14.f * 4095.f) - 4095.f);
                if (pre > 4095.f)
                    pre = 4095.f;
                else if (pre < 0.f)
                    pre = 0.f;
                return (uint16_t)pre;
            }

            inline void WriteCvOut(int channel, float voltage, bool raw)
            {
                if (channel == 0 || channel == 1)
                    dac_output_[0] = raw ? (uint16_t)voltage : VoltageToCode(voltage);
                if (channel == 0 || channel == 2)
                    dac_output_[1] = raw ? (uint16_t)voltage : VoltageToCode(voltage);
            }

            size_t dac_buffer_size_;
            uint16_t *internal_dac_buffer_[2];
            uint16_t dac_output_[2];
            DacHandle dac_;

        private:
            bool dac_running_;
        };

        /** Static Local Object */
        static DPT::Impl patch_sm_hw;

        /** Impl function definintions */

        void DPT::Impl::InitDac()
        {
            DacHandle::Config dac_config;
            dac_config.mode = DacHandle::Mode::DMA;
            dac_config.bitdepth = DacHandle::BitDepth::
                BITS_12; /**< Sets the output value to 0-4095 */
            dac_config.chn = DacHandle::Channel::BOTH;
            dac_config.buff_state = DacHandle::BufferState::ENABLED;
            dac_config.target_samplerate = 48000;
            dac_.Init(dac_config);
        }

        void DPT::Impl::StartDac(DacHandle::DacCallback callback)
        {
            if (dac_running_)
                dac_.Stop();
            dac_.Start(internal_dac_buffer_[0],
                       internal_dac_buffer_[1],
                       dac_buffer_size_,
                       callback == nullptr ? InternalDacCallback : callback);
            dac_running_ = true;
        }

        void DPT::Impl::StopDac()
        {
            dac_.Stop();
            dac_running_ = false;
        }

        void DPT::Impl::InternalDacCallback(uint16_t **output, size_t size)
        {
            /** We could add some smoothing, interp, or something to make this a bit less waste-y */
            // std::fill(&output[0][0], &output[0][size], patch_sm_hw.dac_output_[0]);
            // std::fill(&output[1][1], &output[1][size], patch_sm_hw.dac_output_[1]);
            for (size_t i = 0; i < size; i++)
            {
                output[0][i] = patch_sm_hw.dac_output_[0];
                output[1][i] = patch_sm_hw.dac_output_[1];
            }
        }

        /** Actual DPT implementation
         *  With the pimpl model in place, we can/should probably
         *  move the rest of the implementation to the Impl class
         */

        void DPT::Init()
        {
            /** Assign pimpl pointer */
            pimpl_ = &patch_sm_hw;
            /** Initialize the MCU and clock tree */
            System::Config syscfg;
            syscfg.Boost();

            auto memory = System::GetProgramMemoryRegion();
            if (memory != System::MemoryRegion::INTERNAL_FLASH)
                syscfg.skip_clocks = true;

            system.Init(syscfg);
            /** Memories */
            if (memory == System::MemoryRegion::INTERNAL_FLASH)
            {
                /** FMC SDRAM */
                sdram.Init();
            }
            if (memory != System::MemoryRegion::QSPI)
            {
                /** QUADSPI FLASH */
                QSPIHandle::Config qspi_config;
                qspi_config.device = QSPIHandle::Config::Device::IS25LP064A;
                qspi_config.mode = QSPIHandle::Config::Mode::MEMORY_MAPPED;
                qspi_config.pin_config.io0 = Pin(PORTF, 8);
                qspi_config.pin_config.io1 = Pin(PORTF, 9);
                qspi_config.pin_config.io2 = Pin(PORTF, 7);
                qspi_config.pin_config.io3 = Pin(PORTF, 6);
                qspi_config.pin_config.clk = Pin(PORTF, 10);
                qspi_config.pin_config.ncs = Pin(PORTG, 6);
                qspi.Init(qspi_config);
            }
            /** Audio */
            // Audio Init
            SaiHandle::Config sai_config;
            sai_config.periph = SaiHandle::Config::Peripheral::SAI_1;
            sai_config.sr = SaiHandle::Config::SampleRate::SAI_48KHZ;
            sai_config.bit_depth = SaiHandle::Config::BitDepth::SAI_24BIT;
            sai_config.a_sync = SaiHandle::Config::Sync::MASTER;
            sai_config.b_sync = SaiHandle::Config::Sync::SLAVE;
            sai_config.a_dir = SaiHandle::Config::Direction::RECEIVE;
            sai_config.b_dir = SaiHandle::Config::Direction::TRANSMIT;
            sai_config.pin_config.fs = Pin(PORTE, 4);
            sai_config.pin_config.mclk = Pin(PORTE, 2);
            sai_config.pin_config.sck = Pin(PORTE, 5);
            sai_config.pin_config.sa = Pin(PORTE, 6);
            sai_config.pin_config.sb = Pin(PORTE, 3);
            SaiHandle sai_1_handle;
            sai_1_handle.Init(sai_config);
            I2CHandle::Config i2c_cfg;
            i2c_cfg.periph = I2CHandle::Config::Peripheral::I2C_2;
            i2c_cfg.mode = I2CHandle::Config::Mode::I2C_MASTER;
            i2c_cfg.speed = I2CHandle::Config::Speed::I2C_400KHZ;
            i2c_cfg.pin_config.scl = Pin(PORTB, 10);
            i2c_cfg.pin_config.sda = Pin(PORTB, 11);
            I2CHandle i2c2;
            i2c2.Init(i2c_cfg);
            codec.Init(i2c2);

            AudioHandle::Config audio_config;
            audio_config.blocksize = 48;
            audio_config.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
            audio_config.postgain = 1.f;
            audio.Init(audio_config, sai_1_handle);
            callback_rate_ = AudioSampleRate() / AudioBlockSize();

            /** ADC Init */
            AdcChannelConfig adc_config[ADC_LAST];
            /** Order of pins to match enum expectations */
            Pin adc_pins[] = {
                PIN_ADC_CTRL_1,
                PIN_ADC_CTRL_2,
                PIN_ADC_CTRL_3,
                PIN_ADC_CTRL_4,
                PIN_ADC_CTRL_8,
                PIN_ADC_CTRL_7,
                PIN_ADC_CTRL_5,
                PIN_ADC_CTRL_6,
                PIN_ADC_CTRL_9,
                PIN_ADC_CTRL_10,
                PIN_ADC_CTRL_11,
                PIN_ADC_CTRL_12,
            };

            for (int i = 0; i < ADC_LAST; i++)
            {
                adc_config[i].InitSingle(adc_pins[i]);
            }
            adc.Init(adc_config, ADC_LAST);
            /** Control Init */
            for (size_t i = 0; i < ADC_LAST; i++)
            {
                if (i < ADC_9)
                    controls[i].InitBipolarCv(adc.GetPtr(i), callback_rate_);
                else
                    controls[i].Init(adc.GetPtr(i), callback_rate_);
            }

            /** Fixed-function Digital I/O */
            user_led.Init(PIN_USER_LED, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);
            // libdaisy v4: GateIn::Init takes dsy_gpio_pin*, needs cast from Pin
            gate_in_1.Init((dsy_gpio_pin *)&B10);
            gate_in_2.Init((dsy_gpio_pin *)&B9);
            gate_in_1_trig = false;
            gate_in_2_trig = false;

            // B5 = PORTC, 13 = GATE OUT 1
            // B6 = PORTC, 14 = GATE OUT 2
            gate_out_1.mode = DSY_GPIO_MODE_OUTPUT_PP;
            gate_out_1.pull = DSY_GPIO_NOPULL;
            gate_out_1.pin = B5;
            dsy_gpio_init(&gate_out_1);

            gate_out_2.mode = DSY_GPIO_MODE_OUTPUT_PP;
            gate_out_2.pull = DSY_GPIO_NOPULL;
            gate_out_2.pin = B6;
            dsy_gpio_init(&gate_out_2);

            // Startup blink test to verify GPIO initialization
            // Blink pattern: gate_out_1 ON, gate_out_2 OFF -> both OFF -> gate_out_1 OFF, gate_out_2 ON -> both OFF (repeat 3x)
            for (int i = 0; i < 3; i++)
            {
                dsy_gpio_write(&gate_out_1, true);
                dsy_gpio_write(&gate_out_2, false);
                System::Delay(100);
                dsy_gpio_write(&gate_out_1, false);
                dsy_gpio_write(&gate_out_2, false);
                System::Delay(50);
                dsy_gpio_write(&gate_out_1, false);
                dsy_gpio_write(&gate_out_2, true);
                System::Delay(100);
                dsy_gpio_write(&gate_out_1, false);
                dsy_gpio_write(&gate_out_2, false);
                System::Delay(50);
            }
            // Ensure gate outputs start LOW after blink test
            dsy_gpio_write(&gate_out_1, false);
            dsy_gpio_write(&gate_out_2, false);

            // Init MIDI i/o
            if (ENABLE_MIDI)
            {
                InitMidi();
            }
            else if (ENABLE_E4)
            {
                // These two GPIO pins will conflict w/ MIDI
                clicker1.Init(A8, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);
                clicker2.Init(A9, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);
            }

            pimpl_->InitDac();

            dac_exp.Init();

            /** Start any background stuff */
            StartAdc();
            StartDac();

            codec.SetAdcAttenuation(215);
            /** Init Timer */
        }

        void DPT::InitTimer(daisy::dpt_internal::PeriodElapsedCallback cb, void *data)
        {
            // libdaisy v4: TimerHandle doesn't have PeriodElapsedCallback, enable_irq, or SetCallback
            // Timer callbacks are not supported in this version - stub out for API compatibility
            (void)cb;
            (void)data;

            // TIM5->PSC = 1;
            // TIM5->DIER = TIM_DIER_UIE;
            // tim5.Instance = ((TIM_TypeDef *)TIM5_BASE);
            // tim5.Init.CounterMode = TIM_COUNTERMODE_UP;
            // tim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
            // tim5.Init.Prescaler = 1;

            // HAL_TIM_Base_Init(&tim5);
        }

        void DPT::InitMidi()
        {
            /*
            MidiUsbHandler::Config midi_cfg;
            midi_cfg.transport_config.periph = MidiUsbTransport::Config::EXTERNAL;
            usb_midi.Init(midi_cfg);
            */

            // This is using USART1 here
            MidiUartHandler::Config midi_config;
            midi_config.transport_config.rx = DPT::A9;
            midi_config.transport_config.tx = DPT::A8;
            midi.Init(midi_config);
        }

        void DPT::StartAudio(AudioHandle::AudioCallback cb)
        {
            audio.Start(cb);
        }

        void DPT::StartAudio(AudioHandle::InterleavingAudioCallback cb)
        {
            audio.Start(cb);
        }

        void DPT::ChangeAudioCallback(AudioHandle::AudioCallback cb)
        {
            audio.ChangeCallback(cb);
        }

        void
        DPT::ChangeAudioCallback(AudioHandle::InterleavingAudioCallback cb)
        {
            audio.ChangeCallback(cb);
        }

        void DPT::StopAudio() { audio.Stop(); }

        void DPT::SetAudioBlockSize(size_t size)
        {
            audio.SetBlockSize(size);
            callback_rate_ = AudioSampleRate() / AudioBlockSize();
        }

        void DPT::SetAudioSampleRate(float sr)
        {
            SaiHandle::Config::SampleRate sai_sr;
            switch (int(sr))
            {
            case 8000:
                sai_sr = SaiHandle::Config::SampleRate::SAI_8KHZ;
                break;
            case 16000:
                sai_sr = SaiHandle::Config::SampleRate::SAI_16KHZ;
                break;
            case 32000:
                sai_sr = SaiHandle::Config::SampleRate::SAI_32KHZ;
                break;
            case 48000:
                sai_sr = SaiHandle::Config::SampleRate::SAI_48KHZ;
                break;
            case 96000:
                sai_sr = SaiHandle::Config::SampleRate::SAI_96KHZ;
                break;
            default:
                sai_sr = SaiHandle::Config::SampleRate::SAI_48KHZ;
                break;
            }
            audio.SetSampleRate(sai_sr);
            callback_rate_ = AudioSampleRate() / AudioBlockSize();
        }

        void
        DPT::SetAudioSampleRate(SaiHandle::Config::SampleRate sample_rate)
        {
            audio.SetSampleRate(sample_rate);
            callback_rate_ = AudioSampleRate() / AudioBlockSize();
        }

        size_t DPT::AudioBlockSize()
        {
            return audio.GetConfig().blocksize;
        }

        float DPT::AudioSampleRate() { return audio.GetSampleRate(); }

        float DPT::AudioCallbackRate() { return callback_rate_; }

        void DPT::StartAdc() { adc.Start(); }

        void DPT::StopAdc() { adc.Stop(); }

        void DPT::ProcessAnalogControls()
        {
            for (int i = 0; i < ADC_LAST; i++)
            {
                controls[i].Process();
            }
        }

        void DPT::ProcessDigitalControls()
        {
            // Update gate trigger state for edge detection
            gate_in_1_trig = gate_in_1.Trig();
            gate_in_2_trig = gate_in_2.Trig();
        }

        float DPT::GetAdcValue(int idx) { return controls[idx].Value(); }

        Pin DPT::GetPin(const PinBank bank, const int idx)
        {
            if (idx <= 0 || idx > 10)
                return DUMMYPIN;
            else
                return kPinMap[static_cast<int>(bank)][idx - 1];
        }

        void DPT::StartDac(DacHandle::DacCallback callback)
        {
            pimpl_->StartDac(callback);
        }

        void DPT::StartDacExp(DacHandle::DacCallback callback)
        {
            // Stub for Oopsy compatibility - delegates to StartDac
            StartDac(callback);
        }

        void DPT::StopDac() { pimpl_->StopDac(); }

        void DPT::WriteCvOut(const int channel, float voltage, bool raw)
        {
            pimpl_->WriteCvOut(channel, voltage, raw);
        }

        // Scale -7v to 7v
        uint16_t DPT::VoltageToCodeExp(float input)
        {
            // Outputs are inverted, so have to flip the literal voltage
            float pre = DSY_CLAMP(4095.f - ((input + 7.f) / 14.f * 4095.f), 0, 4095);

            if (pre > 4095.f)
                pre = 4095.f;
            else if (pre < 0.f)
                pre = 0.f;

            return (uint16_t)pre;
        }

        void DPT::WriteCvOutExp(float a, float b, float c, float d, bool raw)
        {
            uint16_t gogo[4];

            // Outputs are inverted, so have to flip the inputs
            gogo[0] = raw ? 4095 - (uint16_t)a : VoltageToCodeExp(a);
            gogo[1] = raw ? 4095 - (uint16_t)b : VoltageToCodeExp(b);
            gogo[2] = raw ? 4095 - (uint16_t)c : VoltageToCodeExp(c);
            gogo[3] = raw ? 4095 - (uint16_t)d : VoltageToCodeExp(d);

            dac_exp.Write(gogo);
            dac_exp.WriteDac7554();
        }

        void DPT::SetLed(bool state) { user_led.Write(state); }

        bool DPT::ValidateSDRAM()
        {
            uint32_t *sdramptr = (uint32_t *)0xc0000000;
            uint32_t size_in_words = 16777216;
            uint32_t testval = 0xdeadbeef;
            uint32_t num_failed = 0;
            /** Write test val */
            for (uint32_t i = 0; i < size_in_words; i++)
            {
                uint32_t *word = sdramptr + i;
                *word = testval;
            }
            /** Compare written */
            for (uint32_t i = 0; i < size_in_words; i++)
            {
                uint32_t *word = sdramptr + i;
                if (*word != testval)
                    num_failed++;
            }
            /** Write Zeroes */
            for (uint32_t i = 0; i < size_in_words; i++)
            {
                uint32_t *word = sdramptr + i;
                *word = 0x00000000;
            }
            /** Compare Cleared */
            for (uint32_t i = 0; i < size_in_words; i++)
            {
                uint32_t *word = sdramptr + i;
                if (*word != 0)
                    num_failed++;
            }
            return num_failed == 0;
        }

        bool DPT::ValidateQSPI(bool quick)
        {
            uint32_t start;
            uint32_t size;
            if (quick)
            {
                start = 0x400000;
                size = 0x4000;
            }
            else
            {
                start = 0;
                size = 0x800000;
            }
            // Erase the section to be tested
            qspi.Erase(start, start + size);
            // Create some test data
            std::vector<uint8_t> test;
            test.resize(size);
            uint8_t *testmem = test.data();
            for (size_t i = 0; i < size; i++)
                testmem[i] = (uint8_t)(i & 0xff);
            // Write the test data to the device
            qspi.Write(start, size, testmem);
            // Read it all back and count any/all errors
            // I supppose any byte where ((data & 0xff) == data)
            // would be able to false-pass..
            size_t fail_cnt = 0;
            for (size_t i = 0; i < size; i++)
                if (testmem[i] != (uint8_t)(i & 0xff))
                    fail_cnt++;
            return fail_cnt == 0;
        }

    } // namespace dpt
} // namespace daisy
