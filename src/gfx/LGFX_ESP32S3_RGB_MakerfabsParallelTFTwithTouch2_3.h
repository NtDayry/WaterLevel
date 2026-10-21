#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ILI9341 _panel_instance;
    lgfx::Bus_SPI _bus_instance;
    lgfx::Touch_XPT2046 _touch_instance;
    lgfx::Light_PWM _light_instance;

public:
    LGFX(void) {
        { // Configure bus control settings
            auto cfg = _bus_instance.config();

            // SPI bus settings
            cfg.spi_host = SPI2_HOST;    // Use SPI2
            cfg.spi_mode = 0;            // SPI mode 0
            cfg.freq_write = 40000000;   // SPI clock for transmission (40MHz)
            cfg.freq_read = 16000000;    // SPI clock for receiving (16MHz)
            cfg.spi_3wire = true;        // Use 3-wire SPI (MISO and MOSI shared)
            cfg.use_lock = true;         // Enable transaction lock
            cfg.dma_channel = 1;         // Use DMA channel 1
            cfg.pin_sclk = 14;          // SCLK pin
            cfg.pin_mosi = 13;          // MOSI pin
            cfg.pin_miso = 12;          // MISO pin (-1 = disable)
            cfg.pin_dc = 2;             // D/C pin

            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        { // Configure panel settings
            auto cfg = _panel_instance.config();

            cfg.pin_cs = 15;           // CS pin
            cfg.pin_rst = -1;           // RST pin (-1 = disable)
            cfg.pin_busy = -1;          // BUSY pin (-1 = disable)

            cfg.memory_width = 320;     // Display width
            cfg.memory_height = 240;     // Display height
            cfg.panel_width = 320;      // Visible width
            cfg.panel_height = 240;      // Visible height
            cfg.offset_x = 0;           // X offset
            cfg.offset_y = 0;           // Y offset
            cfg.offset_rotation = 0;    // Rotation offset (0~7)
            cfg.readable = true;        // Enable read
            cfg.invert = false;         // Disable color invert
            cfg.rgb_order = false;      // RGB order (false = normal)
            cfg.dlen_16bit = false;     // 16-bit data length
            cfg.bus_shared = true;      // Shared bus with touch

            _panel_instance.config(cfg);
        }

        { // Configure backlight
            auto cfg = _light_instance.config();

            cfg.pin_bl = 21;           // Backlight pin
            cfg.invert = false;         // Disable invert
            cfg.freq = 44100;          // PWM frequency
            cfg.pwm_channel = 7;       // PWM channel

            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }

        { // Configure touch screen
            auto cfg = _touch_instance.config();

            cfg.x_min = 0;             // Minimum X value
            cfg.x_max = 319;            // Maximum X value
            cfg.y_min = 0;             // Minimum Y value
            cfg.y_max = 239;           // Maximum Y value
            cfg.pin_int = 36;          // INT pin
            cfg.bus_shared = true;     // Shared bus with display
            cfg.offset_rotation = 0;   // Rotation offset

            // SPI settings
            cfg.spi_host = SPI2_HOST;  // Use SPI2
            cfg.freq = 50000;        // SPI clock (50kHz)
            cfg.pin_sclk = 25;        // SCLK pin
            cfg.pin_mosi = 32;        // MOSI pin
            cfg.pin_miso = 39;        // MISO pin
            cfg.pin_cs = 33;          // CS pin

            _touch_instance.config(cfg);
            _panel_instance.setTouch(&_touch_instance);
        }

        setPanel(&_panel_instance);
    }
};