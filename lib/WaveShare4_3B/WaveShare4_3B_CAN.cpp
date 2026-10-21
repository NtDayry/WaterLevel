/**
 * @file WaveShare4_3B_CAN.cpp
 * @brief Implementation of CAN Bus (TWAI) communication driver for Waveshare ESP32-S3-Touch-LCD-4.3B.
 */

#include "WaveShare4_3B_CAN.h"

WaveShare4_3B_CAN::WaveShare4_3B_CAN()
    : _isInstalled(false), _isRunning(false),
      _txPin((gpio_num_t)WS43B_CAN_TX_PIN), _rxPin((gpio_num_t)WS43B_CAN_RX_PIN),
      _baudrate(WS43B_CAN_DEFAULT_BAUD)
{
}

WaveShare4_3B_CAN::~WaveShare4_3B_CAN() {
    end();
}

bool WaveShare4_3B_CAN::begin(uint32_t baudrate, gpio_num_t txPin, gpio_num_t rxPin) {
    if (_isRunning) {
        end();
    }

    _baudrate = baudrate;
    _txPin = txPin;
    _rxPin = rxPin;

    // General configuration
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(_txPin, _rxPin, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 20;
    g_config.rx_queue_len = 20;

    // Timing configuration based on requested baud rate
    twai_timing_config_t t_config;
    switch (baudrate) {
        case 25000:
            t_config = TWAI_TIMING_CONFIG_25KBITS();
            break;
        case 50000:
            t_config = TWAI_TIMING_CONFIG_50KBITS();
            break;
        case 100000:
            t_config = TWAI_TIMING_CONFIG_100KBITS();
            break;
        case 125000:
            t_config = TWAI_TIMING_CONFIG_125KBITS();
            break;
        case 250000:
            t_config = TWAI_TIMING_CONFIG_250KBITS();
            break;
        case 500000:
            t_config = TWAI_TIMING_CONFIG_500KBITS();
            break;
        case 800000:
            t_config = TWAI_TIMING_CONFIG_800KBITS();
            break;
        case 1000000:
            t_config = TWAI_TIMING_CONFIG_1MBITS();
            break;
        default:
            t_config = TWAI_TIMING_CONFIG_500KBITS();
            break;
    }

    // Filter configuration: accept all IDs
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        return false;
    }
    _isInstalled = true;

    // Start TWAI driver
    err = twai_start();
    if (err != ESP_OK) {
        twai_driver_uninstall();
        _isInstalled = false;
        return false;
    }

    _isRunning = true;
    return true;
}

void WaveShare4_3B_CAN::end() {
    if (_isRunning) {
        twai_stop();
        _isRunning = false;
    }
    if (_isInstalled) {
        twai_driver_uninstall();
        _isInstalled = false;
    }
}

uint32_t WaveShare4_3B_CAN::available() {
    if (!_isRunning) return 0;
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        return status.msgs_to_rx;
    }
    return 0;
}

bool WaveShare4_3B_CAN::send(uint32_t id, const uint8_t *data, uint8_t len,
                             bool extended, bool rtr, unsigned long timeoutMs) {
    if (!_isRunning) return false;

    twai_message_t message;
    memset(&message, 0, sizeof(message));
    message.identifier = id;
    message.extd = extended ? 1 : 0;
    message.rtr = rtr ? 1 : 0;
    message.data_length_code = (len > 8) ? 8 : len;

    if (data && len > 0 && !rtr) {
        memcpy(message.data, data, message.data_length_code);
    }

    return sendMessage(message, timeoutMs);
}

bool WaveShare4_3B_CAN::sendMessage(const twai_message_t &message, unsigned long timeoutMs) {
    if (!_isRunning) return false;
    TickType_t ticks = (timeoutMs == 0) ? 0 : pdMS_TO_TICKS(timeoutMs);
    esp_err_t err = twai_transmit(&message, ticks);
    return (err == ESP_OK);
}

bool WaveShare4_3B_CAN::receive(uint32_t &id, uint8_t *data, uint8_t &len, bool &extended, unsigned long timeoutMs) {
    twai_message_t message;
    if (receiveMessage(message, timeoutMs)) {
        id = message.identifier;
        extended = message.extd != 0;
        len = message.data_length_code;
        if (data && len > 0) {
            memcpy(data, message.data, len);
        }
        return true;
    }
    return false;
}

bool WaveShare4_3B_CAN::receiveMessage(twai_message_t &message, unsigned long timeoutMs) {
    if (!_isRunning) return false;
    TickType_t ticks = (timeoutMs == 0) ? 0 : pdMS_TO_TICKS(timeoutMs);
    esp_err_t err = twai_receive(&message, ticks);
    return (err == ESP_OK);
}

bool WaveShare4_3B_CAN::getStatus(twai_status_info_t &statusInfo) {
    if (!_isRunning) return false;
    return (twai_get_status_info(&statusInfo) == ESP_OK);
}

bool WaveShare4_3B_CAN::recoverBus() {
    if (!_isInstalled) return false;
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        if (status.state == TWAI_STATE_BUS_OFF) {
            if (twai_initiate_recovery() == ESP_OK) {
                // Wait for recovery
                vTaskDelay(pdMS_TO_TICKS(100));
                return (twai_start() == ESP_OK);
            }
        }
    }
    return false;
}

void WaveShare4_3B_CAN::printMessage(const twai_message_t &msg, Stream &out) {
    out.print("[CAN] ID: 0x");
    if (msg.extd) {
        if (msg.identifier < 0x1000000) out.print("0");
        if (msg.identifier < 0x100000) out.print("0");
        if (msg.identifier < 0x10000) out.print("0");
        if (msg.identifier < 0x1000) out.print("0");
        if (msg.identifier < 0x100) out.print("0");
        if (msg.identifier < 0x10) out.print("0");
    } else {
        if (msg.identifier < 0x100) out.print("0");
        if (msg.identifier < 0x10) out.print("0");
    }
    out.print(msg.identifier, HEX);
    out.print(msg.extd ? " (EXT)" : " (STD)");

    if (msg.rtr) {
        out.println(" [RTR]");
    } else {
        out.print(" DLC: ");
        out.print(msg.data_length_code);
        out.print(" DATA: ");
        for (int i = 0; i < msg.data_length_code; i++) {
            if (msg.data[i] < 0x10) out.print("0");
            out.print(msg.data[i], HEX);
            out.print(" ");
        }
        out.println();
    }
}
