/**
 * @file WaveShare4_3B_CAN.h
 * @brief CAN Bus (TWAI) communication driver for Waveshare ESP32-S3-Touch-LCD-4.3B
 * using the onboard TJA1051T transceiver.
 */

#ifndef WAVESHARE_4_3B_CAN_H
#define WAVESHARE_4_3B_CAN_H

#include <Arduino.h>
#include "driver/twai.h"
#include "WaveShare4_3B_Config.h"

class WaveShare4_3B_CAN {
public:
    WaveShare4_3B_CAN();
    ~WaveShare4_3B_CAN();

    /**
     * @brief Initialize and start the CAN (TWAI) driver.
     * @param baudrate Bus speed in bps (e.g. 125000, 250000, 500000, 1000000). Default is 500000.
     * @param txPin TX GPIO pin (default WS43B_CAN_TX_PIN = 15).
     * @param rxPin RX GPIO pin (default WS43B_CAN_RX_PIN = 16).
     * @return true if initialized and started successfully.
     */
    bool begin(uint32_t baudrate = WS43B_CAN_DEFAULT_BAUD,
               gpio_num_t txPin = (gpio_num_t)WS43B_CAN_TX_PIN,
               gpio_num_t rxPin = (gpio_num_t)WS43B_CAN_RX_PIN);

    /**
     * @brief Stop and uninstall the TWAI driver.
     */
    void end();

    /**
     * @brief Check if the CAN driver is running and ready.
     */
    bool isRunning() const { return _isRunning; }

    /**
     * @brief Check how many unread messages are in the receive buffer.
     * @return Number of queued messages.
     */
    uint32_t available();

    /**
     * @brief Send a standard or extended CAN frame.
     * @param id Message ID (11-bit for standard, 29-bit for extended).
     * @param data Pointer to data bytes (0 to 8 bytes).
     * @param len Number of bytes (0 to 8).
     * @param extended true for 29-bit extended ID, false for standard 11-bit ID.
     * @param rtr true for Remote Transmission Request.
     * @param timeoutMs Timeout in milliseconds to enqueue the message.
     * @return true if successfully queued/transmitted.
     */
    bool send(uint32_t id, const uint8_t *data, uint8_t len = 8,
              bool extended = false, bool rtr = false, unsigned long timeoutMs = 100);

    /**
     * @brief Send a pre-configured twai_message_t frame.
     */
    bool sendMessage(const twai_message_t &message, unsigned long timeoutMs = 100);

    /**
     * @brief Receive a CAN frame into individual variables.
     * @param[out] id Received message ID.
     * @param[out] data Buffer for data bytes (must be at least 8 bytes).
     * @param[out] len Received data length (0 to 8).
     * @param[out] extended True if extended ID frame.
     * @param timeoutMs Timeout to wait for a frame.
     * @return true if a frame was successfully received.
     */
    bool receive(uint32_t &id, uint8_t *data, uint8_t &len, bool &extended, unsigned long timeoutMs = 50);

    /**
     * @brief Receive a CAN frame into a twai_message_t structure.
     */
    bool receiveMessage(twai_message_t &message, unsigned long timeoutMs = 50);

    /**
     * @brief Get TWAI controller status info (error counts, queue size, state).
     */
    bool getStatus(twai_status_info_t &statusInfo);

    /**
     * @brief Attempt bus recovery if the controller went into Bus-Off state.
     */
    bool recoverBus();

    /**
     * @brief Pretty-print a CAN frame to a serial stream for debugging.
     */
    static void printMessage(const twai_message_t &msg, Stream &out = Serial);

private:
    bool _isInstalled;
    bool _isRunning;
    gpio_num_t _txPin;
    gpio_num_t _rxPin;
    uint32_t _baudrate;
};

#endif // WAVESHARE_4_3B_CAN_H
