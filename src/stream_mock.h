/**
 * @file
 * @brief A header-only file for the helper StreamMock class that simulates the base Stream class used by the
 * SerializedTransferProtocol class.
 *
 * @subsection description Description:
 * This class allows on-controller testing of the SerializedTransferProtocol class without establishing a fully functional
 * bidirectional USB/UART connection with the PC. To do so, it implements two large (600 bytes each) public buffers to
 * mimic the tx and rx buffers used by the real Stream class.
 *
 * Additionally, it includes the following methods:
 * - read().
 * - write() [Overloaded to work for single byte-inputs and array inputs].
 * - reset().
 * - flush().
 * - available().
 * - peek().
 *
 * @note The methods in this class use the same arguments and naming convention as the original class and are derived
 * by overloading the virtual methods of the base class where necessary.
 *
 * @subsection developer_notes Developer Notes:
 * This class is used solely to enable on-controller testing of the SerializedTransferProtocol class. Since on-controller
 * testing is currently realized only through the broader AMC, this class is for all intends and purposes useless
 * for the Arduino-targeted version of this library. For the AMC codebase, see appropriate test suite functions for the
 * examples of how to use this class in custom testing.
 *
 * The class reserves a lot of memory (1200 bytes total) to support it's buffers, which is far from the most elegant
 * solution. It is designed to be used with teensy boards where memory is not an issue and it is likely that the class
 * has to be manually adjusted for boards such as Uno. To simplify this process, the class comes with in-code parameters
 * that can be used to control buffer sizes.
 *
 * @attention The class uses int16_t buffers, so all functions had to be modified to work with elements rather than
 * bytes, as each 'byte' is actually represented by a signed short datatype in this class. This information is
 * particularly relevant for writing test functions using the class that directly check buffer states against byte
 * inputs.
 *
 * @note The methods of this class are documented using Google, rather than Doxygen style used by the rest of the
 * library. This is intentional, as it is deemed that the end-user of the API does not generally need to know how this
 * class works and developers altering test suites would have access to the API documentation parsed by the Intellisense
 * of the IDE / code editor used.
 *
 * @subsection dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - Stream.h for the base Stream class that is overloaded to form this Mock class.
 * - stdint.h for fixed-width integer types.
 */

#ifndef AMC_STREAM_MOCK_H
#define AMC_STREAM_MOCK_H

// Dependencies:
#include "Arduino.h"
#include "Stream.h"
#include "stdint.h"

// An implementation of the Stream class that publicly exposes its rx and tx buffers. Note, the class buffers use
// int16_t datatype with any value outside 0 through 255 range considered and treated as invalid. All class methods will
// still work as if they are operating byte-buffers, like the original Stream class, but the implementation of integer
// buffers allows manually setting portions of the buffer to invalid values as necessary for certain test scenarios.
class StreamMock : public Stream
{
  public:
    // Fixed size for buffers to avoid 'magic numbers'. Use this parameter to adjust the buffers to the length
    // appropriate for your testing uses
    static constexpr uint16_t buffer_size = 300;
    int16_t rx_buffer[buffer_size];  // Reception buffer. Only values from 0 through 255 are treated as valid.
    int16_t tx_buffer[buffer_size];  // Transmission buffer. Only values from 0 through 255 are treated as valid.
    size_t rx_buffer_index = 0;      // Tracks the last evaluated index in rx_buffer. Incremented by read operations.
    size_t tx_buffer_index = 0;      // Tracks the last evaluated index in tx_buffer. Incremented by write operations.

    // Initializes class object instance. Sets rx and tx buffers to 0.
    StreamMock()
    {
        memset(rx_buffer, 0, sizeof(rx_buffer));
        memset(tx_buffer, 0, sizeof(tx_buffer));
    }

    // Reads one byte from the rx_buffer and returns it to caller. Returns -1 if no valid bytes are available. Note,
    // the buffer uses int16_t type, but only values inside the uint8_t (0 through 255) range are considered valid.
    virtual int read() override
    {
        // If read index is within the confines of the rx_buffer, reads the byte currently pointed to by the index.
        if (rx_buffer_index < sizeof(rx_buffer) / sizeof(rx_buffer[0]))  // Adjusts to count elements, not bytes
        {
            // Checks if the value at the current index is within the valid uint8_t range
            if (rx_buffer[rx_buffer_index] >= 0 && rx_buffer[rx_buffer_index] <= 255)
            {
                int value = rx_buffer[rx_buffer_index];  // Reads the value from the buffer as an int
                rx_buffer_index++;                       // Increments after reading the value
                return value;                            // Returns the read value
            }
        }

        // If the index is beyond the bounds of the buffer or invalid data is encountered, returns -1 without
        // incrementing the index to indicate there is no data to read.
        return -1;
    }

    // Writes the requested number of bytes from the input buffer array to the tx_buffer. Each writing cycle starts at
    // index 0 of the tx_buffer, overwriting as many indices as necessary to fully consume the input buffer. Note, the
    // input buffer has to be uint8_t, but the values will be converted to int16_t to be saved to the class buffer.
    // Returns the number of bytes written to the tx_buffer.
    virtual size_t write(const uint8_t *buffer, size_t bytes_to_write) override
    {
        // Writes requested number of bytes from the input buffer to the tx_buffer of the class. Note, the operation
        // will be terminated prematurely if the writing process reaches the end of the tx_buffer without consuming all
        // requested bytes.
        size_t i;
        for (i = 0; i < bytes_to_write && tx_buffer_index < sizeof(tx_buffer) / sizeof(tx_buffer[0]); i++)
        {
            tx_buffer[tx_buffer_index++] = static_cast<int16_t>(buffer[i]);
        }
        return i;  // Returns the number of bytes written to the tx_buffer.
    }

    // Writes the input byte value to the tx_buffer. Converts the value to the uint16_t type to be stored inside the
    // tx_buffer. Returns 1 when the method succeeds and 0 otherwise.
    virtual size_t write(uint8_t byte_value) override
    {
        if (tx_buffer_index < (sizeof(tx_buffer) / sizeof(tx_buffer[0])))
        {
            tx_buffer[tx_buffer_index++] = static_cast<int16_t>(byte_value);
            return 1;  // Number of bytes written
        }
        else
        {
            return 0;  // Buffer full, nothing written
        }
    }

    // Returns the number of elements in the rx_buffer available for reading. To do so, scans the buffer contents from
    // the rx_buffer_index either to the end of the buffer or the first invalid value and returns the length of the
    // scanned data stretch. Uses elements rather than bytes due to the uint16_t type of the buffer.
    virtual int available() override
    {
        size_t count = 0;

        // Iterates over the rx_buffer elements starting from rx_buffer_index until the end of the buffer or the first
        // invalid value
        for (size_t i = rx_buffer_index; i < sizeof(rx_buffer) / sizeof(rx_buffer[0]); ++i)
        {
            // Checks if the current value is within the uint8_t range.
            if (rx_buffer[i] >= 0 && rx_buffer[i] <= 255)
            {
                // If so, this is considered available data, so increments the count.
                count++;
            }
            else
            {
                // If an invalid value is encountered, brakes out of the loop as there is no more valid data to count.
                break;
            }
        }

        // Returns the count of available data bytes
        return static_cast<int>(count);  // Cast count to int to match the return type
    }

    // Returns the value currently pointed by the rx_buffer_index without incrementing the index (without consuming the
    // data). Returns -1 if there is no valid byte-value to read (if there is no more data available).
    virtual int peek() override
    {
        // Checks whether the value pointed by rx_buffer_index is within the boundaries of the rx_buffer and is a valid
        // uint8_t value (between 0 and 255 inclusive).
        if (rx_buffer_index < sizeof(rx_buffer) / sizeof(rx_buffer[0]) &&
            (rx_buffer[rx_buffer_index] >= 0 && rx_buffer[rx_buffer_index] <= 255))
        {
            // If so, returns the value without incrementing the index.
            return rx_buffer[rx_buffer_index];
        }
        else
        {
            return -1;  // If there is no valid data to peek, returns -1.
        }
    }

    // Simulates the data being sent to the PC (flushed) by resetting the tx_buffer to the default -1 (no data) value
    // and resetting the tx_buffer_index to 0.
    virtual void flush() override
    {
        // Resets the tx_buffer_index and the buffer itself to simulate the data being sent ('flushed') to the PC.
        for (size_t i = 0; i < (sizeof(tx_buffer) / sizeof(tx_buffer[0])); ++i)
        {
            tx_buffer[i] = -1;  // Sets every value of the buffer to -1 (invalid / no data)
        }

        tx_buffer_index = 0;  // Sets the index to 0
    }

    // Resets the rx and tx buffers and their index tracker variables. This is typically used during testing to reset
    // the buffers between tests. Sets each variable inside each buffer to -1 (no data). This ensures that the buffers
    // default to an empty state, mimicking the standard behavior for the Stream class.
    void reset()
    {
        // Initializes buffers to a "no data" value. Here -1 is chosen for simplicity, but any value outside the
        // 0 through 255 range would work the same way.
        for (size_t i = 0; i < (sizeof(rx_buffer) / sizeof(rx_buffer[0])); ++i)
        {
            rx_buffer[i] = -1;
        }
        for (size_t i = 0; i < (sizeof(tx_buffer) / sizeof(tx_buffer[0])); ++i)
        {
            tx_buffer[i] = -1;
        }
        // Resets the tracker indices
        rx_buffer_index = 0;
        tx_buffer_index = 0;
    }
};

#endif  //AMC_STREAM_MOCK_H
