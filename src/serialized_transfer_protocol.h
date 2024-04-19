/**
 * @file
 * @brief The header file for the SerializedTransferProtocol class, which aggregates all methods for sending and receiving
 * data over Serial Stream interface (USB / UART).
 *
 * @subsection description Description:
 * This class provides an intermediate-level API that enables receiving and sending data over the USB or UART serial
 * ports. It conducts all necessary steps to properly encode and decode payloads, verifies their integrity and moves
 * them to and from the bundled Stream class buffers. This class instantiates private _transmission_buffer and
 * _reception_buffer arrays, which are used as the intermediate storage / staging areas for the processed payloads. Both
 * buffers have to be treated as temporary work areas as they are frequently reset during data transmission and
 * reception and they can only be accessed using class methods.
 *
 * This class contains the following methods:
 * - Available(): To check whether the bundled Stream class has received data that can be moved to the reception_buffer.
 * - ResetTransmissionBuffer(): To reset the overhead byte and the size tracker fro the transmission_buffer.
 * - ResetReceptionBuffer(): To reset the overhead byte and the size tracker from the reception_buffer.
 * - SendData(): To package and send the contents of the transmission_buffer via the bundled Serial class.
 * - ReceiveData(): To unpack the data received by the bundled Serial class into the reception_buffer.
 * - WriteData(): To write an arbitrary object to the transmission_buffer as bytes.
 * - ReadData(): To read an arbitrary object from the reception_buffer as bytes.
 * - CopyTxDataToBuffer() and CopyRXDataToBuffer: To assist testing by copying the contents of the _transmission_buffer
 * or the _reception_buffer, into the input buffer. Indirectly exposes the buffers for testing purposes.
 * - CopyTxBufferPayloadToRxBuffer: To assist testing by copying the payload bytes inside the _transmission_buffer to
 * the _reception_buffer. This method is safe and allows to maintain the general guard of only writing to
 * _transmission_buffer (and only via WriteData() method) while also giving the ability to safely transfer the data to
 * the _reception_buffer.
 * - Multiple minor methods that are used to by the listed major methods as well as accessor methods that communicate
 * the values of certain private tracker variables (such as the payload byte trackers for the internal buffers).
 *
 * @attention This class is implemented as a template and many methods adapt to the template arguments used during
 * class instantiation. See developer notes below for more information.
 *
 * @subsection developer_notes Developer Notes:
 * This class functions as the root hub of the library, relying on CRCProcessor and COBSProcessor helper-classes to
 * cary out the specific packet processing and integrity verification steps. The SerializedTransferProtocol class abstracts
 * these two classes and a bundled Stream class instance by providing an API that can be used to realize serial
 * communication functionality through 4 major methods (SendData(), ReceiveData(), WriteData() and
 * ReadData()).
 *
 * The class uses a number of template parameters to allows users to pseudo-dynamically configure various class runtime
 * aspects, especially when template parameters are combined with Constructor arguments. That does minorly complicate
 * class instantiation and forces all methods of the class to be implemented inside the .h file, which is somewhat
 * against convention. That said, since this optimization simplifies code maintainability, it is considered an
 * acceptable tradeoff.
 *
 * @note This whole library is written specifically for the AMC codebase and, as such, was optimized for the specific
 * role it has in that codebase (as the main provider of serial communication services over the USB port). The library
 * should be compatible with most Arduino boards and UART protocol (which is much slower compared to USB). It is offered
 * as a standalone tool for other developers that may benefit from having a backend specifically dedicated to USB / UART
 * serial communication between Arduino and Python.
 *
 * @attention This class permanently reserves up to 256 x 2 + 3 = 515 bytes of memory for its buffers. This value can
 * be reduced by changing the maximum transmission / reception buffer sizes (by altering kMaximumTransmittedPayloadSize
 * and kMaximumReceivedPayloadSize template parameters). As such, while the class may not support boards like Uno
 * out of the box, it can easily be configured in a way where it can on almost any board.
 *
 * @subsection packet_anatomy Packet Anatomy:
 * This class sends and receives data in the form of packets. Each packet is expected to adhere to the following general
 * layout:
 * [START BYTE] [COBS OVERHEAD BYTE] [PAYLOAD (1 to 254 bytes)] [DELIMITER BYTE] [CRC CHECKSUM POSTAMBLE (1 to 4 bytes)]
 *
 * When using WriteData() and ReadData() methods, the users are only accessing the payload section of the overall
 * packet. The rest of the packet anatomy is controlled internally by this class and is not exposed to the users.
 *
 * @subsection dependencies Dependencies:
 * - Arduino.h for Arduino platform methods and macros and cross-compatibility with Arduino IDE (to an extent).
 * - cobs_processor.h for COBS encoding and decoding methods.
 * - crc_processor.h for CRC-16 encoding and decoding methods, as well as crc-specific buffer manipulation methods.
 * - cstring for std namespace.
 * - elapsedMillis.h for managing packet reception timers.
 * - stdint.h for fixed-width integer types. Using stdint.h instead of cstdint for compatibility (with Arduino) reasons.
 */

#ifndef AMC_SERIAL_TRANSFER_PROTOCOL_H
#define AMC_SERIAL_TRANSFER_PROTOCOL_H

// Dependencies
#include "Arduino.h"
#include "cobs_processor.h"
#include "crc_processor.h"
#include "elapsedMillis.h"

/**
 * @class SerializedTransferProtocol
 * @brief Provides a collection of methods that can be used to send and receive packetised data over the serial Stream
 * interface (USB or UART).
 *
 * This class wraps other low-level helper classes of the library that are used to encode, validate and bidirectionally
 * transmit arbitrary data over the USB or UART serial port. To facilitate this process, the class provides two internal
 * buffers: _transmission_buffer (stages the data to be transmitted) and _reception_buffer (stores the data received
 * from the PC). Note, both buffers are treated as temporary storage areas and are reset by each SendData() and
 * ReceiveData() calls.
 *
 * @warning Since the buffers follow a very specific layout pattern that is required for this class to work properly,
 * they are stored as private members of this class. The buffers can be manipulated using ReadData() and WriteData()
 * methods to read received data and write the data to be transmitted. They can also be reset at any time by calling
 * ResetTransmissionBuffer() and ResetReceptionBuffer() respectively. Additionally, GetTransmissionBufferBytes() and
 * GetReceptionBufferBytes() can be used to retrieve the number of bytes currently stored inside each of the buffers.
 *
 * @attention The class tracks how many bytes are stored in each of the buffers. Specifically for the
 * _transmission_buffer, this is critical, as the tracker determines how many bytes are packed and sent to the PC. The
 * tracker is reset by calling ResetTransmissionBuffer() or SendData() methods and does not care if already tracked
 * bytes are overwritten. As such, if you add 50 bytes to the buffer and then overwrite the first 20, the class will
 * remember and send all 50 bytes unless you reset the tracker before overwriting the bytes. Additionally, the tracker
 * always assumes that the bytes to send stretch from the beginning of the buffer. So, if you write 10 bytes to the
 * middle of the buffer (say, at index 100+), the tracker will assume that 100 bytes were somehow added before the 10
 * bytes you provided and send 110 bytes, including potentially meaningless 100 bytes. See the documentation for the
 * WriteData() and ReadData() methods for more information on byte trackers.
 *
 * @note The class is broadly configured through the combination of class template parameters and constructor arguments.
 * The template parameters (see below) need to be defined at compilation time and are needed to support proper static
 * initialization of local arrays and subclasses. All currently used template parameters indirectly control how much RAM
 * is reserved by teh class for it's buffers and the CRC lookup table (via the bundled CRCProcessor class instance). The
 * constructor arguments allow to further configure class runtime behavior in a way that is not compile-time dependent.
 *
 * @tparam PolynomialType The datatype of the CRC polynomial to be used by the bundled (internal) CRCProcessor class.
 * Valid types are uint8_t, uint16_t and uint32_t. The class contains a compile-time guard against any other input
 * datatype. See CRCProcessor documentation for more details.
 * @tparam kMaximumTransmittedPayloadSize The maximum size of the payload that is expected to be transmitted during
 * class runtime. Note, the number is capped to 254 bytes due to COBS protocol and it is used to determine the size of
 * the transmission_buffer array. Use this number to indirectly control the memory reserved by the transmission_buffer
 * (at a maximum of 254 + 2 = 256 bytes).
 * @tparam kMaximumReceivedPayloadSize The maximum size of the payload that is expected to be received during class
 * runtime. Works the same way as kMaximumTransmittedPayloadSize, but allows to independently control the size of the
 * reception_buffer.
 *
 * Example instantiation:
 * @code
 * Serial.begin(9600);
 * uint8_t maximum_tx_payload_size = 254;
 * uint8_t maximum_rx_payload_size = 200;
 * uint16_t polynomial = 0x1021;
 * uint16_t initial_value = 0xFFFF;
 * uint16_t final_xor_value = 0x0000;
 * uint8_t start_byte = 129;
 * uint8_t delimiter_byte = 0;
 * uint32_t timeout = 20000; // In microseconds
 *
 * // Instantiates a new SerializedTransferProtocol object
 * SerializedTransferProtocol<uint16_t, maximum_tx_payload_size, maximum_rx_payload_size> stp_class(
 * Serial,
 * polynomial,
 * initial_value,
 * final_xor_value,
 * start_byte,
 * delimiter_byte,
 * timeout
 * );
 * @endcode
 */
template <
    typename PolynomialType,
    const uint8_t kMaximumTransmittedPayloadSize,
    const uint8_t kMaximumReceivedPayloadSize>
class SerializedTransferProtocol
{
    // Ensures that the class only accepts uint8, 16 or 32 as valid CRC types, as no other type can be used to store a
    // CRC polynomial at the time of writing.
    static_assert(
        stp_shared_assets::is_same_v<PolynomialType, uint8_t> ||
            stp_shared_assets::is_same_v<PolynomialType, uint16_t> ||
            stp_shared_assets::is_same_v<PolynomialType, uint32_t>,
        "SerializedTransferProtocol class template PolynomialType argument must be either uint8_t, uint16_t, or "
        "uint32_t."
    );

    // Verifies that the maximum Transmitted and Received payload sizes do not exceed 254 bytes (due to COBS, this is
    // the maximum supported size).
    static_assert(
        kMaximumTransmittedPayloadSize < 255,
        "SerializedTransferProtocol class template MaximumTxPayloadSize must be less than 255."
    );
    static_assert(
        kMaximumReceivedPayloadSize < 255,
        "SerializedTransferProtocol class template MaximumRxPayloadSize must be less than 255."
    );

  public:
    /// Stores the runtime status of the most recently called method. Note, this variable can either store the status
    /// derived from the kSerializedTransferProtocolStatusCodes enumeration if it originates from this class or use the
    /// enumerations for the COBSProcessor and CRCProcessor helper classes, if the status (error) originates from one
    /// of these classes. As such, you may need to use all of the enumerations available through stp_shared_assets
    /// namespace to determine the status of the most recently called method. All status codes used by this library are
    /// unique across the library, so any returned byte-code always has a single meaning.
    uint8_t transfer_status = static_cast<uint8_t>(stp_shared_assets::kSerializedTransferProtocolStatusCodes::kStandby);

    /**
     * @brief Instantiates a new SerializedTransferProtocol object.
     *
     * The constructor resets the _transmission_buffer and _reception_buffer of the instantiated class to 0 following
     * initialization. Also initializes the CRCProcessor class using the provided CRC parameters. Note, the CRCProcessor
     * class is defined using the PolynomialType template parameter and, as such, expects and casts all input CRC
     * arguments to the same type.
     *
     * @param communication_port A reference to the fully configured instance of base Stream class, such as Serial or
     * USB Serial. This class is used as a low-level access point that physically manages the hardware used to
     * communicate with the PC.
     * @param crc_polynomial The polynomial to use for the generation of the CRC lookup table used by the internal
     * CRCProcessor class. Can be provided as an appropriately-sized HEX number (e.g. 0x1021). Note, currently only
     * non-reversed polynomials are supported. Defaults to 0x1021 (CRC-16 CCITT-FAlSE).
     * @param crc_initial_value The initial value to which the CRC checksum variable is initialized during calculation.
     * This value is based on the polynomial parameter. Can be provided as an appropriately-sized HEX number
     * (e.g. 0xFFFF). Defaults to 0xFFFF (CRC-16 CCITT-FAlSE).
     * @param crc_final_xor_value The final XOR value to be applied to the calculated CRC checksum value. This value is
     * based on the polynomial parameter. Can be provided as an appropriately-sized HEX number (e.g. 0x0000). Defaults
     * to 0x0000 (CRC-16 CCITT-FAlSE).
     * @param start_byte The byte-range value (from 0 through 255) to be used as the start byte of each transmitted and
     * received byte-stream. The presence of this value inside the incoming byte-stream instructs the receiver to enter
     * packet parsing mode both on the Controller and PC. This value ideally should be different from the delimiter_byte
     * to maintain higher packet reliability, but it does not have to be. Also, it is advised to use a value that is
     * unlikely to be encountered due to random noise). Defaults to 129.
     * @param delimiter_byte The byte-range value (from 0 through 255) to be used as the delimiter (stop) byte of each
     * packet. Encountering a delimiter_byte value is the only non-error way of ending the packet reception loop. During
     * packet construction, this value is eliminated from the payload using COBS encoding. It is advised to use the
     * value of 0x00 (0) as this is the only value that is guaranteed to not occur anywhere in the packet. All other
     * values may also show up as the overhead byte (see COBSProcessor documentation for more details). Defaults to 0.
     * @param timeout The number of microseconds to wait between receiving any two consecutive bytes of the packet. The
     * algorithm uses this value to detect stale packets, as it expects all bytes of the same packet to arrive close in
     * time to each other. Primarily, this is a safe-guard to break out of stale packet reception cycles and avoid
     * deadlocking the controller into the packet reception mode. Defaults to 20000 us (20 ms).
     * @param allow_start_byte_errors A boolean flag that determines whether the class raises errors when it is unable
     * to find the start_byte value in the incoming byte-stream. It is advised to keep this set to False for most use
     * cases. This is because it is fairly common to see noise-generated bytes inside the reception buffer that
     * are then silently cleared by the algorithm until a real packet becomes available. However, enabling this
     * option may be helpful for certain debugging scenarios. Defaults to False.
     *
     * Example instantiation:
     * @code
     * Serial.begin(9600);
     * uint8_t maximum_tx_payload_size = 254;
     * uint8_t maximum_rx_payload_size = 200;
     * uint16_t polynomial = 0x1021;
     * uint16_t initial_value = 0xFFFF;
     * uint16_t final_xor_value = 0x0000;
     * uint8_t start_byte = 129;
     * uint8_t delimiter_byte = 0;
     * uint32_t timeout = 20000; // In microseconds
     * bool allow_start_byte_errors = false;
     *
     * // Instantiates a new SerializedTransferProtocol object
     * SerializedTransferProtocol<uint16_t, maximum_tx_payload_size, maximum_rx_payload_size> stp_class(
     * Serial,
     * polynomial,
     * initial_value,
     * final_xor_value,
     * start_byte,
     * delimiter_byte,
     * timeout,
     * allow_start_byte_errors
     * );
     * @endcode
     */
    explicit SerializedTransferProtocol(
        Stream& communication_port,
        const PolynomialType crc_polynomial      = 0x1021,
        const PolynomialType crc_initial_value   = 0xFFFF,
        const PolynomialType crc_final_xor_value = 0x0000,
        const uint8_t start_byte                 = 129,
        const uint8_t delimiter_byte             = 0,
        const uint32_t timeout                   = 20000,
        const bool allow_start_byte_errors       = false
    ) :
        _port(communication_port),
        _crc_processor(crc_polynomial, crc_initial_value, crc_final_xor_value),
        kStartByte(start_byte),
        kDelimiterByte(delimiter_byte),
        kTimeout(timeout),
        kAllowStartByteErrors(allow_start_byte_errors),
        _transmission_buffer {},  // Initialization doubles up as resetting buffers to 0
        _reception_buffer {}
    {}

    /**
     * @brief Evaluates whether the reception buffer of the bundled Stream class has bytes to read (checks whether
     * bytes have been received from the PC).
     *
     * This is a simple wrapper around available() method of the bundled Stream class (such as Serial) that can be used
     * to quickly evaluate whether ReceiveData() method needs to be called to parse the incoming data. This allows to
     * save computation time by avoiding unnecessary ReceiveData() calls.
     *
     * @note This is a public utility method and, as such, it does not modify internal class instance transfer_status
     * variable.
     *
     * @returns bool True if there are bytes to be read from the reception buffer, false otherwise.
     *
     * Example usage:
     * @code
     * Serial.begin(9600);
     * SerializedTransferProtocol<uint16_t, 254, 254> stp_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
     * bool available = serial_protocol.Available();
     * @endcode
     */
    bool Available()
    {
        // If bytes are available, returns 'true' to indicate there are bytes to read
        if (_port.available() > 0)
        {
            return true;
        }

        // Otherwise returns 'false' to indicate there are no bytes available to read
        return false;
    }

    /**
     * @brief Resets them internal _bytes_in_transmission_buffer tracker variable and sets the first variable of the
     * internal _transmission_buffer (overhead placeholder) to 0 (default value used by unencoded payloads).
     *
     * This method is primarily used by other class methods to prepare the buffer and data tracker to store a new
     * payload after sending the old payload or encountering a transmission error. It can be called externally if a
     * particular pipeline requires forcibly resetting the buffer's overhead byte variable and tracker.
     *
     * @note This is a public utility method and, as such, it does not modify internal class instance transfer_status
     * variable.
     *
     * Example usage:
     * @code
     * Serial.begin(9600);
     * SerializedTransferProtocol<uint16_t, 254, 254> stp_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
     * serial_protocol.ResetTransmissionBuffer();
     * @endcode
     */
    void ResetTransmissionBuffer()
    {
        _transmission_buffer[0]       = 0;
        _bytes_in_transmission_buffer = 0;
    }

    /**
     * @brief Resets the internal bytes_in_reception_buffer tracker variable and sets the first variable of the internal
     * _reception_buffer (overhead placeholder) to 0.
     *
     * This method is primarily used by other class methods to prepare the buffer and data tracker for the reception
     * of a new data packet after fully consuming a received payload or encountering reception error. It can be called
     * externally if a particular pipeline requires forcibly resetting the buffer's overhead byte variable and tracker.
     *
     * @note This is a public utility method and, as such, it does not modify internal class instance transfer_status
     * variable.
     *
     * Example usage:
     * @code
     * Serial.begin(9600);
     * SerializedTransferProtocol<uint16_t, 254, 254> stp_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
     * serial_protocol.ResetReceptionBuffer();
     * @endcode
     */
    void ResetReceptionBuffer()
    {
        _reception_buffer[0]       = 0;
        _bytes_in_reception_buffer = 0;
    }

    /**
     * @brief Copies the _transmission_buffer into the input destination buffer.
     *
     * This method is designed to assist developers writing test functions. It accepts a buffer of the same type and
     * size as the _transmission_buffer via reference and copies the contents of the _transmission_buffer into the
     * input buffer. This way, the _transmission_buffer can be indirectly exposed for evaluation without in any way
     * modifying the buffer's contents.
     *
     * @warning Do not use this method for anything other than testing! It does not care about buffer layout and
     * partitioning, which are important for correctly working with meaningful payload data.
     *
     * @note This is a public utility method and, as such, it does not modify internal class instance transfer_status
     * variable.
     *
     * @tparam DestinationSize The byte-size of the destination buffer array. Inferred automatically and used to ensure
     * the input buffer size exactly matches the _transmission_buffer size.
     *
     * Example usage:
     * @code
     * Serial.begin(9600);
     * SerializedTransferProtocol<uint16_t, 254, 254> stp_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
     * uint8_t test_buffer[254];
     * serial_protocol.CopyTxDataToBuffer(test_buffer);
     * @endcode
     */
    template <size_t DestinationSize>
    void CopyTxDataToBuffer(uint8_t (&destination)[DestinationSize])
    {
        // Ensures that the destination is the same size as the _transmission_buffer
        static_assert(
            DestinationSize == kMaximumTxBufferSize,
            "Destination buffer size must be equal to the maximum _transmission_buffer size."
        );

        // Copies the _transmission_buffer into the referenced destination buffer
        memcpy(destination, _transmission_buffer, DestinationSize);
    }

    /**
     * @brief Copies the _reception_buffer into the input destination buffer.
     *
     * This method is designed to assist developers writing test functions. It accepts a buffer of the same type and
     * size as the _reception_buffer via reference and copies the contents of the _reception_buffer into the
     * input buffer. This way, the _reception_buffer can be indirectly exposed for evaluation without in any way
     * modifying the buffer's contents.
     *
     * @warning Do not use this method for anything other than testing! It does not care about buffer layout and
     * partitioning, which are important for correctly working with meaningful payload data.
     *
     * @note This is a public utility method and, as such, it does not modify internal class instance transfer_status
     * variable.
     *
     * @tparam DestinationSize The byte-size of the destination buffer array. Inferred automatically and used to ensure
     * the input buffer size exactly matches the _reception_buffer size.
     *
     * Example usage:
     * @code
     * Serial.begin(9600);
     * SerializedTransferProtocol<uint16_t, 254, 254> stp_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
     * uint8_t test_buffer[254];
     * serial_protocol.CopyRxDataToBuffer(test_buffer);
     * @endcode
     */
    template <size_t DestinationSize>
    void CopyRxDataToBuffer(uint8_t (&destination)[DestinationSize])
    {
        // Ensures that the destination is the same size as the _reception_buffer
        static_assert(
            DestinationSize == kMaximumRxBufferSize,
            "Destination buffer size must be equal to the maximum _reception_buffer size."
        );

        // Copies the _reception_buffer into the referenced destination buffer
        memcpy(destination, _reception_buffer, DestinationSize);
    }

    /**
     * @brief Copies the payload bytes from _transmission_buffer to the _reception_buffer.
     *
     * This method is designed to assist developers writing test functions. It checks that the payload written to
     * _transmission_buffer (via WriteData()) is sufficiently small to fit into the payload region of the
     * _reception_buffer and, if so, copies it to the _reception_buffer. This allows to safely 'write' to the
     * _reception_buffer.
     *
     * @warning Do not use this method for anything other than testing! It upholds all safety standards, but, in
     * general, there should never be a need to write to reception_buffer outside of testing scenarios.
     *
     * @note This is a public utility method and, as such, it does not modify internal class instance transfer_status
     * variable.
     *
     * @returns bool True if the _reception_buffer was successfully updated, false otherwise.
     *
     * Example usage:
     * @code
     * Serial.begin(9600);
     * SerializedTransferProtocol<uint16_t, 254, 254> stp_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
     * uint8_t test_value = 50;
     * serial_protocol.WriteData(test_value); // Saves the test value to the _transmission_buffer payload
     * serial_protocol.CopyTxBufferPayloadToRxBuffer();  // Moves the payload over to the _reception_buffer
     * @endcode
     */
    bool CopyTxBufferPayloadToRxBuffer()
    {
        // Ensures that the payload size to move will fit inside the payload region of the _reception_buffer
        if (_bytes_in_transmission_buffer > kMaximumReceivedPayloadSize)
        {
            return false;
        }

        // Copies the payload from _transmission_buffer to _reception_buffer
        memcpy(&_reception_buffer[1], &_transmission_buffer[1], _bytes_in_transmission_buffer);

        // Updates the _bytes_in_reception_buffer to match the copied payload size
        _bytes_in_reception_buffer = _bytes_in_transmission_buffer;

        return true;
    }

    /// Returns the current value of the _bytes_in_transmission_buffer variable as a uint16_t integer.
    [[nodiscard]]
    // No reason to call this method in the first place if returned value is discarded.
    uint16_t get_bytes_in_transmission_buffer() const
    {
        return _bytes_in_transmission_buffer;
    }

    /// Returns the current value of the _bytes_in_reception_buffer variable as a uint16_t integer.
    [[nodiscard]]
    // No reason to call this method in the first place if returned value is discarded.
    uint16_t get_bytes_in_reception_buffer() const
    {
        return _bytes_in_reception_buffer;
    }

    // Returns the value of the kMaximumTransmittedPayloadSize template parameter of the class
    static constexpr uint16_t get_maximum_tx_payload_size()
    {
        return kMaximumTransmittedPayloadSize;
    }

    // Returns the value of the kMaximumReceivedPayloadSize template parameter of the class
    static constexpr uint16_t get_maximum_rx_payload_size()
    {
        return kMaximumReceivedPayloadSize;
    }

    // Returns the size of the _transmission_buffer used by the class
    static constexpr uint16_t get_tx_buffer_size()
    {
        return kMaximumTxBufferSize;
    }

    // Returns the size of the _reception_buffer used by the class
    static constexpr uint16_t get_rx_buffer_size()
    {
        return kMaximumRxBufferSize;
    }

    /**
     * @brief Packages the data inside the transmission_buffer into a packet and transmits it to the PC using the
     * bundled Stream class.
     *
     * This is a master-method that aggregates all steps necessary to correctly transmit the payload stored in the
     * _transmission_buffer as a byte-stream using the bundled Stream class. Specifically, it first encodes the payload
     * using COBS protocol and then calculates and adds the CRC checksum for the encoded packet to the end of the
     * packet. If all packet construction operations are successful, the method then transmits the data using write()
     * method of the Stream class. If there is enough space in the _transmission buffer of the Stream class, the
     * operation is carried out immediately and if not, the method blocks in-place until all bytes to be transmitted are
     * moved to the transmission buffer of the Stream class.
     *
     * @attention This method relies on the _bytes_in_transmission_buffer tracker variable to determine how many
     * bytes inside the _transmission_buffer need to be encoded and added to the packet. Use that tracker before calling
     * this method if you need to determine the portion of the buffer that will be sent.
     *
     * @returns bool True if the packet was successfully constructed and sent and false otherwise. If method runtime
     * fails, use the transfer_status variable to determine the reason for the failure, as it would be set to the
     * specific error code of the failed operation. Transfer_status values are guaranteed to uniquely match one of the
     * enumerators stored inside the kCOBSProcessorCodes, kCRCProcessorCodes or kSerializedTransferProtocolStatusCodes
     * enumerations available through the stp_shared_assets namespace.
     *
     * Example usage:
     * @code
     * Serial.begin(9600);
     * SerializedTransferProtocol<uint16_t, 254, 254> stp_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
     * bool packet_sent = serial_protocol.SendData();
     * @endcode
     */
    bool SendData()
    {
        // Constructs the packet to be transmitted using the _transmission_buffer. Note, during this process, a CRC
        // checksum is calculated for the packet and appended to the end of the encoded packet. The returned size of
        // the data to send includes the CRC checksum size.
        uint16_t combined_size = ConstructPacket();

        // If the returned combined_size is not 0, this indicates that the packet has been constructed successfully.
        if (combined_size != 0)
        {
            // Transmits the packet using the selected stream interface. Starts by transmitting the preamble, which
            // includes kStartByte value and the size of the transmitted payload. The start_byte notifies the receiver
            // that the following data is a real packet and the payload size is used primarily to optimize python
            // runtime speed (see python library code for details). Then, transmits the COBS-encoded packet and the
            // following CRC checksum postamble. Expects the receiver to know that the packet stops at the delimiter
            // byte value and to know the fixed size of the postamble to properly parse the CRC checksum. Combining as
            // much of the data into a unified packet as possible allows optimizing the transmission process, which
            // works better for bigger chunks of data than many small byte-sized inputs.
            uint8_t preamble[2] = {kStartByte, static_cast<uint8_t>(_bytes_in_transmission_buffer)};

            // Note, the preamble concept is only used when sending data intended to be processed by the Python
            // companion library. When receiving data from PC, only the start byte is expected, as compiled c-code does
            // not really gain anything from knowing the size of the payload ahead of time. This asymmetry is
            // unfortunate for code maintainability, but saves sending an extra byte where it is not really needed.
            _port.write(preamble, sizeof(preamble));
            _port.write(_transmission_buffer, combined_size);

            // Communicates that the packet has been sent via the transfer_status variable
            transfer_status =
                static_cast<uint8_t>(stp_shared_assets::kSerializedTransferProtocolStatusCodes::kPacketSent);

            // Resets the transmission_buffer after every successful transmission
            ResetTransmissionBuffer();

            return true;  // Returns true to indicate that the packet has been sent
        }

        // If the combined_size is 0, this indicates that the packet has not been constructed successfully. Then,
        // returns 0 to indicate data sending failed. Since ConstructPacket() method automatically sets the
        // transfer_status variable to the appropriate error code, does not modify the status code.
        return false;
    }

    /**
     * @brief Parses the bytes available from the bundled Serial class reception buffer into a packet, verifies its
     * integrity and unpacks its payload into the _reception_buffer.
     *
     * This is a master-method that aggregates all steps necessary to correctly receive a data packet encoded in a byte
     * stream stored inside the reception circular buffer of the bundled Stream class. Specifically, when triggered, the
     * method first reads the packet and the CRC checksum postamble for the packet from the Stream reception buffer into
     * the class _reception_buffer. If this operation succeeds, the method then validates the integrity of the packet
     * using CRC checksum and then unpacks the payload of the message using COBS decoding in-place.
     *
     * @note Following the successful runtime of this method, the number of payload bytes received from the PC can be
     * obtained using get_bytes_in_reception_buffer() method.
     *
     * @returns bool true if the packet was successfully received and unpacked, false otherwise. If method runtime
     * fails, use the transfer_status variable to determine the reason for the failure, as it would be set to the
     * specific error code of the failed operation. Transfer_status values are guaranteed to uniquely match one of the
     * enumerators stored inside the kCOBSProcessorCodes, kCRCProcessorCodes or kSerializedTransferProtocolStatusCodes
     * enumerations available through the stp_shared_assets namespace.
     * enumeration.
     *
     * Example usage:
     * @code
     * Serial.begin(9600);
     * SerializedTransferProtocol<uint16_t, 254, 254> stp_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
     * bool data_received = serial_protocol.ReceiveData();
     * @endcode
     */
    bool ReceiveData()
    {
        // Resets the reception buffer, and it's bytes tracker variable to prepare for receiving the next packet.
        ResetReceptionBuffer();

        // Attempts to parse a packet (and its CRC checksum postamble) from the Serial port buffer if bytes to read are
        // available
        uint16_t packet_size = ParsePacket();

        // If the returned packet_size is 0, that indicates an error has occurred during the parsing process.
        // ParsePacket() automatically sets the class transfer_status variable to the appropriate error code, so just
        // returns 'false' top explicitly indicate failure and break method runtime.
        if (packet_size == 0)
        {
            return false;
        }

        // If the returned packet size is not 0, this indicates that the packet has been successfully parsed and now
        // occupies packet_size number of bytes inside the _reception_buffer. Additionally, this means that
        // kPostambleSize bytes immediately following the packet are filled with the CRC checksum for the packet. In
        // this case, verifies the integrity of the packet by running the CRC checksum calculator on the packet + the
        // appended CRC checksum.
        uint16_t payload_size = ValidatePacket(packet_size);

        // Similar to the above, if returned payload_size is 0, that indicates that an error was encountered during
        // packet validation or decoding, which is communicated to the caller process by returning false. The
        // transfer_status is automatically set to error code during ValidatePacket() runtime.
        if (payload_size == 0)
        {
            return false;
        }

        // If the method reaches this point, the packet has been successfully received, validated and unpacked. The
        // payload is now available for consumption through the _reception_buffer.
        _bytes_in_reception_buffer = payload_size;  // Records the number of unpacked payload bytes to tracker

        // Sets the status appropriately and returns 'true' to indicate successful runtime.
        transfer_status =
            static_cast<uint8_t>(stp_shared_assets::kSerializedTransferProtocolStatusCodes::kPacketReceived);
        return true;
    }

    /**
     * @brief Writes the input object as bytes into the _transmission_buffer payload region starting at the specified
     * start_index.
     *
     * This method modifies the _transmission_buffer by (over)writing the specific portion of the buffer with the bytes
     * copied from the input object. That means, the buffer remains intact (as-is) everywhere except for the overwritten
     * area. To reset the whole buffer, use the ResetTransmissionBuffer() method. The input object is not modified in
     * any way by this method.
     *
     * @warning If the requested start_index and provided_bytes combination exceeds kMaximumTransmittedPayloadSize
     * class template parameter value, the method will abort and return 0 to indicates no bytes were
     * written.
     *
     * @note This method operates specifically on the bytes allocated for the payload of the data packet. It implicitly
     * handles the necessary transformations of the start_index to make sure start_index 0 corresponds to the start
     * index of the payload (at least 1) and that the end index never exceeds the maximum end_index of the payload
     * (254). This makes this method a safe way of modifying the payload with respect to the _transmission_buffer layout
     * heuristics necessary for other class methods to work as intended.
     *
     * @tparam ObjectType The type of the object to from which the bytes will be copied over to the
     * _transmission_buffer. This parameter used by the template to correctly configure the method instance to accept
     * any input object type and therefore be type-agnostic.
     * @param object The object from which the bytes are copied. Passed as a constant reference to reduce memory
     * overhang as the object itself is not modified in any way.
     * @param start_index The index inside the _transmission_buffer payload, from which to start writing bytes. Minimum
     * value is 0, maximum value is defined by kMaximumTransmittedPayloadSize (but no more than 254). This index
     * specifically applies to the payload, not the buffer as a whole.
     * @param provided_bytes The number of bytes to write to the transmission_buffer. In most cases this should be left
     * blank as it allows the method to use the value returned by sizeOf() of the ObjectType (writing as many bytes as
     * supported by the object type).
     *
     * @returns uint16_t The index immediately following the last overwritten index of the _transmission_buffer. This
     * value can be used as the starting index for subsequent write operations to ensure data contiguity. If method
     * runtime fails, returns 0 to indicate no bytes were written to the payload and saves the specific error code that
     * describes the failure to transfer_status class variable.
     *
     * Example usage:
     * @code
     * Serial.begin(9600);
     * SerializedTransferProtocol<uint16_t, 254, 254> stp_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
     * uint16_t value = 44321;
     * uint8_t array[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
     * struct MyStruct
     * {
     *    uint8_t a = 60;
     *    uint16_t b = 12345;
     *    uint32_t c = 1234567890;
     * } test_structure;
     *
     * // Writes tested objects to the _transmission_buffer
     * uint16_t next_index = serial_protocol.WriteData(value);
     * uint16_t next_index = serial_protocol.WriteData(array, next_index);
     * uint16_t next_index = serial_protocol.WriteData(test_structure, next_index);
     * @endcode
     */
    template <typename ObjectType>
    uint16_t WriteData(
        const ObjectType& object,
        const uint16_t& start_index    = 0,
        const uint16_t& provided_bytes = sizeof(ObjectType)
    )
    {
        // Increments the start_index by 1. This is to conform to the _transmission_buffer layout where the index 0 is
        // always reserved for the overhead byte. Payloads can only be found between indices 1 and 254.
        uint16_t local_start_index = start_index + 1;

        // Calculates the total size of the payload that would be required to accommodate provided_bytes number of bytes
        // inserted starting with start_index.
        uint16_t required_size = local_start_index + provided_bytes;

        // Verifies that the payload has enough space to accommodate writing the input object using provided start index
        // Note, uses kMaximumTransmittedPayloadSize + 1 to account for the overhead byte that implicitly modifies the
        // 'required_size' calculation.
        if (required_size > kMaximumTransmittedPayloadSize + 1)
        {
            // If the payload does not have enough space, returns 0 to indicate no bytes were written and sets
            // transfer_status to the appropriate error code
            transfer_status = static_cast<uint8_t>(
                stp_shared_assets::kSerializedTransferProtocolStatusCodes::kWritePayloadTooSmallError
            );
            return 0;
        }

        // If there is enough space in the payload to accommodate the data, uses memcpy to efficiently copy the data
        // into the _transmission_buffer.
        memcpy(
            static_cast<void*>(&_transmission_buffer[local_start_index]
            ),                                  // Destination in the buffer to start writing to
            static_cast<const void*>(&object),  // Source object address to copy from
            provided_bytes                      // The number of bytes to write into the buffer
        );

        // If writing to buffer caused the size of the payload to be larger than the size tracker by the
        // _bytes_in_transmission_buffer, updates the tracker to store the new size. This way, the tracker is only
        // updated whenever the used size of the payload increases and ignores overwrite operations. This is somewhat
        // risky, and it mandates that the tracker is reset after each SendData() operation. Also, the entire buffer
        // has to be cleared and re-written if the payload size needs to be reduced (there is no other mechanism to do
        // it right now). Note, -1 subtracts the size of the overhead byte so that the user always inputs and receives
        // the size of the payload that excludes the overhead byte.
        _bytes_in_transmission_buffer = max(_bytes_in_transmission_buffer, static_cast<uint16_t>(required_size - 1));

        // Sets the status code to indicate writing to buffer was successful
        transfer_status =
            static_cast<uint8_t>(stp_shared_assets::kSerializedTransferProtocolStatusCodes::kBytesWrittenToBuffer);

        // Also returns the index immediately following the last updated (overwritten) index of the buffer to caller to
        // support chained method calls.
        return required_size - 1;
    }

    /**
     * @brief Reads the requested_bytes number of bytes from the _reception_buffer payload region starting at the
     * start_index into the provided object.
     *
     * @note This operation does not modify _reception_buffer.
     *
     * This method copies the data from the _reception_buffer into the provided object, modifying the object. Data can
     * be copied from the _reception_buffer any number of times using arbitrary byte-counts and starting positions, as
     * long as the ending index of the read operation remains within the payload boundaries. The method explicitly
     * prevents reading the data outside of the payload boundaries as it may be set to valid values that nevertheless
     * are meaningless as they are leftover from previously processed payloads (this stems from the fact the buffer is
     * never fully reset and instead only partially overwritten with data). To actually update the _reception_buffer,
     * you need to use ReceiveData() or ResetReceptionBuffer() methods.
     *
     * @warning If the requested start_index and requested_bytes combination exceeds the size of the available payload,
     * provided by the value of the _bytes_in_reception_buffer tracker, the method will abort and return 0 to indicates
     * no bytes were read. Use get_bytes_in_reception_buffer() method to obtain the current value of the tracker.
     *
     * @note This method operates specifically on the bytes allocated for the payload of the data packet. It implicitly
     * handles the necessary transformations of the start_index to make sure start_index 0 corresponds to the start
     * index of the payload (at least 1) and that the end index never exceeds the end index of the payload inside the
     * buffer (provided by _bytes_in_reception_buffer) value. This makes this method a safe way of modifying the payload
     * with respect to the _reception_buffer layout heuristics necessary for other class methods to work as intended.
     *
     * @tparam ObjectType The type of the object to which the read bytes would be written. This is used by template to
     * correctly configure the method instance to accept any input object type and therefore be type-agnostic.
     * @param object The object to write the data to. Passed as the reference to the object to enable direct object
     * manipulation.
     * @param start_index The index inside the reception_buffer's payload, from which to start reading bytes.
     * Minimum value is 0, maximum value is defined by _bytes_in_transmission_buffer - 1.
     * @param requested_bytes The number of bytes to read from the _reception_buffer. In most cases this should be
     * left blank as it allows the function to use the value returned by sizeOf() of the ObjectType (requesting as many
     * bytes as supported by the object type).
     *
     * @returns uint16_t The index inside the payload stored in the _reception_buffer that immediately follows the
     * final index of the data that was read by the method into the provided object. This allows to use the output of
     * the method as a start_index for subsequent read operations to ensure data contiguity. If method runtime fails,
     * returns 0 to indicate no bytes were read from the payload and saves the specific error code that describes the
     * failure to transfer_status class variable.
     *
     * Example usage:
     * @code
     * Serial.begin(9600);
     * SerializedTransferProtocol<uint16_t, 254, 254> stp_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
     * uint16_t value = 44321;
     * uint8_t array[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
     * struct MyStruct
     * {
     *    uint8_t a = 60;
     *    uint16_t b = 12345;
     *    uint32_t c = 1234567890;
     * } test_structure;
     *
     * // Overwrites the test objects with the data stored inside the buffer
     * uint16_t next_index = serial_protocol.ReadData(value);
     * uint16_t next_index = serial_protocol.ReadData(array, next_index);
     * uint16_t next_index = serial_protocol.ReadData(test_structure, next_index);
    * @endcode
     */
    template <typename ObjectType>
    uint16_t ReadData(
        ObjectType& object,  // Not constant by design, while the reference is constant, the object itself is mutable.
        const uint16_t& start_index     = 0,
        const uint16_t& requested_bytes = sizeof(ObjectType)
    )
    {
        // Increments the start_index by 1. This is to conform to the _reception_buffer layout where the index 0 is
        // always reserved for the overhead byte. Payloads can only be found between indices 1 and 254.
        uint16_t local_start_index = start_index + 1;

        // Calculates the total size of the payload that is sufficient to accommodate _requested_bytes number of bytes
        // stored upstream of the start_index.
        uint16_t requested_size = local_start_index + requested_bytes;

        // Verifies that the payload has enough bytes to be read. Uses _bytes_in_reception_buffer as it stores the
        // accurate size of the payload inside the buffer. The payload size tracker is modified by +1 to cancel out the
        // local_start_index including the overhead byte.
        if (requested_size > _bytes_in_reception_buffer + 1)
        {
            // If the payload does not have enough bytes, returns 0 to indicate no bytes were read and sets
            // transfer_status to the appropriate error code
            transfer_status = static_cast<uint8_t>(
                stp_shared_assets::kSerializedTransferProtocolStatusCodes::kReadPayloadTooSmallError
            );
            return 0;
        }

        // If there are enough bytes in the payload to read, uses memcpy to efficiently copy the data into the
        // object from the reception_buffer.
        memcpy(
            static_cast<void*>(&object),                                      // Destination object to write the data to
            static_cast<const void*>(&_reception_buffer[local_start_index]),  // Source to read the data from
            requested_bytes  // The number of bytes to read into the object
        );

        // Critical difference from the WriteData() method: There is no _bytes_in_reception_buffer updating. This is
        // because the read operation does not modify the buffer in any way. There is still a returned value that
        // indicates where the read operation stopped, relative to the payload stored inside the buffer.

        // Sets the status code to indicate reading from buffer was successful
        transfer_status =
            static_cast<uint8_t>(stp_shared_assets::kSerializedTransferProtocolStatusCodes::kBytesReadFromBuffer);

        // Also returns the index immediately following the index of the final read byte (relative to the payload) to
        // caller. This index can be used as the next input start_index if multiple read calls are chained together.
        return requested_size - 1;
    }

  private:
    /// The reference to the Stream class object used to transmit and receive data. This variable is made public to
    /// support class testing via the use of StreamMock class. When it is set to an instance of StreamMock class, it can
    /// be used to directly access the mock buffers to evaluate the performance of the SerializedTransferProtocol class.
    Stream& _port;

    /// The local instance of the COBSProcessor class that provides the methods to encode and decode packets using
    /// COBS protocol. See the class documentation for more details on the process and the functionality of the class.
    COBSProcessor _cobs_processor;

    /// The local instance of the CRCProcessor class that provides the methods to calculate the CRC checksum for packets
    /// and save and read the checksum from class buffers. See the class documentation for more details on the process
    /// and the functionality of the class.
    CRCProcessor<PolynomialType> _crc_processor;

    /// The byte used to indicate the start of the packet. Encountering this byte in the evaluated incoming byte-stream
    /// is the only trigger that starts packet reception cycle.
    const uint8_t kStartByte;

    /// The byte used to indicate the end of the packet. All instances of this byte in the payload are eliminated using
    /// COBS. Encountering this byte is only of way to end packet reception procedure without triggering an error.
    const uint8_t kDelimiterByte;

    /// The maximum number of microseconds (us) to wait between receiving bytes of the packet. When in packet reception
    /// mode, the algorithm will wait for the specified number of microseconds before declaring the packet stale and
    /// aborting the reception procedure. This is the only way to abort packet reception cycle other than encountering
    /// the delimiter byte value.
    const uint32_t kTimeout;

    /// A boolean flag that controls whether ParsePacket() method raises errors when it is unable to find the start
    /// byte of the packet. The default behavior is to disable such errors as they are somewhat common due to having
    /// noise in the communication lines. However, some users m,ay want to enable these errors to assist debugging,
    /// so the option is preserved to be user-controllable.
    const bool kAllowStartByteErrors;

    /// Stores the byte-size of the postamble, which currently statically depends on the declared PolynomialType.
    /// The postamble is the portion of the data that immediately follows each received and transmitted packet and, at
    /// the time of writing, only includes the CRC checksum for the packet. To optimize data transfer, the postamble
    /// is appended to the specifically reserved portion of the _transmission_buffer and received into the specific
    /// portion of the _reception_buffer, rather than being stored in a separate buffer.

    static constexpr uint8_t kPostambleSize = sizeof(PolynomialType);  // NOLINT(*-dynamic-static-initializers)

    /// Stores the size of the _transmission_buffer array, which is statically set to the maximum transmitted payload
    /// size (kMaximumTransmittedPayloadSize template parameter) + 2 + size of the postamble. The +2 accounts for the
    /// overhead byte and delimiter byte and the + kPostambleSize accounts for the CRC checksum placeholder space at the
    /// end of the buffer (used to do zero-return CRC checks on the packet).
    static constexpr uint16_t kMaximumTxBufferSize =  // NOLINT(*-dynamic-static-initializers)
        kMaximumTransmittedPayloadSize + 2 + kPostambleSize;

    /// Stores the size of the _reception_buffer array, which is statically set to the maximum received payload size
    /// (kMaximumReceivedPayloadSize template parameter) + 2 + size of the postamble. The +2 accounts for the overhead
    /// byte and delimiter byte and the + kPostambleSize accounts for the CRC checksum placeholder space at the end of
    /// the buffer (used to do zero-return CRC checks on the packet).
    static constexpr uint16_t kMaximumRxBufferSize =  // NOLINT(*-dynamic-static-initializers)
        kMaximumReceivedPayloadSize + 2 + kPostambleSize;

    /// The buffer that stages the payload data before it is transmitted to the PC. The buffer is constructed with the
    /// assumption that the first index is always reserved for the overhead byte of each transmitted packet (after the
    /// payload is packetized using COBS). Also, the size of the buffer ensures there is always enough space after the
    /// data is packetized with COBS to accommodate appending the postamble to the packet stored inside the buffer.
    uint8_t _transmission_buffer[kMaximumTxBufferSize];

    /// The buffer that stores the data received from the PC. The buffer is constructed with the assumption that the
    /// first index is always reserved for the overhead byte of each received packet. The buffer is always set to
    /// accommodate the maximum allowed payload size + overhead and delimiter bytes of the packet + the postamble.
    uint8_t _reception_buffer[kMaximumRxBufferSize];

    /// Tracks the number of payload bytes inside the transmission buffer. This variable is modified each time data
    /// is written to the buffer and it is set to the index immediately following the last index of the buffer that was
    /// written to.
    uint16_t _bytes_in_transmission_buffer = 0;

    /// Tracks the number of payload bytes inside the reception buffer. This variable is modified each time a new
    /// payload is received from the PC. Since reading from buffer is non-destructive, reading does not modify this
    /// tracker variable.
    uint16_t _bytes_in_reception_buffer = 0;

    /**
     * @brief Constructs the serial packet using the payload stored inside the _transmission_buffer and the
     * bytes_in_transmission_buffer payload size tracker.
     *
     * Specifically, first uses COBS encoding to eliminate all instances of the delimiter_byte value inside the payload
     * before appending the delimiter to the end of the newly constructed packet. This generates a packet with a maximum
     * size of 256 and it includes both the overhead byte at index 0 and the delimiter that immediately follows the last
     * byte of the payload.
     *
     * Next, calculates the CRC checksum for the constructed packet and adds it to the _transmission_buffer right after
     * the packet (the maximum packet size achieved after this operation is currently 260 bytes).
     *
     * @note This method is intended to be called only within the SerializedTransferProtocol class.
     *
     * @returns uint16_t The combined size of the packet and the CRC checksum postamble in bytes (260 bytes maximum).
     * If method runtime fails, returns 0 to indicate that the packet was not constructed and uses transfer_status to
     * communicate the error code of the specific operation that failed.
     *
     * Example usage:
     * @code
     * uint16_t combined_size = ConstructPacket();
     * @endcode
     */
    inline uint16_t ConstructPacket()
    {
        // Carries out in-place payload encoding using COBS algorithm. Relies on the bytes_in_transmission_buffer
        // tracker to communicate the payload size. Implicitly uses overhead byte placeholder at index 0, this class is
        // constructed in a way that the user does not need to care about where the placeholder as all class methods
        // handle this heuristic requirement implicitly.
        uint16_t packet_size =
            _cobs_processor.EncodePayload(_transmission_buffer, _bytes_in_transmission_buffer, kDelimiterByte);

        // If the encoder runs into an error, it returns 0 to indicate that the payload was not encoded. In this
        // case, transfers the error status code from the COBS processor status tracker to transfer_status and returns
        // 0 to indicate packet construction failed.
        if (packet_size == 0)
        {
            transfer_status = _cobs_processor.cobs_status;
            return 0;
        }

        // If COBS encoding succeeded, calculates the CRC checksum on the encoded packet. This includes the overhead
        // byte and the delimiter byte added during COBS encoding. Note, uses the CRC type specified by the
        // PolynomialType class template parameter to automatically scale with all supported CRC types.
        PolynomialType checksum = _crc_processor.CalculatePacketCRCChecksum(_transmission_buffer, 0, packet_size);

        // If the CRC calculator runs into an error, as indicated by its status code not matching the expected success
        // code, transfers the error status to the transfer_status and returns 0 to indicate packet construction failed
        if (_crc_processor.crc_status !=
            static_cast<uint8_t>(stp_shared_assets::kCRCProcessorCodes::kCRCChecksumCalculated))
        {
            transfer_status = _crc_processor.crc_status;
            return 0;
        }

        // Writes the calculated CRC checksum to the _transmission_buffer at the position immediately following the
        // encoded packet. This way, the PC can runt he CRC calculation on the received data and quickly verify its
        // integrity using the zero-expected-return CRC check method. Note, this relies on the storage buffers being
        // constructed in a way that always reserves enough space for the used CRC checksum, regardless of the
        // payload size.
        uint16_t combined_size = _crc_processor.AddCRCChecksumToBuffer(_transmission_buffer, packet_size, checksum);

        // If CRC addition fails, as indicated by the returned combined size being 0, transfers the specific error
        // status to the transfer_status and returns 0 to indicate packet construction failed.
        if (combined_size == 0)
        {
            transfer_status = _crc_processor.crc_status;
            return 0;
        }

        // If the algorithm reaches this point, this means that the payload has been successfully encoded and
        // checksummed and the CRC checksum has been added to the end of the encoded packet. Sets the transfer_status
        // appropriately and returns the combined size of the packet and the added CRC checksum to let the caller know
        // how many bytes to transmit to the PC.
        transfer_status =
            static_cast<uint8_t>(stp_shared_assets::kSerializedTransferProtocolStatusCodes::kPacketConstructed);
        return combined_size;
    }

    /**
     * @brief Parses the incoming bytes from the bundled Stream class reception buffer into the reception_buffer array
     * of this class.
     *
     * Specifically, if bytes are available for reading, scans through all available bytes until a start byte value
     * defined by kStartByte is found. Once the start byte is found, enters a while loop that iteratively reads all
     * following bytes into the _reception_buffer until either kDelimiterByte value is encountered, the loop reads the
     * kMaximumRxBufferSize - kPostambleSize of bytes or a timeout of kTimeout microseconds is reached while waiting for
     * more bytes to become available.
     *
     * @attention This method is asymmetric to the similar method used in the PC (python) version of the library.
     * Specifically, whereas python parses makes use of payload_size byte sent as part of the preamble, this method
     * does NOT expect or account for the payload size and expects the preamble to be made up solely of the start_byte,
     * immediately followed by the overhead byte. This is a bit sub-optimal for code maintainability, but has to be done
     * this way as PC does benefit from knowing payload_size before parsing and the controller does not (has to do
     * purely with how python client is implemented).
     *
     * If the packet's delimiter byte was found, the method then reads the kPostambleSize number of bytes that follow
     * the delimiter adn saves them into the _reception_buffer immediately after the packet.
     *
     * @note This method is intended to be called only within the SerializedTransferProtocol class.
     *
     * @returns uint16_t The number of packet bytes read into the reception_buffer. If method runtime fails, returns 0
     * to indicate no packet bytes were read. If 0 is returned, transfer_status is set to the appropriate error code
     * that communicates the specific operation that resulted in the failure.
     *
     * Example usage:
     * @code
     * uint16_t packet_size = ParsePacket();
     * @endcode
     */
    inline uint16_t ParsePacket()
    {
        elapsedMicros timeout_timer;  // The timer that disengages the loop if the packet stales
        uint16_t bytes_read = 0;      // Tracks the number of bytes read from the Serial port into the buffer

        // First, attempts to find the start byte of the packet. The start byte is used to tell the receiver that the
        // following data belongs to a supposedly well-formed packet and should be retained (written to the buffer) and
        // not discarded.
        while (_port.available())
        {
            // Note, the start byte itself is not saved, which is the intended behavior.
            if (_port.read() == kStartByte)
            {
                // Sets the status to indicate start byte has been found. The status is immediately used below to
                // evaluate loop runtime
                transfer_status = static_cast<uint8_t>(
                    stp_shared_assets::kSerializedTransferProtocolStatusCodes::kPacketStartByteFound
                );
                break;
            }
        }

        // If the start byte was found, as indicated by the status variable, enters packet parsing loop
        if (transfer_status ==
            static_cast<uint8_t>(stp_shared_assets::kSerializedTransferProtocolStatusCodes::kPacketStartByteFound))
        {
            timeout_timer = 0;  // Resets the timer to 0 before entering the loop

            // Loops either until the process times out (packet stales) or until the maximum writeable buffer index is
            // reached. The loop contains an internal 'break' to end early if delimiter_byte is encountered (correct
            // exit condition, the other two exit conditions are error-conditions). Note, the buffer space for the crc
            // checksum is excluded from the space available for the packet, which is done to make sure it is always
            // available for the CRC check.
            while (timeout_timer < kTimeout && bytes_read < kMaximumRxBufferSize - kPostambleSize)
            {
                // Uses a separate 'if' to check whether bytes to parse are available to enable waiting for the packet
                // bytes to become available if at some point the entire reception buffer becomes consumed. In this
                // case, 'while' loop will block in-place until more bytes are received or the timeout is reached.
                if (_port.available())
                {
                    // Consumes and writes the next available byte to a temporary storage variable
                    uint8_t byte_value = _port.read();
                    // Saves the byte to the appropriate buffer position
                    _reception_buffer[bytes_read] = byte_value;
                    // Increments the bytes_read to iteratively move along the buffer and add new data
                    bytes_read += 1;

                    // If delimiter byte value is encountered, breaks out of the while loop and goes into postamble
                    // parsing mode. Due to COBS encoding, the delimiter byte is ONLY found at the end of the packet and
                    // not inside the packet payload unless the packet has been corrupted. Packet corruption is jointly
                    // verified via the CRC checksum and COBS decoding.
                    if (byte_value == kDelimiterByte)
                    {
                        // Uses the status to communicate that the delimiter byte has been found
                        transfer_status = static_cast<uint8_t>(
                            stp_shared_assets::kSerializedTransferProtocolStatusCodes::kPacketDelimiterByteFound
                        );
                        break;
                    }

                    timeout_timer = 0;  // Resets the timer whenever a byte is successfully read and the loop is active
                }
            }
        }
        // If the start byte was not found, aborts the method and returns 0 to indicate that no data was parsed as no
        // packet was available.
        else
        {
            // Note, selects the status based on the value of the kAllowStartByteErrors flag
            if (kAllowStartByteErrors)
            {
                transfer_status = static_cast<uint8_t>(
                    stp_shared_assets::kSerializedTransferProtocolStatusCodes::kPacketStartByteNotFoundError
                );
            }
            else
            {
                transfer_status = static_cast<uint8_t>(
                    stp_shared_assets::kSerializedTransferProtocolStatusCodes::kNoBytesToParseFromBuffer
                );
            }
            return 0;
        }

        // If the delimiter byte has been found and, therefore, the packet has been fully parsed, enters postamble
        // parsing mode. The postamble size is fixed and, at the time of writing, depends on the used PolynomialType
        // (the postamble only stores the CRC checksum for the received packet), so uses a 'for' loop here.
        // Each iteration of the loop blocks until either a timeout occurs or the bytes are available for reading to
        // once-again realize a mechanism of waiting for bytes to become available if the reception buffer becomes fully
        // consumed.
        if (transfer_status ==
            static_cast<uint8_t>(stp_shared_assets::kSerializedTransferProtocolStatusCodes::kPacketDelimiterByteFound))
        {
            // Loops over each postamble byte and attempts to read it from the incoming stream
            for (uint8_t i = 0; i < kPostambleSize; i++)
            {
                timeout_timer = 0;  // Resets timeout timer for each instance of the loop

                // Blocks until bytes are available for reading
                while (!(_port.available()))
                {
                    // If the timer exceeds the timeout value while inside the loop, immediately breaks with code 0 and
                    // uses the status to communicate that the postamble parsing failed
                    if (timeout_timer >= kTimeout)
                    {
                        transfer_status = static_cast<uint8_t>(
                            stp_shared_assets::kSerializedTransferProtocolStatusCodes::kPostambleTimeoutError
                        );
                        return 0;
                    }

                    // Note, there is no check for the saving index to be outside buffer boundaries. This is
                    // intentional and safe, as the buffer size always includes the kPostambleSize and due to
                    // the discounting in the loop above, the packet is never allowed to use the postamble placeholder
                    // space.
                }

                // If the postamble byte is available, reads it into the _reception_buffer buffer. Uses bytes_read to
                // offset the writing index so that the postamble is added directly after the parsed packet
                _reception_buffer[bytes_read + i] = _port.read();
            }

            // If this point in the processing pipeline is reached, this means that the postamble was successfully
            // parsed. Since this is the last step of packet reception, sets the status appropriately and returns the
            // packet size to caller.
            transfer_status =
                static_cast<uint8_t>(stp_shared_assets::kSerializedTransferProtocolStatusCodes::kPacketParsed);

            // Since bytes_read directly corresponds to the packet size, returns this to the caller
            return bytes_read;
        }
        // Otherwise, if the delimiter byte has not been found, returns code 0 and sets the status to appropriately
        // communicate the reason for parsing failure
        else
        {
            // If bytes_read tracker is set to a value beyond the indexable range of the buffer, this indicates
            // that the packet was not parsed due to running out of buffer space (most likely an issue with the
            // delimiter byte)
            if (bytes_read >= kMaximumRxBufferSize - kPostambleSize)
            {
                transfer_status = static_cast<uint8_t>(
                    stp_shared_assets::kSerializedTransferProtocolStatusCodes::kPacketOutOfBufferSpaceError
                );
            }

            // Otherwise, the packet was not parsed due to timing out while waiting for packet bytes. This may be both
            // due to staling (transmission error) and invalid delimiter byte setting (for example, if the PC uses a
            // different value from what the class is set to expect).
            else
            {
                transfer_status =
                    static_cast<uint8_t>(stp_shared_assets::kSerializedTransferProtocolStatusCodes::kPacketTimeoutError
                    );
            }

            return 0;
        }
    }

    /**
     * @brief Validates successfully parsed packets and decodes them to extract the payload.
     *
     * @attention Assumes that the ParsePacket() appends the crc checksum received with the packet to the end of each
     * parsed packet.
     *
     * Specifically, first runs the CRC calculator on the portion of the buffer that holds the packet and the postamble
     * CRC checksum, expecting to get a 0 returned checksum. If the CRC check is successful (returns 0), decodes the
     * packet using COBS to obtain the payload. See CRCProcessor documentation if you wonder why this looks for a 0
     * return value for the CRC check, but this is a property of the CRC.
     *
     * @note This method is intended to be called only within the SerializedTransferProtocol class and only after the
     * ParsePacket() method has been successfully called.
     *
     * @param packet_size The size of the packet that was parsed into the _reception_buffer by the ParsePacket() method
     * in bytes.
     *
     * @returns uint16_t The number of bytes making up the payload. If method runtime fails, returns 0 to indicate that
     * no bytes passed the verification and uses transfer_status to communicate the error code of the specific operation
     * that resulted in the failure.
     *
     * Example usage:
     * @code
     * uint16_t packet_size = ParsePacket();
     * uint16_t payload_size = ValidatePacket(packet_size);
     * @endcode
     */
    inline uint16_t ValidatePacket(uint16_t packet_size)
    {
        // Calculates the combined data size of the packet + crc bytes. This is needed to correctly calculate the
        // portion of the buffer to run the CRC check on.
        uint16_t combined_size = packet_size + static_cast<uint16_t>(sizeof(PolynomialType));

        // Runs the CRC calculator on the reception buffer stretch that holds the data + CRC checksum. This relies on
        // the signature property of the CRC, namely that adding the CRC checksum to the data for which the checksum was
        // calculated and re-running the CRC on this combined data will always return 0. Assumes that the CRC was stored
        // as bytes starting with the highest byte first, otherwise this check will not work. Also assumes that the
        // ParsePacket() method adds the CRC postamble to the portion of the buffer immediately following the packet.
        PolynomialType packet_checksum = _crc_processor.CalculatePacketCRCChecksum(_reception_buffer, 0, combined_size);

        // Verifies that the CRC calculator ran without errors and returned the success status. If not, sets
        // the transfer_status to the returned crc status and returns 0 to indicate runtime error.
        if (_crc_processor.crc_status !=
            static_cast<uint8_t>(stp_shared_assets::kCRCProcessorCodes::kCRCChecksumCalculated))
        {
            transfer_status = _crc_processor.crc_status;
            return 0;
        }

        // If the crc calculation runtime was successful, ensures that the calculated CRC checksum is 0, which is the
        // expected correct checksum for an uncorrupted packet
        else if (packet_checksum != 0)
        {
            // If the returned checksum is not 0, that means that the packet failed the CRC check and is likely
            // corrupted
            transfer_status =
                static_cast<uint8_t>(stp_shared_assets::kSerializedTransferProtocolStatusCodes::kCRCCheckFailed);
            return 0;
        }

        // If the CRC check succeeds, attempts to decode COBS-encoded data. This serves two purposes. First, it restores
        // any encoded variable that was previously an instance of kDelimiterByte back to the original value. Second,
        // it acts as a secondary verification step, since COBS encoding ensures the data is organized in a particular
        // fashion and if that is not true, the data is likely corrupted and the CRC failed to recognize that.
        uint16_t payload_size = _cobs_processor.DecodePayload(_reception_buffer, packet_size, kDelimiterByte);

        // Verifies that the COBS decoder runtime was successful. Uses the heuristic that the successful COBS
        // decoder runtime always returns a non-zero payload_size, and an erroneous one always returns 0 to simplify the
        // 'if' check. If payload_size is 0, sets the transfer_status to the returned status code and returns 0 to
        // indicate runtime error.
        if (payload_size == 0)
        {
            transfer_status = _cobs_processor.cobs_status;
            return 0;
        }

        // If COBS decoding was successful, sets the packet status appropriately and returns the payload size to caller
        transfer_status =
            static_cast<uint8_t>(stp_shared_assets::kSerializedTransferProtocolStatusCodes::kPacketValidated);
        return payload_size;
    }
};

#endif  //AMC_SERIAL_TRANSFER_PROTOCOL_H
