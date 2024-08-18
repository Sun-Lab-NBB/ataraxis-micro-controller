/**
 * @file
 * @brief The header-only file that stores all general assets intended to be shared between library classes wrapped
 * into a common namespace.
 *
 * @section axmc_sa_description Description:
 *
 * This file contains:
 * - kCOBSProcessorCodes enumeration that stores status bytecodes used by the COBSProcessor to report its runtime
 * status.
 * - kCRCProcessorCodes enumeration that stores status bytecodes used by the CRCProcessor to report its runtime status.
 * - kTransportLayerStatusCodes enumeration that stores status bytecodes used by the TransportLayer to report its
 * runtime status.
 * - kCommunicationStatusCodes enumeration that stores status bytecodes used by the Communication to report its runtime
 * status.
 * - A custom implementation of the is_same_v() standard method used to verify that two input objects have the same
 * type.
 *
 * @section axmc_sa_developer_notes Developer Notes:
 *
 * The primary reason for having this file is to store all byte-code enumerations in the same place. To simplify
 * error handling, all codes available through this namespace have to be unique relative to each other. For example, if
 * value 11 is used to represent 'Standby' state for CRCProcessor, no other status should be using value 11).
 *
 * @section axmc_sa_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 */

#ifndef AXMC_SHARED_ASSETS_H
#define AXMC_SHARED_ASSETS_H

// Dependencies
#include <Arduino.h>

/**
 * @namespace shared_assets
 * @brief Provides all assets (structures, enumerations, functions) that are intended to be shared between the classes
 * of the library.
 *
 * The shared assets are primarily used to simplify library development by storing co-dependent assets in the same
 * place. Additionally, it simplifies using these assets with template classes from the library.
 */
namespace shared_assets
{
    /**
     * @enum kCOBSProcessorCodes
     * @brief Assigns meaningful names to all status codes used by the COBSProcessor class.
     *
     * @note Due to the unified approach to status-coding in this library, this enumeration should only use code values
     * in the range of 11 through 50. This is to simplify chained error handling in the TransportLayer class of the
     * library.
     */
    enum class kCOBSProcessorCodes : uint8_t
    {
        kStandby                       = 11,  ///< The value used to initialize the cobs_status variable
        kEncoderTooSmallPayloadSize    = 12,  ///< Encoder failed to encode payload because payload size is too small
        kEncoderTooLargePayloadSize    = 13,  ///< Encoder failed to encode payload because payload size is too large
        kEncoderPacketLargerThanBuffer = 14,  ///< Encoded payload buffer is too small to accommodate the packet
        kPayloadAlreadyEncoded         = 15,  ///< Cannot encode payload as it is already encoded (overhead value != 0)
        kPayloadEncoded                = 16,  ///< Payload was successfully encoded into a transmittable packet
        kDecoderTooSmallPacketSize     = 17,  ///< Decoder failed to decode the packet because packet size is too small
        kDecoderTooLargePacketSize     = 18,  ///< Decoder failed to decode the packet because packet size is too large
        kDecoderPacketLargerThanBuffer = 19,  ///< Packet size to be decoded is larger than the storage buffer size
        kDecoderUnableToFindDelimiter  = 20,  ///< Decoder failed to find the delimiter at the end of the packet
        kDecoderDelimiterFoundTooEarly = 21,  ///< Decoder found a delimiter before reaching the end of the packet
        kPacketAlreadyDecoded          = 22,  ///< Cannot decode the packet as it is already decoded (overhead == 0)
        kPayloadDecoded                = 23,  ///< Payload was successfully decoded from the received packet
    };

    /**
     * @enum kCRCProcessorCodes
     * @brief Assigns meaningful names to all status codes used by the CRCProcessor class.
     *
     * @note Due to the unified approach to error-code handling in this library, this enumeration should only use code
     * values in the range of 51 through 100. This is to simplify chained error handling in the
     * TransportLayer class of the library.
     */
    enum class kCRCProcessorCodes : uint8_t
    {
        kStandby                            = 51,  ///< The value used to initialize the crc_status variable
        kCalculateCRCChecksumBufferTooSmall = 52,  ///< Checksum calculator failed, the packet exceeds buffer space
        kCRCChecksumCalculated              = 53,  ///< Checksum was successfully calculated
        kAddCRCChecksumBufferTooSmall       = 54,  ///< Not enough remaining buffer space to add checksum to buffer
        kCRCChecksumAddedToBuffer           = 55,  ///< Checksum was successfully added to the buffer
        kReadCRCChecksumBufferTooSmall      = 56,  ///< Not enough remaining space inside buffer to get checksum from it
        kCRCChecksumReadFromBuffer          = 57,  ///< Checksum was successfully read from the buffer
    };

    /**
     * @enum kTransportLayerStatusCodes
     * @brief Assigns meaningful names to all status codes used by the TransportLayer class.
     *
     * @note Due to the unified approach to error-code handling in this library, this enumeration should only use code
     * values in the range of 101 through 150. This is to simplify chained error handling in the
     * TransportLayer class of the library.
     */
    enum class kTransportLayerStatusCodes : uint8_t
    {
        kStandby                      = 101,  ///< The default value used to initialize the transfer_status variable
        kPacketConstructed            = 102,  ///< Packet was successfully constructed
        kPacketSent                   = 103,  ///< Packet was successfully transmitted
        kPacketStartByteFound         = 104,  ///< Packet start byte was found
        kPacketStartByteNotFoundError = 105,  ///< Packet start byte was not found in the incoming stream
        kPayloadSizeByteFound         = 106,  ///< Payload size byte was found
        kPayloadSizeByteNotFound      = 107,  ///< Payload size byte was not found in the incoming stream
        kInvalidPayloadSize           = 108,  ///< Received payload size is not valid
        kPacketTimeoutError           = 109,  ///< Packet parsing failed due to stalling (reception timeout)
        kNoBytesToParseFromBuffer     = 110,  ///< Stream class reception buffer had no packet bytes to parse
        kPacketParsed                 = 111,  ///< Packet was successfully parsed
        kCRCCheckFailed               = 112,  ///< CRC check failed, the incoming packet is corrupted
        kPacketValidated              = 113,  ///< Packet was successfully validated
        kPacketReceived               = 114,  ///< Packet was successfully received
        kWriteObjectBufferError       = 115,  ///< Not enough space in the buffer payload region to write the object
        kObjectWrittenToBuffer        = 116,  ///< The object has been written to the buffer
        kReadObjectBufferError        = 117,  ///< Not enough bytes in the buffer payload region to read the object from
        kObjectReadFromBuffer         = 118,  ///< The object has been read from the buffer
    };

    /**
     * @enum kCommunicationStatusCodes
     * @brief Assigns meaningful names to all status codes used by the Communication class.
     *
     * @note Due to the unified approach to error-code handling in this library, this enumeration should only use code
     * values in the range of 151 through 200. This is to simplify chained error handling in the Communication class
     * of the library.
     */
    enum class kCommunicationStatusCodes : uint8_t
    {
        kStandby           = 151,  ///< Standby placeholder used to initialize the status tracker.
    };

    /**
     * @struct ControllerRuntimeParameters
     * @brief Stores global runtime parameters that are addressable through the Communication interface to broadly
     * configure Controller behavior.
     *
     * The scope of modification realized through these parameters is very broad and affects many downstream methods and
     * runtime behavior patterns. Additionally, since all of these parameters can be dynamically changed during runtime,
     * this allows to change Controller behavior on-the-fly, which is in contrast to some other broad parameters that
     * are statically defined. Primarily, this is helpful when working with precisely configured Controllers tightly
     * embedded in an actively used experimental control system that should not be radically altered by software or
     * physical pinout modifications.
     *
     * @warning This structure is defined in shared_assets.h as it is used by multiple Core level classes. However,
     * the @b only class allowed to modify this structure is the Kernel class. While each Module-derived
     * class requires an instance of this structure to properly support runtime behavior, that instance is introduced
     * by referencing the root instance of the Kernel class. Interfering with this declaration and use pattern may
     * result in unexpected behavior and should be avoided if possible.
     *
     * To enforce the user-pattern described above, the only class that contains the functionality and, more
     * importantly, the address-codes enumeration is the Kernel class. If you need to add or modify variables of
     * this structure, be sure to properly update structure modification methods and address-codes stored inside the
     * Kernel class.
     */
    struct ControllerRuntimeParameters
    {
        /// Enables running the controller logic without physically issuing commands. This is helpful for testing and
        /// debugging microcontroller systems. Specifically, blocks @b action pins from writing. Sensor and TTL pins are
        /// unaffected by this variable's state.
        bool action_lock = true;

        /// Same as action_lock, but specifically locks or unlocks output TTL pin activity. Currently only applies to
        /// TTL Module commands, but, in the future, may be tied to any Module that sends a ttl pulse of any kind
        /// using digital pins. The reason this is separate from action_lock is because action pins connected to
        /// physical systems are often capable of damaging connected systems if the code that runs them is not properly
        /// calibrated. Conversely, this is usually not a concern for ttl signals that convey information without
        /// triggering any physical changes in the connected systems. As such, it is not uncommon to have a use case
        /// where action_lock needs to be enabled without enabling the ttl_lock.
        bool ttl_lock = true;

        /// Controls whether the controller uses simulated or real sensor readout values. To support python-driven AMC
        /// tests, all code comes with simulation capacity where necessary. Specifically, when set to @b true, this
        /// parameter forces all functions to return code-generated values rather than real-world derived values.
        /// For example, instead of reading a sensor the code will return a random or predefined sensor value as if it
        /// was obtained by reading the sensor.
        bool simulate_responses = false;

        /// If simulate_responses is set to @b true, determines whether simulated values are generated using random()
        /// function or a predefined simulated_value. The random approach is good for longitudinal speed / pipeline
        /// resilience testing as it is a better approximation of real world performance in certain scenarios and the
        /// predefined approach is good for unit-testing PC-Controller interactions and logic. If random generation
        /// (simulation) is used, the range of random generation is set to the maximum resolution of the expected output
        /// variable type.
        bool simulate_specific_responses = false;

        /// If simulate_specific_responses is @b true, this is the value that will be returned by EVERY sensor-reading
        /// function call. If this value does not fit into the bit-size of the variable expected from a particular
        /// function that calls the simulation sub-routine, the returned value will actually be the maximum value
        /// supported by the variable's bit-size. For example, if this variable is a 1000, and it is used to simulate
        /// the response of a sensor that can only read HIGH or LOW (bool), the number 1000 will be converted to HIGH
        /// and returned as HIGH (1).
        uint32_t simulated_value = 1000;

        /// Allows to enforce a specific @em minimum sensor-loop lock duration in @b microseconds. This variable
        /// specifically affects the loops that lock until particular sensor value is encountered. These loops are
        /// modified indirectly via the simulate_specific_responses variable that, if @b false, introduces a random
        /// readout component where each call to the sensor reading function will return a random value that may or may
        /// not satisfy the loop exit requirement. This variable acts as an additional simulation step which will
        /// prevent loop breaking until the specified number of microseconds has passed, even if the sensor value
        /// criterion has been satisfied.
        uint32_t minimum_lock_duration = 20000;
    };

    // Since Arduino Mega (the lower-end board this code was tested with) boards do not have access to 'cstring' header
    // that is available to Teensy, some assets had to be reimplemented manually. They are implemented in as
    // similar of a way as possible to be drop-in replaceable with std:: namespace.

    /**
     * @brief A type trait that determines if two types are the same.
     *
     * @tparam T The first type.
     * @tparam U The second type.
     *
     * This struct is used to compare two types at compile-time. It defines a static constant member `value` which is
     * set to `false` by default, indicating that the two types are different.
     */
    template <typename T, typename U>
    struct is_same
    {
        /// The default value used by the specification for two different input types.
        static constexpr bool value = false;
    };

    /**
      * @brief Specialization of is_same for the case when both types are the same.
      *
      * @tparam T The type to compare.
      *
      * This specialization is used when both type parameters are the same. In this case, the static constant member
      * `value` is set to `true`, indicating that the types are indeed the same.
      */
    template <typename T>
    struct is_same<T, T>
    {
        /// The default value used by the specification for two identical types.
        static constexpr bool value = true;
    };

    /**
     * @brief A helper variable template that provides a convenient way to access the value of is_same.
     *
     * @tparam T The first type.
     * @tparam U The second type.
     *
     * This variable template is declared as `constexpr`, allowing it to be used in compile-time expressions. It
     * provides a more concise way to check if two types are the same, without the need to explicitly access the
     * `value` member of the `is_same` struct.
     *
     * Example usage:
     * @code
     * static_assert(is_same_v<int, int>, "int and int are the same");
     * static_assert(!is_same_v<int, float>, "int and float are different");
     * @endcode
     */
    template <typename T, typename U>
    constexpr bool is_same_v = is_same<T, U>::value;  // NOLINT(*-dynamic-static-initializers)

}  // namespace shared_assets

#endif  //AXMC_SHARED_ASSETS_H
