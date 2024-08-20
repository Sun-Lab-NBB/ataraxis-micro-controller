/**
 * @file
 * @brief The header-only file that stores all assets intended to be shared between library classes.
 *
 * @section axmc_sa_description Description:
 *
 * This file contains:
 * - shared_assets namespace that stores general-purpose assets shared between library classes. This includes
 * implementations of some std:: namespace functions, such as is_same_v.
 * - communication_assets namespace that stores structures and enumerations used by the Communication, Kernel and
 * (base) Module classes to support bidirectional communication with other Ataraxis systems.
 *
 * @section axmc_sa_developer_notes Developer Notes:
 *
 * The primary reason for having this file is to store shared byte-code enumerations and structures in the same place.
 * Many of the codes available through this file have to be unique across the library and / or Ataraxis project as
 * a whole.
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
     * @enum kCoreStatusCodes
     * @brief Assigns meaningful names to all status codes used by the Communication, Kernel and (base) Module classes.
     *
     * This enumeration jointly covers the core class 'triad'. Unlike other shared enumerations, these classes have
     * relatively few unique status codes. Together with the enumerations above, this ensures that each core class
     * status code is unique across the library, simplifying error handling.
     *
     * @note Due to the unified approach to error-code handling in this library, this enumeration should only use code
     * values in the range of 151 through 200.
     */
    enum class kCoreStatusCodes : uint8_t
    {
        kCommunicationStandby =
            151,               ///< Standby placeholder used to initialize the Communication class status tracker.
        kKernelStandby = 152,  ///< Standby placeholder used to initialize the Kernel class status tracker.
        kModuleStandby = 153,  ///< Standby placeholder used to initialize the (base) Module class status tracker.
    };

    /**
     * @struct ControllerRuntimeParameters
     * @brief Stores global runtime parameters that are addressable through the Communication interface to broadly
     * configure Controller behavior.
     *
     * These parameters are primarily used through the Kernel and (base) Module classes, and they broadly
     * affect the runtime of all classes derived from the base Module class.
     *
     * @warning The @b only class allowed to instantiate and modify this structure is the Kernel class. Each
     * Module-derived class requires an instance of this structure to properly support runtime behavior, but that
     * instance should be automatically obtained from the Kernel class during module initialization.
     *
     * @note If you need to add or modify variables of this structure, be sure to properly update structure
     * modification methods and address-codes stored inside the Kernel class.
     */
    struct ControllerRuntimeParameters
    {
        /// Enables running the controller logic without physically issuing commands. This is helpful for testing and
        /// debugging microcontroller systems. Specifically, blocks @b action pins from writing. Sensor and TTL pins are
        /// unaffected by this variable's state.
        bool action_lock = true;

        /// Same as action_lock, but specifically locks or unlocks output TTL pin activity. Currently only applies to
        /// TTL Module commands, as there is no 'automatic' way of knowing if any other output pin is a TTL pin.
        bool ttl_lock = true;
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

/**
 * @namespace communication_assets
 * @brief Provides all assets (structures, enumerations, variables) necessary to interface with and support the
 * runtime of the Communication class.
 *
 * @attention These assets are designed to be used exclusively by the core classes: Communication, Kernel and (base)
 * Module. Do not modify or access these assets from any class derived from the base Module class or any other custom
 * class.
 */
namespace communication_assets
{
    /**
     * @enum kProtocols
     * @brief Stores protocol codes used by the Communication class to specify incoming and outgoing message structures.
     *
     * The Protocol byte-code instructs message parsers on how to process the incoming message. Each message has a
     * unique payload structure which cannot be parsed unless the underlying protocol is known.
     *
     * @attention The protocol code, derived from this enumeration, should be the first 'payload' byte of each message.
     * This enumeration should only be used by the Communication and Kernel classes. Protocol codes are designed to
     * be unique across the entire Ataraxis project.
     */
    enum class kProtocols : uint8_t
    {
        /// Undefined protocol. This value is used as the default initializer and is not a valid protocol code.
        /// Messages with this protocol code will be interpreted as communication errors.
        kUndefined = 0,

        /// Command message protocol.
        kCommand = 1,

        /// Parameters message protocol.
        kParameters = 2,

        /// Data message protocol. This protocol is used for all types of data messages, including error messages.
        kData = 3,

        /// Reception acknowledgement protocol. This is a minimalistic protocol used to notify the sender that the
        /// controller received the Command or Parameters message.
        kReceptionCode = 4,

        /// Idle protocol. This protocol is used by controllers in the 'standby' mode. It serves as a sign that
        /// the controller is not currently used by other Ataraxis systems.
        kIdle = 5,
    };

    /**
     * @struct CommandMessage
     * @brief The payload structure used by the incoming Command messages.
     *
     * When Communication class encounters a message with the first value (assumed to be the protocol ID) set to
     * 1, it parses the message using this structure. Command messages are designed to trigger specific module runtimes.
     * These runtimes may be further configured by setting module parameters via the Parameters message.
     *
     * @notes Currently, controllers can only receive Command messages, but they cannot send them.
     */
    struct CommandMessage
    {
        /// The type-code of the module to which the command is addressed.
        uint8_t module_type = 0;

        /// The specific module ID within the broader module family specified by module_type.
        uint8_t module_id = 0;

        /// When this field is set to a value other than 0, the Communication class will send this code back to the
        /// sender upon successfully processing the received command. This is to notify the sender that the command was
        /// received intact, ensuring message delivery. Setting this field to 0 disables delivery assurance.
        uint8_t return_code = 0;

        /// The unique code of the command to execute.
        uint8_t command = 0;

        /// Determines whether the command is executed once or repeatedly cycled with a certain periodicity.
        /// Together with cycle_duration, this allows triggering both one-shot and cyclic command runtimes.
        bool cycle = false;

        /// The period of time, in milliseconds, to delay before repeating (cycling) the command. This is only used if
        /// the cycle flag is True.
        uint32_t cycle_duration = 0;
    } __attribute__((packed));

    /**
     * @struct ParameterMessage
     * @brief The payload structure used by the incoming Parameter messages.
     *
     * When Communication class encounters a message with the first value (assumed to be the protocol ID) set to
     * 2, it parses the message using this structure. Parameter messages are used to flexibly configure the addressed
     * module by overwriting its parameter structure with the object provided in the message.
     *
     * @notes Parameter messages are expected to contain this ID structure as the header, with remaining payload
     * space occupied by the parameter structure object. The structure should exactly match the configured module
     * 'runtime_parameters' structure. During parsing, the Kernel will first use the information from this structure
     * to find the addressed module and then use that module's runtime_parameters structure as the prototype to parse
     * the rest of the message payload. See Kernel class ParseParameters() method for more details.
     */
    struct ParameterMessage
    {
        /// The type-code of the module to which the parameter configuration is addressed.
        uint8_t module_type = 0;

        /// The specific module ID within the broader module family specified by module_type.
        uint8_t module_id = 0;

        /// When this field is set to a value other than 0, the Communication class will send this code back to the
        /// sender upon successfully processing the received command. This is to notify the sender that the command was
        /// received intact, ensuring message delivery. Setting this field to 0 disables delivery assurance.
        uint8_t return_code = 0;
    } __attribute__((packed));

    /**
     * @struct DataMessage
     * @brief The payload structure used by the outgoing Data messages.
     *
     * When the Communication class sends the Data message, it uses this message structure, preceded by
     * protocol value 3. Data messages are used to communicate all controller-originating information, such as
     * runtime data and error messages to other Ataraxis systems.
     *
     * @tparam ObjectType The type of object to be sent with the message. The object can be of any supported type,
     * including arrays and structures.
     */
    template <typename ObjectType>
    struct DataMessage
    {
        /// The type-code of the module which sent the data message.
        uint8_t module_type = 0;

        /// The specific module ID within the broader module family specified by module_type.
        uint8_t module_id = 0;

        /// The unique code of the command the module was executing when it sent the data message.
        uint8_t command = 0;

        /// The unique code of the event within the command runtime that prompted the data transmission.
        uint8_t event = 0;

        /// The transmitted data object. This can be any valid object type, as long as it fits the
        /// specification imposed by the maximum message payload size.
        ObjectType object = 0;
    } __attribute__((packed));

    /**
     * @struct ReceptionMessage
     * @brief The payload structure used by the outgoing Reception messages.
     *
     * When the Communication class receives a message that contains a non-zero return_code field, it sends a response
     * using this message structure, preceded by protocol value 4. This is a one-variable protocol that returns the
     * reception code to the sender, to indicate that the message was received intact.
     */
    struct ReceptionMessage
    {
        /// The return code from the received message that requested the reception acknowledgement service.
        uint8_t return_code = 0;
    } __attribute__((packed));

    /**
     * @struct IdleMessage
     * @brief The payload structure used by the outgoing Idle messages.
     *
     * When the Communication class sends the Idle message, it uses this message structure, preceded by
     * protocol value 5. Idle messages are periodically sent to signal that the controller is not currently in use by
     * other Ataraxis systems. Additionally, it contains controller ID code.
     *
     * @notes Idle message periodicity is configured using kStaticRuntimeParameters structure inside the main.cpp
     * file.
     */
    struct IdleMessage
    {
        /// The user-defined ID-code of the controller.
        uint8_t controller_id = 0;
    } __attribute__((packed));

}  // namespace communication_assets

#endif  //AXMC_SHARED_ASSETS_H
