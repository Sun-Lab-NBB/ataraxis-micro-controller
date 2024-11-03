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
     * @enum kTransportLayerCodes
     * @brief Assigns meaningful names to all status codes used by the TransportLayer class.
     *
     * @note Due to the unified approach to error-code handling in this library, this enumeration should only use code
     * values in the range of 101 through 150. This is to simplify chained error handling in the
     * TransportLayer class of the library.
     */
    enum class kTransportLayerCodes : uint8_t
    {
        kStandby                     = 101,  ///< The default value used to initialize the transfer_status variable
        kPacketConstructed           = 102,  ///< Packet was successfully constructed
        kPacketSent                  = 103,  ///< Packet was successfully transmitted
        kPacketStartByteFound        = 104,  ///< Packet start byte was found
        kPacketStartByteNotFound     = 105,  ///< Packet start byte was not found in the incoming stream
        kPayloadSizeByteFound        = 106,  ///< Payload size byte was found
        kPayloadSizeByteNotFound     = 107,  ///< Payload size byte was not found in the incoming stream
        kInvalidPayloadSize          = 108,  ///< Received payload size is not valid
        kPacketTimeoutError          = 109,  ///< Packet parsing failed due to stalling (reception timeout)
        kNoBytesToParseFromBuffer    = 110,  ///< Stream class reception buffer had no packet bytes to parse
        kPacketParsed                = 111,  ///< Packet was successfully parsed
        kCRCCheckFailed              = 112,  ///< CRC check failed, the incoming packet is corrupted
        kPacketValidated             = 113,  ///< Packet was successfully validated
        kPacketReceived              = 114,  ///< Packet was successfully received
        kWriteObjectBufferError      = 115,  ///< Not enough space in the buffer payload region to write the object
        kObjectWrittenToBuffer       = 116,  ///< The object has been written to the buffer
        kReadObjectBufferError       = 117,  ///< Not enough bytes in the buffer payload region to read the object from
        kObjectReadFromBuffer        = 118,  ///< The object has been read from the buffer
        kDelimiterNotFoundError      = 119,  ///< Delimiter byte not found at the end of the packet
        kDelimiterFoundTooEarlyError = 120,  ///< Delimiter byte was found before reaching the end of the packet
        kPostambleTimeoutError       = 121,  ///< The Postamble was not received within the specified time frame
    };

    /**
     * @enum kCommunicationCodes
     * @brief Assigns meaningful names to all status codes used by the Communication class.
     *
     * @note Due to the unified approach to error-code handling in this library, this enumeration should only use code
     * values in the range of 151 through 200.
     */
    enum class kCommunicationCodes : uint8_t
    {
        kStandby             = 151,  ///< The default value used to initialize the communication_status variable.
        kReceptionError      = 152,  ///< Communication class ran into an error when receiving a message.
        kParsingError        = 153,  ///< Communication class ran into an error when parsing (reading) a message.
        kPackingError        = 154,  ///< Communication class ran into an error when writing a message to payload.
        kTransmissionError   = 155,  ///< Communication class ran into an error when transmitting a message.
        kMessageSent         = 156,  ///< Communication class successfully transmitted a message.
        kMessageReceived     = 157,  ///< Communication class successfully received a message.
        kInvalidProtocol     = 158,  ///< The message protocol code is not valid for the type of operation (Rx or Tx).
        kNoBytesToReceive    = 159,  ///< Communication class did not receive enough bytes to process the message.
        kParameterMismatch   = 160,  ///< The size of the received parameters structure does not match expectation.
        kParametersExtracted = 161,  ///< Parameter data has been successfully extracted.
        kExtractionForbidden = 162,  ///< Attempted to extract parameters from message other than ModuleParameters.
    };

    /**
     * @struct DynamicRuntimeParameters
     * @brief Stores global runtime parameters shared by all core classes and addressable through the Kernel class.
     *
     * These parameters broadly affect the runtime of all classes derived from the base Module class. They are
     * addressable through the Kernel class using the Communication interface.
     *
     * @warning The @b only class allowed to modify this structure is the Kernel class. Each (base) Module-derived
     * class requires an instance of this structure to properly support runtime behavior, but they should not modify the
     * instance.
     */
    struct DynamicRuntimeParameters
    {
            /// Enables running the controller logic without physically issuing commands. This is helpful for testing
            /// and debugging Microcontroller code. Specifically, blocks @b action pins from writing. Sensor and TTL
            /// pins are unaffected by this variable's state. Note, this only works for digital and analog writing
            /// methods inherited from the base Module class.
            bool action_lock = true;

            /// Same as action_lock, but specifically locks or unlocks output TTL pin activity. Same as action_lock,
            /// this only works for digital and analog writing methods inherited from the base Module class.
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
        /// Not a valid protocol code. This is used to initialize the Communication class.
        kUndefined = 0,

        /// Protocol for receiving Module-addressed commands that should be repeated (executed recurrently).
        KRepeatedModuleCommand = 1,

        /// Protocol for receiving Module-addressed commands that should not be repeated (executed only once).
        kOneOffModuleCommand = 2,

        /// Protocol for receiving Module-addressed commands that remove all queued commands
        /// (including recurrent commands). This command is actually fulfilled by the Kernel that directly manipulates
        /// the local queue of the targeted Module.
        kDequeueModuleCommand = 3,

        /// Protocol for receiving Kernel-addressed commands. All Kernel commands are always non-repeatable (one-shot).
        kKernelCommand = 4,

        /// Protocol for receiving Module-addressed parameters. This relies on transmitting arbitrary sized parameter
        /// objects and requires each module to define a function for handling these parameters.
        kModuleParameters = 5,

        /// Protocol for receiving Kernel-addressed parameters. The parameters transmitted via these messages will be
        /// used to overwrite the fields of the global DynamicRuntimeParameters structure shared by all Modules.
        kKernelParameters = 6,

        /// Protocol for sending Module data or error messages that include an arbitrary data object in addition to
        /// event state-code.
        kModuleData = 7,

        /// Protocol for sending Kernel data or error messages that include an arbitrary data object in addition to
        /// event state-code.
        kKernelData = 8,

        /// Protocol for sending Module data or error messages that do not include additional data objects. These
        /// messages are optimized for delivering single event state-codes.
        kModuleState = 9,

        /// Protocol for sending Kernel data or error messages that do not include additional data objects. These
        /// messages are optimized for delivering single event state-codes.
        kKernelState = 10,

        /// Protocol used to acknowledge the reception of command and parameter messages. This is a minimalistic
        /// service protocol used to notify the PC that the controller received the message.
        kReceptionCode = 11,

        /// Protocol used to identify the controller to the PC. This is a service message protocol that transmits the
        /// unique ID of the controller to the PC. This is primarily used to determine the USB ports used by specific
        /// controllers.
        kIdentification = 12,
    };

    /**
     * @enum kPrototypes
     * @breif Stores prototype codes used by the Communication class to specify the data structure object that can be
     * used to parse DataMessage objects.
     *
     * Since most transmitted data objects use a small set of data structures, it is possible to uniquely map each
     * used data object structure to a unique prototype code. In turn, this allows optimizing data reception and logging
     * on the PC side as objet types can be obtained during a single reception cycle (the message instructs parser on
     * how to read the object).'
     *
     * @note While this approach essentially limits the number of valid prototype codes to 255 (256 if 0 is made a valid
     * code), realistically, this si more than enough to cover a very wide range of runtime cases.
     */
    enum class kPrototypes : uint8_t
    {
        /// A single uint8_t value.
        kOneUnsignedByte = 1,
        /// An array made up of two uint8_t values.
        kTwoUnsignedBytes = 2,
        /// An array made up of three uint8_t values.
        kThreeUnsignedBytes = 3,
        /// An array made up of four uint8_t values.
        kFourUnsignedBytes = 4,
        /// A single uint32_t value.
        kOneUnsignedLong = 5,
        /// A single uint16_t value.
        kOneUnsignedShort = 6,
    };

    /**
     * @struct RepeatedModuleCommand
     * @brief Instructs the addressed Module to run the specified command repeatedly (recurrently).
     */
    struct RepeatedModuleCommand
    {
            /// The type-code of the module to which the command is addressed.
            uint8_t module_type = 0;

            /// The specific module ID within the broader module family specified by module_type.
            uint8_t module_id = 0;

            /// When this field is set to a value other than 0, the Communication class will send this code back to the
            /// sender upon successfully processing the received command. This is to notify the sender that the command
            /// was received intact, ensuring message delivery. Setting this field to 0 disables delivery assurance.
            uint8_t return_code = 0;

            /// The unique code of the command to execute.
            uint8_t command = 0;

            /// Determines whether the command runs in blocking or non-blocking mode. If set to false, the controller
            /// runtime will block in-place for any sensor- or time-waiting loops during command execution. Otherwise,
            /// the controller will run other commands concurrently, while waiting for the block to complete.
            bool noblock = false;

            /// The period of time, in microseconds, to delay before repeating (cycling) the command. This is only used
            /// if the cycle flag is True.
            uint32_t cycle_delay = 0;
    } __attribute__((packed));

    /**
     * @struct OneOffModuleCommand
     * @brief Instructs the addressed Module to run the specified command exactly once (non-recurrently).
     */
    struct OneOffModuleCommand
    {
            /// The type-code of the module to which the command is addressed.
            uint8_t module_type = 0;

            /// The specific module ID within the broader module family specified by module_type.
            uint8_t module_id = 0;

            /// When this field is set to a value other than 0, the Communication class will send this code back to the
            /// sender upon successfully processing the received command. This is to notify the sender that the command
            /// was received intact, ensuring message delivery. Setting this field to 0 disables delivery assurance.
            uint8_t return_code = 0;

            /// The unique code of the command to execute.
            uint8_t command = 0;

            /// Determines whether the command runs in blocking or non-blocking mode. If set to false, the controller
            /// runtime will block in-place for any sensor- or time-waiting loops during command execution. Otherwise,
            /// the controller will run other commands concurrently, while waiting for the block to complete.
            bool noblock = false;
    } __attribute__((packed));

    /**
     * @struct DequeueModuleCommand
     * @brief Instructs the addressed Module to clear (empty) its command queue.
     */
    struct DequeueModuleCommand
    {
            /// The type-code of the module to which the command is addressed.
            uint8_t module_type = 0;

            /// The specific module ID within the broader module family specified by module_type.
            uint8_t module_id = 0;

            /// When this field is set to a value other than 0, the Communication class will send this code back to the
            /// sender upon successfully processing the received command. This is to notify the sender that the command
            /// was received intact, ensuring message delivery. Setting this field to 0 disables delivery assurance.
            uint8_t return_code = 0;
    } __attribute__((packed));

    /**
     * @struct KernelCommand
     * @brief Instructs the Kernel to run the specified command exactly once.
     *
     * Currently, the Kernel only supports blocking one-off commands.
     */
    struct KernelCommand
    {
            /// When this field is set to a value other than 0, the Communication class will send this code back to the
            /// sender upon successfully processing the received command. This is to notify the sender that the command
            /// was received intact, ensuring message delivery. Setting this field to 0 disables delivery assurance.
            uint8_t return_code = 0;

            /// The unique code of the command to execute.
            uint8_t command = 0;
    } __attribute__((packed));

    /**
     * @struct ModuleParameters
     * @brief Instructs the addressed Module to overwrite its custom parameters object with the included object data.
     *
     * @warning Module parameters are stored in a module-family-specific structure, whose layout will not be known at
     * by the parser at message reception. The Kernel class will use the included module type and id information from
     * the message to call the addressed Module's public API method that will extract the parameters.
     */
    struct ModuleParameters
    {
            /// The type-code of the module to which the parameter configuration is addressed.
            uint8_t module_type = 0;

            /// The specific module ID within the broader module family specified by module_type.
            uint8_t module_id = 0;

            /// When this field is set to a value other than 0, the Communication class will send this code back to the
            /// sender upon successfully processing the received command. This is to notify the sender that the command
            /// was received intact, ensuring message delivery. Setting this field to 0 disables delivery assurance.
            uint8_t return_code = 0;
    } __attribute__((packed));

    /**
     * @struct KernelParameters
     * @brief Instructs the Kernel to update the shared DynamicRuntimeParameters object with included data.
     *
     * @notes Since the target data structure for this method is static and known at compile-time, the parser
     * will be able to resolve the target structure in-place.
     */
    struct KernelParameters
    {
            /// When this field is set to a value other than 0, the Communication class will send this code back to the
            /// sender upon successfully processing the received command. This is to notify the sender that the command
            /// was received intact, ensuring message delivery. Setting this field to 0 disables delivery assurance.
            uint8_t return_code = 0;

            /// Since Kernel parameter structure is known at compile-time, this message structure automatically knows
            /// the shape of the transmitted parameters' data.
            shared_assets::DynamicRuntimeParameters dynamic_parameters;
    } __attribute__((packed));

    /**
     * @struct ModuleData
     * @brief Communicates the event state-code of the sender Module and includes an additional data object.
     *
     * @notes For messages that only need to transmit an event state-code, use ModuleStateMessage structure for better
     * efficiency.
     *
     * @warning Remember to add the data object matching the included prototype code to the message payload before
     * sending the message.
     */
    struct ModuleData
    {
            /// The unique message protocol used by this structure. Including the protocol with the message structure
            /// allows optimizing data transmission.
            uint8_t protocol;

            /// The type-code of the module which sent the data message.
            uint8_t module_type;

            /// The specific module ID within the broader module family specified by module_type.
            uint8_t module_id;

            /// The unique code of the command the module was executing when it sent the data message.
            uint8_t command;

            /// The unique code of the event within the command runtime that prompted the data transmission.
            uint8_t event;

            /// The byte-code of the prototype object that can be used to decode the object data included with this
            /// message. The PC will read the object data into the prototype object specified by this code.
            uint8_t prototype;

    } __attribute__((packed));

    /**
     * @struct KernelData
     * @brief Communicates the event state-code of the Kernel and includes an additional data object.
     *
     * @attention For messages that only need to transmit an event state-code, use KernelState structure for better
     * efficiency.
     *
     * @warning Remember to add the data object matching the included prototype code to the message payload before
     * sending the message.
     * */
    struct KernelData
    {
            /// The unique message protocol used by this structure. Including the protocol with the message structure
            /// allows optimizing data transmission.
            uint8_t protocol;

            /// The unique code of the command the kernel was executing when it sent the data message.
            uint8_t command;

            /// The unique code of the event within the command runtime that prompted the data transmission.
            uint8_t event;

            /// The byte-code of the prototype object that can be used to decode the object data included with this
            /// message. The PC will read the object data into the prototype object specified by this code.
            uint8_t prototype;

    } __attribute__((packed));

    /**
     * @struct ModuleState
     * @brief Communicates the event state-code of the sender Module.
     *
     * @attention For messages that need to transmit a data object in addition to the state event-code, use
     * ModuleData structure.
     */
    struct ModuleState
    {
            /// The unique message protocol used by this structure. Including the protocol with the message structure
            /// allows optimizing data transmission.
            uint8_t protocol;

            /// The type-code of the module which sent the data message.
            uint8_t module_type;

            /// The specific module ID within the broader module family specified by module_type.
            uint8_t module_id;

            /// The unique code of the command the module was executing when it sent the data message.
            uint8_t command;

            /// The unique code of the event within the command runtime that prompted the data transmission.
            uint8_t event;
    } __attribute__((packed));

    /**
     * @struct KernelState
     * @brief Communicates the event state-code of the Kernel.
     *
     * @attention For messages that need to transmit a data object in addition to the state event-code, use
     * KernelData structure.
     */
    struct KernelState
    {
            /// The unique message protocol used by this structure. Including the protocol with the message structure
            /// allows optimizing data transmission.
            uint8_t protocol;

            /// The unique code of the command the kernel was executing when it sent the data message.
            uint8_t command;

            /// The unique code of the event within the command runtime that prompted the data transmission.
            uint8_t event;
    } __attribute__((packed));

}  // namespace communication_assets

#endif  //AXMC_SHARED_ASSETS_H
