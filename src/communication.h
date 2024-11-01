/**
 * @file
 * @brief The header file for the Communication class, which is used to establish and maintain bidirectional
 * communication with other Ataraxis systems over the USB or UART interface.
 *
 * @section comm_description Description:
 *
 * @note A single instance of this class should be created inside the main.cpp file and provided as a reference input
 * to Kernel and (base) Module-derived classes.
 *
 * Communication class functions as an opinionated extension of the TransportLayer class, enforcing consistent
 * message structures and runtime error handling behaviors.
 *
 * @subsection developer_notes Developer Notes:
 *
 * This is a high-level API wrapper for the low-level TransportLayer class. It is used to define the
 * communicated payload microstructure and abstracts away all communication steps.
 *
 * @attention For this class to work as intended, its configuration, message layout structures and status codes should
 * be the same as those used by the Ataraxis system this class communicates with. Make sure that both communicating
 * parties are configured appropriately!
 *
 * @subsubsection rx_tx_data_structures_overview Rx/Tx Data Structures Overview
 *
 * These datastructures are used to serialize and deserialize the data for Serial transmission. Each incoming or
 * outgoing serial stream consists of a package macrostructure (handled by TransportLayer class), and microstructure,
 * defined via Communication class message layouts (Protocols). Each supported message layout, available through the
 * communication_assets namespace, is uniquely configured to optimally support the intended communication format.
 *
 * @attention It is expected that every incoming or outgoing payload uses the first non-metadata byte to store the
 * protocol code. This code is used to interpret the rest of the message. Further, it is expected that each transmitted
 * packet contains a single message.
 *
 * @note The physical payload limit is 254 bytes, so there is a cap on how much data can be sent as a single packet.
 * This class contains validation mechanisms that raise errors if the buffer overflows, although this is considered an
 * unlikely scenario if the class is used as intended.
 *
 * @subsubsection rx_tx_datastructures_visualization Byte-Stream Anatomy Visualized:
 *
 * This is a rough outline of how the Rx and Tx data streams look. Each |block| represents a structure either provided
 * by this class or TransportLayer class. Only the datastructures defined inside this class are relevant for data
 * serialization and deserialization, TransportLayer handles processing the preamble and postamble structures of
 * each stream before deserializing the custom datastructures of this project. Note, only the Protocol Code and
 * Message Payload count towards the 254-byte limit for message payload.
 *
 * - **Protocol**: |Start Byte| |Header Preamble| |Protocol Code| |Message Payload| |Delimiter Byte| |CRC Postamble|.
 *
 * @subsection dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - transport_layer.h for low-level methods for sending and receiving messages over USB or UART Serial port.
 * - shared_assets.h for globally shared assets, including communication_assets namespace.
 */

#ifndef AXMC_COMMUNICATION_H
#define AXMC_COMMUNICATION_H

// Dependencies:
#include <Arduino.h>
#include "shared_assets.h"
#include "transport_layer.h"

// Statically defines the size of the Serial class reception buffer associated with different Arduino and Teensy board
// architectures. This is required to ensure the Communication class is configured appropriately. If you need to adjust
// the TransportLayer class buffers (for example, because you manually increased the buffer size used by the Serial
// class of your board), do it by editing or specifying a new preprocessor directive below. It is HIGHLY advised not to
// tamper with these settings, however, and to always have the kSerialBufferSize set exactly to the size of the Serial
// class reception buffer.
#if defined(ARDUINO_ARCH_SAM)
// Arduino Due (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 256;

#elif defined(ARDUINO_ARCH_SAMD)
// Arduino Zero, MKR series (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 256;

#elif defined(ARDUINO_ARCH_NRF52)
// Arduino Nano 33 BLE (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 256;

// Note, teensies are identified based on the processor model. This WILL need to be updated for future versions of
// Teensy boards.
#elif defined(CORE_TEENSY)
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || \
    defined(__IMXRT1062__)
// Teensy 3.x, 4.x (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 1024;
#else
// Teensy 2.0, Teensy++ 2.0 (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 256;
#endif

#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA) ||  \
    defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega168__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega16U4__)
// Arduino Uno, Mega, and other AVR-based boards (UART serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 64;

#else
// Default fallback for unsupported boards is the reasonable minimum buffer size
static constexpr uint16_t kSerialBufferSize = 64;

#endif

/**
 * @class Communication
 * @brief Exposes methods that allow establishing and maintaining bidirectional serialized communication with other
 * Ataraxis systems via the internal binding of the TransportLayer library.
 *
 * This class handles all communications between the Microcontroller and the PC over the USB (preferred) or UART serial
 * interface. To do so, the class statically extends the TransportLayer class methods to enforce the consistent
 * communication structure used by the Ataraxis project.
 *
 * @note This class is intended to be used both by custom modules (classes derived from the base Module class) and
 * the Kernel class. Overall, this class defines and maintains a rigid payload microstructure that acts on top of the
 * serialized message structure enforced by the TransportLayer. Combined, these two classes define a robust
 * communication system specifically designed to support realtime communication between Ataraxis infrastructure.
 *
 * @warning This class is explicitly designed to hide most of the configuration parameters used to control the
 * communication interface from the user. Do not modify any internal parameters or subclasses of this class,
 * unless you know what you are doing.
 */
class Communication
{
    public:
        /// Tracks the most recent class runtime status.
        uint8_t communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kStandby);

        /// Stores the protocol code of the last received message.
        uint8_t protocol_code = static_cast<uint8_t>(communication_assets::kProtocols::kUndefined);

        /// Stores the last received Module-addressed command message data. Note, this structure is used by both
        /// Repeated (recurrent) and OneOff (non-recurrent) commands. If the cycle_duration is set to 0, this structure
        /// stores a non-recurrent command. Otherwise, the structure stores a recurrent command. Using the same
        /// structure for both commands improves code readability and conserves memory.
        communication_assets::RepeatedModuleCommand module_command;

        /// Stores the last received Kernel-addressed command message data.
        communication_assets::KernelCommand kernel_command;

        /// Stores the last received Module-addressed dequeue message data.
        communication_assets::DequeueModuleCommand module_dequeue;

        /// Stores the last received Module-addressed parameters message header data. Note, the parameter object
        /// is not contained in this structure! Instead, it has to be parsed separately by calling the
        /// ExtractModuleParameters() method.
        communication_assets::ModuleParameters module_parameter;

        /// Stores the last received Kernel-addressed parameters message data. This structure stores the entire
        /// message, including the parameter object.
        communication_assets::KernelParameters kernel_parameters;

        /**
         * @brief Instantiates a new Communication class object.
         *
         * Critically, the constructor also initializes the TransportLayer class as part of its runtime, which is
         * essential for the correct functioning of the communication interface.
         *
         * @note Make sure that the referenced Stream interface is initialized via its 'begin' method before calling
         * Communication class methods.
         *
         * @param communication_port A reference to a Stream interface class, such as Serial or USBSerial. This class
         * is used as a low-level access point that physically manages the hardware used to transmit and receive the
         * serialized data.
         *
         * Example instantiation:
         * @code
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         * Serial.begin(9600);  // Initializes serial interface.
         * @endcode
         */
        explicit Communication(Stream& communication_port) :
            _transport_layer(
                communication_port,  // Stream
                0x1021,              // 16-bit CRC Polynomial
                0xFFFF,              // Initial CRC value
                0x0000,              // Final CRC XOR value
                129,                 // Start byte value
                0,                   // Delimiter byte value
                20000,               // Packet reception timeout (in microseconds)
                false                // Disables start byte detection errors
            )
        {}

        /**
         * @brief Returns the latest status code of the TransportLayer class used by the Communication class.
         *
         * This method is used by the Kernel class when handling Communication class errors. Specifically, knowing the
         * status code of the TransportLayer class is crucial for understanding the root cause of Communication
         * class errors.
         */
        [[nodiscard]]
        uint8_t GetTransportLayerStatus() const
        {
            return _transport_layer.transfer_status;
        }

        /**
         * @brief Packages the input data into the appropriate DataMessage structure and sends it over to the
         * connected Ataraxis system.
         *
         * This method is used for sending both data and error messages. The message will be interpreted according to
         * the bundled event_code. This library treats errors as generic event instances, same as data or 'success'
         * messages.
         *
         * @warning If you only need to communicate the event-code state without any additional data, use
         * SendStateMessage() for more optimal transmission protocol!
         *
         * Data messages consist of a rigid header structure that addresses the data payload and a flexible
         * data object, that stores additional information to be sent along with the message. Messages designed to not
         * include a data object are still required to provide a placeholder object to use this method.
         *
         * @note This method automatically determines whether to use KernelDataMessage or ModuleDataMessage structure,
         * based on the value of the module_type and module_id parameters. When both are set to 0, KernelDataMessage is
         * used.
         *
         * @tparam ObjectType The type of the data object to be sent along with the message. This is inferred
         * automatically by the template constructor.
         * @param module_type The byte-code specifying the type of the module that sent the data message. Set to 0 when
         * sending the data from Kernel.
         * @param module_id The ID byte-code of the specific module within the broader module_type family that
         * sent the data. Set to 0 when sending the data from Kernel.
         * @param command The byte-code specifying the command executed by the module that sent the data message.
         * @param event_code The byte-code specifying the event that triggered the data message.
         * @param prototype The byte-code specifying the prototype object that can be used to deserialize the included
         * object data. For this to work as expected, the microcontroller and the PC need to share the same
         * code-to-prototype mapping.
         * @param object Additional data object to be sent along with the message. Currently, all data messages
         * have to contain a data object, but you can use a sensible placeholder for calls that do not have a valid
         * object to include.
         *
         * @returns True if the message was successfully sent, false otherwise.
         *
         * Example usage:
         * @code
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         * Serial.begin(9600);  // Initializes serial interface.
         *
         * const uint8_t module_type = 112        // Example module type
         * const uint8_t module_id = 12;          // Example module ID
         * const uint8_t command = 88;            // Example command code
         * const uint8_t event_code = 221;        // Example event code
         * const uint8_t prototype = 1;           // Prototype codes are available from communication_assets namespace.
         * const uint8_t placeholder_object = 0;  // Meaningless, placeholder object
         * comm_class.SendDataMessage(module_type, module_id, command, event_code, prototype, placeholder_object);
         * @endcode
         */
        template <typename ObjectType>
        bool SendDataMessage(
            const uint8_t module_type,
            const uint8_t module_id,
            const uint8_t command,
            const uint8_t event_code,
            const communication_assets::kPrototypes prototype,
            const ObjectType& object
        )
        {
            uint16_t
                next_index;  // This index is used to sequentially write the necessary data into the payload buffer.

            // Determines the protocol and message structure to use
            if (module_type == 0 && module_id == 0)
            {
                communication_assets::KernelData message {};
                message.command   = command;
                message.event     = event_code;
                message.prototype = static_cast<uint8_t>(prototype);

                // Constructs the payload by writing the protocol code
                next_index = _transport_layer.WriteData(
                    static_cast<uint8_t>(communication_assets::kProtocols::kKernelData),
                    kProtocolIndex
                );

                // Writes the message payload header if the protocol number was successfully written
                if (next_index != 0) next_index = _transport_layer.WriteData(message, next_index);
            }
            else
            {
                communication_assets::ModuleData message {};
                message.module_id   = module_id;
                message.module_type = module_type;
                message.command     = command;
                message.event       = event_code;
                message.prototype   = static_cast<uint8_t>(prototype);

                // Constructs the payload by writing the protocol code
                next_index = _transport_layer.WriteData(
                    static_cast<uint8_t>(communication_assets::kProtocols::kModuleData),
                    kProtocolIndex
                );

                // Writes the message payload header if the protocol number was successfully written
                if (next_index != 0) next_index = _transport_layer.WriteData(message, next_index);
            }

            // Writes the data object if the header was successfully written
            if (next_index != 0) next_index = _transport_layer.WriteData(object, next_index);

            // If any of the write operations above fail, breaks the runtime with an error status.
            if (next_index == 0)
            {
                communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kPackingError);
                return false;
            }

            // Sends data to the PC
            if (!_transport_layer.SendData())
            {
                // If write operation fails, returns with an error status
                communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kTransmissionError);
                return false;
            }

            // If write operation succeeds, returns with a success code
            communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kMessageSent);
            return true;
        }

        /**
         * @brief Packages the input data into a StateMessage structure and sends it over to the connected Ataraxis
         * system.
         *
         * @note This method is a more efficient for of DataMessage structure that does not use additional data objects.
         * Otherwise, the two message layouts are very similar, but optimized for slightly different ways of reporting
         * data (with or without objects).
         *
         * This method is used for sending both data and error messages. The message will be interpreted according to
         * the bundled event_code. This library treats errors as generic event instances, same as data or 'success'
         * messages.
         *
         * @note This method automatically determines whether to use KernelStateMessage or ModuleStateMessage structure,
         * based on the value of the module_type and module_id parameters. When both are set to 0, KernelStateMessage is
         * used.
         *
         * @param module_type The byte-code specifying the type of the module that sent the data message. Set to 0 when
         * sending the state from Kernel.
         * @param module_id The ID byte-code of the specific module within the broader module_type family that
         * sent the data. Set to 0 when sending the state from Kernel.
         * @param command The byte-code specifying the command executed by the module that sent the data message.
         * @param event_code The byte-code specifying the event that triggered the data message.
         *
         * @returns True if the message was successfully sent, false otherwise.
         *
         * Example usage:
         * @code
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         * Serial.begin(9600);  // Initializes serial interface.
         *
         * const uint8_t module_type = 112        // Example module type
         * const uint8_t module_id = 12;          // Example module ID
         * const uint8_t command = 88;            // Example command code
         * const uint8_t event_code = 221;        // Example event code
         * comm_class.SendDataMessage(module_type, module_id, command, event_code);
         * @endcode
         */
        bool SendStateMessage(
            const uint8_t module_type,
            const uint8_t module_id,
            const uint8_t command,
            const uint8_t event_code
        )
        {
            uint16_t
                next_index;  // This index is used to sequentially write the necessary data into the payload buffer.

            // Determines the protocol and message structure to use
            if (module_type == 0 && module_id == 0)
            {
                communication_assets::KernelState message {};
                message.command = command;
                message.event   = event_code;

                // Constructs the payload by writing the protocol code
                next_index = _transport_layer.WriteData(
                    static_cast<uint8_t>(communication_assets::kProtocols::kKernelState),
                    kProtocolIndex
                );

                // Writes the message payload if the protocol number was successfully written
                if (next_index != 0) next_index = _transport_layer.WriteData(message, next_index);
            }
            else
            {
                communication_assets::ModuleState message {};
                message.module_id   = module_id;
                message.module_type = module_type;
                message.command     = command;
                message.event       = event_code;

                // Constructs the payload by writing the protocol code
                next_index = _transport_layer.WriteData(
                    static_cast<uint8_t>(communication_assets::kProtocols::kModuleState),
                    kProtocolIndex
                );

                // Writes the message payload if the protocol number was successfully written
                if (next_index != 0) next_index = _transport_layer.WriteData(message, next_index);
            }

            // If any of the write operations above fail, breaks the runtime with an error status.
            if (next_index == 0)
            {
                communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kPackingError);
                return false;
            }

            // Sends data to the PC
            if (!_transport_layer.SendData())
            {
                // If write operation fails, returns with an error status
                communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kTransmissionError);
                return false;
            }

            // If write operation succeeds, returns with a success code
            communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kMessageSent);
            return true;
        }

        /**
         * @brief Sends the input service_code to the connected Ataraxis system using the requested protocol code.
         *
         * This method is used to transmit all Service messages. Service messages use the same minimalistic one-byte
         * payload structure, but the meaning of the transmitted byte-code depends on the used protocol.
         *
         * @note Currently, this method only supports kIdentification and kReceptionCode protocols from the
         * kProtocols enumeration available from the communication_assets namespace.
         *
         * @param protocol_code The byte-code specifying the protocol to use for the transmitted message.
         * @param service_code The byte-code specifying the byte-code to be transmitted as message payload.
         *
         * @returns True if the message was successfully sent, false otherwise.
         *
         * Example usage:
         * @code
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         * Serial.begin(9600);  // Initializes serial interface.
         *
         * const uint8_t protocol_code = static_cast<uint8_t>(communication_assets::kProtocols::kReceptionCode);
         * const uint8_t code = 112;      // Example service code
         * comm_class.SendServiceMessage(protocol_code, code);
         * @endcode
         */
        bool SendServiceMessage(const uint8_t protocol_code, const uint8_t service_code)
        {
            // Casts protocol code to the kProtocols enumeration to simplify the 'if' check below
            const auto protocol = static_cast<communication_assets::kProtocols>(protocol_code);

            // Currently, only Idle and ReceptionCode protocols are considered valid ServiceMessage protocols.
            if (protocol == communication_assets::kProtocols::kReceptionCode ||
                protocol == communication_assets::kProtocols::kIdentification)
            {
                // Writes message payload. Note, unlike other messages, the bytes are written directly to the buffer,
                // without using the structure system. Structure would only unnecessarily complicate the procedure
                // here.
                uint16_t next_index = _transport_layer.WriteData(protocol, kProtocolIndex);
                if (next_index != 0) next_index = _transport_layer.WriteData(service_code, next_index);

                // If write operation fails, breaks the runtime with an error status. This will be executed regardless
                // of which of the two writing calls failed.
                if (next_index == 0)
                {
                    communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kPackingError);
                    return false;
                }

                // Sends data to the other Ataraxis system
                if (_transport_layer.SendData())
                {
                    // If write operation succeeds, returns with a success code
                    communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kMessageSent);
                    return true;
                }

                // If send operation fails, returns with an error status
                communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kTransmissionError);
                return false;
            }

            // If input protocol code is not one of the valid Service protocol codes, aborts with an error
            communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kInvalidProtocol);
            return false;
        }

        /**
         * @brief Receives the next available message from the connected Ataraxis system.
         *
         * This method will only attempt message reception if the Serial interface buffer, managed by the
         * TransportLayer class, has the minimum number of bytes required to represent the smallest possible message.
         * If possible, the method will parse as much of the received message into the appropriate class attribute as
         * possible.
         *
         * @note If this method returns true, use protocol class attribute to access the protocol code of the parsed
         * message. Based on the protocol code, use command_message or parameter_header attributes to access the
         * parsed message data.
         *
         * @attention If the received message is a ModuleParameters message, call ExtractModuleParameters() method to
         * finalize message parsing. This method DOES NOT extract Module parameter data from the received message.
         *
         * @returns True if a message was successfully received and parsed, false otherwise. Note, if this method
         * returns false, this does not necessarily indicate runtime error. Use communication_status attribute to
         * determine the  cause of the failure. kCommunicationNoBytesToReceive status code from
         * shared_assets::kCommunicationCodes indicates a non-error failure.
         *
         * Example usage:
         * @code
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         * Serial.begin(9600);  // Initializes serial interface.
         *
         * bool success = comm_class.ReceiveMessage();  // Attempts to receive the message
         * @endcode
         */
        bool ReceiveMessage()
        {
            // Attempts to receive the next available message
            if (!_transport_layer.ReceiveData())
            {
                // The reception protocol can 'fail' gracefully if the reception buffer does not have enough bytes to
                // attempt message reception. In this case, the TransportLayer status is set to the
                // kNoBytesToParseFromBuffer constant. If message reception failed with any other code, returns with
                // error status.
                if (_transport_layer.transfer_status !=
                    static_cast<uint8_t>(shared_assets::kTransportLayerCodes::kNoBytesToParseFromBuffer))
                {
                    communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kReceptionError);
                    return false;
                }

                // Otherwise, returns with a specialized status that indicates non-error-failure.
                communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kNoBytesToReceive);
                return false;
            }

            // If the message is received and decoded, parses the protocol code of the received message. If
            // protocol code was parsed, uses it to read message ID data into an appropriate structure
            if (const uint16_t next_index = _transport_layer.ReadData(protocol_code, kProtocolIndex); next_index != 0)
            {
                // Pre-sets the status to the success code, assuming the switch below will resolve the message. This
                // makes the code below more readable.
                communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kMessageReceived);

                // Uses efficient switch logic to resolve and handle the data based on the protocol code
                switch (static_cast<communication_assets::kProtocols>(protocol_code))
                {
                    // Recurrent Module Command. This is likely one of the larger messages.
                    case communication_assets::kProtocols::KRepeatedModuleCommand:
                        if (_transport_layer.ReadData(module_command, next_index)) return true;
                        break;

                    // Non-recurrent Module Command. This message is half the size of the recurrent command message.
                    case communication_assets::kProtocols::kOneOffModuleCommand:
                        if (_transport_layer.ReadData(non_recurrent_module_command_message, next_index)) return true;
                        break;

                    // Kernel Command. This is one of the smallest messages.
                    case communication_assets::kProtocols::kKernelCommand:
                        if (_transport_layer.ReadData(kernel_command, next_index)) return true;
                        break;

                    // Module Parameters. Note, the reception cannot be completed until the target module is resolve and
                    // is used to call the ExtractModuleParameters() method
                    case communication_assets::kProtocols::kModuleParameters:
                        // Reads the HEADER of the message into the storage structure. This gives Kernel class enough
                        // information to address the message, but this is NOT the whole message. To retrieve parameter
                        // data bundled with the message, use ExtractModuleParameters() method.
                        if (_transport_layer.ReadData(module_parameter, next_index)) return true;
                        break;

                    // Kernel Parameters. Unlike module parameters, the exact object prototype for the Kernel is known.
                    // Therefore, this step parses the whole message data.
                    case communication_assets::kProtocols::kKernelParameters:
                        if (_transport_layer.ReadData(kernel_parameters, next_index)) return true;
                        break;

                    // Module command Queue reset command.
                    case communication_assets::kProtocols::kDequeueModuleCommand:
                        if (_transport_layer.ReadData(module_dequeue, next_index)) return true;
                        break;

                    default:
                        // If input protocol code is not one of the valid protocols, aborts with an error status.
                        communication_status =
                            static_cast<uint8_t>(shared_assets::kCommunicationCodes::kInvalidProtocol);
                        return false;
                }
            }

            // If any of the reading method calls failed (returns a next_index == 0), returns the error status
            communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kParsingError);
            return false;
        }

        /**
         * @brief Extracts the parameter data transmitted with the ModuleParameters message and uses it to set the input
         * structure values.
         *
         * This method executed the second step of the ModuleParameter reception process. It uses the data parsed by the
         * ReceiveMessage() method to extract the necessary number of bytes to fill the input structure with data.
         * Kernel class uses this method to set the target Module's parameter structure to use the new data.
         *
         * @warning This method will fail if it is called for any other message type, including KernelParameters
         * message.
         *
         * @tparam ObjectType The type of the input structure object. This is resolved automatically and allows using
         * this method regardless of the input structure layout.
         * @param prototype The data structure instance which needs to be filled with received parameter values.
         * @param bytes_to_read The number of bytes that make up the structure. This is used as an extra safety check to
         * ensure the parameter data extracted from the message (whose size is known) matches the size expected by the
         * structure.
         *
         * @returns True if the parameter data was successfully extracted and set, false otherwise.
         *
         * Example usage:
         * @code
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         * Serial.begin(9600);  // Initializes serial interface.
         *
         * // Initializes a test structure
         * struct DataMessage
         * {
         *     uint8_t id = 1;
         *     uint8_t data = 10;
         * } data_message;
         *
         * bool success = comm_class.ExtractParameters(data_message);  // Attempts to receive the message
         * @endcode
         */
        template <typename ObjectType>
        bool ExtractModuleParameters(ObjectType& prototype, const uint16_t& bytes_to_read = sizeof(ObjectType))
        {
            // Ensures this method cannot be called (successfully) unless the message currently stored in the reception
            // buffer is a ModuleParameters message.
            if (protocol_code != static_cast<uint8_t>(communication_assets::kProtocols::kModuleParameters))
            {
                communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kExtractionForbidden);
                return false;
            }

            // Verifies that the size of the prototype structure exactly matches the number of object bytes received
            // with the message. The '-1' accounts for the protocol code (first variable of each message) that precedes
            // the message structure.
            if (static_cast<uint8_t>(bytes_to_read) !=
                _transport_layer.get_rx_payload_size() - sizeof(communication_assets::ModuleParameters) - 1)
            {
                communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kParameterMismatch);
                return false;
            }

            if (const uint16_t next_index = _transport_layer.ReadData(prototype, kParameterObjectIndex);
                next_index != 0)
            {
                communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kParametersExtracted);
                return true;  // Successfully extracted parameters.
            }

            communication_status = static_cast<uint8_t>(shared_assets::kCommunicationCodes::kParsingError);
            return false;  // Failed to extract parameters.
        }

    private:
        /// Stores the minimum valid incoming payload size. Currently, this is the size of the KernelCommandMessage
        /// (2 bytes) + 1 Protocol byte. This value is used to optimize incoming message reception behavior.
        static constexpr uint16_t kMinimumPayloadSize = sizeof(communication_assets::KernelCommand) + 1;

        /// Stores the size of the CRC Checksum postamble in bytes. This is directly dependent on the variable type used
        /// for the PolynomialType template parameter when specializing the TransportLayer class. At the time of
        /// writing, valid values are 1 (for uint8_t), 2 (for uint16_t) and 4 (for uint32_t). The local TransportLayer
        /// binding uses CRC-16 by default (byte size: 2).
        static constexpr uint8_t kCRCSize = 2;

        /// Stores the size of the metadata variables used by the TransportLayer class. At the time of writing, this
        /// includes: the start byte, payload size byte, COBS overhead byte and COBS delimiter byte. A total of 4 bytes.
        static constexpr uint8_t kMetadataSize = 4;

        /// Combines the CRC and Metadata sizes to calculate the transmission buffer space reserved for TransportLayer
        /// variables. This value is used to calculate the (remaining) payload buffer space.
        static constexpr uint8_t kReservedNonPayloadSize = kCRCSize + kMetadataSize;

        /// Calculates maximum transmitted and received payload size. At a maximum, this can be 254 bytes
        /// (COBS limitation). For most controllers, this will be a lower value that depends on the available buffer
        /// space and the space reserved for TransportLayer variables.
        static constexpr uint16_t kMaximumPayloadSize =  // NOLINT(*-dynamic-static-initializers)
            min(kSerialBufferSize - kReservedNonPayloadSize, 254);

        /// Stores the index of the protocol code in the incoming message payload sequence. The protocol code should
        /// always occupy the first byte of each received and transmitted payload.
        static constexpr uint16_t kProtocolIndex = 0;

        /// Stores the first index of the Parameter message object data in the incoming message payload sequence.
        /// Parameter messages are different from other messages, as the parameter object itself is not part of the
        /// message structure. However, the message header has a fixed size, so the first parameter data index
        /// is static: it is the size of the ParameterMessage structure and Protocol byte (1).
        static constexpr uint16_t kParameterObjectIndex = sizeof(communication_assets::ModuleParameters) + 1;

        /// The bound TransportLayer instance that abstracts low-level data communication steps. This class statically
        /// specializes and initializes the TransportLayer to use sensible defaults. It is highly advised not to alter
        /// default initialization parameters unless you know what you are doing.
        TransportLayer<uint16_t, kMaximumPayloadSize, kMaximumPayloadSize, kMinimumPayloadSize> _transport_layer;
};

#endif  //AXMC_COMMUNICATION_H
