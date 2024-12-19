/**
 * @file
 * @brief The header file for the Communication class, which is used to establish and maintain bidirectional
 * communication with host-computers (PCs) running ataraxis-communication-interface Python library, over the USB or UART
 * interface.
 *
 * @section comm_description Description:
 *
 * @note A single instance of this class should be created inside the main.cpp file and provided as a reference input
 * to Kernel and Module-derived classes.
 *
 * Communication class functions as an opinionated extension of the TransportLayer class, enforcing consistent
 * message structures and runtime error handling behaviors for all classes derived from this library. Generally, only
 * Core developers should work directly with the Communication class. All custom modules should use the API derived
 * from the base Module class to send and receive data messages.
 *
 * @section developer_notes Developer Notes:
 *
 * This is a high-level API wrapper for the low-level TransportLayer class. It is used to define the
 * communicated payload microstructure and abstracts away all communication steps.
 *
 * @attention For this class to work as intended, its configuration, message layout structures and status codes should
 * be identical to those used by the PC companion library (ataraxis-communication-interface).
 *
 * @section comm_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - axtlmc_shared_assets.h for TransportLayer status code enumerations.
 * - transport_layer.h for low-level methods for sending and receiving messages over USB or UART Serial port.
 * - axmc_shared_assets.h for globally shared assets, including axmc_communication_assets namespace.
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 */

#ifndef AXMC_COMMUNICATION_H
#define AXMC_COMMUNICATION_H

// Dependencies:
#include <Arduino.h>
#include <axtlmc_shared_assets.h>
#include <transport_layer.h>
#include "axmc_shared_assets.h"
#include "digitalWriteFast.h"

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
        uint8_t communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kStandby);

        /// Stores the protocol code of the last received message.
        uint8_t protocol_code = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kUndefined);

        /// Stores the last received Module-addressed recurrent (repeated) command message data.
        axmc_communication_assets::RepeatedModuleCommand repeated_module_command;

        /// Stores the last received Module-addressed non-recurrent (one-off) command message data.
        axmc_communication_assets::OneOffModuleCommand one_off_module_command;

        /// Stores the last received Kernel-addressed command message data.
        axmc_communication_assets::KernelCommand kernel_command;

        /// Stores the last received Module-addressed dequeue message data.
        axmc_communication_assets::DequeueModuleCommand module_dequeue;

        /// Stores the last received Module-addressed parameters message header data. Note, the parameter object
        /// is not contained in this structure! Instead, it has to be parsed separately by calling the
        /// ExtractModuleParameters() method.
        axmc_communication_assets::ModuleParameters module_parameter;

        /// Stores the last received Kernel-addressed parameters message data. This structure stores the entire
        /// message, including the parameter object.
        axmc_communication_assets::KernelParameters kernel_parameters;

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
         * @brief Packages the input data into the ModuleData structure and sends it to the connected PC.
         *
         * This method is used for sending both data and error messages. This library treats errors as generic event
         * instances, same as data or success-state messages.
         *
         * @note This method is specialized to send Module messages. There is an overloaded version of this method that
         * does not take module_type and module_id arguments, which allows sending data messages from the Kernel class.
         *
         * @warning If you only need to communicate the state event-code without any additional data, use
         * SendStateMessage() method for more optimal transmission protocol!
         *
         * @tparam ObjectType The type of the data object to be sent along with the message. This is inferred
         * automatically by the template constructor.
         * @param module_type The byte-code specifying the type of the module that sent the data message.
         * @param module_id The ID byte-code of the specific module within the broader module_type family that
         * sent the data.
         * @param command The byte-code specifying the command executed by the module that sent the data message.
         * @param event_code The byte-code specifying the event that triggered the data message.
         * @param prototype The byte-code specifying the prototype object that can be used to deserialize the included
         * object data. For this to work as expected, the microcontroller and the PC need to share the same
         * prototype_code to object mapping.
         * @param object Additional data object to be sent along with the message. Currently, only one object is
         * supported per each Data message.
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
            const axmc_communication_assets::kPrototypes prototype,
            const ObjectType& object
        )
        {
            // Ensures that the input fits inside the message payload buffer. Since this statement is evaluated at
            // compile time, it does not impact runtime speed.
            static_assert(
                sizeof(ObjectType) <= kMaximumPayloadSize - sizeof(axmc_communication_assets::ModuleData),
                "The provided object is too large to fit inside the message payload buffer. This check accounts for "
                "the size of the ModuleData header that will be sent with the object."
            );

            // Constructs the message header
            const axmc_communication_assets::ModuleData message {
                static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleData),
                module_type,
                module_id,
                command,
                event_code,
                static_cast<uint8_t>(prototype)
            };

            // Writes the message into the payload buffer
            uint16_t next_index = _transport_layer.WriteData(message);

            // Writes the data object if the message header was successfully written to the buffer, as indicated by a
            // non-zero next_index value
            if (next_index != 0) next_index = _transport_layer.WriteData(object, next_index);

            // If any writing attempt from above fails, breaks the runtime with an error status.
            if (next_index == 0)
            {
                communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kPackingError);
                return false;
            }

            // If the data was written to the buffer, sends it to the PC
            if (!_transport_layer.SendData())
            {
                // If data sending fails, returns with an error status
                communication_status =
                    static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kTransmissionError);
                return false;
            }

            // Returns with a success code
            communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent);
            return true;
        }

        /// Overloads SendDataMessage to use it for Kernel class. The function works the same as the module-oriented
        /// SendDataMessage, but does not accept module_type and module_id.
        template <typename ObjectType>
        bool SendDataMessage(
            const uint8_t command,
            const uint8_t event_code,
            const axmc_communication_assets::kPrototypes prototype,
            const ObjectType& object
        )
        {
            // Ensures that the input fits inside the message payload buffer. Since this statement is evaluated at
            // compile time, it does not impact runtime speed.
            static_assert(
                sizeof(ObjectType) <= kMaximumPayloadSize - sizeof(axmc_communication_assets::KernelData),
                "The provided object is too large to fit inside the message payload buffer. This check accounts for "
                "the size of the KernelData header that will be sent with the object."
            );

            // Constructs the message header
            const axmc_communication_assets::KernelData message {
                static_cast<uint8_t>(axmc_communication_assets::kProtocols::kKernelData),
                command,
                event_code,
                static_cast<uint8_t>(prototype)
            };

            // Writes the message into the payload buffer
            uint16_t next_index = _transport_layer.WriteData(message);

            // Writes the data object if the message header was successfully written to the buffer, as indicated by a
            // non-zero next_index value
            if (next_index != 0) next_index = _transport_layer.WriteData(object, next_index);

            // If any writing attempt from above fails, breaks the runtime with an error status.
            if (next_index == 0)
            {
                communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kPackingError);
                return false;
            }

            // If the data was written to the buffer, sends it to the PC
            if (!_transport_layer.SendData())
            {
                // If data sending fails, returns with an error status
                communication_status =
                    static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kTransmissionError);
                return false;
            }

            // Returns with a success code
            communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent);
            return true;
        }

        /**
         * @brief Packages the input data into the ModuleState structure and sends it to the connected PC.
         *
         * @note This method is essentially a version of SendDataMessage that is optimized to not use additional data
         * objects and execute faster. Otherwise, it behaves exactly like SendDataMessage.
         *
         * @note This method is specialized to send Module messages. There is an overloaded version of this method that
         * only takes command and event_code arguments, which allows sending data messages from the Kernel class.
         *
         * @param module_type The byte-code specifying the type of the module that sent the state message.
         * @param module_id The ID byte-code of the specific module within the broader module_type family that
         * sent the state message.
         * @param command The byte-code specifying the command executed by the module that sent the state message.
         * @param event_code The byte-code specifying the event that triggered the state message.
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
         * comm_class.SendStateMessage(module_type, module_id, command, event_code);
         * @endcode
         */
        bool SendStateMessage(
            const uint8_t module_type,
            const uint8_t module_id,
            const uint8_t command,
            const uint8_t event_code
        )
        {
            // Constructs the message header
            const axmc_communication_assets::ModuleState message {
                static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleState),
                module_id,
                module_type,
                command,
                event_code
            };

            //Writes the message into the payload buffer. If writing fails, breaks the runtime with an error status.
            if (!_transport_layer.WriteData(message))
            {
                communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kPackingError);
                return false;
            }

            // If the data was written to the buffer, sends it to the PC
            if (!_transport_layer.SendData())
            {
                // If data sending fails, returns with an error status
                communication_status =
                    static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kTransmissionError);
                return false;
            }

            // Returns with a success code
            communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent);
            return true;
        }

        /// Overloads SendStateMessage to use it for Kernel class. The function works the same as the module-oriented
        /// SendStateMessage, but does not accept module_type and module_id.
        bool SendStateMessage(const uint8_t command, const uint8_t event_code)
        {
            // Constructs the message header
            const axmc_communication_assets::KernelState message {
                static_cast<uint8_t>(axmc_communication_assets::kProtocols::kKernelState),
                command,
                event_code
            };

            // Writes the message into the payload buffer. If writing fails, breaks the runtime with an error status.
            if (!_transport_layer.WriteData(message))
            {
                communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kPackingError);
                return false;
            }

            // If the data was written to the buffer, sends it to the PC
            if (!_transport_layer.SendData())
            {
                // If data sending fails, returns with an error status
                communication_status =
                    static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kTransmissionError);
                return false;
            }

            // Returns with a success code
            communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent);
            return true;
        }

        /**
         * @brief Sends a communication error message to the connected PC and activates the built-in LED to indicate
         * the error.
         *
         * @warning This method should be used exclusively for communication errors. Use regular SendDataMessage or
         * SendStateMessage to send all other types of errors.
         *
         * This method is an extension of the SendDataMessage method that is specialized to send communication errors to
         * the PC. The primary reason for making this method distinct is to aggregate unique aspects of handling
         * communication (versus other types of errors), such as setting the LED. This special treatment is due to the
         * fact that if communication works as expected, all other errors will reach the PC and will be handled there.
         * If communication fails, however, no information will ever reach the PC.
         *
         * @note This method is specialized to send Module messages. There is an overloaded version of this method that
         * only takes command and error_code arguments, which allows sending communication error messages from the
         * Kernel class.
         *
         * @param module_type The byte-code specifying the type of the module that encountered the error.
         * @param module_id The ID byte-code of the specific module within the broader module_type family that
         * encountered the error.
         * @param command The byte-code specifying the command executed by the module that encountered the error.
         * @param error_code The byte-code specifying the specific module-level error code for this type of
         * communication error.
         */
        void SendCommunicationErrorMessage(
            const uint8_t module_type,
            const uint8_t module_id,
            const uint8_t command,
            const uint8_t error_code
        )
        {
            // Combines the latest status of the Communication class (likely an error code) and the TransportLayer
            // status code into a 2-byte array. Jointly, this information should be enough to diagnose the error.
            const uint8_t errors[2] = {communication_status, _transport_layer.transfer_status};

            // Attempts sending the error message. Does not evaluate the status of sending the error message to avoid
            // recursions.
            SendDataMessage(
                module_type,
                module_id,
                command,
                error_code,
                axmc_communication_assets::kPrototypes::kTwoUint8s,
                errors
            );

            // As a fallback in case the error message does not reach the connected system, sets the class status to
            // the error code and activates the built-in LED. The LED is used as a visual indicator for a potentially
            // unhandled runtime error. The Kernel class manages the indicator inactivation.
            digitalWriteFast(LED_BUILTIN, HIGH);
        }

        /// Overloads SendCommunicationErrorMessage to use it for Kernel class. The function works the same as the
        /// module-oriented SendCommunicationErrorMessage, but does not accept module_type and module_id.
        void SendCommunicationErrorMessage(const uint8_t command, const uint8_t error_code)
        {
            // Combines the latest status of the Communication class (likely an error code) and the TransportLayer
            // status code into a 2-byte array. Jointly, this information should be enough to diagnose the error.
            const uint8_t errors[2] = {communication_status, _transport_layer.transfer_status};

            // Attempts sending the error message. Does not evaluate the status of sending the error message to avoid
            // recursions.
            SendDataMessage(command, error_code, axmc_communication_assets::kPrototypes::kTwoUint8s, errors);

            // As a fallback in case the error message does not reach the connected system, sets the class status to
            // the error code and activates the built-in LED. The LED is used as a visual indicator for a potentially
            // unhandled runtime error. The Kernel class manages the indicator inactivation.
            digitalWriteFast(LED_BUILTIN, HIGH);
        }

        /**
         * @brief Uses the requested protocol to send the input service_code to the PC.
         *
         * This method is used to transmit all Service messages. All service messages use the same minimalistic one-byte
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
            // Currently, only Idle and ReceptionCode protocols are considered valid ServiceMessage protocols.
            const auto protocol = static_cast<axmc_communication_assets::kProtocols>(protocol_code);
            if (protocol == axmc_communication_assets::kProtocols::kReceptionCode ||
                protocol == axmc_communication_assets::kProtocols::kIdentification)
            {
                // Constructs the message array. This is qualitatively similar to how the rest of the message structures
                // work, but it is easier to use an array here.
                const uint8_t message[2] = {protocol_code, service_code};

                // If writing the dat to transmission buffer, breaks the runtime with an error status.
                const uint16_t next_index = _transport_layer.WriteData(message);
                if (next_index == 0)
                {
                    communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kPackingError);
                    return false;
                }

                // Sends the data to the PC
                if (!_transport_layer.SendData())
                {
                    // If send operation fails, returns with an error status
                    communication_status =
                        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kTransmissionError);
                    return false;
                }

                // Returns with a success code
                communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent);
                return true;
            }

            // If input protocol code is not one of the valid Service protocol codes, aborts with an error
            communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kInvalidProtocol);
            return false;
        }

        /**
         * @brief Parses the next available message received from the PC and stored inside the serial port reception
         * buffer.
         *
         * This method will only attempt message reception (parsing) if the Serial interface buffer, managed by the
         * TransportLayer class, has the minimum number of bytes required to represent the smallest possible message.
         *
         * @note If this method returns true, use 'protocol_code' class attribute to access the protocol code of the
         * parsed message. Based on the protocol code, use the appropriate class attribute to access the parsed message
         * data.
         *
         * @attention If the received message is a ModuleParameters message, call ExtractModuleParameters() method to
         * finalize message parsing. This method DOES NOT extract Module parameter data from the received message.
         *
         * @returns True if a message was successfully received and parsed, false otherwise. Note, if this method
         * returns false, this does not necessarily indicate runtime error. Use 'communication_status' class attribute
         * to determine the cause of the failure. kCommunicationNoBytesToReceive status code from
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
                    static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kNoBytesToParseFromBuffer))
                {
                    communication_status =
                        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kReceptionError);
                    return false;
                }

                // Otherwise, returns with a status that indicates non-error failure.
                communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kNoBytesToReceive);
                return false;
            }

            // If the message is received and decoded, extracts the protocol code of the received message and uses it to
            // parse the rest of the message
            const uint16_t next_index = _transport_layer.ReadData(protocol_code);
            if (next_index != 0)
            {
                // Pre-sets the status to the success code, assuming the switch below will resolve the message.
                // The status will be overridden as necessary if the switch statement fails for any reason.
                communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageReceived);

                // Uses efficient switch logic to resolve and handle the data based on the protocol code
                switch (static_cast<axmc_communication_assets::kProtocols>(protocol_code))
                {
                    case axmc_communication_assets::kProtocols::KRepeatedModuleCommand:
                        // The 'if' checks for the implicit 'next_index' returned by the ReadData() method to not be 0.
                        // 0 indicates that the data was not read successfully. Any other number indicates successful
                        // read operation. Inlining everything makes the code more readable.
                        if (_transport_layer.ReadData(repeated_module_command, next_index)) return true;
                        break;

                    case axmc_communication_assets::kProtocols::kOneOffModuleCommand:
                        if (_transport_layer.ReadData(one_off_module_command, next_index)) return true;
                        break;

                    case axmc_communication_assets::kProtocols::kDequeueModuleCommand:
                        if (_transport_layer.ReadData(module_dequeue, next_index)) return true;
                        break;

                    case axmc_communication_assets::kProtocols::kKernelCommand:
                        if (_transport_layer.ReadData(kernel_command, next_index)) return true;
                        break;

                    case axmc_communication_assets::kProtocols::kModuleParameters:
                        // Reads the HEADER of the message into the storage structure. This gives Kernel class enough
                        // information to address the message, but this is NOT the whole message. To retrieve parameter
                        // data bundled with the message, use ExtractModuleParameters() method.
                        if (_transport_layer.ReadData(module_parameter, next_index)) return true;
                        break;

                    case axmc_communication_assets::kProtocols::kKernelParameters:
                        if (_transport_layer.ReadData(kernel_parameters, next_index)) return true;
                        break;

                    default:
                        // If input protocol code is not one of the valid protocols, aborts with an error status.
                        communication_status =
                            static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kInvalidProtocol);
                        return false;
                }
            }

            // If any of the data reading method calls failed (returned a next_index == 0), returns with error status
            communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParsingError);
            return false;
        }

        /**
         * @brief Extracts the parameter data transmitted with the ModuleParameters message and uses it to set the input
         * structure values.
         *
         * This method executes the second step of the ModuleParameter message reception process. It extracts the
         * necessary number of bytes to overwrite the input structure's memory with incoming message data. Kernel
         * class uses this method to set the target Module's parameter structure to use the new data.
         *
         * @warning This method will fail if it is called for any other message type, including KernelParameters
         * message.
         *
         * @tparam ObjectType The type of the input structure object. This is resolved automatically and allows using
         * this method with most valid objet structures.
         * @param prototype The data structure instance whose memory needs to be overwritten with received data.
         * @param bytes_to_read The number of bytes that make up the prototype. This is used as an extra safety check to
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
            if (protocol_code != static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleParameters))
            {
                communication_status =
                    static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kExtractionForbidden);
                return false;
            }

            // Verifies that the size of the prototype structure exactly matches the number of object bytes received
            // with the message. The '-1' accounts for the protocol code (first variable of each message) that precedes
            // the message structure.
            if (static_cast<uint8_t>(bytes_to_read) !=
                _transport_layer.get_rx_payload_size() - sizeof(axmc_communication_assets::ModuleParameters) - 1)
            {
                communication_status =
                    static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParameterMismatch);
                return false;
            }

            // If both checks above are passed, extracts the parameter data from the incoming message into the provided
            // structure (by reference). If ReadData() returns 0 (false), the extraction has failed.
            if (!_transport_layer.ReadData(prototype, kParameterObjectIndex))
            {
                communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParsingError);
                return false;
            }

            // Returns with success code
            communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParametersExtracted);
            return true;  // Successfully extracted parameters.
        }

    private:
        /// Stores the minimum valid incoming payload size. Currently, this is the size of the KernelCommandMessage
        /// (2 bytes) + 1 Protocol byte. This value is used to optimize incoming message reception behavior.
        static constexpr uint16_t kMinimumPayloadSize = sizeof(axmc_communication_assets::KernelCommand) + 1;

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

        /// Stores the first index of the parameter object in the incoming ModuleParameters message payload.
        /// Since Module-addressed parameter objects can vary to a great extent, they are retrieved in two steps. The
        /// first step parses the message header and identifies the target module. The second step asks the module to
        /// extract the parameter data, since the module knows what data structure to expect. This index is used to
        /// continue reading message data from the point where the parser finished reading the message header.
        static constexpr uint16_t kParameterObjectIndex = sizeof(axmc_communication_assets::ModuleParameters) + 1;

        /// The bound TransportLayer instance that abstracts low-level data communication steps. This class statically
        /// specializes and initializes the TransportLayer to use sensible defaults. It is highly advised not to alter
        /// default initialization parameters unless you know what you are doing.
        TransportLayer<uint16_t, kMaximumPayloadSize, kMaximumPayloadSize, kMinimumPayloadSize> _transport_layer;
};

#endif  //AXMC_COMMUNICATION_H
