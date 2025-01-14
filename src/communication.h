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
 * - axtlmc_shared_assets.h for TransportLayer status code enumerations and is_same_v implementation.
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - transport_layer.h for low-level methods for sending and receiving messages over USB or UART Serial port.
 * - axmc_shared_assets.h for globally shared assets, including axmc_communication_assets namespace.
 */

#ifndef AXMC_COMMUNICATION_H
#define AXMC_COMMUNICATION_H

// Dependencies:
#include <Arduino.h>
#include <axtlmc_shared_assets.h>
#include <digitalWriteFast.h>
#include <transport_layer.h>
#include "axmc_shared_assets.h"

/**
 * @class Communication
 * @brief Exposes methods that allow establishing and maintaining bidirectional serialized communication with a
 * host-computer (PC) running the ataraxis-communication-interface library via the internal binding of the
 * TransportLayer class.
 *
 * This class handles all communications between the microcontroller and the PC over the USB (preferred) or UART serial
 * interface. To do so, the class extends the TransportLayer class methods to send and receive a statically defined set
 * of messages with known layout. All supported message structures are available from the axmc_communication_assets
 * namespace.
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
         * essential for the correct functioning of the communication interface. The class uses CRC-16 polynomial for
         * data integrity verification and dynamically configures TransportLayer buffer sizes to not exceed the size
         * of the microcontroller's serial buffer.
         *
         * @warning This class can reserve up to ~1kB of RAM to store runtime data. On supported lower-end
         * microcontrollers, this number may be lowered up to ~700 bytes due to adaptive optimization.
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
         * @brief Returns the most recent status code of the TransportLayer class used by the Communication class.
         *
         * This method is used  when handling Communication class errors. Knowing the status code of the TransportLayer
         * class is crucial for understanding the root cause of Communication class errors.
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
         * object data. All valid prototype codes are stored in the axmc_communication_assets::kPrototypes
         * enumeration.
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
         * const uint8_t module_type = 112  // Example module type
         * const uint8_t module_id = 12;    // Example module ID
         * const uint8_t command = 88;      // Example command code
         * const uint8_t event_code = 221;  // Example event code
         * auto prototype = axmc_communication_assets::kPrototypes::kOneUint8;  // Prototype code.
         * const uint8_t placeholder_object = 255;  // Has to be a single unsigned integer!
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
            // Ensures that the input fits inside the message payload buffer.
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
         * This method is very similar to SendDataMessage, but is optimized to not use additional data objects. It
         * will execute slightly faster, and the payloads transmitted by this method may be significantly smaller than
         * those transmitted by SendDataMessage.
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
                module_type,
                module_id,
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
         * This method is used to transmit all Service messages. All service messages contain a protocol code and a
         * single scalar 'code' value. However, the bit-width and the meaning of each service code depends on the
         * particular protocol code value.
         *
         * @tparam protocol The byte-code specifying the protocol to use for the transmitted message. Has to be either
         * axmc_communication_assets::kProtocols::kReceptionCode,
         * axmc_communication_assets::kProtocols::kControllerIdentification or
         * axmc_communication_assets::kProtocols::kModuleIdentification.
         * @tparam ObjectType The type of the service code. Has to be either uint8_t, uint16_t or uint32_t.
         * @param service_code The scalar service code to be transmitted to the PC.
         *
         * @returns True if the message was successfully sent, false otherwise.
         *
         * Example usage:
         * @code
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         * Serial.begin(9600);  // Initializes serial interface.
         *
         * // Protocol is given as template, service code as an uint8_t argument
         * comm_class.SendServiceMessage<axmc_communication_assets::kProtocols::kReceptionCode>(112);
         * @endcode
         */
        template <const axmc_communication_assets::kProtocols protocol, typename ObjectType>
        bool SendServiceMessage(const ObjectType service_code)
        {
            // Ensures the communication protocol is one of the supported Service protocols.
            static_assert(
                protocol == axmc_communication_assets::kProtocols::kReceptionCode ||
                    protocol == axmc_communication_assets::kProtocols::kControllerIdentification ||
                    protocol == axmc_communication_assets::kProtocols::kModuleIdentification,
                "Encountered an invalid ServiceMessage protocol code. Use one of the supported Service protocols from "
                "the axmc_communication_assets::kProtocols enumerations."
            );

            // Ensures that the provide service_code is of the correct type.
            static_assert(
                axtlmc_shared_assets::is_same_v<ObjectType, uint8_t> ||
                    axtlmc_shared_assets::is_same_v<ObjectType, uint16_t> ||
                    axtlmc_shared_assets::is_same_v<ObjectType, uint32_t>,
                "Encountered an invalid ServiceMessage service code type. Currently, only scalar uint8_t, uint16_t or "
                "uint32_t service codes are supported."
            );

            // Packages hte input protocol code and the service code into the transmission buffer
            uint16_t next_index = _transport_layer.WriteData(static_cast<uint8_t>(protocol));
            if (next_index!= 0) next_index          = _transport_layer.WriteData(service_code, next_index);

            // If writing the data to transmission buffer, breaks the runtime with an error status.
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

        /**
         * @brief Parses the next available PC-sent message stored inside the serial port reception buffer.
         *
         * This method will only attempt message reception (parsing) if the serial interface buffer, managed by the
         * TransportLayer class, has the minimum number of bytes required to represent the smallest valid message.
         *
         * @note If this method returns true, it uses the 'protocol_code' to store the protocol of the received message.
         * In turn, that determines which of the class attributes stores the parsed message data.
         *
         * @attention If the received message is a ModuleParameters message, call ExtractModuleParameters() method to
         * finalize message parsing. This method DOES NOT extract Module parameter data from the serial buffer.
         *
         * @returns True if a message was successfully received and parsed, false otherwise. If this method returns
         * false, this does not necessarily indicate runtime error. Use 'communication_status' class attribute to
         * determine the cause of the failure. kNoBytesToReceive status code from
         * axmc_shared_assets::kCommunicationCodes indicates a non-error failure.
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
         * @brief Extracts the parameter data transmitted with the ModuleParameters message and uses it to overwrite the
         * memory of the input 'prototype' object.
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
         * @tparam bytes_to_read The number of bytes that make up the prototype. This is used as an extra safety check
         * to ensure the parameter data extracted from the message (whose size is known) matches the size expected by
         * the structure.
         * @param prototype The object whose memory needs to be overwritten with received data.
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
        template <typename ObjectType, const size_t bytes_to_read = sizeof(ObjectType)>
        bool ExtractModuleParameters(ObjectType& prototype)
        {
            // This guards against invalid calls at compile time
            static_assert(
                bytes_to_read > 0 && bytes_to_read <= 250,
                "Unable to extract received module parameters as the method has received an invalid 'prototype' input. "
                "Since a single received message payload exceed 254 bytes and ModuleParameters message header data "
                "reserves 4 bytes, a valid prototype object cannot exceed 250 bytes in size. Also, since sending empty"
                "parameter messages is not allowed, the prototype has to be greater than 0."
            );

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

        /// Calculates the maximum transmitted and received payload sizes. At a maximum, this can be 254 bytes
        /// (COBS limitation). For most controllers, this will be a lower value that depends on the available buffer
        /// space and the space reserved for TransportLayer variables. Note, this reuses kSerialBufferSize constant
        /// defined inside transport_layer.h to determine the serial buffer size!
        static constexpr uint8_t kMaximumPayloadSize = min(kSerialBufferSize - 6, 254);

        /// Stores the start index of the parameter object data in the incoming ModuleParameters message payload. This
        /// relies on the fact that the header of each ModuleParameter is always the same size, whereas the parameter
        /// data section varies in length.
        static constexpr uint8_t kParameterObjectIndex = sizeof(axmc_communication_assets::ModuleParameters) + 1;

        /// The bound TransportLayer instance that abstracts low-level data communication steps. Communication
        /// statically specializes and initializes the TransportLayer to use sensible defaults. It is highly advised
        /// not to alter default initialization parameters unless you know what you are doing.
        TransportLayer<uint16_t, kMaximumPayloadSize, kMaximumPayloadSize, kMinimumPayloadSize> _transport_layer;
};

#endif  //AXMC_COMMUNICATION_H
