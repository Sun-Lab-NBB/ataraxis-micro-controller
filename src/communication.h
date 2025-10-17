/**
 * @file
 * @brief Provides the Communication class that establishes and maintains bidirectional communication with
 * host-computers (PCs) running the ataraxis-communication-interface Python library.
 *
 * @section comm_description Description:
 *
 * This class defines the communication message structures and provides the API used by other library components to
 * exchange data with the interface running on the PC.
 *
 * @note A single shared instance of this class should be created inside the main.cpp file and provided as an
 * initialization argument to the Kernel instance and all Module-derived instances.
 */

#ifndef AXMC_COMMUNICATION_H
#define AXMC_COMMUNICATION_H

// Dependencies:
#include <Arduino.h>
#include <axtlmc_shared_assets.h>
#include <digitalWriteFast.h>
#include <transport_layer.h>
#include "axmc_shared_assets.h"

using namespace axmc_shared_assets;
using namespace axtlmc_shared_assets;
using namespace axmc_communication_assets;

/**
 * @class Communication
 * @brief Exposes methods that allow exchanging data with a host-computer (PC) running the
 * ataraxis-communication-interface library.
 *
 * @warning This class is explicitly designed to be used by other library assets and should not be used directly by
 * end users. A single shared instance of this class must be provided to the Kernel and all Module-derived classes as
 * initialization arguments.
 */
class Communication
{
    public:
        /// Stores the runtime status of the most recently called method.
        uint8_t communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kStandby);

        /// Stores the protocol code of the last received message.
        uint8_t protocol_code = static_cast<uint8_t>(kProtocols::kUndefined);

        /// Stores the last received Module-addressed recurrent (repeated) command message data.
        RepeatedModuleCommand repeated_module_command;

        /// Stores the last received Module-addressed non-recurrent (one-off) command message data.
        OneOffModuleCommand one_off_module_command;

        /// Stores the last received Kernel-addressed command message data.
        KernelCommand kernel_command;

        /// Stores the last received Module-addressed dequeue command message data.
        DequeueModuleCommand module_dequeue;

        /// Stores the last received Module-addressed parameters message header data.
        ModuleParameters module_parameters_header;

        /// Stores the last received Kernel-addressed parameters message data.
        KernelParameters kernel_parameters;

        /**
         * @brief Instantiates a specialized TransportLayer instance to handle the microcontroller-PC communication.
         *
         * @warning This class reserves up to ~1kB of RAM during runtime. On supported lower-end microcontrollers this
         * number may be lowered up to ~700 bytes due to adaptive optimization.
         *
         * @param communication_port The initialized communication interface instance, such as Serial or USB Serial.
         *
         * Example instantiation:
         * @code
         * Serial.begin(9600);  // Initializes the serial communication interface.
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         * @endcode
         */
        explicit Communication(Stream& communication_port) :
            _transport_layer(
                communication_port,  // Stream
                0x1021,              // 16-bit CRC Polynomial
                0xFFFF,              // Initial CRC value
                0x0000               // Final CRC XOR value
            )
        {}

        /**
         * @brief Returns the most recent TransportLayer's status code.
         */
        [[nodiscard]]
        uint8_t GetTransportLayerStatus() const
        {
            return _transport_layer.runtime_status;
        }

        /**
         * @brief Sends the input event code and data object to the PC.
         *
         * @warning Use the SendStateMessage() method to communicate the event-code without any additional data for
         * faster transmission.
         *
         * @tparam ObjectType The type of the data object to be sent along with the message.
         * @param module_type The type of the module that sent the message.
         * @param module_id The ID of the specific module instance that sent the message.
         * @param command The command executed by the module that sent the message.
         * @param event_code The event that triggered the message.
         * @param prototype The type of the data object transmitted with the message. Must be one of the kPrototypes
         * enumeration members.
         * @param object The data object to be sent along with the message.
         *
         * @returns True if the message is sent, false otherwise.
         *
         * Example usage:
         * @code
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         * Serial.begin(9600);  // Initializes serial interface.
         *
         * // Sends the example data message.
         * const uint8_t module_type = 112
         * const uint8_t module_id = 12;
         * const uint8_t command = 88;
         * const uint8_t event_code = 221;
         * auto prototype = kPrototypes::kOneUint8;
         * const uint8_t placeholder_object = 255;
         * comm_class.SendDataMessage(module_type, module_id, command, event_code, prototype, placeholder_object);
         * @endcode
         */
        template <typename ObjectType>
        bool SendDataMessage(
            const uint8_t module_type,
            const uint8_t module_id,
            const uint8_t command,
            const uint8_t event_code,
            const kPrototypes prototype,
            const ObjectType& object
        )
        {
            // Ensures that the input fits inside the message payload buffer.
            static_assert(
                sizeof(ObjectType) <= kMaximumPayloadSize - sizeof(ModuleData),
                "The provided object is too large to fit inside the message payload buffer. This check accounts for "
                "the size of the ModuleData header sent with the object."
            );

            // Constructs the message header
            const ModuleData message {
                static_cast<uint8_t>(kProtocols::kModuleData),
                module_type,
                module_id,
                command,
                event_code,
                static_cast<uint8_t>(prototype)
            };

            // Writes the message to the transmission buffer.
            bool success = true;
            if (!_transport_layer.WriteData(message)) success = false;
            if (success && !_transport_layer.WriteData(object)) success = false;

            // If serializing the message and the data payload fails, breaks the runtime with an error status.
            if (!success)
            {
                communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kPackingError);
                return false;
            }

            // If the data was written to the buffer, sends it to the PC.
            _transport_layer.SendData();
            communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kMessageSent);
            return true;
        }

        /// Overloads SendDataMessage to support sending Kernel data messages.
        template <typename ObjectType>
        bool SendDataMessage(
            const uint8_t command,
            const uint8_t event_code,
            const kPrototypes prototype,
            const ObjectType& object
        )
        {
            // Ensures that the input fits inside the message payload buffer.
            static_assert(
                sizeof(ObjectType) <= kMaximumPayloadSize - sizeof(KernelData),
                "The provided object is too large to fit inside the message payload buffer. This check accounts for "
                "the size of the KernelData header sent with the object."
            );

            // Constructs the message header
            const KernelData message {
                static_cast<uint8_t>(kProtocols::kKernelData),
                command,
                event_code,
                static_cast<uint8_t>(prototype)
            };

            // Writes the message to the transmission buffer.
            bool success = true;
            if (!_transport_layer.WriteData(message)) success = false;
            if (success && !_transport_layer.WriteData(object)) success = false;

            // If serializing the message and the data payload fails, breaks the runtime with an error status.
            if (!success)
            {
                communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kPackingError);
                return false;
            }

            // If the data was written to the buffer, sends it to the PC.
            _transport_layer.SendData();
            communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kMessageSent);
            return true;
        }

        /**
         * @brief Sends the input event code to the PC.
         *
         * @note Use the SendDataMessage() method to send a message with an additional arbitrary data object.
         *
         * @param module_type The type of the module that sent the message.
         * @param module_id The ID of the specific module instance that sent the message.
         * @param command The command executed by the module that sent the message.
         * @param event_code The event that triggered the message.
         *
         * @returns True if the message is sent, false otherwise.
         *
         * Example usage:
         * @code
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         * Serial.begin(9600);  // Initializes serial interface.
         *
         * // Sends the example state message.
         * const uint8_t module_type = 112;
         * const uint8_t module_id = 12;
         * const uint8_t command = 88;
         * const uint8_t event_code = 221;
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
            // Constructs the message header.
            const ModuleState
                message {static_cast<uint8_t>(kProtocols::kModuleState), module_type, module_id, command, event_code};

            // Writes the message into the payload buffer. If writing fails, breaks the runtime with an error status.
            if (!_transport_layer.WriteData(message))
            {
                communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kPackingError);
                return false;
            }

            // If the data was written to the buffer, sends it to the PC.
            _transport_layer.SendData();
            communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kMessageSent);
            return true;
        }

        /// Overloads SendDataMessage to support sending Kernel state messages.
        bool SendStateMessage(const uint8_t command, const uint8_t event_code)
        {
            // Constructs the message header.
            const KernelState message {static_cast<uint8_t>(kProtocols::kKernelState), command, event_code};

            // Writes the message into the payload buffer. If writing fails, breaks the runtime with an error status.
            if (!_transport_layer.WriteData(message))
            {
                communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kPackingError);
                return false;
            }

            // If the data was written to the buffer, sends it to the PC.
            _transport_layer.SendData();
            communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kMessageSent);
            return true;
        }

        /**
         * @brief Sends the communication error message to the PC and activates the built-in LED.
         *
         * @warning This method is reserved for Communication class errors. Use SendDataMessage and SendStateMessage
         * methods for all other errors.
         *
         * @param module_type The type of the module that sent the message.
         * @param module_id The ID of the specific module instance that sent the message.
         * @param command The command executed by the module that sent the message.
         * @param error_code The encountered communication error.
         */
        void SendCommunicationErrorMessage(
            const uint8_t module_type,
            const uint8_t module_id,
            const uint8_t command,
            const uint8_t error_code
        )
        {
            // Combines the latest statuses of the Communication class and the TransportLayer class into a 2-byte array.
            // Jointly, this information should be enough to diagnose the error.
            const uint8_t errors[2] = {communication_status, _transport_layer.runtime_status};

            // Attempts sending the error message. Does not evaluate the status of sending the error message to avoid
            // recursions.
            SendDataMessage(module_type, module_id, command, error_code, kPrototypes::kTwoUint8s, errors);

            // As a fallback in case the error message does not reach the connected system, sets the class status to
            // the error code and activates the built-in LED. The LED is used as a visual indicator for a potentially
            // unhandled runtime error. The Kernel class manages the indicator inactivation.
            digitalWriteFast(LED_BUILTIN, HIGH);
        }

        /// Overloads SendCommunicationErrorMessage to support sending Kernel communication error messages.
        void SendCommunicationErrorMessage(const uint8_t command, const uint8_t error_code)
        {
            // Combines the latest statuses of the Communication class and the TransportLayer class into a 2-byte array.
            // Jointly, this information should be enough to diagnose the error.
            const uint8_t errors[2] = {communication_status, _transport_layer.runtime_status};

            // Attempts sending the error message. Does not evaluate the status of sending the error message to avoid
            // recursions.
            SendDataMessage(command, error_code, kPrototypes::kTwoUint8s, errors);

            // As a fallback in case the error message does not reach the connected system, sets the class status to
            // the error code and activates the built-in LED. The LED is used as a visual indicator for a potentially
            // unhandled runtime error. The Kernel class manages the indicator inactivation.
            digitalWriteFast(LED_BUILTIN, HIGH);
        }

        /**
         * @brief Uses the specified service message protocol to send the input service code to the PC.
         *
         * @tparam protocol The protocol to use for the transmitted message. Has to be one of the following kProtocols
         * enumeration members: kReceptionCode, kControllerIdentification or kModuleIdentification.
         * @tparam ObjectType The data type of the service code value.
         * @param service_code The service code to be transmitted to the PC.
         *
         * @returns True if the message is sent, false otherwise.
         *
         * Example usage:
         * @code
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         * Serial.begin(9600);  // Initializes serial interface.
         *
         * // Protocol is given as template, service code as an uint8_t argument.
         * comm_class.SendServiceMessage<kProtocols::kReceptionCode>(112);
         * @endcode
         */
        template <const kProtocols protocol, typename ObjectType>
        bool SendServiceMessage(const ObjectType service_code)
        {
            // Ensures the communication protocol is one of the supported Service protocols.
            static_assert(
                protocol == kProtocols::kReceptionCode || protocol == kProtocols::kControllerIdentification ||
                    protocol == kProtocols::kModuleIdentification,
                "Encountered an invalid ServiceMessage protocol code. Use one of the supported Service protocols from "
                "the kProtocols enumeration."
            );

            // Ensures that the provide service_code is of the correct type.
            static_assert(
                axtlmc_shared_assets::is_same_v<ObjectType, uint8_t> ||
                    axtlmc_shared_assets::is_same_v<ObjectType, uint16_t> ||
                    axtlmc_shared_assets::is_same_v<ObjectType, uint32_t>,
                "Encountered an invalid ServiceMessage service code type. Currently, only uint8_t, uint16_t or "
                "uint32_t service codes are supported."
            );

            // Packages hte input protocol code and the service code into the transmission buffer.
            bool success = true;
            if (!_transport_layer.WriteData(static_cast<uint8_t>(protocol))) success = false;
            if (success && !_transport_layer.WriteData(service_code)) success = false;

            // If the data was written to the buffer, sends it to the PC.
            _transport_layer.SendData();
            communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kMessageSent);
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
         * false, this does not necessarily indicate a runtime error. Use 'communication_status' class attribute to
         * determine the cause of the failure. kNoBytesToReceive status code from
         * kCommunicationCodes indicates a non-error failure.
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
                if (_transport_layer.transfer_status != static_cast<uint8_t>(kTransportStatusCodes::kNoBytesToParse))
                {
                    communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kReceptionError);
                    return false;
                }

                // Otherwise, returns with a status that indicates non-error failure.
                communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kNoBytesToReceive);
                return false;
            }

            // If the message is received and decoded, extracts the protocol code of the received message and uses it to
            // parse the rest of the message
            const uint16_t next_index = _transport_layer.ReadData(protocol_code);
            if (next_index != 0)
            {
                // Pre-sets the status to the success code, assuming the switch below will resolve the message.
                // The status will be overridden as necessary if the switch statement fails for any reason.
                communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kMessageReceived);

                // Uses efficient switch logic to resolve and handle the data based on the protocol code
                switch (static_cast<kProtocols>(protocol_code))
                {
                    case kProtocols::KRepeatedModuleCommand:
                        // The 'if' checks for the implicit 'next_index' returned by the ReadData() method to not be 0.
                        // 0 indicates that the data was not read successfully. Any other number indicates successful
                        // read operation. Inlining everything makes the code more readable.
                        if (_transport_layer.ReadData(repeated_module_command, next_index)) return true;
                        break;

                    case kProtocols::kOneOffModuleCommand:
                        if (_transport_layer.ReadData(one_off_module_command, next_index)) return true;
                        break;

                    case kProtocols::kDequeueModuleCommand:
                        if (_transport_layer.ReadData(module_dequeue, next_index)) return true;
                        break;

                    case kProtocols::kKernelCommand:
                        if (_transport_layer.ReadData(kernel_command, next_index)) return true;
                        break;

                    case kProtocols::kModuleParameters:
                        // Reads the HEADER of the message into the storage structure. This gives Kernel class enough
                        // information to address the message, but this is NOT the whole message. To retrieve parameter
                        // data bundled with the message, use ExtractModuleParameters() method.
                        if (_transport_layer.ReadData(module_parameters_header, next_index)) return true;
                        break;

                    case kProtocols::kKernelParameters:
                        if (_transport_layer.ReadData(kernel_parameters, next_index)) return true;
                        break;

                    default:
                        // If input protocol code is not one of the valid protocols, aborts with an error status.
                        communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kInvalidProtocol);
                        return false;
                }
            }

            // If any of the data reading method calls failed (returned a next_index == 0), returns with error status
            communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kParsingError);
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
            if (protocol_code != static_cast<uint8_t>(kProtocols::kModuleParameters))
            {
                communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kExtractionForbidden);
                return false;
            }

            // Verifies that the size of the prototype structure exactly matches the number of object bytes received
            // with the message. The '-1' accounts for the protocol code (first variable of each message) that precedes
            // the message structure.
            if (static_cast<uint8_t>(bytes_to_read) !=
                _transport_layer.get_rx_payload_size() - sizeof(ModuleParameters) - 1)
            {
                communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kParameterMismatch);
                return false;
            }

            // If both checks above are passed, extracts the parameter data from the incoming message into the provided
            // structure (by reference). If ReadData() returns 0 (false), the extraction has failed.
            if (!_transport_layer.ReadData(prototype, kModuleParameterObjectIndex))
            {
                communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kParsingError);
                return false;
            }

            // Returns with success code
            communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kParametersExtracted);
            return true;  // Successfully extracted parameters.
        }

    private:
        /// The maximum possible size for the received and transmitted payloads. Reuses the
        /// kSerialBufferSize constant defined inside transport_layer.h to determine the serial buffer size of the
        /// host microcontroller.
        static constexpr uint8_t kMaximumPayloadSize = min(kSerialBufferSize - 6, 254);

        /// The TransportLayer instance that handles the bidirectional communication with the PC.
        TransportLayer<uint16_t, kMaximumPayloadSize, kMaximumPayloadSize> _transport_layer;
};

#endif  //AXMC_COMMUNICATION_H
