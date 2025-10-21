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
         * // Protocol is given as a template, service code as an uint8_t argument.
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
         * @brief If a message is currently stored in the serial interface's reception buffer, moves it into the
         * instance's reception buffer.
         *
         * Depending on the protocol used by the received message, the message header data is read into the appropriate
         * instance's attribute structure.
         *
         * @attention If the received message is a ModuleParameters message, call the ExtractModuleParameters() method
         * to extract the data payload. This method DOES NOT extract Module parameter data from the serial buffer.
         *
         * @returns True if a message was successfully received, false otherwise. If this method returns false, this
         * does not by itself indicate a runtime error. Use the 'communication_status' instance property
         * to determine if the method encountered a runtime error.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);  // Initializes serial interface.
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         * bool success = comm_class.ReceiveMessage();  // Attempts to receive the message
         * @endcode
         */
        bool ReceiveMessage()
        {
            // Attempts to receive the next available message
            if (!_transport_layer.ReceiveData())
            {
                // The reception protocol can 'fail' gracefully if the reception buffer does not have enough bytes to
                // attempt message reception.
                if (_transport_layer.runtime_status == static_cast<uint8_t>(kTransportStatusCodes::kNoBytesToParse))
                {
                    communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kNoBytesToReceive);
                    return false;
                }

                // Otherwise, returns with a status that indicates runtime failure.
                communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kReceptionError);
                return false;
            }

            // If the message is received and decoded, extracts the protocol code of the received message and uses
            // it to parse the rest of the message
            if (_transport_layer.ReadData(protocol_code))
            {
                communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kMessageReceived);
                // Unpacks the message header into the appropriate attribute structure.
                switch (static_cast<kProtocols>(protocol_code))
                {
                    case kProtocols::KRepeatedModuleCommand:
                        if (_transport_layer.ReadData(repeated_module_command)) return true;
                        break;

                    case kProtocols::kOneOffModuleCommand:
                        if (_transport_layer.ReadData(one_off_module_command)) return true;
                        break;

                    case kProtocols::kDequeueModuleCommand:
                        if (_transport_layer.ReadData(module_dequeue)) return true;
                        break;

                    case kProtocols::kKernelCommand:
                        if (_transport_layer.ReadData(kernel_command)) return true;
                        break;

                    case kProtocols::kModuleParameters:
                        // Reads the HEADER of the message into the storage structure. This gives the Kernel class
                        // enough information to address the message, but this is NOT the whole message. To retrieve
                        // the parameter data bundled with the message, use the ExtractModuleParameters() method.
                        if (_transport_layer.ReadData(module_parameters_header)) return true;
                        break;

                    default:
                        // If input protocol code is not one of the valid protocols, aborts with an error status.
                        communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kInvalidProtocol);
                        return false;
                }
            }

            // If any of the data reading calls above fail, returns with an error status.
            communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kParsingError);
            return false;
        }

        /**
         * @brief Extracts the parameter data payload transmitted with the last received ModuleParameters message into
         * the destination object's memory.
         *
         * @warning This method is intended to be called by end users as part of the SetCustomParameters() virtual
         * method implementation. Do not call this method from any other context.
         *
         * @tparam ObjectType The type of the destination object.
         * @tparam object_size The size of the destination object, in bytes.
         * @param destination The object where to unpack the received parameters. Typically, this is a packed structure.
         *
         * @returns True if the parameter data was successfully extracted into the destination object and false
         * otherwise.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);  // Initializes serial interface.
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         *
         * // Initializes a test structure.
         * struct DataMessage
         * {
         *     uint8_t id = 1;
         *     uint8_t data = 10;
         * } data_message;
         *
         * bool success = comm_class.ExtractParameters(data_message);  // Attempts to extract the parameters.
         * @endcode
         */
        template <typename ObjectType, const size_t object_size = sizeof(ObjectType)>
        bool ExtractModuleParameters(ObjectType& destination)
        {
            // Ensures that the prototype compiles with the limitations of the transport layer.
            static_assert(
                object_size > 0 && object_size <= 250,
                "Unable to extract the target module's parameters as the method has received an invalid "
                "'destination' input. A valid destination object must have a size between 1 and 250 bytes."
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
            if (static_cast<uint8_t>(object_size) !=
                _transport_layer.get_bytes_in_reception_buffer() - sizeof(ModuleParameters) - 1)
            {
                communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kParameterMismatch);
                return false;
            }

            // If both checks above are passed, extracts the parameter data from the incoming message into the provided
            // structure (by reference).
            if (!_transport_layer.ReadData(destination))
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
