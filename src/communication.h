/**
 * @file
 *
 * @brief Provides the Communication class that establishes and maintains bidirectional communication with
 * host-computers (PCs) running the ataraxis-communication-interface Python library.
 *
 * Defines the communication message structures and provides the API used by other library components to
 * exchange data with the interface running on the PC.
 *
 * @note A single shared instance of this class should be created inside the main.cpp file and provided as an
 * initialization argument to the Kernel instance and all Module-derived instances.
 */

#ifndef AXMC_COMMUNICATION_H
#define AXMC_COMMUNICATION_H

#include <Arduino.h>
#include <axtlmc_shared_assets.h>
#include <digitalWriteFast.h>
#include <transport_layer.h>
#include "axmc_shared_assets.h"

// The digitalWriteFast library aliases digitalWriteFast and digitalReadFast to the standard Arduino functions on every
// non-AVR target. Teensy boards ship always-inline register-level implementations of both, so removing the aliases
// exposes them. The pinModeFast alias is kept, as Teensy provides no fast counterpart for it.
#if defined(CORE_TEENSY)
#undef digitalWriteFast
#undef digitalReadFast
#endif

using namespace axmc_shared_assets;
using namespace axtlmc_shared_assets;
using namespace axmc_communication_assets;

/**
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
        /**
         * @brief Instantiates a specialized TransportLayer instance to handle the microcontroller-PC communication.
         *
         * @warning This class reserves up to ~1kB of RAM during runtime. On supported lower-end microcontrollers this
         * number may be lowered up to ~700 bytes due to adaptive optimization.
         *
         * @param communication_port The initialized communication interface instance, such as Serial or USB Serial.
         */
        explicit Communication(Stream& communication_port) :
            _transport_layer(
                communication_port,  // communication_port
                kCRCPolynomial,      // crc_polynomial
                kCRCInitialValue,    // crc_initial_value
                kCRCFinalXORValue,   // crc_final_xor_value
                false                // crc_reflected
            )
        {}

        /// Returns the runtime status of the Communication class.
        [[nodiscard]]
        uint8_t get_communication_status() const
        {
            return _communication_status;
        }

        /// Sets the runtime status of the Communication class.
        void set_communication_status(const uint8_t status)
        {
            _communication_status = status;
        }

        /// Returns the protocol code of the last received message.
        [[nodiscard]]
        uint8_t get_protocol_code() const
        {
            return _protocol_code;
        }

        /// Sets the protocol code of the last received message.
        void set_protocol_code(const uint8_t code)
        {
            _protocol_code = code;
        }

        /// Returns the last received Module-addressed recurrent (repeated) command message data.
        [[nodiscard]]
        const RepeatedModuleCommand& get_repeated_module_command() const
        {
            return _repeated_module_command;
        }

        /// Returns the last received Module-addressed non-recurrent (one-off) command message data.
        [[nodiscard]]
        const OneOffModuleCommand& get_one_off_module_command() const
        {
            return _one_off_module_command;
        }

        /// Returns the last received Kernel-addressed command message data.
        [[nodiscard]]
        const KernelCommand& get_kernel_command() const
        {
            return _kernel_command;
        }

        /// Returns the last received Module-addressed dequeue command message data.
        [[nodiscard]]
        const DequeueModuleCommand& get_module_dequeue() const
        {
            return _module_dequeue;
        }

        /// Returns the last received Module-addressed parameters message header data.
        [[nodiscard]]
        const ModuleParameters& get_module_parameters_header() const
        {
            return _module_parameters_header;
        }

        /// Returns the most recent TransportLayer's status code.
        [[nodiscard]]
        uint8_t get_transport_layer_status() const
        {
            return _transport_layer.get_runtime_status();
        }

        /**
         * @brief Sends the input event code and data object to the PC.
         *
         * The prototype code for the wire protocol is resolved automatically from the C++ type of the data object at
         * compile time via ResolvePrototype.
         *
         * @warning Use the SendStateMessage() method to communicate the event-code without any additional data for
         * faster transmission.
         *
         * @tparam ObjectType The type of the data object to be sent along with the message.
         * @param module_type The type of the module that sent the message.
         * @param module_id The ID of the specific module instance that sent the message.
         * @param command The command executed by the module that sent the message.
         * @param event_code The event that triggered the message.
         * @param object The data payload appended to the message after the header.
         *
         * @returns true if the message is sent, false otherwise.
         */
        template <typename ObjectType>
        bool SendDataMessage(
            const uint8_t module_type,
            const uint8_t module_id,
            const uint8_t command,
            const uint8_t event_code,
            const ObjectType& object
        )
        {
            // Ensures that the input fits inside the message payload buffer.
            static_assert(
                sizeof(ObjectType) <= kMaximumPayloadSize - sizeof(ModuleData),
                "The provided object is too large to fit inside the message payload buffer. This check accounts for "
                "the size of the ModuleData header sent with the object."
            );

            // Constructs the message header. The prototype code is resolved at compile time from ObjectType.
            const ModuleData message {
                static_cast<uint8_t>(kProtocols::kModuleData),
                module_type,
                module_id,
                command,
                event_code,
                static_cast<uint8_t>(ResolvePrototype<ObjectType>())
            };

            bool success = true;
            if (!_transport_layer.WriteData(message)) success = false;
            if (success && !_transport_layer.WriteData(object)) success = false;

            // If serializing the message and the data payload fails, breaks the runtime with an error status.
            if (!success)
            {
                // Discards the header the first write stages before the second one fails. SendData() clears the
                // transmission buffer on every send, and this path returns before reaching it, so the staged bytes
                // would otherwise shrink the payload region available to every later message.
                _transport_layer.ResetTransmissionBuffer();
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kPackingError);
                return false;
            }

            // Sends the buffered data to the PC. Transmission fails if the interface accepts only part of the packet.
            if (!_transport_layer.SendData())
            {
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kTransmissionError);
                return false;
            }

            _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kMessageSent);
            return true;
        }

        /**
         * @brief Sends the input event code and data object to the PC as a Kernel-addressed message.
         *
         * The prototype code for the wire protocol is resolved automatically from the C++ type of the data object at
         * compile time via ResolvePrototype.
         *
         * @tparam ObjectType The type of the data object to be sent along with the message.
         * @param command The command the Kernel was executing when it sent the message.
         * @param event_code The event that triggered the message.
         * @param object The data payload appended to the message after the header.
         *
         * @returns true if the message is sent, false otherwise.
         */
        template <typename ObjectType>
        bool SendDataMessage(const uint8_t command, const uint8_t event_code, const ObjectType& object)
        {
            // Ensures that the input fits inside the message payload buffer.
            static_assert(
                sizeof(ObjectType) <= kMaximumPayloadSize - sizeof(KernelData),
                "The provided object is too large to fit inside the message payload buffer. This check accounts for "
                "the size of the KernelData header sent with the object."
            );

            // Constructs the message header. The prototype code is resolved at compile time from ObjectType.
            const KernelData message {
                static_cast<uint8_t>(kProtocols::kKernelData),
                command,
                event_code,
                static_cast<uint8_t>(ResolvePrototype<ObjectType>())
            };

            bool success = true;
            if (!_transport_layer.WriteData(message)) success = false;
            if (success && !_transport_layer.WriteData(object)) success = false;

            // If serializing the message and the data payload fails, breaks the runtime with an error status.
            if (!success)
            {
                // Discards the header the first write stages before the second one fails. SendData() clears the
                // transmission buffer on every send, and this path returns before reaching it, so the staged bytes
                // would otherwise shrink the payload region available to every later message.
                _transport_layer.ResetTransmissionBuffer();
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kPackingError);
                return false;
            }

            // Sends the buffered data to the PC. Transmission fails if the interface accepts only part of the packet.
            if (!_transport_layer.SendData())
            {
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kTransmissionError);
                return false;
            }

            _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kMessageSent);
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
         * @returns true if the message is sent, false otherwise.
         */
        bool SendStateMessage(
            const uint8_t module_type,
            const uint8_t module_id,
            const uint8_t command,
            const uint8_t event_code
        )
        {
            const ModuleState
                message {static_cast<uint8_t>(kProtocols::kModuleState), module_type, module_id, command, event_code};

            // Writes the message into the payload buffer. If writing fails, breaks the runtime with an error status.
            if (!_transport_layer.WriteData(message))
            {
                // Restores the empty transmission buffer that SendData() maintains, as this path returns before
                // reaching it.
                _transport_layer.ResetTransmissionBuffer();
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kPackingError);
                return false;
            }

            // Sends the buffered data to the PC. Transmission fails if the interface accepts only part of the packet.
            if (!_transport_layer.SendData())
            {
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kTransmissionError);
                return false;
            }

            _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kMessageSent);
            return true;
        }

        /**
         * @brief Sends the input event code to the PC as a Kernel-addressed state message.
         *
         * @param command The command the Kernel was executing when it sent the message.
         * @param event_code The event that triggered the message.
         *
         * @returns true if the message is sent, false otherwise.
         */
        bool SendStateMessage(const uint8_t command, const uint8_t event_code)
        {
            const KernelState message {static_cast<uint8_t>(kProtocols::kKernelState), command, event_code};

            // Writes the message into the payload buffer. If writing fails, breaks the runtime with an error status.
            if (!_transport_layer.WriteData(message))
            {
                // Restores the empty transmission buffer that SendData() maintains, as this path returns before
                // reaching it.
                _transport_layer.ResetTransmissionBuffer();
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kPackingError);
                return false;
            }

            // Sends the buffered data to the PC. Transmission fails if the interface accepts only part of the packet.
            if (!_transport_layer.SendData())
            {
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kTransmissionError);
                return false;
            }

            _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kMessageSent);
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
            const uint8_t errors[2] = {_communication_status, _transport_layer.get_runtime_status()};

            // Attempts sending the error message. Does not evaluate the status of sending the error message to avoid
            // recursions.
            SendDataMessage(module_type, module_id, command, error_code, errors);

            // As a fallback in case the error message does not reach the connected system, activates the built-in LED.
            // The LED is used as a visual indicator for a potentially unhandled runtime error. The Kernel class manages
            // the indicator inactivation.
            digitalWriteFast(LED_BUILTIN, HIGH);
        }

        /**
         * @brief Sends the communication error message to the PC as a Kernel-addressed message and activates the
         * built-in LED.
         *
         * @param command The command the Kernel was executing when it encountered the error.
         * @param error_code The encountered communication error.
         */
        void SendCommunicationErrorMessage(const uint8_t command, const uint8_t error_code)
        {
            // Combines the latest statuses of the Communication class and the TransportLayer class into a 2-byte array.
            // Jointly, this information should be enough to diagnose the error.
            const uint8_t errors[2] = {_communication_status, _transport_layer.get_runtime_status()};

            // Attempts sending the error message. Does not evaluate the status of sending the error message to avoid
            // recursions.
            SendDataMessage(command, error_code, errors);

            // As a fallback in case the error message does not reach the connected system, activates the built-in LED.
            // The LED is used as a visual indicator for a potentially unhandled runtime error. The Kernel class manages
            // the indicator inactivation.
            digitalWriteFast(LED_BUILTIN, HIGH);
        }

        /**
         * @brief Uses the specified service message protocol to send the input service code to the PC.
         *
         * @tparam kProtocol The protocol to use for the transmitted message. Has to be one of the following kProtocols
         * enumeration members: kReceptionCode, kControllerIdentification or kModuleIdentification.
         * @tparam ObjectType The data type of the service code value.
         * @param service_code The service code to be transmitted to the PC.
         *
         * @returns true if the message is sent, false otherwise.
         */
        template <const kProtocols kProtocol, typename ObjectType>
        bool SendServiceMessage(const ObjectType service_code)
        {
            // Ensures the communication protocol is one of the supported Service protocols.
            static_assert(
                kProtocol == kProtocols::kReceptionCode || kProtocol == kProtocols::kControllerIdentification ||
                    kProtocol == kProtocols::kModuleIdentification,
                "Encountered an invalid ServiceMessage protocol code. Use one of the supported Service protocols from "
                "the kProtocols enumeration."
            );

            // Ensures that the provided service_code is of the correct type.
            static_assert(
                axtlmc_shared_assets::is_same_v<ObjectType, uint8_t> ||
                    axtlmc_shared_assets::is_same_v<ObjectType, uint16_t> ||
                    axtlmc_shared_assets::is_same_v<ObjectType, uint32_t>,
                "Encountered an invalid ServiceMessage service code type. Currently, only uint8_t, uint16_t or "
                "uint32_t service codes are supported."
            );

            // Ensures the packed structure carries no padding, so it serializes to the protocol byte followed by the
            // service code bytes.
            static_assert(
                sizeof(ServiceMessage<ObjectType>) == sizeof(uint8_t) + sizeof(ObjectType),
                "The ServiceMessage structure must be packed to match the service message wire layout."
            );

            const ServiceMessage<ObjectType> message {static_cast<uint8_t>(kProtocol), service_code};

            // Writes the message into the payload buffer. If writing fails, breaks the runtime with an error status.
            if (!_transport_layer.WriteData(message))
            {
                // Restores the empty transmission buffer that SendData() maintains, as this path returns before
                // reaching it.
                _transport_layer.ResetTransmissionBuffer();
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kPackingError);
                return false;
            }

            // Sends the buffered data to the PC. Transmission fails if the interface accepts only part of the packet.
            if (!_transport_layer.SendData())
            {
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kTransmissionError);
                return false;
            }

            _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kMessageSent);
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
         * @returns true if a message was successfully received, false otherwise. If this method returns false, this
         * does not by itself indicate a runtime error. Use the get_communication_status() accessor to determine if
         * the method encountered a runtime error.
         */
        bool ReceiveMessage()
        {
            if (!_transport_layer.ReceiveData())
            {
                // The reception protocol can 'fail' gracefully if the reception buffer does not have enough bytes to
                // attempt message reception.
                if (_transport_layer.get_runtime_status() ==
                    static_cast<uint8_t>(kTransportStatusCodes::kNoBytesToParse))
                {
                    _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kNoBytesToReceive);
                    return false;
                }

                // Otherwise, returns with a status that indicates runtime failure.
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kReceptionError);
                return false;
            }

            // If the message is received and decoded, extracts the protocol code of the received message and uses
            // it to parse the rest of the message.
            if (_transport_layer.ReadData(_protocol_code))
            {
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kMessageReceived);
                // Unpacks the message header into the appropriate attribute structure.
                switch (static_cast<kProtocols>(_protocol_code))
                {
                    case kProtocols::kRepeatedModuleCommand:
                        if (_transport_layer.ReadData(_repeated_module_command)) return true;
                        break;

                    case kProtocols::kOneOffModuleCommand:
                        if (_transport_layer.ReadData(_one_off_module_command)) return true;
                        break;

                    case kProtocols::kDequeueModuleCommand:
                        if (_transport_layer.ReadData(_module_dequeue)) return true;
                        break;

                    case kProtocols::kKernelCommand:
                        if (_transport_layer.ReadData(_kernel_command)) return true;
                        break;

                    case kProtocols::kModuleParameters:
                        // Reads the HEADER of the message into the storage structure. This gives the Kernel class
                        // enough information to address the message, but this is NOT the whole message. To retrieve
                        // the parameter data bundled with the message, use the ExtractModuleParameters() method.
                        if (_transport_layer.ReadData(_module_parameters_header)) return true;
                        break;

                    default:
                        // If input protocol code is not one of the valid protocols, aborts with an error status.
                        _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kInvalidProtocol);
                        return false;
                }
            }

            // If any of the data reading calls above fail, returns with an error status.
            _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kParsingError);
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
         * @tparam kObjectSize The size of the destination object, in bytes.
         * @param destination The object where to unpack the received parameters. Typically, this is a packed structure.
         *
         * @returns true if the parameter data was successfully extracted into the destination object and false
         * otherwise.
         */
        template <typename ObjectType, const size_t kObjectSize = sizeof(ObjectType)>
        bool ExtractModuleParameters(ObjectType& destination)
        {
            // Ensures that the prototype complies with the limitations of the transport layer. The '-1' accounts for
            // the protocol code that precedes the ModuleParameters header inside the message payload.
            static_assert(
                kObjectSize > 0 && kObjectSize <= kMaximumPayloadSize - sizeof(ModuleParameters) - 1,
                "Unable to extract the target module's parameters as the method has received an invalid "
                "'destination' input. A valid destination object must fit inside the message payload buffer "
                "alongside the ModuleParameters header and the protocol code."
            );

            // Stores the number of payload bytes a valid parameters message for this destination carries. The '+1'
            // accounts for the protocol code that precedes the ModuleParameters header inside the message payload.
            static constexpr auto kExpectedPayloadSize =
                static_cast<uint8_t>(kObjectSize + sizeof(ModuleParameters) + 1);

            // Ensures this method cannot be called (successfully) unless the message currently stored in the reception
            // buffer is a ModuleParameters message.
            if (_protocol_code != static_cast<uint8_t>(kProtocols::kModuleParameters))
            {
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kExtractionForbidden);
                return false;
            }

            // Verifies that the number of payload bytes received with the message matches the size expected for the
            // destination object.
            if (_transport_layer.get_bytes_in_reception_buffer() != kExpectedPayloadSize)
            {
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kParameterMismatch);
                return false;
            }

            // If both checks above are passed, extracts the parameter data from the incoming message into the provided
            // structure (by reference).
            if (!_transport_layer.ReadData(destination))
            {
                _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kParsingError);
                return false;
            }

            _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kParametersExtracted);
            return true;
        }

    private:
        /// Stores the polynomial of the CRC-16 checksum algorithm that verifies the integrity of exchanged messages.
        /// The PC interface library has to use the same CRC parameters to communicate with this microcontroller.
        static constexpr uint16_t kCRCPolynomial = 0x1021;

        /// Stores the value to which the CRC-16 checksum is initialized before calculation.
        static constexpr uint16_t kCRCInitialValue = 0xFFFF;

        /// Stores the value with which the CRC-16 checksum is XORed after calculation.
        static constexpr uint16_t kCRCFinalXORValue = 0x0000;

        /// Stores the number of bytes each transmitted packet reserves for the framing metadata and the 16-bit CRC
        /// checksum postamble.
        static constexpr uint16_t kPacketMetadataSize = 6;

        /// Defines the maximum possible size for the received and transmitted payloads. Reuses the
        /// kSerialBufferSize constant defined inside transport_layer.h to determine the serial buffer size of the
        /// host microcontroller.
        static constexpr uint8_t kMaximumPayloadSize =
            static_cast<uint8_t>(min(kSerialBufferSize - kPacketMetadataSize, kBufferLayout::kMaximumPayloadSize));

        /// Stores the runtime status of the most recently called method.
        uint8_t _communication_status = static_cast<uint8_t>(kCommunicationStatusCodes::kStandby);

        /// Stores the protocol code of the last received message.
        uint8_t _protocol_code = static_cast<uint8_t>(kProtocols::kUndefined);

        /// Stores the last received Module-addressed recurrent (repeated) command message data.
        RepeatedModuleCommand _repeated_module_command;

        /// Stores the last received Module-addressed non-recurrent (one-off) command message data.
        OneOffModuleCommand _one_off_module_command;

        /// Stores the last received Kernel-addressed command message data.
        KernelCommand _kernel_command;

        /// Stores the last received Module-addressed dequeue command message data.
        DequeueModuleCommand _module_dequeue;

        /// Stores the last received Module-addressed parameters message header data.
        ModuleParameters _module_parameters_header;

        /// Manages the bidirectional communication with the PC.
        TransportLayer<uint16_t, kMaximumPayloadSize, kMaximumPayloadSize> _transport_layer;
};

#endif  // AXMC_COMMUNICATION_H
