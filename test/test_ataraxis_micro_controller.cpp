/**
 * @file
 *
 * @brief Verifies the behavior of the Communication, Module, and Kernel classes that make up the library.
 *
 * Covers the Communication class send, receive, and parameter extraction methods, the compile-time prototype
 * resolution, the Module command queue and execution state transitions, and the Kernel message routing, module setup,
 * and keepalive runtimes. All tests drive the library through a mock serial stream, so they require no PC connection.
 */

#include <Arduino.h>
#include <unity.h>  // This is the C testing framework, no connection to the Unity game engine
#include "axmc_shared_assets.h"
#include "cobs_processor.h"
#include "communication.h"
#include "crc_processor.h"
#include "kernel.h"
#include "module.h"
#include "stream_mock.h"

using namespace axmc_communication_assets;

/// The byte capacity of the mock serial stream used to back the Communication instance in each test.
static constexpr uint16_t kTestBufferSize = 60;

/// The byte capacity of the mock serial stream used to truncate outgoing packets. Any packet the Communication class
/// produces exceeds this capacity, which forces the stream to accept only a part of the packet.
static constexpr uint16_t kTruncatingBufferSize = 4;

/// The byte capacity of the mock serial stream used to back the Kernel tests. A single Kernel runtime cycle emits
/// several messages, all of which have to fit into the transmission buffer for the test to read them back.
static constexpr uint16_t kRuntimeBufferSize = 300;

/// The value the StreamMock class treats as the absence of data. Filling a buffer region with this value hides that
/// region from the reception and decoding logic.
static constexpr int16_t kInvalidStreamValue = -1;

/// The polynomial of the CRC-16 checksum algorithm the Communication class uses to verify message integrity.
static constexpr uint16_t kTestCRCPolynomial = 0x1021;

/// The value to which the CRC-16 checksum is initialized before calculation.
static constexpr uint16_t kTestCRCInitialValue = 0xFFFF;

/// The value with which the CRC-16 checksum is XORed after calculation.
static constexpr uint16_t kTestCRCFinalXORValue = 0x0000;

/// The number of bytes each packet reserves for the framing metadata and the CRC checksum postamble.
static constexpr size_t kPacketMetadataSize = 6;

/// The largest packet, in bytes, the tests decode out of the mock transmission buffer.
static constexpr size_t kMaximumTestPacketSize = 64;

/// The type (family) code shared by every MockModule instance the tests manage.
static constexpr uint8_t kMockModuleType = 77;

/// The identifier of the first MockModule instance the tests manage.
static constexpr uint8_t kFirstMockModuleId = 11;

/// The identifier of the second MockModule instance the tests manage.
static constexpr uint8_t kSecondMockModuleId = 22;

/// The identifier of the third MockModule instance the Kernel setup tests manage.
static constexpr uint8_t kThirdMockModuleId = 33;

/// The identifier of a module instance no test manages, used to exercise the target resolution failure runtime.
static constexpr uint8_t kAbsentMockModuleId = 44;

/// The identifier of the microcontroller the Kernel tests emulate.
static constexpr uint8_t kTestControllerId = 123;

/// A command code no MockModule instance recognizes, used to exercise the unrecognized command runtime.
static constexpr uint8_t kUnrecognizedCommand = 99;

/// A kernel command code the Kernel does not support, used to exercise the unrecognized kernel command runtime.
static constexpr uint8_t kUnrecognizedKernelCommand = 88;

/// The acknowledgement code the tests attach to the messages that verify reception code echoing.
static constexpr uint8_t kTestReceptionCode = 55;

/// The value passed as the 'noblock' argument to queue a command that runs in non-blocking mode.
static constexpr bool kNonBlockingCommand = true;

/// The value passed as the 'noblock' argument to queue a command that runs in blocking mode.
static constexpr bool kBlockingCommand = false;

/// The recurrent command cycle delay, in microseconds, that the recurrent activation tests wait out.
static constexpr uint32_t kRecurrentCycleDelay = 20000;

/// The time, in milliseconds, the tests wait to outlast kRecurrentCycleDelay and the non-blocking stage delay.
static constexpr uint32_t kDelayOvershootMilliseconds = 30;

/// The stage delay, in microseconds, the non-blocking WaitForMicros() test waits out.
static constexpr uint32_t kNonBlockingStageDelay = 20000;

/// The stage delay, in microseconds, the blocking WaitForMicros() test suspends the runtime for.
static constexpr uint32_t kBlockingStageDelay = 2000;

/// The keepalive interval, in milliseconds, that the keepalive timeout test allows to expire.
static constexpr uint32_t kKeepaliveTestInterval = 5;

/// The effective keepalive timeout, in milliseconds, the Kernel derives from kKeepaliveTestInterval by doubling it.
static constexpr uint32_t kKeepaliveTestTimeout = kKeepaliveTestInterval * 2;

/// The keepalive interval, in milliseconds, whose doubling overflows the 32-bit interval tracker. Doubling this value
/// without the saturation guard wraps it to 8 milliseconds, which makes the Kernel report a keepalive timeout within
/// the window kDelayOvershootMilliseconds covers.
static constexpr uint32_t kOverflowingKeepaliveInterval = (UINT32_MAX / 2) + 5;

/// The number of readouts the pin polling tests average into a single returned value.
static constexpr uint16_t kTestPoolSize = 10;

/// The number of individual analog readouts the pin polling test samples on each side of the pooled readout to
/// establish the range that pooled readout has to fall inside.
static constexpr uint16_t kAnalogProbeCount = 32;

/// The digital pin the pin polling tests drive and read back.
static constexpr uint8_t kTestDigitalPin = 2;

/// The analog pin the pin polling tests drive and read back.
static constexpr uint8_t kTestAnalogPin = A0;

/// Stores the runtime parameters a MockModule instance receives from the PC.
struct MockParameters
{
        uint8_t identifier = 0;  ///< Verifies the extraction of a single-byte parameter field.
        uint16_t payload   = 0;  ///< Verifies the extraction of a multi-byte parameter field.
} PACKED_STRUCT;

/// Stores the calls a MockModule instance records as the Kernel drives its runtime.
struct MockModuleRecord
{
        uint16_t setup_calls     = 0;  ///< The number of SetupModule() calls the instance has received.
        uint16_t parameter_calls = 0;  ///< The number of SetCustomParameters() calls the instance has received.
        uint16_t command_calls   = 0;  ///< The number of RunActiveCommand() calls the instance has received.
        uint8_t last_command     = 0;  ///< The command code active during the most recent RunActiveCommand() call.
        uint8_t last_stage       = 0;  ///< The execution stage active during the most recent RunActiveCommand() call.
};

/// Stores a single message decoded out of the mock transmission buffer.
struct DecodedMessage
{
        uint8_t payload[kMaximumTestPacketSize] = {};  ///< The decoded message payload.
        uint8_t size                            = 0;   ///< The number of payload bytes the message carries.
};

/**
 * @brief Records the calls the Kernel makes against the instance and exposes the command execution utilities of the
 * base Module class to the test functions.
 *
 * The instance recognizes the command codes the kMockCommands enumeration defines and reports every other code as
 * unrecognized, which drives the runtime that discards the commands no module is able to execute.
 *
 * @warning The instance's SetupModule() method reports the outcome each test configures. This is a testing aid that
 * reaches the runtime the Kernel enters after a module fails to set up. Every module that drives real hardware has to
 * be written so that its setup cannot fail.
 */
class MockModule final : public Module
{
    public:
        /// Defines the codes for the commands the instance recognizes.
        enum class kMockCommands : uint8_t
        {
            kComplete   = 1,  ///< Completes during its first execution stage.
            kMultiStage = 2,  ///< Reports each of its two execution stages before completing.
            kAbort      = 3,  ///< Aborts itself during its first execution stage.
            kIdle       = 4,  ///< Stays active without advancing its stage or completing.
        };

        /// Defines the codes the instance uses when communicating its runtime state to the PC.
        enum class kMockStatusCodes : uint8_t
        {
            kStageEntered = 51,  ///< The instance has entered a new stage of the kMultiStage command.
        };

        // Widens the access of the command execution utilities the base class reserves for its subclasses. This lets
        // the test functions drive and inspect the instance's command state directly instead of through the Kernel.
        using Module::AbortCommand;
        using Module::AdvanceCommandStage;
        using Module::AnalogRead;
        using Module::CompleteCommand;
        using Module::DigitalRead;
        using Module::get_active_command;
        using Module::get_command_stage;
        using Module::SendData;
        using Module::WaitForMicros;

        /// Initializes the base Module class with the provided type, id, and communication instance.
        MockModule(const uint8_t module_type, const uint8_t module_id, Communication& communication) :
            Module(module_type, module_id, communication)
        {}

        /// Overwrites the instance's runtime parameters with the data received from the PC.
        bool SetCustomParameters() override
        {
            _record.parameter_calls++;
            return ExtractParameters(_parameters);
        }

        /// Resolves and executes the currently active command.
        bool RunActiveCommand() override
        {
            _record.command_calls++;
            _record.last_command = get_active_command();
            _record.last_stage   = get_command_stage();

            switch (static_cast<kMockCommands>(get_active_command()))
            {
                case kMockCommands::kComplete: CompleteCommand(); return true;
                case kMockCommands::kMultiStage: RunMultiStage(); return true;
                case kMockCommands::kAbort: AbortCommand(); return true;
                case kMockCommands::kIdle: return true;
                default: return false;
            }
        }

        /// Restores the instance's parameter defaults and leaves the instance in the idle state.
        bool SetupModule() override
        {
            _record.setup_calls++;
            _parameters = {};
            return _setup_outcome;
        }

        /// Returns the calls the instance has recorded since the last recording reset.
        [[nodiscard]]
        const MockModuleRecord& get_record() const
        {
            return _record;
        }

        /// Returns the runtime parameters the instance most recently received from the PC.
        [[nodiscard]]
        const MockParameters& get_parameters() const
        {
            return _parameters;
        }

        /// Sets the outcome the instance's SetupModule() method reports.
        void set_setup_outcome(const bool outcome)
        {
            _setup_outcome = outcome;
        }

        /// Destroys the instance during cleanup.
        ~MockModule() override = default;

    private:
        /// Stores the instance's PC-addressable runtime parameters.
        MockParameters _parameters;

        /// Stores the calls the Kernel has made against the instance.
        MockModuleRecord _record;

        /// Determines whether the instance's SetupModule() method reports success.
        bool _setup_outcome = true;

        /// Reports each of the command's two execution stages to the PC before completing the command.
        void RunMultiStage()
        {
            switch (get_command_stage())
            {
                // Stage 1: Reports the stage and moves on to the second stage.
                case 1:
                    SendData(static_cast<uint8_t>(kMockStatusCodes::kStageEntered));
                    AdvanceCommandStage();
                    return;

                // Stage 2: Reports the stage and completes the command.
                case 2:
                    SendData(static_cast<uint8_t>(kMockStatusCodes::kStageEntered));
                    CompleteCommand();
                    return;

                default: AbortCommand(); return;
            }
        }
};

/**
 * @brief Frames the payload stored in the input buffer into a packet and writes that packet into the mock reception
 * buffer.
 *
 * The buffer has to follow the packet layout, which reserves the first three bytes for the framing metadata and the
 * last three bytes for the packet delimiter and the CRC checksum postamble.
 *
 * @tparam kBufferSize The size of the mock stream's buffers, in elements.
 * @tparam kPacketSize The size of the input buffer, in bytes.
 * @param port The mock stream whose reception buffer receives the packet.
 * @param packet The buffer that stores the payload to frame.
 */
template <const uint16_t kBufferSize, const size_t kPacketSize>
void InjectReceivedMessage(StreamMock<kBufferSize>& port, uint8_t (&packet)[kPacketSize])
{
    static_assert(
        kPacketSize <= kBufferSize,
        "The mock stream's reception buffer must be large enough to store the injected packet."
    );

    COBSProcessor::EncodePayload(packet);
    CRCProcessor<uint16_t> crc_processor(kTestCRCPolynomial, kTestCRCInitialValue, kTestCRCFinalXORValue);
    crc_processor.CalculateChecksum<false>(packet);

    // Masks every element past the packet, as the mock stream reports the elements holding valid byte values as the
    // data available for reception.
    for (size_t index = 0; index < kBufferSize; ++index)
    {
        port.rx_buffer[index] = index < kPacketSize ? static_cast<int16_t>(packet[index]) : kInvalidStreamValue;
    }

    port.rx_buffer_index = 0;
}

/**
 * @brief Discards the messages currently stored in the mock transmission buffer.
 *
 * @tparam kBufferSize The size of the mock stream's buffers, in elements.
 * @param port The mock stream whose transmission buffer is cleared.
 */
template <const uint16_t kBufferSize>
void ClearTransmittedMessages(StreamMock<kBufferSize>& port)
{
    for (size_t index = 0; index < kBufferSize; ++index)
    {
        port.tx_buffer[index] = kInvalidStreamValue;
    }

    port.tx_buffer_index = 0;
}

/**
 * @brief Decodes the message stored at the requested transmission buffer offset and advances the offset past that
 * message.
 *
 * @tparam kBufferSize The size of the mock stream's buffers, in elements.
 * @param port The mock stream whose transmission buffer stores the message.
 * @param offset The transmission buffer index at which the message starts.
 *
 * @returns The decoded message, or a message with the size of 0 if the buffer stores no valid message at the offset.
 */
template <const uint16_t kBufferSize>
DecodedMessage ReadTransmittedMessage(const StreamMock<kBufferSize>& port, size_t& offset)
{
    DecodedMessage message;

    if (offset + kPacketMetadataSize > kBufferSize) return message;
    if (port.tx_buffer[offset + kBufferLayout::kStartByteIndex] != kBufferLayout::kStartByte) return message;

    const auto payload_size = static_cast<size_t>(port.tx_buffer[offset + kBufferLayout::kPayloadSizeIndex]);
    if (payload_size == 0 || payload_size + kPacketMetadataSize > kMaximumTestPacketSize) return message;

    // Copies the packet out of the mock buffer, as decoding restores the delimiter bytes the encoding replaced and
    // therefore modifies the buffer it operates on.
    uint8_t packet[kMaximumTestPacketSize] = {};
    for (size_t index = 0; index < payload_size + kPacketMetadataSize; ++index)
    {
        packet[index] = static_cast<uint8_t>(port.tx_buffer[offset + index]);
    }

    if (static_cast<size_t>(COBSProcessor::DecodePayload(packet)) != payload_size) return message;

    for (size_t index = 0; index < payload_size; ++index)
    {
        message.payload[index] = packet[index + kBufferLayout::kPayloadStartIndex];
    }

    message.size = static_cast<uint8_t>(payload_size);
    offset += payload_size + kPacketMetadataSize;
    return message;
}

/**
 * @brief Verifies that the message stored at the requested transmission buffer offset carries the expected payload.
 *
 * @tparam kBufferSize The size of the mock stream's buffers, in elements.
 * @tparam kExpectedSize The size of the expected payload, in bytes.
 * @param port The mock stream whose transmission buffer stores the message.
 * @param offset The transmission buffer index at which the message starts.
 * @param expected_payload The payload the message is expected to carry.
 */
template <const uint16_t kBufferSize, const size_t kExpectedSize>
void AssertTransmittedMessage(
    const StreamMock<kBufferSize>& port,
    size_t& offset,
    const uint8_t (&expected_payload)[kExpectedSize]
)
{
    const DecodedMessage message = ReadTransmittedMessage(port, offset);
    TEST_ASSERT_EQUAL_UINT8(kExpectedSize, message.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_payload, message.payload, kExpectedSize);
}

/**
 * @brief Verifies that the mock transmission buffer stores no message past the requested offset.
 *
 * @tparam kBufferSize The size of the mock stream's buffers, in elements.
 * @param port The mock stream whose transmission buffer is evaluated.
 * @param offset The transmission buffer index past which no message is expected.
 */
template <const uint16_t kBufferSize>
void AssertNoTransmittedMessage(const StreamMock<kBufferSize>& port, const size_t offset)
{
    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(offset), static_cast<uint32_t>(port.tx_buffer_index));
}

/// Called automatically before each test function. Currently unused.
void setUp()
{}

/// Called automatically after each test function. Currently unused.
void tearDown()
{}

/// Verifies the Communication's SendDataMessage() method.
void test_send_data_message()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);

    // Defines static message payload components.
    constexpr uint8_t module_type = 112;  // Example module type
    constexpr uint8_t module_id   = 12;   // Example module ID
    constexpr uint8_t command     = 88;   // Example command code
    constexpr uint8_t event_code  = 221;  // Example event code
    constexpr uint8_t test_object = 255;  // Test object

    // The prototype byte-code is auto-resolved from the object type. For uint8_t, the expected code is kOneUint8.
    constexpr auto prototype_code = static_cast<uint8_t>(axmc_communication_assets::kPrototypes::kOneUint8);

    // Kernel test
    constexpr uint16_t kernel_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kKernelData);
    TEST_ASSERT_TRUE(communication_class.SendDataMessage(command, event_code, test_object));
    constexpr uint16_t expected_kernel[6] = {kernel_protocol, command, event_code, prototype_code, test_object, 0};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        communication_class.get_communication_status()
    );
    for (size_t i = 0; i < 6; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_kernel[i], mock_port.tx_buffer[i + 3]);
    }

    mock_port.Reset();

    // Module test
    constexpr uint16_t module_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleData);
    TEST_ASSERT_TRUE(communication_class.SendDataMessage(module_type, module_id, command, event_code, test_object));
    constexpr uint16_t expected_module[8] =
        {module_protocol, module_type, module_id, command, event_code, prototype_code, test_object, 0};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        communication_class.get_communication_status()
    );
    for (size_t i = 0; i < 8; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 3]);
    }
}

/// Verifies that the Communication's SendDataMessage() method resolves the prototype code of an array data object.
void test_send_data_message_array_object()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    size_t offset = 0;

    constexpr uint8_t module_type    = 112;
    constexpr uint8_t module_id      = 12;
    constexpr uint8_t command        = 88;
    constexpr uint8_t event_code     = 221;
    constexpr uint8_t test_object[3] = {7, 8, 9};

    TEST_ASSERT_TRUE(communication_class.SendDataMessage(module_type, module_id, command, event_code, test_object));

    const uint8_t expected_message[9] = {
        static_cast<uint8_t>(kProtocols::kModuleData),
        module_type,
        module_id,
        command,
        event_code,
        static_cast<uint8_t>(kPrototypes::kThreeUint8s),
        test_object[0],
        test_object[1],
        test_object[2],
    };
    AssertTransmittedMessage(mock_port, offset, expected_message);
    AssertNoTransmittedMessage(mock_port, offset);
}

/// Verifies the Communication's SendStateMessage() method.
void test_send_state_message()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);

    // Defines static message payload components.
    constexpr uint8_t module_type = 112;  // Example module type
    constexpr uint8_t module_id   = 12;   // Example module ID
    constexpr uint8_t command     = 88;   // Example command code
    constexpr uint8_t event_code  = 221;  // Example event code

    constexpr uint16_t kernel_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kKernelState);
    TEST_ASSERT_TRUE(communication_class.SendStateMessage(command, event_code));
    constexpr uint16_t expected_kernel[4] = {kernel_protocol, command, event_code, 0};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        communication_class.get_communication_status()
    );
    for (size_t i = 0; i < 4; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_kernel[i], mock_port.tx_buffer[i + 3]);
    }

    mock_port.Reset();

    // Module test
    constexpr uint16_t module_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleState);
    TEST_ASSERT_TRUE(communication_class.SendStateMessage(module_type, module_id, command, event_code));
    constexpr uint16_t expected_module[6] = {module_protocol, module_type, module_id, command, event_code, 0};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        communication_class.get_communication_status()
    );
    for (size_t i = 0; i < 6; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 3]);
    }
}

/// Verifies the Communication's SendCommunicationErrorMessage() method.
void test_send_communication_error_message()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);

    // Defines static payload components.
    constexpr uint8_t module_type = 1;
    constexpr uint8_t module_id   = 2;
    constexpr uint8_t command     = 3;
    constexpr uint8_t error_code  = 4;
    // Manually sets the Communication class status code.
    communication_class.set_communication_status(189);

    // Extracts the current Transport Layer status.
    const uint8_t transport_layer_status = communication_class.get_transport_layer_status();

    // Defines the prototype byte-code. Communication errors contain two uint8 values: Communication and
    // TransportLayer status codes.
    constexpr auto prototype_code = static_cast<uint8_t>(axmc_communication_assets::kPrototypes::kTwoUint8s);

    // Kernel error message
    // Error messages use the appropriate Data protocol code.
    constexpr uint16_t kernel_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kKernelData);
    communication_class.SendCommunicationErrorMessage(command, error_code);
    const uint16_t expected_kernel[7] = {kernel_protocol, 3, 4, prototype_code, 189, transport_layer_status, 0};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        communication_class.get_communication_status()
    );
    for (size_t i = 0; i < 7; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_kernel[i], mock_port.tx_buffer[i + 3]);
    }

    mock_port.Reset();
    communication_class.set_communication_status(65);  // Resets the communication class status.

    // Re-extracts the Transport Layer status.
    const uint8_t transport_layer_status_2 = communication_class.get_transport_layer_status();

    // Module error message
    constexpr uint16_t module_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleData);
    communication_class.SendCommunicationErrorMessage(module_type, module_id, command, error_code);
    const uint16_t expected_module[9] = {module_protocol, 1, 2, 3, 4, prototype_code, 65, transport_layer_status_2, 0};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        communication_class.get_communication_status()
    );
    for (size_t i = 0; i < 9; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 3]);
    }
}

/// Verifies the Communication's SendServiceMessage() method for all valid protocols.
void test_send_service_message()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);

    constexpr uint8_t service_code = 111;  // Uses the same test service code for all messages.

    // Reception message
    constexpr uint16_t expected_reception[2] = {
        static_cast<uint8_t>(axmc_communication_assets::kProtocols::kReceptionCode),
        service_code
    };
    TEST_ASSERT_TRUE(
        communication_class.SendServiceMessage<axmc_communication_assets::kProtocols::kReceptionCode>(service_code)
    );
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        communication_class.get_communication_status()
    );
    for (size_t i = 0; i < 2; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_reception[i], mock_port.tx_buffer[i + 3]);
    }

    mock_port.Reset();

    // ControllerIdentification message
    constexpr uint16_t expected_identification[2] = {
        static_cast<uint8_t>(axmc_communication_assets::kProtocols::kControllerIdentification),
        service_code
    };
    TEST_ASSERT_TRUE(
        communication_class.SendServiceMessage<axmc_communication_assets::kProtocols::kControllerIdentification>(
            service_code
        )
    );
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        communication_class.get_communication_status()
    );
    for (size_t i = 0; i < 2; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_identification[i], mock_port.tx_buffer[i + 3]);
    }

    mock_port.Reset();

    // ModuleIdentification message
    constexpr uint16_t module_type_id                    = 300;
    constexpr uint16_t expected_module_identification[3] = {
        static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleIdentification),
        44,
        1
    };
    TEST_ASSERT_TRUE(
        communication_class.SendServiceMessage<axmc_communication_assets::kProtocols::kModuleIdentification>(
            module_type_id
        )
    );
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        communication_class.get_communication_status()
    );
    for (size_t i = 0; i < 3; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_module_identification[i], mock_port.tx_buffer[i + 3]);
    }
}

/// Verifies that the Communication's SendServiceMessage() method serializes a 32-bit service code.
void test_send_service_message_wide_code()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    size_t offset = 0;

    // Uses a service code whose four bytes all differ, which detects both a truncated code and a reversed byte order.
    constexpr uint32_t service_code = 0x04030201;

    TEST_ASSERT_TRUE(communication_class.SendServiceMessage<kProtocols::kReceptionCode>(service_code));

    const uint8_t expected_message[5] = {static_cast<uint8_t>(kProtocols::kReceptionCode), 1, 2, 3, 4};
    AssertTransmittedMessage(mock_port, offset, expected_message);
    AssertNoTransmittedMessage(mock_port, offset);
}

/// Verifies that the Communication class reports a transmission error when the serial interface truncates the packet.
void test_send_message_transmission_error()
{
    StreamMock<kTruncatingBufferSize> mock_port;
    Communication communication_class(mock_port);

    // Defines static message payload components.
    constexpr uint8_t module_type = 112;  // Example module type
    constexpr uint8_t module_id   = 12;   // Example module ID
    constexpr uint8_t command     = 88;   // Example command code
    constexpr uint8_t event_code  = 221;  // Example event code
    constexpr uint8_t test_object = 255;  // Test object

    // State message test
    TEST_ASSERT_FALSE(communication_class.SendStateMessage(command, event_code));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kTransmissionError),
        communication_class.get_communication_status()
    );
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kTransportStatusCodes::kPacketPartiallySent),
        communication_class.get_transport_layer_status()
    );

    mock_port.Reset();

    // Data message test
    TEST_ASSERT_FALSE(communication_class.SendDataMessage(module_type, module_id, command, event_code, test_object));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kTransmissionError),
        communication_class.get_communication_status()
    );

    mock_port.Reset();

    // Service message test
    TEST_ASSERT_FALSE(
        communication_class.SendServiceMessage<axmc_communication_assets::kProtocols::kReceptionCode>(event_code)
    );
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kTransmissionError),
        communication_class.get_communication_status()
    );

    mock_port.Reset();

    // Module-addressed state message test
    TEST_ASSERT_FALSE(communication_class.SendStateMessage(module_type, module_id, command, event_code));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kTransmissionError),
        communication_class.get_communication_status()
    );

    mock_port.Reset();

    // Kernel-addressed data message test
    TEST_ASSERT_FALSE(communication_class.SendDataMessage(command, event_code, test_object));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kTransmissionError),
        communication_class.get_communication_status()
    );
}

/// Verifies the Communication's ReceiveMessage() method.
void test_receive_message()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor cobs_class;

    // Verifies the correct non-error no-success scenario, where the buffer does not contain any bytes to receive.
    communication_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kNoBytesToReceive),
        communication_class.get_communication_status()
    );

    mock_port.Reset();

    // Verifies RepeatedModuleCommand reception.
    uint8_t test_buffer_1[16] = {129, 10, 0, 1, 2, 3, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_1);
    crc_class.CalculateChecksum<false>(test_buffer_1);
    for (size_t i = 0; i < sizeof(test_buffer_1); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_1[i]);
    }

    // Receives and verifies the message data.
    communication_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageReceived),
        communication_class.get_communication_status()
    );
    TEST_ASSERT_EQUAL_UINT8(2, communication_class.get_repeated_module_command().module_type);
    TEST_ASSERT_EQUAL_UINT8(3, communication_class.get_repeated_module_command().module_id);
    TEST_ASSERT_EQUAL_UINT8(4, communication_class.get_repeated_module_command().return_code);
    TEST_ASSERT_EQUAL_UINT8(5, communication_class.get_repeated_module_command().command);
    TEST_ASSERT_FALSE(communication_class.get_repeated_module_command().noblock);
    TEST_ASSERT_EQUAL_UINT32(0, communication_class.get_repeated_module_command().cycle_delay);

    mock_port.Reset();

    // Verifies OneOffModuleCommand reception.
    uint8_t test_buffer_2[12] = {129, 6, 0, 2, 0, 1, 2, 3, 1, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_2);
    crc_class.CalculateChecksum<false>(test_buffer_2);
    for (size_t i = 0; i < sizeof(test_buffer_2); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_2[i]);
    }

    // Receives and verifies the message data.
    communication_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageReceived),
        communication_class.get_communication_status()
    );
    TEST_ASSERT_EQUAL_UINT8(0, communication_class.get_one_off_module_command().module_type);
    TEST_ASSERT_EQUAL_UINT8(1, communication_class.get_one_off_module_command().module_id);
    TEST_ASSERT_EQUAL_UINT8(2, communication_class.get_one_off_module_command().return_code);
    TEST_ASSERT_EQUAL_UINT8(3, communication_class.get_one_off_module_command().command);
    TEST_ASSERT_TRUE(communication_class.get_one_off_module_command().noblock);

    mock_port.Reset();

    // Verifies DequeueModuleCommand reception.
    uint8_t test_buffer_3[10] = {129, 4, 0, 3, 1, 2, 3, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_3);
    crc_class.CalculateChecksum<false>(test_buffer_3);
    for (size_t i = 0; i < sizeof(test_buffer_3); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_3[i]);
    }

    // Receives and verifies the message data.
    communication_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageReceived),
        communication_class.get_communication_status()
    );
    TEST_ASSERT_EQUAL_UINT8(1, communication_class.get_module_dequeue().module_type);
    TEST_ASSERT_EQUAL_UINT8(2, communication_class.get_module_dequeue().module_id);
    TEST_ASSERT_EQUAL_UINT8(3, communication_class.get_module_dequeue().return_code);

    mock_port.Reset();

    // Verifies KernelCommand reception.
    uint8_t test_buffer_4[9] = {129, 3, 0, 4, 1, 2, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_4);
    crc_class.CalculateChecksum<false>(test_buffer_4);
    for (size_t i = 0; i < sizeof(test_buffer_4); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_4[i]);
    }

    // Receives and verifies the message data.
    communication_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageReceived),
        communication_class.get_communication_status()
    );
    TEST_ASSERT_EQUAL_UINT8(1, communication_class.get_kernel_command().return_code);
    TEST_ASSERT_EQUAL_UINT8(2, communication_class.get_kernel_command().command);

    mock_port.Reset();

    // Verifies ModuleParameters reception. Note, this only verifies the header of the message. Parameter extraction is
    // verified by a different test function.
    uint8_t test_buffer_5[11] = {129, 4, 0, 5, 1, 2, 3, 4, 0, 0, 0};  // 4 simulates parameter object data

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_5);
    crc_class.CalculateChecksum<false>(test_buffer_5);
    for (size_t i = 0; i < sizeof(test_buffer_5); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_5[i]);
    }

    // Receives and verifies the message data.
    communication_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageReceived),
        communication_class.get_communication_status()
    );
    TEST_ASSERT_EQUAL_UINT8(1, communication_class.get_module_parameters_header().module_type);
    TEST_ASSERT_EQUAL_UINT8(2, communication_class.get_module_parameters_header().module_id);
    TEST_ASSERT_EQUAL_UINT8(3, communication_class.get_module_parameters_header().return_code);

    mock_port.Reset();
}

/// Verifies the error-handling behavior of the Communication's ReceiveMessage() method.
void test_receive_message_errors()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor cobs_class;

    // Verifies that failing one of the reception steps, such as COBS decoding or CRC verification, correctly raises
    // kReceptionError.
    const uint8_t test_buffer_1[16] = {129, 10, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Skips COBS and CRC to produce an invalid packet. Writes the invalid packet into the mock reception buffer.
    for (size_t i = 0; i < sizeof(test_buffer_1); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_1[i]);
    }

    // Triggers and verifies the error.
    communication_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kReceptionError),
        communication_class.get_communication_status()
    );

    mock_port.Reset();

    // Verifies that receiving a message with an invalid protocol code correctly raises kInvalidProtocol. Note,
    // protocols used by the outgoing messages (such as KernelData) are also considered invalid.
    constexpr auto invalid_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kKernelData);
    uint8_t test_buffer_2[16]       = {129, 10, 0, invalid_protocol, 2, 3, 4, 5, 1, 2, 3, 4, 5, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_2);
    crc_class.CalculateChecksum<false>(test_buffer_2);
    for (size_t i = 0; i < sizeof(test_buffer_2); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_2[i]);
    }

    // Triggers and verifies the error.
    communication_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kInvalidProtocol),
        communication_class.get_communication_status()
    );

    mock_port.Reset();

    // Verifies that receiving an incomplete message (message that deviates from its mandated layout) correctly raises
    // kParsingError.
    uint8_t test_buffer_3[16] = {129, 9, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_3);
    crc_class.CalculateChecksum<false>(test_buffer_3);
    for (size_t i = 0; i < sizeof(test_buffer_3); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_3[i]);
    }

    // Triggers and verifies the error.
    communication_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kParsingError),
        communication_class.get_communication_status()
    );
}

/// Verifies the Communication's ExtractModuleParameters() method.
void test_extract_module_parameters()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor cobs_class;

    // Verifies that extracting parameters into an array works as expected
    uint8_t test_buffer_1[16] = {129, 10, 0, 5, 2, 3, 4, 5, 1, 2, 3, 4, 5, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_1);
    crc_class.CalculateChecksum<false>(test_buffer_1);
    for (size_t i = 0; i < sizeof(test_buffer_1); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_1[i]);
    }

    // Defines the test array which serves as a parameter extraction target.
    uint8_t extract_data[6] = {};

    // Receives the message, extracts and verifies parameter data.
    communication_class.ReceiveMessage();
    TEST_ASSERT_TRUE(communication_class.ExtractModuleParameters(extract_data));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kParametersExtracted),
        communication_class.get_communication_status()
    );
    const uint8_t expected_data[6] = {
        5,
        1,
        2,
        3,
        4,
        5,
    };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_data, extract_data, sizeof(expected_data));

    mock_port.Reset();

    // Verifies that extracting parameter data into a structure works as expected
    uint8_t test_buffer_2[16] = {129, 10, 0, 5, 2, 3, 4, 9, 1, 2, 3, 4, 5, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_2);
    crc_class.CalculateChecksum<false>(test_buffer_2);
    for (size_t i = 0; i < sizeof(test_buffer_2); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_2[i]);
    }

    // Defines the test structure which serves as a parameter extraction target.
    struct TestStructure
    {
            uint8_t id      = 1;
            uint8_t data[5] = {};
    } PACKED_STRUCT test_structure;  // Has to be packed to properly align the data

    // Calls ExtractModuleParameters(), expecting a successful extraction.
    communication_class.ReceiveMessage();
    TEST_ASSERT_TRUE(communication_class.ExtractModuleParameters(test_structure));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kParametersExtracted),
        communication_class.get_communication_status()
    );
    TEST_ASSERT_EQUAL_UINT8(9, test_structure.id);
    const uint8_t expected_data_2[5] = {
        1,
        2,
        3,
        4,
        5,
    };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_data_2, test_structure.data, sizeof(expected_data_2));
}

/// Verifies the error-handling behavior of the Communication's ExtractModuleParameters() method.
void test_extract_module_parameters_errors()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor cobs_class;

    // Verifies that calling ExtractParameters() after receiving a message with a protocol code other than
    // kModuleParameters raises the ExtractionForbidden error.
    constexpr auto protocol_code = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kUndefined);
    uint8_t test_buffer_1[16]    = {129, 10, 0, protocol_code, 2, 3, 4, 5, 1, 2, 3, 4, 5, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_1);
    crc_class.CalculateChecksum<false>(test_buffer_1);
    for (size_t i = 0; i < sizeof(test_buffer_1); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_1[i]);
    }

    // Triggers and verifies the error.
    communication_class.ReceiveMessage();
    uint8_t extract_into[6] = {};
    communication_class.ExtractModuleParameters(extract_into);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kExtractionForbidden),
        communication_class.get_communication_status()
    );

    mock_port.Reset();

    // Verifies that calling ExtractParameters() with a prototype whose size does not match the size of the parameters
    // block inside the serial buffer raises a kParameterMismatch error.
    communication_class.set_protocol_code(5);  // Manually sets the protocol code to kModuleParameters

    // The prototype is larger than the stored data size
    uint8_t invalid_prototype_2[12] = {};
    TEST_ASSERT_FALSE(communication_class.ExtractModuleParameters(invalid_prototype_2));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kParameterMismatch),
        communication_class.get_communication_status()
    );
}

/// Verifies compile-time prototype resolution for representative type/count combinations.
void test_resolve_prototype()
{
    // Scalar types
    static_assert(
        ResolvePrototype<bool>() == kPrototypes::kOneBool,
        "ResolvePrototype must map bool to kPrototypes::kOneBool."
    );
    static_assert(
        ResolvePrototype<uint8_t>() == kPrototypes::kOneUint8,
        "ResolvePrototype must map uint8_t to kPrototypes::kOneUint8."
    );
    static_assert(
        ResolvePrototype<int8_t>() == kPrototypes::kOneInt8,
        "ResolvePrototype must map int8_t to kPrototypes::kOneInt8."
    );
    static_assert(
        ResolvePrototype<uint16_t>() == kPrototypes::kOneUint16,
        "ResolvePrototype must map uint16_t to kPrototypes::kOneUint16."
    );
    static_assert(
        ResolvePrototype<int16_t>() == kPrototypes::kOneInt16,
        "ResolvePrototype must map int16_t to kPrototypes::kOneInt16."
    );
    static_assert(
        ResolvePrototype<uint32_t>() == kPrototypes::kOneUint32,
        "ResolvePrototype must map uint32_t to kPrototypes::kOneUint32."
    );
    static_assert(
        ResolvePrototype<int32_t>() == kPrototypes::kOneInt32,
        "ResolvePrototype must map int32_t to kPrototypes::kOneInt32."
    );
    static_assert(
        ResolvePrototype<float>() == kPrototypes::kOneFloat32,
        "ResolvePrototype must map float to kPrototypes::kOneFloat32."
    );
    static_assert(
        ResolvePrototype<uint64_t>() == kPrototypes::kOneUint64,
        "ResolvePrototype must map uint64_t to kPrototypes::kOneUint64."
    );
    static_assert(
        ResolvePrototype<int64_t>() == kPrototypes::kOneInt64,
        "ResolvePrototype must map int64_t to kPrototypes::kOneInt64."
    );
    static_assert(
        ResolvePrototype<double>() == kPrototypes::kOneFloat64,
        "ResolvePrototype must map double to kPrototypes::kOneFloat64."
    );

    // Array types (representative samples)
    static_assert(
        ResolvePrototype<uint8_t[2]>() == kPrototypes::kTwoUint8s,
        "ResolvePrototype must map uint8_t[2] to kPrototypes::kTwoUint8s."
    );
    static_assert(
        ResolvePrototype<uint16_t[3]>() == kPrototypes::kThreeUint16s,
        "ResolvePrototype must map uint16_t[3] to kPrototypes::kThreeUint16s."
    );
    static_assert(
        ResolvePrototype<float[4]>() == kPrototypes::kFourFloat32s,
        "ResolvePrototype must map float[4] to kPrototypes::kFourFloat32s."
    );
    static_assert(
        ResolvePrototype<double[15]>() == kPrototypes::kFifteenFloat64s,
        "ResolvePrototype must map double[15] to kPrototypes::kFifteenFloat64s."
    );
    static_assert(
        ResolvePrototype<bool[8]>() == kPrototypes::kEightBools,
        "ResolvePrototype must map bool[8] to kPrototypes::kEightBools."
    );

    // Extended prototypes: platform cap counts
    static_assert(
        ResolvePrototype<bool[52]>() == kPrototypes::kFiftyTwoBools,
        "ResolvePrototype must map bool[52] to kPrototypes::kFiftyTwoBools."
    );
    static_assert(
        ResolvePrototype<bool[248]>() == kPrototypes::kTwoHundredFortyEightBools,
        "ResolvePrototype must map bool[248] to kPrototypes::kTwoHundredFortyEightBools."
    );
    static_assert(
        ResolvePrototype<uint8_t[52]>() == kPrototypes::kFiftyTwoUint8s,
        "ResolvePrototype must map uint8_t[52] to kPrototypes::kFiftyTwoUint8s."
    );
    static_assert(
        ResolvePrototype<uint8_t[128]>() == kPrototypes::kOneHundredTwentyEightUint8s,
        "ResolvePrototype must map uint8_t[128] to kPrototypes::kOneHundredTwentyEightUint8s."
    );
    static_assert(
        ResolvePrototype<uint8_t[248]>() == kPrototypes::kTwoHundredFortyEightUint8s,
        "ResolvePrototype must map uint8_t[248] to kPrototypes::kTwoHundredFortyEightUint8s."
    );
    static_assert(
        ResolvePrototype<int8_t[52]>() == kPrototypes::kFiftyTwoInt8s,
        "ResolvePrototype must map int8_t[52] to kPrototypes::kFiftyTwoInt8s."
    );
    static_assert(
        ResolvePrototype<int8_t[248]>() == kPrototypes::kTwoHundredFortyEightInt8s,
        "ResolvePrototype must map int8_t[248] to kPrototypes::kTwoHundredFortyEightInt8s."
    );
    static_assert(
        ResolvePrototype<uint16_t[26]>() == kPrototypes::kTwentySixUint16s,
        "ResolvePrototype must map uint16_t[26] to kPrototypes::kTwentySixUint16s."
    );
    static_assert(
        ResolvePrototype<uint16_t[124]>() == kPrototypes::kOneHundredTwentyFourUint16s,
        "ResolvePrototype must map uint16_t[124] to kPrototypes::kOneHundredTwentyFourUint16s."
    );
    static_assert(
        ResolvePrototype<uint32_t[62]>() == kPrototypes::kSixtyTwoUint32s,
        "ResolvePrototype must map uint32_t[62] to kPrototypes::kSixtyTwoUint32s."
    );
    static_assert(
        ResolvePrototype<float[62]>() == kPrototypes::kSixtyTwoFloat32s,
        "ResolvePrototype must map float[62] to kPrototypes::kSixtyTwoFloat32s."
    );
    static_assert(
        ResolvePrototype<uint64_t[31]>() == kPrototypes::kThirtyOneUint64s,
        "ResolvePrototype must map uint64_t[31] to kPrototypes::kThirtyOneUint64s."
    );
    static_assert(
        ResolvePrototype<double[31]>() == kPrototypes::kThirtyOneFloat64s,
        "ResolvePrototype must map double[31] to kPrototypes::kThirtyOneFloat64s."
    );

    // Extended prototypes: intermediate counts
    static_assert(
        ResolvePrototype<uint8_t[36]>() == kPrototypes::kThirtySixUint8s,
        "ResolvePrototype must map uint8_t[36] to kPrototypes::kThirtySixUint8s."
    );
    static_assert(
        ResolvePrototype<int8_t[132]>() == kPrototypes::kOneHundredThirtyTwoInt8s,
        "ResolvePrototype must map int8_t[132] to kPrototypes::kOneHundredThirtyTwoInt8s."
    );
    static_assert(
        ResolvePrototype<int16_t[48]>() == kPrototypes::kFortyEightInt16s,
        "ResolvePrototype must map int16_t[48] to kPrototypes::kFortyEightInt16s."
    );
    static_assert(
        ResolvePrototype<int32_t[48]>() == kPrototypes::kFortyEightInt32s,
        "ResolvePrototype must map int32_t[48] to kPrototypes::kFortyEightInt32s."
    );
    static_assert(
        ResolvePrototype<int64_t[24]>() == kPrototypes::kTwentyFourInt64s,
        "ResolvePrototype must map int64_t[24] to kPrototypes::kTwentyFourInt64s."
    );

    // If all static_asserts pass, the test trivially succeeds at runtime.
    TEST_ASSERT_TRUE(true);
}

/// Verifies the Module's identifier accessors.
void test_module_identifiers()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    TEST_ASSERT_EQUAL_UINT8(kMockModuleType, module_class.get_module_type());
    TEST_ASSERT_EQUAL_UINT8(kFirstMockModuleId, module_class.get_module_id());

    // The combined code packs the type into the upper byte and the ID into the lower byte.
    constexpr uint16_t expected_type_id = (static_cast<uint16_t>(kMockModuleType) << 8U) | kFirstMockModuleId;
    TEST_ASSERT_EQUAL_UINT16(expected_type_id, module_class.get_module_type_id());
}

/// Verifies that the Module's ResolveActiveCommand() method reports the absence of commands to execute.
void test_resolve_active_command_without_queued_command()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    TEST_ASSERT_FALSE(module_class.ResolveActiveCommand());
    TEST_ASSERT_EQUAL_UINT8(0, module_class.get_active_command());
    TEST_ASSERT_EQUAL_UINT8(0, module_class.get_command_stage());
    AssertNoTransmittedMessage(mock_port, 0);
}

/// Verifies that the Module's ResolveActiveCommand() method activates a newly queued command.
void test_resolve_active_command_activates_queued_command()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    constexpr auto command = static_cast<uint8_t>(MockModule::kMockCommands::kIdle);
    module_class.QueueCommand(command, kBlockingCommand);

    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    TEST_ASSERT_EQUAL_UINT8(command, module_class.get_active_command());
    TEST_ASSERT_EQUAL_UINT8(1, module_class.get_command_stage());

    // The command that is already active is kept active, and its stage is preserved across resolution calls.
    module_class.AdvanceCommandStage();
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    TEST_ASSERT_EQUAL_UINT8(2, module_class.get_command_stage());
}

/// Verifies that the Module's ResolveActiveCommand() method finishes the active command before the queued one.
void test_resolve_active_command_prefers_active_command()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    constexpr auto active_command = static_cast<uint8_t>(MockModule::kMockCommands::kIdle);
    constexpr auto queued_command = static_cast<uint8_t>(MockModule::kMockCommands::kComplete);

    module_class.QueueCommand(active_command, kBlockingCommand);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());

    // Queueing a command while another one runs does not interrupt the running command.
    module_class.QueueCommand(queued_command, kBlockingCommand);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    TEST_ASSERT_EQUAL_UINT8(active_command, module_class.get_active_command());

    // The queued command is activated only once the running command releases the module.
    module_class.CompleteCommand();
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    TEST_ASSERT_EQUAL_UINT8(queued_command, module_class.get_active_command());
}

/// Verifies that the Module's ResolveActiveCommand() method repeats a recurrent command once its delay expires.
void test_resolve_active_command_repeats_recurrent_command()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    constexpr auto command = static_cast<uint8_t>(MockModule::kMockCommands::kIdle);
    module_class.QueueCommand(command, kBlockingCommand, kRecurrentCycleDelay);

    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    module_class.CompleteCommand();
    TEST_ASSERT_EQUAL_UINT8(0, module_class.get_active_command());

    // The command stays inactive until the requested number of microseconds passes.
    TEST_ASSERT_FALSE(module_class.ResolveActiveCommand());

    delay(kDelayOvershootMilliseconds);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    TEST_ASSERT_EQUAL_UINT8(command, module_class.get_active_command());
    TEST_ASSERT_EQUAL_UINT8(1, module_class.get_command_stage());
}

/// Verifies that the Module's ResetCommandQueue() method stops a recurrent command from repeating.
void test_reset_command_queue_stops_recurrent_command()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    constexpr auto command = static_cast<uint8_t>(MockModule::kMockCommands::kIdle);
    module_class.QueueCommand(command, kBlockingCommand, kRecurrentCycleDelay);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    module_class.CompleteCommand();

    ClearTransmittedMessages(mock_port);
    module_class.ResetCommandQueue();

    // Clearing the queue while the recurrent command waits between repetitions reports the command as completed.
    size_t offset                        = 0;
    const uint8_t expected_completion[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kFirstMockModuleId,
        command,
        static_cast<uint8_t>(Module::kCoreStatusCodes::kCommandCompleted),
    };
    AssertTransmittedMessage(mock_port, offset, expected_completion);
    AssertNoTransmittedMessage(mock_port, offset);

    delay(kDelayOvershootMilliseconds);
    TEST_ASSERT_FALSE(module_class.ResolveActiveCommand());
}

/// Verifies that the Module's ResetCommandQueue() method leaves the running command undisturbed.
void test_reset_command_queue_preserves_active_command()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    constexpr auto command = static_cast<uint8_t>(MockModule::kMockCommands::kIdle);
    module_class.QueueCommand(command, kBlockingCommand, kRecurrentCycleDelay);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());

    ClearTransmittedMessages(mock_port);
    module_class.ResetCommandQueue();

    // The running command reports its own completion, so clearing the queue sends nothing while it runs.
    AssertNoTransmittedMessage(mock_port, 0);
    TEST_ASSERT_EQUAL_UINT8(command, module_class.get_active_command());

    module_class.CompleteCommand();
    delay(kDelayOvershootMilliseconds);
    TEST_ASSERT_FALSE(module_class.ResolveActiveCommand());
}

/// Verifies that the Module's QueueCommand() method reports the recurrent command it replaces as completed.
void test_queue_command_retires_replaced_recurrent_command()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    constexpr auto recurrent_command = static_cast<uint8_t>(MockModule::kMockCommands::kIdle);
    constexpr auto new_command       = static_cast<uint8_t>(MockModule::kMockCommands::kComplete);

    module_class.QueueCommand(recurrent_command, kBlockingCommand, kRecurrentCycleDelay);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    module_class.CompleteCommand();

    ClearTransmittedMessages(mock_port);
    module_class.QueueCommand(new_command, kBlockingCommand);

    // The completion message is attributed to the replaced command, which is idle between its repetitions.
    size_t offset                        = 0;
    const uint8_t expected_completion[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kFirstMockModuleId,
        recurrent_command,
        static_cast<uint8_t>(Module::kCoreStatusCodes::kCommandCompleted),
    };
    AssertTransmittedMessage(mock_port, offset, expected_completion);
    AssertNoTransmittedMessage(mock_port, offset);

    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    TEST_ASSERT_EQUAL_UINT8(new_command, module_class.get_active_command());
}

/// Verifies that the Module's CompleteCommand() method reports one-off commands and silences recurrent ones.
void test_complete_command_completion_reporting()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    constexpr auto one_off_command = static_cast<uint8_t>(MockModule::kMockCommands::kComplete);
    module_class.QueueCommand(one_off_command, kBlockingCommand);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());

    ClearTransmittedMessages(mock_port);
    module_class.CompleteCommand();

    size_t offset                        = 0;
    const uint8_t expected_completion[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kFirstMockModuleId,
        one_off_command,
        static_cast<uint8_t>(Module::kCoreStatusCodes::kCommandCompleted),
    };
    AssertTransmittedMessage(mock_port, offset, expected_completion);
    AssertNoTransmittedMessage(mock_port, offset);

    // Completing a one-off command also clears the queue, leaving the module without a command to execute.
    TEST_ASSERT_EQUAL_UINT8(0, module_class.get_active_command());
    TEST_ASSERT_FALSE(module_class.ResolveActiveCommand());

    // A recurrent command reports its completion only when it is canceled or replaced, so each of its repetitions
    // ends silently.
    constexpr auto recurrent_command = static_cast<uint8_t>(MockModule::kMockCommands::kIdle);
    module_class.QueueCommand(recurrent_command, kBlockingCommand, kRecurrentCycleDelay);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());

    ClearTransmittedMessages(mock_port);
    module_class.CompleteCommand();
    AssertNoTransmittedMessage(mock_port, 0);
}

/// Verifies that the Module's AbortCommand() method clears the queue and reports the command as completed.
void test_abort_command_clears_command_queue()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    constexpr auto command = static_cast<uint8_t>(MockModule::kMockCommands::kIdle);
    module_class.QueueCommand(command, kBlockingCommand, kRecurrentCycleDelay);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());

    ClearTransmittedMessages(mock_port);
    module_class.AbortCommand();

    size_t offset                        = 0;
    const uint8_t expected_completion[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kFirstMockModuleId,
        command,
        static_cast<uint8_t>(Module::kCoreStatusCodes::kCommandCompleted),
    };
    AssertTransmittedMessage(mock_port, offset, expected_completion);
    AssertNoTransmittedMessage(mock_port, offset);

    TEST_ASSERT_EQUAL_UINT8(0, module_class.get_active_command());
    delay(kDelayOvershootMilliseconds);
    TEST_ASSERT_FALSE(module_class.ResolveActiveCommand());
}

/// Verifies that the Module's AbortCommand() method keeps a newly queued command in the queue.
void test_abort_command_preserves_queued_command()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    constexpr auto aborted_command = static_cast<uint8_t>(MockModule::kMockCommands::kIdle);
    constexpr auto queued_command  = static_cast<uint8_t>(MockModule::kMockCommands::kComplete);

    module_class.QueueCommand(aborted_command, kBlockingCommand, kRecurrentCycleDelay);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    module_class.QueueCommand(queued_command, kBlockingCommand);

    ClearTransmittedMessages(mock_port);
    module_class.AbortCommand();

    size_t offset                        = 0;
    const uint8_t expected_completion[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kFirstMockModuleId,
        aborted_command,
        static_cast<uint8_t>(Module::kCoreStatusCodes::kCommandCompleted),
    };
    AssertTransmittedMessage(mock_port, offset, expected_completion);
    AssertNoTransmittedMessage(mock_port, offset);

    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    TEST_ASSERT_EQUAL_UINT8(queued_command, module_class.get_active_command());
}

/// Verifies that the Module's DiscardActiveCommand() method removes a recurrent command from the queue.
void test_discard_active_command_clears_recurrent_queue()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    // Queues the command with no cycle delay, which makes it eligible for reactivation on every resolution call.
    module_class.QueueCommand(kUnrecognizedCommand, kBlockingCommand, 0);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());

    ClearTransmittedMessages(mock_port);
    module_class.DiscardActiveCommand();

    // The caller reports the outcome of the discarded command, so the discard itself sends no completion message.
    AssertNoTransmittedMessage(mock_port, 0);
    TEST_ASSERT_EQUAL_UINT8(0, module_class.get_active_command());
    TEST_ASSERT_EQUAL_UINT8(0, module_class.get_command_stage());

    // Removing the command from the queue keeps it from reactivating on any later runtime cycle.
    TEST_ASSERT_FALSE(module_class.ResolveActiveCommand());
    delay(kDelayOvershootMilliseconds);
    TEST_ASSERT_FALSE(module_class.ResolveActiveCommand());
}

/// Verifies that the Module's DiscardActiveCommand() method keeps a newly queued command in the queue.
void test_discard_active_command_preserves_queued_command()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    constexpr auto queued_command = static_cast<uint8_t>(MockModule::kMockCommands::kComplete);

    module_class.QueueCommand(kUnrecognizedCommand, kBlockingCommand, 0);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    module_class.QueueCommand(queued_command, kBlockingCommand);

    ClearTransmittedMessages(mock_port);
    module_class.DiscardActiveCommand();

    AssertNoTransmittedMessage(mock_port, 0);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    TEST_ASSERT_EQUAL_UINT8(queued_command, module_class.get_active_command());
}

/// Verifies the Module's SendCommandActivationError() and SendCommandRejection() methods.
void test_module_command_error_reporting()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    constexpr auto active_command = static_cast<uint8_t>(MockModule::kMockCommands::kIdle);
    module_class.QueueCommand(active_command, kBlockingCommand);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());

    ClearTransmittedMessages(mock_port);
    module_class.SendCommandActivationError();

    // The activation error is attributed to the command the instance is executing.
    size_t offset                        = 0;
    const uint8_t expected_activation[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kFirstMockModuleId,
        active_command,
        static_cast<uint8_t>(Module::kCoreStatusCodes::kCommandNotRecognized),
    };
    AssertTransmittedMessage(mock_port, offset, expected_activation);

    module_class.SendCommandRejection(kUnrecognizedCommand);

    // The rejection is attributed to the rejected command and leaves the running command undisturbed.
    const uint8_t expected_rejection[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kFirstMockModuleId,
        kUnrecognizedCommand,
        static_cast<uint8_t>(Module::kCoreStatusCodes::kCommandNotRecognized),
    };
    AssertTransmittedMessage(mock_port, offset, expected_rejection);
    AssertNoTransmittedMessage(mock_port, offset);
    TEST_ASSERT_EQUAL_UINT8(active_command, module_class.get_active_command());
}

/// Verifies the Module's ResetExecutionParameters() method.
void test_reset_execution_parameters()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    constexpr auto command = static_cast<uint8_t>(MockModule::kMockCommands::kIdle);
    module_class.QueueCommand(command, kBlockingCommand, kRecurrentCycleDelay);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    module_class.AdvanceCommandStage();

    ClearTransmittedMessages(mock_port);
    module_class.ResetExecutionParameters();

    // The reset aborts the running command without reporting it, as it runs before the communication interface is
    // ready to carry the report.
    AssertNoTransmittedMessage(mock_port, 0);
    TEST_ASSERT_EQUAL_UINT8(0, module_class.get_active_command());
    TEST_ASSERT_EQUAL_UINT8(0, module_class.get_command_stage());
    TEST_ASSERT_FALSE(module_class.ResolveActiveCommand());
}

/// Verifies the Module's AdvanceCommandStage() and get_command_stage() methods.
void test_advance_command_stage()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    module_class.QueueCommand(static_cast<uint8_t>(MockModule::kMockCommands::kIdle), kBlockingCommand);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    TEST_ASSERT_EQUAL_UINT8(1, module_class.get_command_stage());

    module_class.AdvanceCommandStage();
    TEST_ASSERT_EQUAL_UINT8(2, module_class.get_command_stage());

    module_class.AdvanceCommandStage();
    TEST_ASSERT_EQUAL_UINT8(3, module_class.get_command_stage());

    // The stage accessor reports 0 for a module that has no command to execute.
    module_class.CompleteCommand();
    TEST_ASSERT_EQUAL_UINT8(0, module_class.get_command_stage());
}

/// Verifies the Module's WaitForMicros() method in the non-blocking execution mode.
void test_wait_for_micros_non_blocking()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    module_class.QueueCommand(static_cast<uint8_t>(MockModule::kMockCommands::kIdle), kNonBlockingCommand);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    module_class.AdvanceCommandStage();  // Restarts the stage delay timer.

    // A zero-length delay has always elapsed, so it never suspends the command.
    TEST_ASSERT_TRUE(module_class.WaitForMicros(0));

    TEST_ASSERT_FALSE(module_class.WaitForMicros(kNonBlockingStageDelay));
    delay(kDelayOvershootMilliseconds);
    TEST_ASSERT_TRUE(module_class.WaitForMicros(kNonBlockingStageDelay));
}

/// Verifies the Module's WaitForMicros() method in the blocking execution mode.
void test_wait_for_micros_blocking()
{
    StreamMock<kTestBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    module_class.QueueCommand(static_cast<uint8_t>(MockModule::kMockCommands::kIdle), kBlockingCommand);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    // Samples the reference point before the stage delay timer restarts, which keeps the measured interval at or
    // above the interval the timer itself observes.
    const uint32_t start_time = micros();
    module_class.AdvanceCommandStage();  // Restarts the stage delay timer.

    // The blocking mode suspends the runtime in place, so the requested duration has always elapsed on return.
    TEST_ASSERT_TRUE(module_class.WaitForMicros(kBlockingStageDelay));
    TEST_ASSERT_TRUE(micros() - start_time >= kBlockingStageDelay);
}

/// Verifies that the Module's SendData() method falls back to the built-in LED when the transmission fails.
void test_module_send_data_transmission_error()
{
    StreamMock<kTruncatingBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule module_class(kMockModuleType, kFirstMockModuleId, communication_class);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    module_class.QueueCommand(static_cast<uint8_t>(MockModule::kMockCommands::kIdle), kBlockingCommand);
    TEST_ASSERT_TRUE(module_class.ResolveActiveCommand());
    module_class.SendData(static_cast<uint8_t>(MockModule::kMockStatusCodes::kStageEntered));

    // The truncating stream fails both the event message and the error message that reports it, which leaves the LED
    // as the only channel able to communicate the failure.
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kTransmissionError),
        communication_class.get_communication_status()
    );
    TEST_ASSERT_EQUAL_INT(HIGH, digitalRead(LED_BUILTIN));

    digitalWrite(LED_BUILTIN, LOW);  // Restores the indicator for the tests that follow.
}

/// Verifies the Module's DigitalRead() method with and without readout averaging.
void test_digital_read_pin_polling()
{
    pinMode(kTestDigitalPin, OUTPUT);

    digitalWrite(kTestDigitalPin, HIGH);
    TEST_ASSERT_TRUE(MockModule::DigitalRead<kTestDigitalPin>());
    TEST_ASSERT_TRUE(MockModule::DigitalRead<kTestDigitalPin>(1));
    TEST_ASSERT_TRUE(MockModule::DigitalRead<kTestDigitalPin>(kTestPoolSize));

    digitalWrite(kTestDigitalPin, LOW);
    TEST_ASSERT_FALSE(MockModule::DigitalRead<kTestDigitalPin>());
    TEST_ASSERT_FALSE(MockModule::DigitalRead<kTestDigitalPin>(1));
    TEST_ASSERT_FALSE(MockModule::DigitalRead<kTestDigitalPin>(kTestPoolSize));
}

/// Verifies the Module's AnalogRead() method with and without readout averaging.
void test_analog_read_pin_polling()
{
    // Leaves the pin as an input, which is the only configuration every supported board routes to its
    // analog-to-digital converter. Driving the pin as a digital output presents the converter with no defined level
    // on the SAM3X, where the readouts then track the floating converter input rather than the pin.
    pinMode(kTestAnalogPin, INPUT_PULLUP);

    uint16_t lowest  = UINT16_MAX;
    uint16_t highest = 0;

    // Samples individual readouts before the pooled call. Requesting no pool size exercises the branch that disables
    // averaging.
    for (uint16_t index = 0; index < kAnalogProbeCount; ++index)
    {
        const uint16_t readout = MockModule::AnalogRead<kTestAnalogPin>();
        if (readout < lowest) lowest = readout;
        if (readout > highest) highest = readout;
    }

    const uint16_t pooled_readout = MockModule::AnalogRead<kTestAnalogPin>(kTestPoolSize);

    // Samples the readouts again after the pooled call, which brackets the pooled value even when the readout level
    // drifts over time. The pool size of 1 exercises the second branch that disables averaging.
    for (uint16_t index = 0; index < kAnalogProbeCount; ++index)
    {
        const uint16_t readout = MockModule::AnalogRead<kTestAnalogPin>(1);
        if (readout < lowest) lowest = readout;
        if (readout > highest) highest = readout;
    }

    // An average always falls inside the range its inputs span. Bounding the pooled readout by the individual ones
    // detects a broken accumulator, divisor, or return width without assuming a readout level, which no board
    // guarantees for a pin the test is free to use.
    TEST_ASSERT_GREATER_OR_EQUAL_UINT16(lowest, pooled_readout);
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(highest, pooled_readout);
}

/// Verifies that the Kernel's Setup() method configures every managed module.
void test_kernel_setup_configures_modules()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    MockModule second_module(kMockModuleType, kSecondMockModuleId, communication_class);
    Module* modules[] = {&first_module, &second_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();

    TEST_ASSERT_EQUAL_UINT16(1, first_module.get_record().setup_calls);
    TEST_ASSERT_EQUAL_UINT16(1, second_module.get_record().setup_calls);

    size_t offset                   = 0;
    const uint8_t expected_setup[3] = {
        static_cast<uint8_t>(kProtocols::kKernelState),
        static_cast<uint8_t>(Kernel::kKernelCommands::kResetController),
        static_cast<uint8_t>(Kernel::kKernelStatusCodes::kSetupComplete),
    };
    AssertTransmittedMessage(mock_port, offset, expected_setup);
    AssertNoTransmittedMessage(mock_port, offset);
}

/// Verifies that the Kernel's Setup() method suspends the runtime when a managed module fails to set up.
void test_kernel_setup_aborts_on_module_failure()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    MockModule second_module(kMockModuleType, kSecondMockModuleId, communication_class);
    MockModule third_module(kMockModuleType, kThirdMockModuleId, communication_class);
    Module* modules[] = {&first_module, &second_module, &third_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    second_module.set_setup_outcome(false);
    kernel_class.Setup();

    // The setup stops at the failing module, so the modules that follow it are never configured.
    TEST_ASSERT_EQUAL_UINT16(1, first_module.get_record().setup_calls);
    TEST_ASSERT_EQUAL_UINT16(1, second_module.get_record().setup_calls);
    TEST_ASSERT_EQUAL_UINT16(0, third_module.get_record().setup_calls);

    size_t offset                   = 0;
    const uint8_t expected_error[6] = {
        static_cast<uint8_t>(kProtocols::kKernelData),
        static_cast<uint8_t>(Kernel::kKernelCommands::kResetController),
        static_cast<uint8_t>(Kernel::kKernelStatusCodes::kModuleSetupError),
        static_cast<uint8_t>(kPrototypes::kTwoUint8s),
        kMockModuleType,
        kSecondMockModuleId,
    };
    AssertTransmittedMessage(mock_port, offset, expected_error);
    AssertNoTransmittedMessage(mock_port, offset);

    // The failed setup suspends the runtime, so the Kernel neither reads the PC-sent messages nor runs module commands.
    uint8_t command_packet[12] = {
        kBufferLayout::kStartByte,
        6,
        0,
        static_cast<uint8_t>(kProtocols::kOneOffModuleCommand),
        kMockModuleType,
        kFirstMockModuleId,
        0,
        static_cast<uint8_t>(MockModule::kMockCommands::kComplete),
        0,
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, command_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(mock_port.rx_buffer_index));
    TEST_ASSERT_EQUAL_UINT16(0, first_module.get_record().command_calls);
    AssertNoTransmittedMessage(mock_port, 0);
}

/// Verifies that the Kernel executes the controller and module identification commands.
void test_kernel_identification_commands()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    MockModule second_module(kMockModuleType, kSecondMockModuleId, communication_class);
    Module* modules[] = {&first_module, &second_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();

    // Requests the controller identification and acknowledges the message, which also verifies reception code echoing.
    uint8_t identify_controller_packet[9] = {
        kBufferLayout::kStartByte,
        3,
        0,
        static_cast<uint8_t>(kProtocols::kKernelCommand),
        kTestReceptionCode,
        static_cast<uint8_t>(Kernel::kKernelCommands::kIdentifyController),
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, identify_controller_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    size_t offset                       = 0;
    const uint8_t expected_reception[2] = {
        static_cast<uint8_t>(kProtocols::kReceptionCode),
        kTestReceptionCode,
    };
    const uint8_t expected_controller[2] = {
        static_cast<uint8_t>(kProtocols::kControllerIdentification),
        kTestControllerId,
    };
    AssertTransmittedMessage(mock_port, offset, expected_reception);
    AssertTransmittedMessage(mock_port, offset, expected_controller);
    AssertNoTransmittedMessage(mock_port, offset);

    // Requests the module identification, which reports the combined type and ID code of every managed module.
    uint8_t identify_modules_packet[9] = {
        kBufferLayout::kStartByte,
        3,
        0,
        static_cast<uint8_t>(kProtocols::kKernelCommand),
        0,
        static_cast<uint8_t>(Kernel::kKernelCommands::kIdentifyModules),
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, identify_modules_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    // The combined code is transmitted as a 16-bit value, which places the module ID ahead of the module type.
    offset                          = 0;
    const uint8_t expected_first[3] = {
        static_cast<uint8_t>(kProtocols::kModuleIdentification),
        kFirstMockModuleId,
        kMockModuleType,
    };
    const uint8_t expected_second[3] = {
        static_cast<uint8_t>(kProtocols::kModuleIdentification),
        kSecondMockModuleId,
        kMockModuleType,
    };
    AssertTransmittedMessage(mock_port, offset, expected_first);
    AssertTransmittedMessage(mock_port, offset, expected_second);
    AssertNoTransmittedMessage(mock_port, offset);
}

/// Verifies that the Kernel reports the kernel commands it does not support.
void test_kernel_unrecognized_command()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    Module* modules[] = {&first_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();

    uint8_t command_packet[9] = {
        kBufferLayout::kStartByte,
        3,
        0,
        static_cast<uint8_t>(kProtocols::kKernelCommand),
        0,
        kUnrecognizedKernelCommand,
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, command_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    size_t offset                   = 0;
    const uint8_t expected_error[3] = {
        static_cast<uint8_t>(kProtocols::kKernelState),
        kUnrecognizedKernelCommand,
        static_cast<uint8_t>(Kernel::kKernelStatusCodes::kCommandNotRecognized),
    };
    AssertTransmittedMessage(mock_port, offset, expected_error);
    AssertNoTransmittedMessage(mock_port, offset);
}

/// Verifies that the Kernel's reset command reruns the setup sequence for every managed module.
void test_kernel_reset_command()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    Module* modules[] = {&first_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();
    TEST_ASSERT_EQUAL_UINT16(1, first_module.get_record().setup_calls);

    uint8_t reset_packet[9] = {
        kBufferLayout::kStartByte,
        3,
        0,
        static_cast<uint8_t>(kProtocols::kKernelCommand),
        0,
        static_cast<uint8_t>(Kernel::kKernelCommands::kResetController),
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, reset_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    TEST_ASSERT_EQUAL_UINT16(2, first_module.get_record().setup_calls);

    size_t offset                   = 0;
    const uint8_t expected_setup[3] = {
        static_cast<uint8_t>(kProtocols::kKernelState),
        static_cast<uint8_t>(Kernel::kKernelCommands::kResetController),
        static_cast<uint8_t>(Kernel::kKernelStatusCodes::kSetupComplete),
    };
    AssertTransmittedMessage(mock_port, offset, expected_setup);
    AssertNoTransmittedMessage(mock_port, offset);
}

/// Verifies that the Kernel routes a module-addressed command to the module it is addressed to.
void test_kernel_module_command_routing()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    MockModule second_module(kMockModuleType, kSecondMockModuleId, communication_class);
    Module* modules[] = {&first_module, &second_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();

    uint8_t command_packet[12] = {
        kBufferLayout::kStartByte,
        6,
        0,
        static_cast<uint8_t>(kProtocols::kOneOffModuleCommand),
        kMockModuleType,
        kSecondMockModuleId,
        0,
        static_cast<uint8_t>(MockModule::kMockCommands::kComplete),
        0,
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, command_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    // Only the addressed module runs the command.
    TEST_ASSERT_EQUAL_UINT16(0, first_module.get_record().command_calls);
    TEST_ASSERT_EQUAL_UINT16(1, second_module.get_record().command_calls);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MockModule::kMockCommands::kComplete),
        second_module.get_record().last_command
    );
    TEST_ASSERT_EQUAL_UINT8(1, second_module.get_record().last_stage);

    size_t offset                        = 0;
    const uint8_t expected_completion[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kSecondMockModuleId,
        static_cast<uint8_t>(MockModule::kMockCommands::kComplete),
        static_cast<uint8_t>(Module::kCoreStatusCodes::kCommandCompleted),
    };
    AssertTransmittedMessage(mock_port, offset, expected_completion);
    AssertNoTransmittedMessage(mock_port, offset);
}

/// Verifies that the Kernel executes a multi-stage command across consecutive runtime cycles.
void test_kernel_staged_command_execution()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    Module* modules[] = {&first_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();

    uint8_t command_packet[12] = {
        kBufferLayout::kStartByte,
        6,
        0,
        static_cast<uint8_t>(kProtocols::kOneOffModuleCommand),
        kMockModuleType,
        kFirstMockModuleId,
        0,
        static_cast<uint8_t>(MockModule::kMockCommands::kMultiStage),
        0,
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, command_packet);
    ClearTransmittedMessages(mock_port);

    // The first cycle runs the opening stage and leaves the command active.
    kernel_class.RuntimeCycle();

    size_t offset                   = 0;
    const uint8_t expected_stage[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kFirstMockModuleId,
        static_cast<uint8_t>(MockModule::kMockCommands::kMultiStage),
        static_cast<uint8_t>(MockModule::kMockStatusCodes::kStageEntered),
    };
    AssertTransmittedMessage(mock_port, offset, expected_stage);
    AssertNoTransmittedMessage(mock_port, offset);
    TEST_ASSERT_EQUAL_UINT16(1, first_module.get_record().command_calls);

    // The second cycle runs the closing stage and completes the command.
    kernel_class.RuntimeCycle();

    const uint8_t expected_completion[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kFirstMockModuleId,
        static_cast<uint8_t>(MockModule::kMockCommands::kMultiStage),
        static_cast<uint8_t>(Module::kCoreStatusCodes::kCommandCompleted),
    };
    AssertTransmittedMessage(mock_port, offset, expected_stage);
    AssertTransmittedMessage(mock_port, offset, expected_completion);
    AssertNoTransmittedMessage(mock_port, offset);
    TEST_ASSERT_EQUAL_UINT8(2, first_module.get_record().last_stage);

    // The completed command releases the module, so no further cycle runs it again.
    kernel_class.RuntimeCycle();
    TEST_ASSERT_EQUAL_UINT16(2, first_module.get_record().command_calls);
    AssertNoTransmittedMessage(mock_port, offset);
}

/// Verifies that the Kernel releases the module whose command aborts itself mid-execution.
void test_kernel_aborted_command_execution()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    Module* modules[] = {&first_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();

    uint8_t command_packet[12] = {
        kBufferLayout::kStartByte,
        6,
        0,
        static_cast<uint8_t>(kProtocols::kOneOffModuleCommand),
        kMockModuleType,
        kFirstMockModuleId,
        0,
        static_cast<uint8_t>(MockModule::kMockCommands::kAbort),
        0,
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, command_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    // Aborting reports the command as completed, which distinguishes it from the discard the Kernel applies to the
    // commands the module does not recognize.
    size_t offset                        = 0;
    const uint8_t expected_completion[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kFirstMockModuleId,
        static_cast<uint8_t>(MockModule::kMockCommands::kAbort),
        static_cast<uint8_t>(Module::kCoreStatusCodes::kCommandCompleted),
    };
    AssertTransmittedMessage(mock_port, offset, expected_completion);
    AssertNoTransmittedMessage(mock_port, offset);

    // The abort clears the queue, so the module runs no further commands.
    ClearTransmittedMessages(mock_port);
    kernel_class.RuntimeCycle();
    AssertNoTransmittedMessage(mock_port, 0);
    TEST_ASSERT_EQUAL_UINT16(1, first_module.get_record().command_calls);
}

/// Verifies that the Kernel reports the module-addressed messages it cannot deliver.
void test_kernel_target_module_not_found()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    Module* modules[] = {&first_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();

    uint8_t command_packet[12] = {
        kBufferLayout::kStartByte,
        6,
        0,
        static_cast<uint8_t>(kProtocols::kOneOffModuleCommand),
        kMockModuleType,
        kAbsentMockModuleId,
        0,
        static_cast<uint8_t>(MockModule::kMockCommands::kComplete),
        0,
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, command_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    size_t offset                   = 0;
    const uint8_t expected_error[6] = {
        static_cast<uint8_t>(kProtocols::kKernelData),
        static_cast<uint8_t>(Kernel::kKernelCommands::kReceiveData),
        static_cast<uint8_t>(Kernel::kKernelStatusCodes::kTargetModuleNotFound),
        static_cast<uint8_t>(kPrototypes::kTwoUint8s),
        kMockModuleType,
        kAbsentMockModuleId,
    };
    AssertTransmittedMessage(mock_port, offset, expected_error);
    AssertNoTransmittedMessage(mock_port, offset);
    TEST_ASSERT_EQUAL_UINT16(0, first_module.get_record().command_calls);
}

/// Verifies that the Kernel rejects the module commands that use the reserved command code 0.
void test_kernel_rejects_reserved_command_code()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    Module* modules[] = {&first_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();

    const uint8_t expected_rejection[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kFirstMockModuleId,
        0,
        static_cast<uint8_t>(Module::kCoreStatusCodes::kCommandNotRecognized),
    };

    // The command queue reserves the code 0 to mark the absence of a command, so a one-off command that carries it is
    // rejected rather than queued.
    uint8_t one_off_packet[12] = {
        kBufferLayout::kStartByte,
        6,
        0,
        static_cast<uint8_t>(kProtocols::kOneOffModuleCommand),
        kMockModuleType,
        kFirstMockModuleId,
        0,
        0,
        0,
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, one_off_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    size_t offset = 0;
    AssertTransmittedMessage(mock_port, offset, expected_rejection);
    AssertNoTransmittedMessage(mock_port, offset);
    TEST_ASSERT_EQUAL_UINT16(0, first_module.get_record().command_calls);

    // A recurrent command that carries the reserved code is rejected the same way.
    uint8_t repeated_packet[16] = {
        kBufferLayout::kStartByte,
        10,
        0,
        static_cast<uint8_t>(kProtocols::kRepeatedModuleCommand),
        kMockModuleType,
        kFirstMockModuleId,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, repeated_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    offset = 0;
    AssertTransmittedMessage(mock_port, offset, expected_rejection);
    AssertNoTransmittedMessage(mock_port, offset);
    TEST_ASSERT_EQUAL_UINT16(0, first_module.get_record().command_calls);
}

/// Verifies that the Kernel discards an unrecognized recurrent command instead of repeating it.
void test_kernel_discards_unrecognized_recurrent_command()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    Module* modules[] = {&first_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();

    // Queues the unrecognized command with no cycle delay, which makes it eligible for repetition on every cycle.
    uint8_t command_packet[16] = {
        kBufferLayout::kStartByte,
        10,
        0,
        static_cast<uint8_t>(kProtocols::kRepeatedModuleCommand),
        kMockModuleType,
        kFirstMockModuleId,
        0,
        kUnrecognizedCommand,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, command_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    size_t offset                   = 0;
    const uint8_t expected_error[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kFirstMockModuleId,
        kUnrecognizedCommand,
        static_cast<uint8_t>(Module::kCoreStatusCodes::kCommandNotRecognized),
    };
    AssertTransmittedMessage(mock_port, offset, expected_error);
    AssertNoTransmittedMessage(mock_port, offset);
    TEST_ASSERT_EQUAL_UINT16(1, first_module.get_record().command_calls);

    // An unrecognized command cannot become recognized on a later repetition, so the discard removes it from the
    // queue and the module stays silent for the rest of the runtime.
    ClearTransmittedMessages(mock_port);
    delay(kDelayOvershootMilliseconds);

    for (uint8_t cycle = 0; cycle < 5; ++cycle)
    {
        kernel_class.RuntimeCycle();
    }

    AssertNoTransmittedMessage(mock_port, 0);
    TEST_ASSERT_EQUAL_UINT16(1, first_module.get_record().command_calls);
}

/// Verifies that the Kernel discards an unrecognized one-off command instead of leaving it active.
void test_kernel_discards_unrecognized_one_off_command()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    Module* modules[] = {&first_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();

    uint8_t command_packet[12] = {
        kBufferLayout::kStartByte,
        6,
        0,
        static_cast<uint8_t>(kProtocols::kOneOffModuleCommand),
        kMockModuleType,
        kFirstMockModuleId,
        0,
        kUnrecognizedCommand,
        0,
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, command_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    size_t offset                   = 0;
    const uint8_t expected_error[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kFirstMockModuleId,
        kUnrecognizedCommand,
        static_cast<uint8_t>(Module::kCoreStatusCodes::kCommandNotRecognized),
    };
    AssertTransmittedMessage(mock_port, offset, expected_error);
    AssertNoTransmittedMessage(mock_port, offset);

    // The discarded command releases the module, which leaves it able to run the commands queued after the failure.
    ClearTransmittedMessages(mock_port);
    kernel_class.RuntimeCycle();
    AssertNoTransmittedMessage(mock_port, 0);
    TEST_ASSERT_EQUAL_UINT16(1, first_module.get_record().command_calls);
}

/// Verifies that the Kernel's dequeue command stops a recurrent command from repeating.
void test_kernel_dequeue_command()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    Module* modules[] = {&first_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();

    uint8_t command_packet[16] = {
        kBufferLayout::kStartByte,
        10,
        0,
        static_cast<uint8_t>(kProtocols::kRepeatedModuleCommand),
        kMockModuleType,
        kFirstMockModuleId,
        0,
        static_cast<uint8_t>(MockModule::kMockCommands::kComplete),
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, command_packet);
    ClearTransmittedMessages(mock_port);

    // The recurrent command repeats on every cycle, as it carries no cycle delay.
    kernel_class.RuntimeCycle();
    kernel_class.RuntimeCycle();
    TEST_ASSERT_EQUAL_UINT16(2, first_module.get_record().command_calls);

    uint8_t dequeue_packet[10] = {
        kBufferLayout::kStartByte,
        4,
        0,
        static_cast<uint8_t>(kProtocols::kDequeueModuleCommand),
        kMockModuleType,
        kFirstMockModuleId,
        0,
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, dequeue_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    // Clearing the queue retires the recurrent command, which reports the command as completed exactly once.
    size_t offset                        = 0;
    const uint8_t expected_completion[5] = {
        static_cast<uint8_t>(kProtocols::kModuleState),
        kMockModuleType,
        kFirstMockModuleId,
        static_cast<uint8_t>(MockModule::kMockCommands::kComplete),
        static_cast<uint8_t>(Module::kCoreStatusCodes::kCommandCompleted),
    };
    AssertTransmittedMessage(mock_port, offset, expected_completion);
    AssertNoTransmittedMessage(mock_port, offset);

    ClearTransmittedMessages(mock_port);
    kernel_class.RuntimeCycle();
    kernel_class.RuntimeCycle();
    TEST_ASSERT_EQUAL_UINT16(2, first_module.get_record().command_calls);
    AssertNoTransmittedMessage(mock_port, 0);
}

/// Verifies that the Kernel applies the parameters addressed to a managed module.
void test_kernel_module_parameters()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    Module* modules[] = {&first_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();

    // The parameter block carries the single-byte identifier followed by the two bytes of the 16-bit payload.
    uint8_t parameters_packet[13] = {
        kBufferLayout::kStartByte,
        7,
        0,
        static_cast<uint8_t>(kProtocols::kModuleParameters),
        kMockModuleType,
        kFirstMockModuleId,
        0,
        222,
        1,
        2,
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, parameters_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    TEST_ASSERT_EQUAL_UINT16(1, first_module.get_record().parameter_calls);
    TEST_ASSERT_EQUAL_UINT8(222, first_module.get_parameters().identifier);
    TEST_ASSERT_EQUAL_UINT16(0x0201, first_module.get_parameters().payload);

    size_t offset                    = 0;
    const uint8_t expected_result[3] = {
        static_cast<uint8_t>(kProtocols::kKernelState),
        static_cast<uint8_t>(Kernel::kKernelCommands::kReceiveData),
        static_cast<uint8_t>(Kernel::kKernelStatusCodes::kModuleParametersSet),
    };
    AssertTransmittedMessage(mock_port, offset, expected_result);
    AssertNoTransmittedMessage(mock_port, offset);
}

/// Verifies that the Kernel reports the parameters a managed module is unable to apply.
void test_kernel_module_parameters_error()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    Module* modules[] = {&first_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();

    // The parameter block is one byte short of the size the module's parameter structure requires, which fails the
    // extraction the module performs.
    uint8_t parameters_packet[12] = {
        kBufferLayout::kStartByte,
        6,
        0,
        static_cast<uint8_t>(kProtocols::kModuleParameters),
        kMockModuleType,
        kFirstMockModuleId,
        0,
        222,
        1,
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, parameters_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    TEST_ASSERT_EQUAL_UINT16(1, first_module.get_record().parameter_calls);

    size_t offset                   = 0;
    const uint8_t expected_error[6] = {
        static_cast<uint8_t>(kProtocols::kKernelData),
        static_cast<uint8_t>(Kernel::kKernelCommands::kReceiveData),
        static_cast<uint8_t>(Kernel::kKernelStatusCodes::kModuleParametersError),
        static_cast<uint8_t>(kPrototypes::kTwoUint8s),
        kMockModuleType,
        kFirstMockModuleId,
    };
    AssertTransmittedMessage(mock_port, offset, expected_error);
    AssertNoTransmittedMessage(mock_port, offset);
}

/// Verifies that the Kernel reports the messages that use a protocol it cannot receive.
void test_kernel_invalid_message_protocol()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    Module* modules[] = {&first_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules);

    kernel_class.Setup();

    // The KernelData protocol is reserved for the outgoing messages, so an incoming message that uses it is invalid.
    constexpr auto invalid_protocol = static_cast<uint8_t>(kProtocols::kKernelData);
    uint8_t invalid_packet[9]       = {kBufferLayout::kStartByte, 3, 0, invalid_protocol, 1, 2, 0, 0, 0};
    InjectReceivedMessage(mock_port, invalid_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();

    size_t offset                   = 0;
    const uint8_t expected_error[5] = {
        static_cast<uint8_t>(kProtocols::kKernelData),
        static_cast<uint8_t>(Kernel::kKernelCommands::kReceiveData),
        static_cast<uint8_t>(Kernel::kKernelStatusCodes::kInvalidMessageProtocol),
        static_cast<uint8_t>(kPrototypes::kOneUint8),
        invalid_protocol,
    };
    AssertTransmittedMessage(mock_port, offset, expected_error);
    AssertNoTransmittedMessage(mock_port, offset);
}

/// Verifies that the Kernel resets the controller when the PC stops sending keepalive messages.
void test_kernel_keepalive_timeout()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    Module* modules[] = {&first_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules, kKeepaliveTestInterval);

    kernel_class.Setup();

    uint8_t keepalive_packet[9] = {
        kBufferLayout::kStartByte,
        3,
        0,
        static_cast<uint8_t>(kProtocols::kKernelCommand),
        0,
        static_cast<uint8_t>(Kernel::kKernelCommands::kKeepAlive),
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, keepalive_packet);
    ClearTransmittedMessages(mock_port);

    // The keepalive message activates the tracking and restarts the timer, so this cycle reports no timeout.
    kernel_class.RuntimeCycle();
    AssertNoTransmittedMessage(mock_port, 0);
    TEST_ASSERT_EQUAL_UINT16(1, first_module.get_record().setup_calls);

    delay(kDelayOvershootMilliseconds);
    kernel_class.RuntimeCycle();

    // The timeout reports the interval it enforced and then reruns the setup sequence to restore the default state.
    size_t offset                     = 0;
    const uint8_t expected_timeout[8] = {
        static_cast<uint8_t>(kProtocols::kKernelData),
        static_cast<uint8_t>(Kernel::kKernelCommands::kReceiveData),
        static_cast<uint8_t>(Kernel::kKernelStatusCodes::kKeepAliveTimeout),
        static_cast<uint8_t>(kPrototypes::kOneUint32),
        kKeepaliveTestTimeout,
        0,
        0,
        0,
    };
    const uint8_t expected_setup[3] = {
        static_cast<uint8_t>(kProtocols::kKernelState),
        static_cast<uint8_t>(Kernel::kKernelCommands::kResetController),
        static_cast<uint8_t>(Kernel::kKernelStatusCodes::kSetupComplete),
    };
    AssertTransmittedMessage(mock_port, offset, expected_timeout);
    AssertTransmittedMessage(mock_port, offset, expected_setup);
    AssertNoTransmittedMessage(mock_port, offset);
    TEST_ASSERT_EQUAL_UINT16(2, first_module.get_record().setup_calls);
}

/// Verifies that the Kernel saturates the keepalive interval whose doubling overflows the interval tracker.
void test_kernel_keepalive_interval_saturation()
{
    StreamMock<kRuntimeBufferSize> mock_port;
    Communication communication_class(mock_port);
    MockModule first_module(kMockModuleType, kFirstMockModuleId, communication_class);
    Module* modules[] = {&first_module};
    Kernel kernel_class(kTestControllerId, communication_class, modules, kOverflowingKeepaliveInterval);

    kernel_class.Setup();

    uint8_t keepalive_packet[9] = {
        kBufferLayout::kStartByte,
        3,
        0,
        static_cast<uint8_t>(kProtocols::kKernelCommand),
        0,
        static_cast<uint8_t>(Kernel::kKernelCommands::kKeepAlive),
        0,
        0,
        0,
    };
    InjectReceivedMessage(mock_port, keepalive_packet);
    ClearTransmittedMessages(mock_port);

    kernel_class.RuntimeCycle();
    delay(kDelayOvershootMilliseconds);
    kernel_class.RuntimeCycle();

    // Saturating the interval keeps the timeout far above the elapsed time, so the watchdog stays silent. Wrapping it
    // instead would shorten the timeout to a few milliseconds and reset the controller within the delay above.
    AssertNoTransmittedMessage(mock_port, 0);
    TEST_ASSERT_EQUAL_UINT16(1, first_module.get_record().setup_calls);
}

/// Specifies the test functions executed at runtime.
int RunUnityTests()
{
    UNITY_BEGIN();

    // ResolvePrototype (compile-time verification)
    RUN_TEST(test_resolve_prototype);

    // SendDataMessage
    RUN_TEST(test_send_data_message);
    RUN_TEST(test_send_data_message_array_object);

    // SendStateMessage
    RUN_TEST(test_send_state_message);

    // SendCommunicationErrorMessage
    RUN_TEST(test_send_communication_error_message);

    // SendServiceMessage
    RUN_TEST(test_send_service_message);
    RUN_TEST(test_send_service_message_wide_code);

    // Transmission error handling
    RUN_TEST(test_send_message_transmission_error);

    // ReceiveMessage
    RUN_TEST(test_receive_message);
    RUN_TEST(test_receive_message_errors);

    // ExtractModuleParameters
    RUN_TEST(test_extract_module_parameters);
    RUN_TEST(test_extract_module_parameters_errors);

    // Module identifiers
    RUN_TEST(test_module_identifiers);

    // Module command resolution
    RUN_TEST(test_resolve_active_command_without_queued_command);
    RUN_TEST(test_resolve_active_command_activates_queued_command);
    RUN_TEST(test_resolve_active_command_prefers_active_command);
    RUN_TEST(test_resolve_active_command_repeats_recurrent_command);

    // Module command queue management
    RUN_TEST(test_reset_command_queue_stops_recurrent_command);
    RUN_TEST(test_reset_command_queue_preserves_active_command);
    RUN_TEST(test_queue_command_retires_replaced_recurrent_command);
    RUN_TEST(test_complete_command_completion_reporting);
    RUN_TEST(test_abort_command_clears_command_queue);
    RUN_TEST(test_abort_command_preserves_queued_command);
    RUN_TEST(test_discard_active_command_clears_recurrent_queue);
    RUN_TEST(test_discard_active_command_preserves_queued_command);
    RUN_TEST(test_module_command_error_reporting);
    RUN_TEST(test_reset_execution_parameters);

    // Module command execution utilities
    RUN_TEST(test_advance_command_stage);
    RUN_TEST(test_wait_for_micros_non_blocking);
    RUN_TEST(test_wait_for_micros_blocking);
    RUN_TEST(test_module_send_data_transmission_error);
    RUN_TEST(test_digital_read_pin_polling);
    RUN_TEST(test_analog_read_pin_polling);

    // Kernel setup
    RUN_TEST(test_kernel_setup_configures_modules);
    RUN_TEST(test_kernel_setup_aborts_on_module_failure);

    // Kernel commands
    RUN_TEST(test_kernel_identification_commands);
    RUN_TEST(test_kernel_unrecognized_command);
    RUN_TEST(test_kernel_reset_command);

    // Kernel module message routing
    RUN_TEST(test_kernel_module_command_routing);
    RUN_TEST(test_kernel_staged_command_execution);
    RUN_TEST(test_kernel_aborted_command_execution);
    RUN_TEST(test_kernel_target_module_not_found);
    RUN_TEST(test_kernel_rejects_reserved_command_code);
    RUN_TEST(test_kernel_discards_unrecognized_recurrent_command);
    RUN_TEST(test_kernel_discards_unrecognized_one_off_command);
    RUN_TEST(test_kernel_dequeue_command);
    RUN_TEST(test_kernel_module_parameters);
    RUN_TEST(test_kernel_module_parameters_error);
    RUN_TEST(test_kernel_invalid_message_protocol);

    // Kernel keepalive monitoring
    RUN_TEST(test_kernel_keepalive_timeout);
    RUN_TEST(test_kernel_keepalive_interval_saturation);

    return UNITY_END();
}

// Defines the baud rates for different boards.

// For Arduino Due, the maximum non-doubled stable rate is 5.25 Mbps at 84 MHz cpu clock.
#if defined(ARDUINO_SAM_DUE)
static constexpr uint32_t kSerialBaudRate = 5250000;

// For Uno, Mega, and other 16 MHz AVR boards, the maximum stable non-doubled rate is 1 Mbps.
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA) ||  \
    defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega168__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega16U4__)
static constexpr uint32_t kSerialBaudRate = 1000000;

// For all other boards the default 9600 rate is used.
#else
static constexpr uint32_t kSerialBaudRate = 9600;
#endif

/// Runs all tests inside setup() as required by the Arduino framework for one-shot testing, which includes
/// Teensy boards.
void setup()
{
    // Starts the serial connection.
    Serial.begin(kSerialBaudRate);

    // Waits ~2 seconds before the Unity test runner establishes connection with a board Serial interface. For teensy,
    // this is less important, since it uses a USB interface which does not reset the board on connection.
    delay(2000);

    // Runs the required tests.
    RunUnityTests();

    // Stops the serial communication interface.
    Serial.end();
}

/// Intentionally empty. All tests run in setup() as one-shot operations.
void loop()
{}
