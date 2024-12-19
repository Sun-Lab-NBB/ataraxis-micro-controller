// Due to certain issues with reconnecting to teensy boards for running separate test suits, this test suit acts as a
// single centralized hub for running all available tests for all supported classes and methods of the
// SerializedTransferProtocol library. Declare all required tests using separate functions (as needed) and then add the
// tests to be evaluated to the RunUnityTests function at the bottom of this file. Comment unused tests out if needed.

// Dependencies
#include <Arduino.h>  // For Arduino functions
#include <unity.h>   // This is the C testing framework, no connection to the Unity game engine
#include "axmc_shared_assets.h"
#include "cobs_processor.h"  // COBSProcessor class
#include "communication.h"
#include "crc_processor.h"    // CRCProcessor class
#include "stream_mock.h"      // StreamMock class required for SerializedTransferProtocol class testing
#include "transport_layer.h"  // SerializedTransferProtocol class

// This function is called automatically before each test function. Currently not used.
void setUp()
{}

// This function is called automatically after each test function. Currently not used.
void tearDown()
{}


// Tests the errors associated with the SendDataMessage() method of the Communication class
// These tests focuses specifically on errors raised by only this method; COBS and CRC errors should be
// tested by their respective test functions. Also, does not test errors that are generally impossible 
// to encounter without modifying the class code, such as COBS or CRC failing and and causing a 
// TransmissionError.
//kCommunicationTransmitted
void TestSendDataMessageTransmittedOneUnsignedByte()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    //OneUnsignedByte
    const uint8_t test_object = 0;

    const uint8_t module_type = 112;       // Example module type
    const uint8_t module_id = 12;          // Example module ID
    const uint8_t command = 88;            // Example command code
    const uint8_t event_code = 221;        // Example event code
  
    const axmc_communication_assets::kPrototypes prototype = axmc_communication_assets::kPrototypes::kOneUint8;

    comm_class.SendDataMessage(command, event_code, prototype, test_object);

    uint16_t expected_kernal[5] = {8, 88, 221, 1, 1}; 
    // Expected payload is: 129, 5, 5, 8, 88, 221, 1, 1, ...

    for (int i = 0; i < 5; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected_kernal[i], mock_port.tx_buffer[i + 3]);
    }

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );

    comm_class.SendDataMessage(module_type, module_id, command, event_code, prototype, test_object);
    uint16_t expected_module[7] = {7, 112, 12, 88, 221, 1, 1}; 
    // Expected payload is: 129, 7, 7, 7, 112, 12, 88, 221, 1, 1,...

    for (int i = 0; i < 7; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 14]);
    }

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
}

void TestSendDataMessageTwoUnsignedBytes()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    //TwoUnsignedBytes
    uint8_t test_object[2] = {0}; 

    const uint8_t module_type = 112;       // Example module type
    const uint8_t module_id = 12;          // Example module ID
    const uint8_t command = 88;            // Example command code
    const uint8_t event_code = 221;        // Example event code
    const axmc_communication_assets::kPrototypes prototype = axmc_communication_assets::kPrototypes::kTwoUint8s;

    comm_class.SendDataMessage(command, event_code, prototype, test_object);

    uint16_t expected_kernal[6] = {8, 88, 221, 2, 1, 1}; 
    // Expected payload is: 129, 6, 5, 8, 88, 221, 2, 1, 1, ...

    for (int i = 0; i < 6; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected_kernal[i], mock_port.tx_buffer[i + 3]);
    }

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );

    comm_class.SendDataMessage(module_type, module_id, command, event_code, prototype, test_object);

    uint16_t expected_module[8] = {7, 112, 12, 88, 221, 2, 1, 1}; 
    // Expected payload is: 129, 8, 7, 7, 112, 12, 88, 221, 2, 1, 1, ...

    for (int i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 15]);
    }

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
}

void TestSendDataMessageTransmittedThreeUnsignedBytes()
{

    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    //ThreeUnsignedBytes
    uint8_t test_object[3] = {1}; 

    const uint8_t module_type = 112;       // Example module type
    const uint8_t module_id = 12;          // Example module ID
    const uint8_t command = 88;            // Example command code
    const uint8_t event_code = 221;        // Example event code
    const axmc_communication_assets::kPrototypes prototype = axmc_communication_assets::kPrototypes::kThreeUint8s;

    comm_class.SendDataMessage(command, event_code, prototype, test_object);

    uint16_t expected_kernal[7] = {8, 88, 221, 3, 1, 1, 1}; 
    // Expected payload is: 129, 7, 8, 8, 88, 221, 3, 1, 1, 1, ...

    for (int i = 0; i < 7; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected_kernal[i], mock_port.tx_buffer[i + 3]);
    }
    
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );

    comm_class.SendDataMessage(module_type, module_id, command, event_code, prototype, test_object);
    
    uint16_t expected_module[9] = {7, 112, 12, 88, 221, 3, 1, 1, 1}; 
    // Expected payload is: 129, 9, 10, 7, 112, 12, 88, 221, 3, 1, 1, 1, ...

    for (int i = 0; i < 9; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 16]);
    }

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
}

void TestSendDataMessageTransmittedFourUnsignedBytes()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    //FourUnsignedBytes
    uint8_t test_object[4] = {1}; 

    const uint8_t module_type = 112;       // Example module type
    const uint8_t module_id = 12;          // Example module ID
    const uint8_t command = 88;            // Example command code
    const uint8_t event_code = 221;        // Example event code
    const axmc_communication_assets::kPrototypes prototype = axmc_communication_assets::kPrototypes::kFourUint8s;

    comm_class.SendDataMessage(command, event_code, prototype, test_object);
    
    uint16_t expected_kernal[8] = {8, 88, 221, 4, 1, 1, 1, 1}; 
    // Expected payload is: 129, 7, 8, 8, 88, 221, 4, 1, 1, 1, 1, ...

    for (int i = 0; i < 7; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected_kernal[i], mock_port.tx_buffer[i + 3]);
    }
    
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );

    comm_class.SendDataMessage(module_type, module_id, command, event_code, prototype, test_object);
    
    uint16_t expected_module[10] = {7, 112, 12, 88, 221, 4, 1, 1, 1, 1}; 
    // Expected payload is: 129, 9, 10, 7, 112, 12, 88, 221, 4, 1, 1, 1, 1, ...

    for (int i = 0; i < 10; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 17]);
    }

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
}

void TestSendDataMessagekOneUnsignedLong()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    //OneUnsignedLong
    const uint32_t test_object = 0;

    const uint8_t module_type = 112;       // Example module type
    const uint8_t module_id = 12;          // Example module ID
    const uint8_t command = 88;            // Example command code
    const uint8_t event_code = 221;        // Example event code
    const axmc_communication_assets::kPrototypes prototype = axmc_communication_assets::kPrototypes::kOneUint32;

    comm_class.SendDataMessage(command, event_code, prototype, test_object);

    uint16_t expected_kernal[8] = {8, 88, 221, 5, 1, 1, 1, 1}; 
    // Expected payload is: 129, 6, 5, 8, 88, 221, 5, 1, 1, 1, 1 ...

    for (int i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected_kernal[i], mock_port.tx_buffer[i + 3]);
    }
    
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );

    comm_class.SendDataMessage(module_type, module_id, command, event_code, prototype, test_object);
    
    uint16_t expected_module[10] = {7, 112, 12, 88, 221, 5, 1, 1, 1, 1}; 
    // Expected payload is: 129, 8, 7, 7, 112, 12, 88, 221, 4, 1, 1, ...

    for (int i = 0; i < 10; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 17]);
    }

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
}

void TestSendDataMessagekOneUnsignedShort()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    //OneUnsignedShort
    const uint16_t test_object = 0;

    const uint8_t module_type = 112;       // Example module type
    const uint8_t module_id = 12;          // Example module ID
    const uint8_t command = 88;            // Example command code
    const uint8_t event_code = 221;        // Example event code
    const axmc_communication_assets::kPrototypes prototype = axmc_communication_assets::kPrototypes::kOneUint16;

    comm_class.SendDataMessage(command, event_code, prototype, test_object);

    uint16_t expected_kernal[6] = {8, 88, 221, 6, 1, 1}; 
    // Expected payload is: 129, 5, 5, 8, 88, 221, 6, 1, ...

    for (int i = 0; i < 6; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected_kernal[i], mock_port.tx_buffer[i + 3]);
    }

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );

    comm_class.SendDataMessage(module_type, module_id, command, event_code, prototype, test_object);
    uint16_t expected_module[8] = {7, 112, 12, 88, 221, 6, 1, 1}; 
    // Expected payload is: 129, 7, 7, 7, 112, 12, 88, 221, 6, 1,...

    for (int i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 15]);
    }

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
}

// Tests the errors associated with the SendStateMessage() method of the Communication class.
// These tests focus specifically on errors raised raised directly by this method. Also, does 
// not test errors that are generally impossible to encounter without modifying the class code,
// such as PackingError and TransmissionError.
void TestSendStateMessageMessageSent(){
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    const uint8_t module_type = 112;       // Example module type
    const uint8_t module_id = 12;          // Example module ID
    const uint8_t command = 88;            // Example command code
    const uint8_t event_code = 221;        // Example event code

    comm_class.SendStateMessage(command, event_code);
    
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );

    comm_class.SendStateMessage(module_type, module_id, command, event_code);
    
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
}

// Tests the errors associated with the SendStateMessage() method of the Communication class.
// These tests focus specifically on errors raised raised directly by this method. Also, does 
// not test errors that are generally impossible to encounter without modifying the class code,
// such as buffer not being large enough to hold the total two byte values protocol_code and 
// code and throwing a PackingError.
void SendCommunicationErrorMessageModule() {
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    const uint8_t module_type = 1;
    const uint8_t module_id = 2;
    const uint8_t command = 3;
    const uint8_t error_code = 4;

    comm_class.communication_status = 65;

    comm_class.SendCommunicationErrorMessage(module_type, module_id, command, error_code);
    uint16_t expected[8] = {7, 1, 2, 3, 4, 2, 65, 101}; 
    // Expected payload is: 129, 8, 9, 7, 1, 2, 3, 4, 2, 65, 11,...

    for (int i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected[i], mock_port.tx_buffer[i + 3]);
    }

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
}

void Test_SendCommunicationErrorMessageKernel() {
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    const uint8_t command = 3;
    const uint8_t error_code = 4;

    comm_class.communication_status = 65;

    comm_class.SendCommunicationErrorMessage(command, error_code);

    uint16_t expected[6] = {8, 3, 4, 2, 65, 101};
    // Expected payload is: 129, 6, 7, 8, 3, 4, 2, 65, 11,...

    for (int i = 0; i < 6; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected[i], mock_port.tx_buffer[i + 3]);
    }

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
}


// Tests the errors associated with the SendServiceMessage() method of the Communication class.
// These tests focus specifically on errors raised directly by this method. Also, does not test 
// errors that are generally impossible to encounter without modifying the class code, such as
// buffer not being large enough to hold the total two byte values protocol_code and code and 
// throwing a PackingError.
void TestSendServiceMessageTransmittedReceptionCode(){
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    const uint8_t protocol_code = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kReceptionCode);
    const uint8_t service_code = static_cast<uint8_t>(axmc_communication_assets::kProtocols::KRepeatedModuleCommand); // Example service code

    comm_class.SendServiceMessage(protocol_code, service_code);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
}

void TestSendServiceMessageTransmittedIdentificationCode(){
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    const uint8_t protocol_code = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kIdentification);
    const uint8_t service_code = static_cast<uint8_t>(axmc_communication_assets::kProtocols::KRepeatedModuleCommand);      // Example service code
    
    comm_class.SendServiceMessage(protocol_code, service_code);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
}

// kCommunicationInvalidProtocolError for kCommand Protocol
void TestSendServiceMessageInvalidProtocolCode() {
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    /// Using KRepeatedModuleCommand as protocol_code
    const uint8_t protocol_code = static_cast<uint8_t>(axmc_communication_assets::kProtocols::KRepeatedModuleCommand);
    const uint8_t service_code = static_cast<uint8_t>(axmc_communication_assets::kProtocols::KRepeatedModuleCommand);   

    comm_class.SendServiceMessage(protocol_code, service_code);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kInvalidProtocol),
        comm_class.communication_status
    );
}



// kCommunicationInvalidProtocolError for kParameters Protocol
void TestSendServiceMessageInvalidProtocolErrorParametersProtocol() {
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    const uint8_t protocol_code = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleParameters);
    const uint8_t code = 112;  // Example service code

    // Using kParameters as protocol_code
    comm_class.SendServiceMessage(protocol_code, code);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kInvalidProtocol),
        comm_class.communication_status
    );
}

//kCommunicationInvalidProtocolError
void TestSendServiceMessageInvalidProtocalErrorArbitraryProtocol(){
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);
    
    const uint8_t protocol_code = 200; // Invalid protocol
    const uint8_t code = 112;      // Example service code
    comm_class.SendServiceMessage(protocol_code, code);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kInvalidProtocol),
        comm_class.communication_status
    );
}


// Tests the errors associated with the ReceiveMessage() method of the Communucation class
// These tests focuses specifically on errors raised by only this method; COBS and CRC related errors should be
// tested by their respective test functions.

void TestReceiveMessageReceptionErrorNoCRCandCOBSCalculation() {
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

    // Currently, the layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL, MODULE, ID, RETURN_CODE, COMMAND, NO_BLOCK, 
    // CYCLE_DELAY[4], DELIMITER, CRC[2]
    uint8_t test_buffer[16] = {129, 10, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
    
    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Converts the test_buffer (uint8_t array) into a uint16_t array for compatibility with the rx_buffer.
    uint16_t test_buffer_uint16[16] = {0};  
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
    test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }
    
    // Does NOT calculate the CRC for the COBS-encoded buffer.
    // Copies the fully encoded package into the rx_buffer to simulate packet reception and test ReceiveData() method.
    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kReceptionError),
        comm_class.communication_status
    );
}

// kCommunicationParsingError
void TestReceiveMessageReceivedNoBytesToReceive() {
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kNoBytesToReceive),
        comm_class.communication_status
    );
}

void TestReceiveMessageReceivedRepeatedModuleCommand() {
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

   // Zero out command_message fields
    memset(&comm_class.repeated_module_command, 0, sizeof(axmc_communication_assets::RepeatedModuleCommand));
    comm_class.repeated_module_command.cycle_delay = 0xFFFFFFFFFF;

    // Currently, the layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL, MODULE, ID, RETURN_CODE, COMMAND, NO_BLOCK, 
    // CYCLE_DELAY[4], DELIMITER, CRC[2]
    uint8_t test_buffer[16] = {129, 10, 0, 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 

     // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);


    uint16_t test_buffer_uint16[16] = {0}; 
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }
    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(2, comm_class.repeated_module_command.module_type);       // Check module_type
    TEST_ASSERT_EQUAL_UINT8(3, comm_class.repeated_module_command.module_id);         // Check module_id
    TEST_ASSERT_EQUAL_UINT8(4, comm_class.repeated_module_command.return_code);       // Check return_code
    TEST_ASSERT_EQUAL_UINT8(0, comm_class.repeated_module_command.command);           // Check command
    TEST_ASSERT_FALSE(comm_class.repeated_module_command.noblock);                    // Check noblock
    
    // Ensure other fields that should not have been modified remain unchanged
    for (size_t i = sizeof(axmc_communication_assets::RepeatedModuleCommand); i < sizeof(comm_class.repeated_module_command); ++i) {
        TEST_ASSERT_EQUAL_UINT8(0xFF, reinterpret_cast<uint8_t*>(&comm_class.repeated_module_command)[i]);
    }
}

void TestReceiveMessageReceivedOneOffModuleCommand() {
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

    // Currently, the layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL, MODULE, ID, RETURN_CODE, COMMAND, NOBLOCK,
    // DELIMITER, CRC[2]
    uint8_t test_buffer[12] = {129, 6, 0, 2, 0, 5, 8, 0, 0, 0, 0, 0}; 

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer. The insertion location has to be statically shifted to account for the
    // metadata preamble bytes
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[12] = {0}; 
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]); 
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(0, comm_class.one_off_module_command.module_type);       // Check module_type
    TEST_ASSERT_EQUAL_UINT8(5, comm_class.one_off_module_command.module_id);         // Check module_id
    TEST_ASSERT_EQUAL_UINT8(8, comm_class.one_off_module_command.return_code);       // Check return_code
    
    // Ensure other fields that should not have been modified remain unchanged
    for (size_t i = sizeof(axmc_communication_assets::OneOffModuleCommand); i < sizeof(comm_class.one_off_module_command); ++i) {
        TEST_ASSERT_EQUAL_UINT8(0xFF, reinterpret_cast<uint8_t*>(&comm_class.one_off_module_command)[i]);
    }
}

void TestReceiveMessageReceivedDequeueModuleCommand() {
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

    // The correct layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL, MODULE, ID, RETURN_CODE, DELIMITER, CRC[2]
    uint8_t test_buffer[10] = {129, 4, 0, 3, 19, 3, 0, 0, 0, 0}; 

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[10] = {0}; 
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
    test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(19, comm_class.module_dequeue.module_type);       // Check module_type
    TEST_ASSERT_EQUAL_UINT8(3, comm_class.module_dequeue.module_id);         // Check module_id
    TEST_ASSERT_EQUAL_UINT8(0, comm_class.module_dequeue.return_code);       // Check return_code

    // Ensure other fields that should not have been modified remain unchanged
    for (size_t i = sizeof(axmc_communication_assets::DequeueModuleCommand); i < sizeof(comm_class.module_dequeue); ++i) {
        TEST_ASSERT_EQUAL_UINT8(0xFF, reinterpret_cast<uint8_t*>(&comm_class.module_dequeue)[i]);
    }
}

void TestReceiveMessageReceivedKernalCommand() {
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

    // The correct layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL, RETURN_CODE, COMMAND, DELIMITER, CRC[2]
    uint8_t test_buffer[9] = {129, 3, 0, 4, 0, 3, 0, 0, 0}; 

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[9] = {0}; 
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
    test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(0, comm_class.kernel_command.return_code);   // Check return_code
    TEST_ASSERT_EQUAL_UINT8(3, comm_class.kernel_command.command);       // Check return_code

    // Ensure other fields that should not have been modified remain unchanged
    for (size_t i = sizeof(axmc_communication_assets::KernelCommand); i < sizeof(comm_class.kernel_command); ++i) {
        TEST_ASSERT_EQUAL_UINT8(0xFF, reinterpret_cast<uint8_t*>(&comm_class.kernel_command)[i]);
    }
}

void TestReceiveMessageReceivedModuleParameters() {
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

    // Instantiates a ModuleParameters withpayload.
    // The correct layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL, MODULE_TYPE, MODULE_ID, RETURN_CODE, DELIMITER, CRC[2]
    uint8_t test_buffer[10] = {129, 4, 0, 5, 1, 20, 1, 0, 0, 0}; 

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[10] = {0}; 
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
    test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(1, comm_class.module_parameter.module_type);       // Check module_type
    TEST_ASSERT_EQUAL_UINT8(20, comm_class.module_parameter.module_id);         // Check module_id
    TEST_ASSERT_EQUAL_UINT8(1, comm_class.module_parameter.return_code);       // Check return_code

    // Ensure other fields that should not have been modified remain unchanged
    for (size_t i = sizeof(axmc_communication_assets::ModuleParameters); i < sizeof(comm_class.module_parameter); ++i) {
        TEST_ASSERT_EQUAL_UINT8(0xFF, reinterpret_cast<uint8_t*>(&comm_class.kernel_command)[i]);
    }
}

void TestReceiveMessageReceivedKernalParameters() {
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

    // The current layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL, RETURN_CODE, DYNAMIC_PARAMETRS, DELIMITER, CRC[2]
    uint8_t test_buffer[10] = {129, 4, 0, 6, 1, 1, 1, 0, 0, 0}; 

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[10] = {0}; 
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
    test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(1, comm_class.kernel_parameters.return_code);       // Check return_code
    TEST_ASSERT_EQUAL_UINT8(1, comm_class.kernel_parameters.dynamic_parameters.action_lock);  // Check action_lock
    TEST_ASSERT_EQUAL_UINT8(1, comm_class.kernel_parameters.dynamic_parameters.ttl_lock);     // Check ttl_lock

    // Ensure other fields that should not have been modified remain unchanged
    for (size_t i = sizeof(axmc_communication_assets::KernelParameters); i < sizeof(comm_class.kernel_parameters); ++i) {
        TEST_ASSERT_EQUAL_UINT8(0xFF, reinterpret_cast<uint8_t*>(&comm_class.kernel_command)[i]);
    }
}

void TestReceiveMessageInvalidProtocolErrorArbitraryProtocolValue() {
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

    // Currently, the layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL,  MODULE, ID, RETURN_CODE , OBJECT_SIZE, OBJECT, DELIMITER, CRC[2]
    uint8_t test_buffer[16] = {129, 10, 0, 100, 2, 3, 4, 5, 1, 2, 3, 4, 5, 0, 0, 0}; // 100 is an invalid protocol code

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[16] = {0}; 
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kInvalidProtocol),
        comm_class.communication_status
    );
}

void TestReceiveMessageParsingErrorRepeatedModuleMissingParameter() {
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class; 

    memset(&comm_class.repeated_module_command, 0, sizeof(axmc_communication_assets::RepeatedModuleCommand));

    // The correct layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL, MODULE, ID, RETURN_CODE, COMMAND, NO_BLOCK,
    // CYCLE[4], DELIMITER, CRC[2]

    // This is missing a byte for CYCLEDELAY
    uint8_t test_buffer[16] = {129, 9, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 

     // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);


    uint16_t test_buffer_uint16[16] = {0}; 
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }
    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParsingError),
        comm_class.communication_status
    );
}

// Tests the errors associated with the ExtractParameter() method of the Communucation class
// These tests focuses specifically on errors raised by only this method; COBS and CRC errors should be
// tested by their respective test functions.
void TestExtractParametersExtractionForbidden()
{
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

    // Currently, the layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL,  MODULE_TYPE, MODULE_ID, OBJECT[6], DELIMITER, CRC[2]
    //The message is not a ModuleParamters message
    uint8_t test_buffer[16] = {129, 10, 0, 3, 2, 3, 4, 5, 1, 2, 3, 4, 5, 0, 0, 0}; 

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[16] = {0}; 
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]); 
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    uint8_t extract_into[6] = {0};
    comm_class.ExtractModuleParameters(extract_into);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kExtractionForbidden),
        comm_class.communication_status
    );
}

void TestExtractParametersArrayDestination()
{
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

    // Currently, the layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL,  MODULE_TYPE, MODULE_ID, OBJECT[6], DELIMITER, CRC[2]
    uint8_t test_buffer[16] = {129, 10, 0, 5, 2, 3, 4, 5, 1, 2, 3, 4, 5, 0, 0, 0}; 

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[16] = {0}; 
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]); 
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    uint8_t extract_into[6] = {0};
    comm_class.ExtractModuleParameters(extract_into);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParametersExtracted),
        comm_class.communication_status
    );
}

void TestExtractParametersStructDestination()
{
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

    // Currently, the layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL, MODULE, ID, RETURN_CODE, OBJECT[6], DELIMITER, CRC[2]
    uint8_t test_buffer[16] = {129, 10, 0, 5, 2, 3, 4, 9, 1, 2, 3, 4, 5, 0, 0, 0}; 

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[16] = {0}; 
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]); 
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();
    // Define a test array to extract parameters into
      struct TestStructure
    {
        uint8_t id = 1;
        uint8_t data[5] = {0};
    } test_structure;


    // Call the ExtractParameters function, expecting a successful extraction
    comm_class.ExtractModuleParameters(test_structure);
    
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParametersExtracted),
        comm_class.communication_status
    );
}

void TestExtractParametersSizeMismatchDestinationSizeLarger() {
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

    // Currently, the layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL,  MODULE, ID, RETURN_CODE, OBJECT[3], DELIMITER, CRC[2]
    uint8_t test_buffer[16] = {129, 10, 0, 5, 2, 3, 4, 5, 1, 5, 0, 0, 0}; 

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer. The insertion location has to be statically shifted to account for the
    // metadata preamble bytes
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[16] = {0}; 
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();
    // Input struture with larger size than transmitted data
    struct DataMessage {
        uint8_t id = 0;
        uint8_t data[10] = {0};
    } data_message;

    bool success = comm_class.ExtractModuleParameters(data_message);

    TEST_ASSERT_FALSE(success);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParameterMismatch),
        comm_class.communication_status
    );
}

void TestExtractParametersSizeMismatchDestinationSizeSmaller() {
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

    // Currently, the layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL,  MODULE, ID, RETURN_CODE, OBJECT[8], DELIMITER, CRC[2]
    uint8_t test_buffer[18] = {129, 12, 0, 5, 2, 3, 4, 5, 1, 2, 3, 4, 5, 18, 30, 0, 0, 0}; 

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer. The insertion location has to be statically shifted to account for the
    // metadata preamble bytes
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[18] = {0}; 
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();
    
    // Input struture with smaller size than transmitted data
    struct DataMessage {
        uint8_t id = 0;
        uint8_t data = 0;
    } data_message;

    bool success = comm_class.ExtractModuleParameters(data_message);

    TEST_ASSERT_FALSE(success);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParameterMismatch),
        comm_class.communication_status
    );
}

void TestExtractParametersParsingErrorLargeTransmittedData() {
    StreamMock<254> mock_port; 
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);  
    COBSProcessor<1, 2> cobs_class;  

    // Instantiates an array with larger size than the max payload size
    // Currently, the layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCAL, MODULE, ID, RETURN_CODE, OBJECT, DELIMITER, CRC[2]

    uint8_t test_buffer[266] = {0}; 

    // Simulates COBS encoding the buffer.
    test_buffer[0] = 129;
    test_buffer[1] = 260;
    test_buffer[2] = 0;
    test_buffer[3] = 5;
    test_buffer[4] = 2;
    test_buffer[5] = 3;
    test_buffer[6] = 4;
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[266] = {0}; 
    for (size_t i = 0; i < sizeof(test_buffer); ++i) {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]); 
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    uint8_t extract_into[256] = {0};
    comm_class.ExtractModuleParameters(extract_into);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParsingError),
        comm_class.communication_status
    );
}

// Specifies the test functions to be executed and controls their runtime. Use this function to determine which tests
// are run during test runtime and in what order. Note, each test function is encapsulated and will run even if it
// depends on other test functions ran before it which fail the tests.
int RunUnityTests()
{
    UNITY_BEGIN();

    // SendDataMessage 
    RUN_TEST(TestSendDataMessageTransmittedOneUnsignedByte);
    RUN_TEST(TestSendDataMessageTwoUnsignedBytes);
    RUN_TEST(TestSendDataMessageTransmittedThreeUnsignedBytes);
    RUN_TEST(TestSendDataMessageTransmittedFourUnsignedBytes);
    RUN_TEST(TestSendDataMessagekOneUnsignedLong);
    RUN_TEST(TestSendDataMessagekOneUnsignedShort);

    //SendStateMessage
    RUN_TEST(TestSendStateMessageMessageSent);

    //SendCommunicationErrorMessage
    RUN_TEST(SendCommunicationErrorMessageModule);
    RUN_TEST(Test_SendCommunicationErrorMessageKernel);

    //SendServiceMessage
    RUN_TEST(TestSendServiceMessageTransmittedReceptionCode);
    RUN_TEST(TestSendServiceMessageTransmittedIdentificationCode);
    RUN_TEST(TestSendServiceMessageInvalidProtocolCode);
    RUN_TEST(TestSendServiceMessageInvalidProtocolErrorParametersProtocol);
    RUN_TEST(TestSendServiceMessageInvalidProtocalErrorArbitraryProtocol);


    // ReceiveMessage
    RUN_TEST(TestReceiveMessageReceptionErrorNoCRCandCOBSCalculation);
    RUN_TEST(TestReceiveMessageReceivedNoBytesToReceive);
    RUN_TEST(TestReceiveMessageReceivedRepeatedModuleCommand);
    RUN_TEST(TestReceiveMessageReceivedOneOffModuleCommand);
    RUN_TEST(TestReceiveMessageReceivedDequeueModuleCommand);
    RUN_TEST(TestReceiveMessageReceivedKernalCommand);
    RUN_TEST(TestReceiveMessageReceivedModuleParameters);
    RUN_TEST(TestReceiveMessageReceivedKernalParameters);
    RUN_TEST(TestReceiveMessageInvalidProtocolErrorArbitraryProtocolValue);
    RUN_TEST(TestReceiveMessageParsingErrorRepeatedModuleMissingParameter);

    //ExtractParameter
    RUN_TEST(TestExtractParametersExtractionForbidden);
    RUN_TEST(TestExtractParametersArrayDestination);
    RUN_TEST(TestExtractParametersStructDestination);
    RUN_TEST(TestExtractParametersSizeMismatchDestinationSizeSmaller);
    RUN_TEST(TestExtractParametersSizeMismatchDestinationSizeLarger);
    RUN_TEST(TestExtractParametersParsingErrorLargeTransmittedData);

    return UNITY_END();
}

// Defines the baud rates for different boards. Note, this list is far from complete and was designed for the boards
// the author happened to have at hand at the time of writing these tests. When testing on an architecture not
// explicitly covered below, it may be beneficial to provide the baudrate optimized for the tested platform.

// For Arduino Due, the maximum non-doubled stable rate is 5.25 Mbps at 84 MHz cpu clock.
#if defined(ARDUINO_SAM_DUE)
#define SERIAL_BAUD_RATE 5250000

// For Uno, Mega, and other 16 MHz AVR boards, the maximum stable non-doubled rate is 1 Mbps.
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA) || \
      defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || \
      defined(__AVR_ATmega168__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega16U4__) || \
      defined(__AVR_ATmega32U4__)
#define SERIAL_BAUD_RATE 1000000

// For all other board the default 9600 rate is used. Note, this is a very slow rate and it is very likely your board
// supports a faster rate. Be sure to select the most appropriate rate based on the CPU clock of your board, as it
// directly affects the error rate at any given baudrate. Boards like Teensy and Dues using USB port ignore the baudrate
// setting and instead default to the fastest speed support by the particular USB port pair (480 Mbps for most modern
// ports).
#else
#define SERIAL_BAUD_RATE 9600
#endif

// This is necessary for the Arduino framework testing to work as expected, which includes teensy. All tests are
// run inside setup function as they are intended to be one-shot tests
void setup()
{
    // Starts the serial connection. Uses the SERIAL_BAUD_RATE defined above based on the specific board architecture
    // (or the generic safe 9600 baudrate, which is VERY slow and should not really be used in production code).
    Serial.begin(SERIAL_BAUD_RATE);

    // Waits ~2 seconds before the Unity test runner establishes connection with a board Serial interface. For teensy,
    // this is less important as, instead of using a UART, it uses a USB interface which does not reset the board on
    // connection.
    delay(2000);

    // Runs the required tests
    RunUnityTests();

    // Stops the serial communication interface.
    Serial.end();
}

// Nothing here as all tests are done in a one-shot fashion using 'setup' function above
void loop()
{}
