/**
 * @file
 * @brief The header-only file that provides assets necessary to support and control the runtime of the Communication
 * class.
 *
 * @section comm_sa_description Description:
 *
 * This file aggregates all assets used to interface with the Communication class and support its runtime. These assets
 * are stored separately from other shared assets to improve future code maintainability, allowing to adjust the
 * Communication class API independently of the rest of the library.
 *
 * This file contains:
 * - kProtocols, kModuleTypes, kModuleIDs, kCommandCodes and kEventCodes enumerations. Jointly, these enumerations
 * provide system-reserved byte codes used in the communication process. These codes should not be directly used by
 * custom modules and are reserved to enable proper functioning of the Communication-Kernel-Module class triad.
 * - kCommunicationStatusCodes enumeration that stores status bytecodes used by the Communication to report its runtime
 * status.
 *
 * @section comm_sa_developer_notes Developer Notes:
 *
 * This file aggregates all byte-codes used by the Communication class to construct and interpret message payloads.
 * Aggregating these codes in the same file improves code readability and maintainability. Additionally, the codes from
 * this file are used by ataraxis-micro-controller library Kernel and (base) Module classes. Jointly, this standardizes
 * communication between all project Ataraxis systems.
 *
 * @attention When appropriate, shared assets should always be used over any private / class-specific assets.
 *
 * @section comm_sa_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 */

#ifndef AXTL_COMMUNICATION_ASSETS_H
#define AXTL_COMMUNICATION_ASSETS_H

// Dependencies
#include <Arduino.h>

/**
 * @namespace communication_assets
 * @brief Provides all assets (structures, enumerations, variables) necessary to interface with and support the
 * runtime of the Communication class.
 *
 * These assets are used to simplify library development by storing co-dependent assets in the same place.
 * Since Communication class is primarily designed to work in-conjunction with the ataraxis-micro-controller
 * project, many assets available from this namespace are specifically designed to be used by classes from that library.
 *
 * @note The assets from this namespace should always be used over any custom assets available through the
 * ataraxis-micro-controller library.
 */
namespace communication_assets
{
    /**
     * @enum kProtocols
     * @brief Stores byte-codes codes for Ataraxis communication protocols supported by the ArenaMicroController
     * systems.
     *
     * The enumerators available through this enumeration should be used to interpret and set the values of the
     * 'protocol' field of the ID structure of each incoming and outgoing message payload during communication.
     *
     * @note This enumeration only stores a subset of all communication protocols as ArenaMicroController systems are
     * not capable of using all of the protocols available to the broader project.
     *
     * @attention All protocol codes are expected to be unique across the entire Ataraxis project. Therefore, avoid
     * using the codes already reserved by other Ataraxis systems, even if the protocols denoted by those codes are not
     * supported by the ArenaMicroController systems.
     */
    enum class kProtocols : uint8_t
    {
        /// Undefined protocol. This is not a valid protocol, and it is used as a default protocol initializer.
        /// Encountering this code is always interpreted as an error case.
        kUndefined = 0,

        /// This is a general data communication protocol designed to communicate arbitrary objects of small-to-medium
        /// size (objects that are likely to fully fit into the COBS-limited packet size). The protocol bundles each
        /// object with an ID that, combined with the header ID structure and the 1-byte event code, allows uniquely
        /// identifying each data object after reception. This protocol is specifically designed for sending data
        /// to other Ataraxis systems in the event-driven fashion and it reserves the first byte after the ID structure
        /// for storing the unique event byte-code. For the protocol designed for sending commands and equipped to store
        /// command byte-codes, see kPackedObjectCommand protocol. Note, the ID structure module_type and module_id
        /// fields are used to communicate the type and id of the module that sent the data.
        kData = 1,

        /// This protocol is almost identical to the kPackedObjectData protocol with one specific exception: the first
        /// byte after the ID structure is reserved for storing the command byte-code, instead of the event byte-code.
        /// The protocol is designed to communicate the specific command to be executed by the target module alongside
        /// an arbitrary number of parameter objects to configure the command execution. Note, the ID structure
        /// module_type and module_id fields are used to communicate the type and id of the target module, to which
        /// the command is addressed (in contrast to the kPackedObjectData protocol).
        kCommand = 2,

        /// This is another protocol from the PackedObject family, specifcially geared for transmitting error-messages.
        /// This protocol is largely similar to the kPackedObjectData protocol as it also makes use of the main
        /// error-event value immediately following the ID header structure. Generally, it is highly recomended to use
        /// only this specific protocol for error handling, as Ataraxis project is configured with the assumption that
        /// all non-handshake errors use this specific message protocol.
        kError = 3,

        kParameter = 4,

        kIdle = 5,
    };

    /**
     * @enum kModuleTypes
     * @brief Stores byte-codes for Core (Kernel-level) AtaraxisMicroController module types.
     *
     * The enumerators available through this enumeration should be used to interpret and set the values of the
     * 'module_type' field of the ID structure of each incoming and outgoing message payload during communication.
     * Specifically, this enumeration stores the codes for Core level modules that support the proper runtime of each
     * AtaraxisMicroController.
     *
     * @attention The codes available through this enumeration should be unique and, as such, should not be used to
     * denote custom module types. The uniqueness claim only applies to the particular ArenaMicroController, the codes
     * can be reused across multiple controllers or Ataraxis systems.
     */
    enum class kModuleTypes : uint8_t
    {
        /// Undefined module type. This value is used as the default initializer and is not a valid module type.
        /// Encountering this value during communication is interpreted as an error.
        kUndefined = 0,

        /// The code for the AtaraxisMicroController Communication module.
        kCommunicationModule = 1,

        /// The code for the AtaraxisMicroController Kernel module
        kKernelModule = 2,
    };

    /**
     * @enum kModuleIDs
     * @brief Stores non-standard byte-codes for AtaraxisMicroController module IDs.
     *
     * The enumerators available through this enumeration should be used to interpret and set the values of the
     * 'module_id' field of the ID structure of each incoming and outgoing message payload during communication.
     * Specifically, this enumeration stores the generic codes that can be encountered in place of a valid module ID.
     * Overall, they are designed to be used in specific cases where a particular module ID is not available or does not
     * apply.
     *
     * @attention The codes available through this enumeration should be unique and, as such, should not be used to
     * denote custom module IDs. These codes are expected to be unique across the entire Ataraxis project.
     */
    enum class kModuleIDs : uint8_t
    {
        /// Undefined module ID. This value is used as the default initializer and is not a valid module ID.
        /// Encountering this value during communication is interpreted as an error.
        kUndefined = 0,

        /// A special code used for module_types that either do not need unique module identification or when all
        /// available modules of a given type are used at the same time. An example of the former case would be a
        /// custom FPGA array to support close-loop 2P-driven experiments that only exists as a singular entity. An
        /// example of the latter case would be a synchronization pulse that briefly sets all TTL pins of the controller
        /// to High.
        kAllSystems = 255,
    };

    enum class kCommandCodes : uint8_t
    {
        kUnknownCommand   = 0,  ///< A placeholder value used when a definitive command ID cannot be determined
        kNoCommand        = 0,  ///< A universal alias used to indicate no requested command to run
    };

    enum class kEventCodes : uint8_t
    {
        kDataCommunicationError = 1,    ///< SerialPCCommunication() class method runtime error
        kPayloadAddressError    = 2,    ///< Unable to address received data payload to any Module or Kernel
        kErrorCode              = 250,  ///< The code used to identify error-communicating Datum values
    };
} // namespace communication_assets

#endif //AXTL_COMMUNICATION_ASSETS_H
