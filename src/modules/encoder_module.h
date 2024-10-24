/**
 * @file
 * @brief The header file for the EncoderModule class, which allows interfacing with a Quadrature encoder to
 * continuously read its pulse vector data.
 */

#ifndef AXMC_ENCODER_MODULE_H
#define AXMC_ENCODER_MODULE_H

#include <Arduino.h>
// Note, this definition has to precede Encoder.h inclusion. This increases the resolution of the encoder, but
// interferes with any other library that makes use of AttachInterrupt() function.
#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>
#include "module.h"
#include "shared_assets.h"

/**
 * @brief Provides access to the quadrature encoder pulse counter.
 *
 * This module is specifically designed to interface with quadrature encoder and track the directional change in overall
 * encoder pulses between class method calls. In turn, this offers a way to precisely monitor the displacement and
 * directional vector of a rotating object connected to the encoder.
 *
 * Largely, this class is an API wrapper around the Paul Stoffregen's Encoder library.
 *
 * @tparam kPinA the digital interrupt pin connected to the 'A' channel of the quadrature encoder.
 * @tparam kPinB the digital interrupt pin connected to the 'B' channel of the quadrature encoder.
 * @param invertDirection if true, inverts the interpretation of the pin trigger order. By default, when pin 2 is
 * triggered before pin 1, the counter decreases and when pin 1 is triggered before pin 2, it increases. Setting this
 * flag to true makes the counter decrease when pin 1 is triggered before pin 2, and increase when pin 2 is triggered
 * before pin 1. This is the same as reversing the CCW / CW direction of the encoder.
 */
template <const uint8_t kPinA, const uint8_t kPinB, const bool kInvertDirection>
class EncoderModule final : public Module
{
        // Verifies that the quadrature pins are different.
        static_assert(kPinA != kPinB, "The two encoder interrupt pins cannot be the same.");

    public:
        /// Assigns meaningful names to byte status-codes used to track module states. Note, this enumeration has to
        /// use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes enumeration inherited from base
        /// Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kStandBy  = 51,  ///< The code used to initialize custom status tracker.
            kCCWDelta = 52,  ///< The encoder was moved in the CCW direction by the included number of pulses.
            kCWDelta  = 53,  ///< The encoder was moved in the CW direction by the included number of pulses.
            kNoDelta  = 54,  ///< The encoder did not move.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kReadEncoderData =
                1,  ///< Gets the encoder direction and displacement in pulses, relative to the previous reading.
            kResetEncoder = 2,  ///< Resets the encoder's position to 0.
        };

        /// Initializes the EncoderModule class by subclassing the base Module class and instantiating the
        /// internal Encoder class.
        EncoderModule(
            const uint8_t module_type,
            const uint8_t module_id,
            Communication& communication,
            const shared_assets::DynamicRuntimeParameters& dynamic_parameters
        ) :
            Module(module_type, module_id, communication, dynamic_parameters), _encoder(kPinA, kPinB)
        {}

        /// Overwrites the custom_parameters structure memory with the data extracted from the Communication
        /// reception buffer.
        bool SetCustomParameters() override
        {
            // Extracts the received parameters into the _custom_parameters structure of the class. If extraction fails,
            // returns false. This instructs the Kernel to execute the necessary steps to send an error message to the
            // PC.
            if (!_communication.ExtractParameters(_custom_parameters)) return false;
            module_status = static_cast<uint8_t>(kCoreStatusCodes::kParametersSet);  // Records the status
            return true;
        }

        /// Executes the currently active command. Unlike other overloaded API methods, this one does not set the
        /// module_status. Instead, it expects the called command to set the status as necessary.
        bool RunActiveCommand() override
        {
            // Depending on the currently active command, executes the necessary logic.
            switch (static_cast<kModuleCommands>(GetActiveCommand()))
            {
                // ReadEncoder
                case kModuleCommands::kReadEncoderData: ReadEncoder(); return true;
                // ResetEncoder
                case kModuleCommands::kResetEncoder: ResetEncoder(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            // Since Encoder class carries out the necessary hardware setup, this method re-initializes the encoder to
            // repeat the hardware setup.
            _encoder      = Encoder(kPinA, kPinB);
            module_status = static_cast<uint8_t>(kCoreStatusCodes::kSetupComplete);  // Records the status
            return true;
        }

        /// Resets the custom_parameters structure fields to their default values.
        bool ResetCustomAssets() override
        {
            _custom_parameters.report_CCW = true;  // Defaults to report changes in the CCW direction.
            _custom_parameters.report_CW  = true;  // Defaults to report changes in the CW direction.
            module_status = static_cast<uint8_t>(kCoreStatusCodes::kModuleAssetsReset);  // Records the status
            return true;
        }

        ~EncoderModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                bool report_CCW = true;  ///< Determines whether to report changes in the CCW direction.
                bool report_CW  = true;  ///< Determines whether to report changes in the CW direction.
        } _custom_parameters;

        /// The encoder class that abstracts low-level access to the Encoder pins and provides an easy API to retrieve
        /// the automatically incremented encoder pulse vector. The vector can be reset by setting to 0, and the class
        /// relies on hardware interrupt functionality to maintain the desired precision.
        Encoder _encoder;

        /// Reads the current encoder position, resets the encoder, and sends the result to the PC. The position is
        /// measured by the number and vector of pulses since the last encoder reset. Note, the result will be
        /// transformed into an absolute value and its direction can be inferred from the event status of the sent
        /// message.
        void ReadEncoder()
        {
            // Note, this is technically a 1-stage command.

            // Retrieves and, if necessary, flips the value of the encoder. The value tracks the number of pulses
            // relative to the previous reset command or the initialization of the encoder.
            const int32_t flipped_value = _encoder.readAndReset() * (kInvertDirection ? -1 : 1);
            uint32_t delta              = 0;  // Stores the absolute displacement in encoder pulses.

            // If encoder has not moved since the last call to this method, sets the status to indicate there was no
            // change in encoder position.
            if (flipped_value == 0)
            {
                module_status = static_cast<uint8_t>(kCustomStatusCodes::kNoDelta);
            }

            // If the value is negative, this is interpreted as the CCW movement direction.
            else if (flipped_value < 0)
            {
                // If reporting the CCW movement is allowed, updates the status as necessary
                if (_custom_parameters.report_CCW)
                {
                    module_status = static_cast<uint8_t>(kCustomStatusCodes::kCCWDelta);
                    delta         = static_cast<uint32_t>(abs(flipped_value));  // Overwrites the delta
                }

                // Otherwise, discards the CCW movement and instead updates the status to indicate there is no change
                // in the encoder position relative to the last call to this method.
                else
                {
                    module_status = static_cast<uint8_t>(kCustomStatusCodes::kNoDelta);
                }
            }

            // Finally, if the value is positive, this is interpreted as the CW movement direction.
            else
            {
                // If reporting the CW movement is allowed, updates the status as necessary
                if (_custom_parameters.report_CW)
                {
                    module_status = static_cast<uint8_t>(kCustomStatusCodes::kCWDelta);
                    delta         = static_cast<uint32_t>(abs(flipped_value));
                }

                // Otherwise, discards the CCW movement and instead updates the status to indicate there is no change
                // in the encoder position relative to the last call to this method.
                else
                {
                    module_status = static_cast<uint8_t>(kCustomStatusCodes::kNoDelta);
                    delta         = 0;
                }
            }

            // Sends the encoder data to the PC. Uses module_status to indicate the movement direction and the delta to
            // communicate the encoder displacement.
            SendData(module_status, delta);
            CompleteCommand();
        }

        /// Resets the encoder pulse counter back to 0. This can be sued to reset the encoder without reading its data.
        void ResetEncoder()
        {
            _encoder.write(0);  // Resets the encoder tracker back to 0 pulses.
            CompleteCommand();
        }
};

#endif  //AXMC_ENCODER_MODULE_H
