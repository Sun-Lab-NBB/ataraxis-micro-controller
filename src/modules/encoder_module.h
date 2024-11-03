/**
 * @file
 * @brief The header file for the EncoderModule class, which allows interfacing with am encoder and using its pulse
 * counter to track the direction and displacement of circular motion.
 *
 * @attention This file is written in a way that is NOT compatible with any other library that uses AttachInterrupt().
 * Disable the 'ENCODER_USE_INTERRUPTS' macro to make this file compatible with other interrupt libraries.
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
 * @brief Internally binds an Encoder class instance and enables accessing its movement direction and displacement.
 *
 * This module is specifically designed to interface with quadrature encoders and track the direction and number of
 * encoder pulses between class method calls. In turn, this allows monitoring the displacement and directional vector
 * of a rotating object connected to the encoder.
 *
 * @note Largely, this class is an API wrapper around the Paul Stoffregen's Encoder library, and it relies on efficient
 * interrupt logic to increase encoder tracking precision. To achieve the best performance, make sure both Pin A and
 * Pin B are hardware interrupt pins.
 *
 * @tparam kPinA the digital interrupt pin connected to the 'A' channel of the quadrature encoder.
 * @tparam kPinB the digital interrupt pin connected to the 'B' channel of the quadrature encoder.
 * @param kInvertDirection if set to true, inverts the sign of the value returned by the encoder. By default, when
 * Pin B is triggered before Pin A, the pulse counter decreases, which corresponds to CW movement. When pin A is
 * triggered before pin B, the counter increases, which corresponds to the CCW movement. This flag allows reversing
 * this relationship, which is helpful to account for the mounting and wiring of the encoder.
 */
template <const uint8_t kPinA, const uint8_t kPinB, const bool kInvertDirection>
class EncoderModule final : public Module
{
        // Verifies that the quadrature pins are different.
        static_assert(kPinA != kPinB, "The two EncoderModule interrupt pins cannot be the same.");

        static_assert(
            kPinA != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Please select a different "
            "pin A for EncoderModule class."
        );

        static_assert(
            kPinB != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Please select a different "
            "pin B for EncoderModule class."
        );

    public:
        /// Assigns meaningful names to byte status-codes used to track module states. Note, this enumeration has to
        /// use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes enumeration inherited from base
        /// Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kStandBy  = 51,  ///< The code used to initialize custom status tracker.
            kCCWDelta = 52,  ///< The encoder was moved in the CCW direction.
            kCWDelta  = 53,  ///< The encoder was moved in the CW.
            kNoDelta  = 54,  ///< The encoder did not move.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kReadEncoder =
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
            return _communication.ExtractModuleParameters(_custom_parameters);
        }

        /// Executes the currently active command. Unlike other overloaded API methods, this one does not set the
        /// module_status. Instead, it expects the called command to set the status as necessary.
        bool RunActiveCommand() override
        {
            // Depending on the currently active command, executes the necessary logic.
            switch (static_cast<kModuleCommands>(GetActiveCommand()))
            {
                // ReadEncoder
                case kModuleCommands::kReadEncoder: ReadEncoder(); return true;
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
            _encoder = Encoder(kPinA, kPinB);

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.report_CCW = true;  // Defaults to report changes in the CCW direction.
            _custom_parameters.report_CW  = true;  // Defaults to report changes in the CW direction.

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
        /// transformed into an absolute value and its direction can be inferred from the event status of the
        /// message transmitted to PC.
        void ReadEncoder()
        {
            // Retrieves and, if necessary, flips the value of the encoder. The value tracks the number of pulses
            // relative to the previous reset command or the initialization of the encoder.
            const int32_t flipped_value = _encoder.readAndReset() * (kInvertDirection ? -1 : 1);

            // If encoder has not moved since the last call to this method, sets the status to indicate there was no
            // change in encoder position.
            if (flipped_value == 0) return;

            // If the value is negative, this is interpreted as the CW movement direction.
            if (flipped_value < 0 && _custom_parameters.report_CW)
            {
                // If reporting the CW movement is allowed, sends the delta to the PC.
                SendData(
                    static_cast<uint8_t>(kCustomStatusCodes::kCWDelta),
                    communication_assets::kPrototypes::kOneUnsignedLong,
                    static_cast<uint32_t>(abs(flipped_value))
                );
            }

            // Finally, if the value is positive, this is interpreted as the CCW movement direction.
            if (_custom_parameters.report_CCW)
            {
                // If reporting the CCW movement is allowed, updates the status as necessary
                SendData(
                    static_cast<uint8_t>(kCustomStatusCodes::kCCWDelta),
                    communication_assets::kPrototypes::kOneUnsignedLong,
                    static_cast<uint32_t>(abs(flipped_value))
                );
            }
            CompleteCommand();
        }

        /// Resets the encoder pulse counter back to 0. This can be used to reset the encoder without reading its data.
        void ResetEncoder()
        {
            _encoder.write(0);  // Resets the encoder tracker back to 0 pulses.
            CompleteCommand();
        }
};

#endif  //AXMC_ENCODER_MODULE_H
