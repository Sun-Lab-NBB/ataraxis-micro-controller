/**
 * @file
 * @brief The header file for the EncoderModule class, which allows interfacing with a rotary encoder to track the
 * direction and distance of a circular object movement.
 *
 * @attention This file is written in a way that is @b NOT compatible with any other library that uses
 * AttachInterrupt(). Disable the 'ENCODER_USE_INTERRUPTS' macro defined at the top of the file to make this file
 * compatible with other interrupt libraries.
 */

#ifndef AXMC_ENCODER_MODULE_H
#define AXMC_ENCODER_MODULE_H

// Note, this definition has to precede Encoder.h inclusion. This increases the resolution of the encoder, but
// interferes with any other library that makes use of AttachInterrupt() function.
#define ENCODER_USE_INTERRUPTS

#include <Arduino.h>
#include <Encoder.h>
#include "module.h"
#include "shared_assets.h"

/**
 * @brief Wraps an Encoder class instance and provides access to its pulse counter to evaluate the direction and
 * distance of a circular object movement.
 *
 * This module is specifically designed to interface with quadrature encoders and track the direction and number of
 * encoder pulses between class method calls. In turn, this allows monitoring the direction and distance moved by the
 * rotating object connected to the encoder.
 *
 * @note Largely, this class is an API wrapper around the Paul Stoffregen's Encoder library, and it relies on efficient
 * interrupt logic to increase encoder tracking precision. To achieve the best performance, make sure both Pin A and
 * Pin B are hardware interrupt pins.
 *
 * @tparam kPinA the digital interrupt pin connected to the 'A' channel of the quadrature encoder.
 * @tparam kPinB the digital interrupt pin connected to the 'B' channel of the quadrature encoder.
 * @tparam kInvertDirection if set to true, inverts the sign of the value returned by the encoder. By default, when
 * Pin B is triggered before Pin A, the pulse counter decreases, which corresponds to CW movement. When pin A is
 * triggered before pin B, the counter increases, which corresponds to the CCW movement. This flag allows reversing
 * this relationship, which is helpful to account for the mounting and wiring of the tracked rotating object.
 */
template <const uint8_t kPinA, const uint8_t kPinB, const bool kInvertDirection>
class EncoderModule final : public Module
{
        // Ensures that the encoder pins are different.
        static_assert(kPinA != kPinB, "The two EncoderModule interrupt pins cannot be the same.");

        // Also ensures that encoder pins do not interfere with LED pin.
        static_assert(
            kPinA != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different pin A for EncoderModule instance."
        );
        static_assert(
            kPinB != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different pin B for EncoderModule instance."
        );

    public:
        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kRotatedCCW = 51,  ///< The encoder was rotated in the CCW direction.
            kRotatedCW  = 52,  ///< The encoder was rotated in the CW.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kCheckState = 1,  ///< Gets the change in pulse counts and sign relative to last check.
            kReset      = 2,  ///< Resets the encoder's pulse tracker to 0.
        };

        /// Initializes the class by subclassing the base Module class and instantiating the
        /// internal Encoder class.
        EncoderModule(
            const uint8_t module_type,
            const uint8_t module_id,
            Communication& communication,
            const shared_assets::DynamicRuntimeParameters& dynamic_parameters
        ) :
            Module(module_type, module_id, communication, dynamic_parameters)
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

        /// Executes the currently active command.
        bool RunActiveCommand() override
        {
            // Depending on the currently active command, executes the necessary logic.
            switch (static_cast<kModuleCommands>(GetActiveCommand()))
            {
                // ReadEncoder
                case kModuleCommands::kCheckState: ReadEncoder(); return true;
                // ResetEncoder
                case kModuleCommands::kReset: ResetEncoder(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            // Since Encoder class carries out the necessary hardware setup, this method re-initializes the encoder to
            // repeat the hardware setup.
            _encoder.write(0);

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.report_CCW      = true;  // Defaults to report rotation in the CCW direction.
            _custom_parameters.report_CW       = true;  // Defaults to report rotation in the CW direction.
            _custom_parameters.delta_threshold = 1;     // By default, the minimum change magnitude to report is 1.

            return true;
        }

        ~EncoderModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                bool report_CCW          = true;  ///< Determines whether to report changes in the CCW direction.
                bool report_CW           = false;  ///< Determines whether to report changes in the CW direction.
                uint32_t delta_threshold = 10;  ///< Sets the minimum pulse count change (delta) for reporting changes.
        } __attribute__((packed)) _custom_parameters;

        /// The encoder class that abstracts low-level access to the Encoder pins and provides an easy API to retrieve
        /// the automatically incremented encoder pulse vector. The vector can be reset by setting it to 0, and the
        /// class relies on hardware interrupt functionality to maintain the desired precision.
        Encoder _encoder = Encoder(kPinA, kPinB);

        /// The multiplier is used to optionally invert the pulse counter sign to virtually flip the direction of
        /// encoder readings. This is helpful if the encoder is mounted and wired in a way where CW rotation of the
        /// tracked object produces CCW readout in the encoder. In other words, this is used to virtually align the
        /// encoder readout direction to the tracked object rotation direction.
        static constexpr int32_t kMultiplier = kInvertDirection ? -1 : 1;  // NOLINT(*-dynamic-static-initializers)

        /// Reads and resets the current encoder pulse counter and sends the result to the PC if it is significantly
        /// different from previous readout. Note, the result will be transformed into an absolute value and its
        /// direction will be codified as the event code of the message transmitted to PC.
        void ReadEncoder()
        {
            // Retrieves and, if necessary, flips the value of the encoder. The value tracks the number of pulses
            // relative to the previous reset command or the initialization of the encoder.
            const int32_t flipped_value = _encoder.read() * kMultiplier;

            // If encoder has not moved since the last call to this method, returns without further processing.
            if (flipped_value == 0)
            {
                CompleteCommand();
                return;
            }

            // Converts the pulse count delta to an absolute value for the threshold checking below.
            auto delta = static_cast<uint32_t>(abs(flipped_value));

            // If the value is negative, this is interpreted as rotation in the Clockwise direction.
            // If reporting CW rotation is allowed, sends the pulse count to the PC as an absolute value, using the
            // transmitted event status code to indicate the direction of movement. Note, the delta is only sent if
            // it is greater than or equal to the readout threshold value.
            if (flipped_value < 0 && _custom_parameters.report_CW && delta >= _custom_parameters.delta_threshold)
            {
                SendData(
                    static_cast<uint8_t>(kCustomStatusCodes::kRotatedCW),
                    communication_assets::kPrototypes::kOneUnsignedLong,
                    delta
                );
            }

            // If the value is positive, this is interpreted as the CCW movement direction.
            // Same as above, if reporting the CCW movement is allowed and the delta is greater than or equal to
            // the readout threshold, sends the data to the PC.
            else if (_custom_parameters.report_CCW && delta >= _custom_parameters.delta_threshold)
            {
                SendData(
                    static_cast<uint8_t>(kCustomStatusCodes::kRotatedCCW),
                    communication_assets::kPrototypes::kOneUnsignedLong,
                    delta
                );
            }

            // Completes the command execution
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
