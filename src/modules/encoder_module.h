/**
 * @file
 * @brief The header-only file for the EncoderModule class. This class allows interfacing with a quadrature rotary
 * encoder to monitor the direction and magnitude of connected object's rotation
 *
 * @attention This file is written in a way that is @b NOT compatible with any other library or class that uses
 * AttachInterrupt(). Disable the 'ENCODER_USE_INTERRUPTS' macro defined at the top of the file to make this file
 * compatible with other interrupt libraries.
 *
 * @section enc_mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - Encoder.h for the low-level API that detects and tracks quadrature encoder pulses using interrupt pins.
 * - module.h for the shared Module class API access (integrates the custom module into runtime flow).
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
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
 * @brief Wraps an Encoder class instance and provides access to its pulse counter to monitor the direction and
 * magnitude of connected object's rotation.
 *
 * This module is specifically designed to interface with quadrature encoders and track the direction and number of
 * encoder pulses between class method calls. The class only works with encoder pulses and expects the ModuleInterface
 * class that receives the data on the PC to perform the necessary conversions to translate pulses into distance.
 *
 * @note Largely, this class is an API wrapper around the Paul Stoffregen's Encoder library, and it relies on efficient
 * interrupt logic to increase encoder tracking precision. To achieve the best performance, make sure both Pin A and
 * Pin B are hardware interrupt pins.
 *
 * @tparam kPinA the digital interrupt pin connected to the 'A' channel of the quadrature encoder.
 * @tparam kPinB the digital interrupt pin connected to the 'B' channel of the quadrature encoder.
 * @tparam kPinX the digital pin connected to the index ('X') channel of the quadrature encoder.
 * @tparam kInvertDirection if true, inverts the sign of the value returned by the encoder. By default, when
 * Pin B is triggered before Pin A, the pulse counter decreases, which corresponds to CW movement. When pin A is
 * triggered before pin B, the counter increases, which corresponds to the CCW movement. This flag allows reversing
 * this relationship, which may be helpful, depending on how the encoder is mounted and wired.
 */
template <const uint8_t kPinA, const uint8_t kPinB, const uint8_t kPinX, const bool kInvertDirection = false>
class EncoderModule final : public Module
{
        // Ensures that the encoder pins are different.
        static_assert(kPinA != kPinB, "EncoderModule PinA and PinB cannot be the same!");
        static_assert(kPinA != kPinX, "EncoderModule PinA and PinX cannot be the same!");
        static_assert(kPinB != kPinX, "EncoderModule PinB and PinX cannot be the same!");

        // Also ensures that encoder pins do not interfere with LED pin.
        static_assert(
            kPinA != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different Channel A pin for EncoderModule "
            "instance."
        );
        static_assert(
            kPinB != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different Channel B pin for EncoderModule "
            "instance."
        );
        static_assert(
            kPinX != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different Index (X) pin for EncoderModule "
            "instance."
        );

    public:
        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kRotatedCCW = 51,  ///< The encoder was rotated in the CCW direction.
            kRotatedCW  = 52,  ///< The encoder was rotated in the CW.
            kPPR        = 53,  ///< The estimated Pulse-Per-Revolution (PPR) value of the encoder.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kCheckState = 1,  ///< Gets the change in pulse counts and sign relative to last check.
            kReset      = 2,  ///< Resets the encoder's pulse tracker to 0.
            kGetPPR     = 3,  ///< Estimates the Pulse-Per-Revolution (PPR) of the encoder.
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
                // GetPPR
                case kModuleCommands::kGetPPR: GetPPR(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            // Since Encoder class does not manage Index pin, it has to be set to INPUT here
            pinMode(kPinX, INPUT);

            // Re-initializing the encoder class leads to runtime errors and is not really needed. Therefore,
            // instead of resetting the encoder hardware via Encoder re-initialization, the setup only resets the
            // pulse counter. The hardware is statically configured during Encoder class instantiation.
            _encoder.write(0);

            // Resets the overflow tracker
            _overflow = 0;

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.report_CCW      = true;
            _custom_parameters.report_CW       = true;
            _custom_parameters.delta_threshold = 20;

            return true;
        }

        ~EncoderModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                bool report_CCW          = true;  ///< Determines whether to report changes in the CCW direction.
                bool report_CW           = true;  ///< Determines whether to report changes in the CW direction.
                uint32_t delta_threshold = 20;    ///< The minimum pulse count change (delta) for reporting changes.
        } PACKED_STRUCT _custom_parameters;

        /// The encoder class that abstracts low-level access to the Encoder pins and provides an easy API to retrieve
        /// the automatically incremented encoder pulse vector. The vector can be reset by setting it to 0, and the
        /// class relies on hardware interrupt functionality to maintain the desired precision.
        Encoder _encoder = Encoder(kPinA, kPinB);  // HAS to be initialized statically or the runtime crashes!

        /// The multiplier is used to optionally invert the pulse counter sign to virtually flip the direction of
        /// encoder readouts. This is helpful if the encoder is mounted and wired in a way where CW rotation of the
        /// tracked object produces CCW readout in the encoder. This is used to virtually align the encoder readout
        /// direction with the tracked object rotation direction.
        static constexpr int32_t kMultiplier = kInvertDirection ? -1 : 1;  // NOLINT(*-dynamic-static-initializers)

        /// This variable is used to accumulate insignificant encoder readouts to be reused during further calls.
        /// This allows filtering the rotation jitter of the tracked object while still accurately tracking small,
        /// incremental movements.
        int32_t _overflow = 0;

        /// Reads and resets the current encoder pulse counter. Sends the result to the PC if it is significantly
        /// different from previous readout. Note, the result will be transformed into an absolute value and its
        /// direction will be codified as the event code of the message transmitted to PC.
        void ReadEncoder()
        {
            // Retrieves and, if necessary, flips the value of the encoder. The value tracks the number of pulses
            // relative to the previous reset command or the initialization of the encoder. Resets the encoder
            // to 0 at each readout.
            const int32_t new_motion = _encoder.readAndReset() * kMultiplier;

            // Uses the current delta_threshold to calculate positive and negative amortization. Amortization allows the
            // overflow to store pulses in the non-reported direction, up to the delta_threshold limit. For example, if
            // CW motion is not allowed, the overflow will be allowed to drop up to at most delta_threshold negative
            // value. This is used to correctly counter (amortize) small, insignificant movements. For example, if the
            // object is locked, it may still move slightly in either direction. Without amortization, the overflow will
            // eventually accumulate small forward jitters and report forward movement, although there is none. With
            // amortization, the positive movement is likely to be counteracted by negative movement, resulting in a net
            // 0 movement. The amortization has to be kept low, however, because if unbounded, the _overflow will
            // accumulate movement in the non-reported direction, which has to be 'canceled' by movement in the reported
            // direction before anything is actually sent to the PC. This accumulated 'inertia' is undesirable and has
            // to be avoided. The best way to use this class is to set delta_threshold to a comfortably small value that
            // allows for adequate amortization of jitter, somewhere ~10-30 pulses.
            const auto positive_amortization    = static_cast<int32_t>(_custom_parameters.delta_threshold);
            const int32_t negative_amortization = -positive_amortization;

            // If encoder has not moved since the last call to this method, returns without further processing.
            if (new_motion == 0)
            {
                CompleteCommand();
                return;
            }

            // If the readout is not 0, uses its sign to determine whether to save or discard this motion.
            if ((new_motion < 0 && _custom_parameters.report_CW) || (new_motion > 0 && _custom_parameters.report_CCW))
            {
                // If the motion was in the direction that is reported, combines the new motion with the accumulated
                // motion stored in the overflow tracker.
                _overflow += new_motion;
            }
            else if (new_motion < 0)
            {
                // The motion in a non-reported CW direction is capped to negative amortization. This does not allow the
                // overflow to become too negative to accumulate inertia.
                _overflow += new_motion;
                _overflow = max(_overflow, negative_amortization);
            }
            else
            {
                // Same as above, but for CCW direction. This is capped to positive amortization.
                _overflow += new_motion;
                _overflow = min(_overflow, positive_amortization);
            }

            // Converts the pulse count delta to an absolute value for the threshold checking below.
            auto delta = static_cast<uint32_t>(abs(_overflow));

            // If the value is negative, this is interpreted as rotation in the Clockwise direction.
            // If the delta is greater than the reporting threshold, sends it to the PC.
            if (_overflow < 0 && delta > _custom_parameters.delta_threshold)
            {
                SendData(
                    static_cast<uint8_t>(kCustomStatusCodes::kRotatedCW),
                    communication_assets::kPrototypes::kOneUnsignedLong,
                    delta
                );
                _overflow = 0;  // Resets the overflow, as all tracked pulses have been 'consumed' and sent to the PC.
            }

            // If the value is positive, this is interpreted as the CCW movement direction.
            // Same as above, if the delta is greater than or equal to the readout threshold, sends the data to the PC.
            else if (_overflow > 0 && delta > _custom_parameters.delta_threshold)
            {
                SendData(
                    static_cast<uint8_t>(kCustomStatusCodes::kRotatedCCW),
                    communication_assets::kPrototypes::kOneUnsignedLong,
                    delta
                );
                _overflow = 0;  // Resets the overflow, as all tracked pulses have been 'consumed' and sent to the PC.
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

        /// Estimates the Pulse-Per-Revolution (PPR) of the encoder by using the index pin to precisely measure the
        /// number of pulses per encoder rotation. Measures up to 11 full rotations and averages the pulse counts per
        /// each rotation to improve the accuracy of the computed PPR value.
        void GetPPR()
        {
            // First, establishes the measurement baseline. Since the algorithm does not know the current position of
            // the encoder, waits until the index pin is triggered. This is used to establish the baseline for tracking
            // the pulses per rotation.
            while (!digitalReadFast(kPinX))
            {
            }
            _encoder.write(0);  // Resets the pulse tracker to 0

            // Measures 10 full rotations (indicated by index pin pulses). Resets the pulse tracker to 0 at each
            // measurement and does not care about the rotation direction.
            uint32_t pprs = 0;
            for (uint8_t i = 0; i < 10; ++i)
            {
                // Delays for 100 milliseconds to ensure the object spins past the range of index pin trigger
                delay(100);

                // Blocks until the index pin is triggered.
                while (!digitalReadFast(kPinX))
                {
                }

                // Accumulates the pulse counter into the summed value and reset the encoder each call.
                pprs += abs(_encoder.readAndReset());
            }

            // Computes the average PPR by using half-up rounding to get a whole number.
            // Currently, it is very unlikely to see a ppr > 10000, so casts the ppr down to uint16_t
            const auto average_ppr = static_cast<uint16_t>((pprs + 10 / 2) / 10);

            // Sends the average PPR count to the PC.
            SendData(
                static_cast<uint8_t>(kCustomStatusCodes::kPPR),
                communication_assets::kPrototypes::kOneUnsignedShort,
                average_ppr
            );

            // Completes the command execution
            CompleteCommand();
        }
};

#endif  //AXMC_ENCODER_MODULE_H
