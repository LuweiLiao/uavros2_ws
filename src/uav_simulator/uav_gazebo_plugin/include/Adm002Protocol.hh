#ifndef UAV_GAZEBO_PLUGIN_ADM002_PROTOCOL_HH_
#define UAV_GAZEBO_PLUGIN_ADM002_PROTOCOL_HH_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace uav_gazebo_plugin {

class Adm002Protocol {
public:
    static constexpr std::size_t kEnableCommandLength = 5;
    static constexpr std::size_t kEnableAckLength = 3;
    static constexpr std::size_t kStreamFrameLength = 5;

    using EnableAck = std::array<uint8_t, kEnableAckLength>;
    using StreamFrame = std::array<uint8_t, kStreamFrameLength>;

    static uint8_t AdditiveChecksum(const uint8_t* data, std::size_t length)
    {
        uint8_t checksum = 0;
        for (std::size_t i = 0; i < length; ++i) {
            checksum = static_cast<uint8_t>(checksum + data[i]);
        }
        return checksum;
    }

    static bool IsEnableStreamCommand(const uint8_t* data, std::size_t length,
                                      uint8_t device_address = 1)
    {
        return data != nullptr &&
               length == kEnableCommandLength &&
               data[0] == device_address &&
               data[1] == 0x28 &&
               data[2] == 0x01 &&
               data[3] == 0x01 &&
               data[4] == AdditiveChecksum(data, kEnableCommandLength - 1);
    }

    static EnableAck EncodeEnableAck(uint8_t device_address = 1)
    {
        EnableAck ack{{device_address, 0x29, 0}};
        ack[2] = AdditiveChecksum(ack.data(), ack.size() - 1);
        return ack;
    }

    static StreamFrame EncodeForceNewton(double force_n, double force_scale = 1.0)
    {
        constexpr double kNewtonToGramForce = 1000.0 / 9.80665;
        constexpr int32_t kMaximumMagnitudeGram = 0xFFFFFF;
        constexpr uint8_t kStatusPositive = 1U << 0;
        constexpr uint8_t kStatusStable = 1U << 1;

        double gram_force = force_n * kNewtonToGramForce * force_scale;
        if (!std::isfinite(gram_force)) {
            gram_force = 0.0;
        }
        if (gram_force > kMaximumMagnitudeGram) {
            gram_force = kMaximumMagnitudeGram;
        } else if (gram_force < -kMaximumMagnitudeGram) {
            gram_force = -kMaximumMagnitudeGram;
        }

        const int32_t signed_weight_g = static_cast<int32_t>(std::llround(gram_force));
        const uint32_t magnitude_g = signed_weight_g < 0 ?
            static_cast<uint32_t>(-static_cast<int64_t>(signed_weight_g)) :
            static_cast<uint32_t>(signed_weight_g);

        StreamFrame frame{{
            static_cast<uint8_t>(kStatusStable | (signed_weight_g >= 0 ? kStatusPositive : 0)),
            static_cast<uint8_t>(magnitude_g >> 16),
            static_cast<uint8_t>(magnitude_g >> 8),
            static_cast<uint8_t>(magnitude_g),
            0,
        }};
        frame[4] = AdditiveChecksum(frame.data(), frame.size() - 1);
        return frame;
    }
};

}  // namespace uav_gazebo_plugin

#endif
