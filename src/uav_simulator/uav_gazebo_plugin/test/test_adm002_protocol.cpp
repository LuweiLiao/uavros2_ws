#include "Adm002Protocol.hh"

#include <gtest/gtest.h>

using uav_gazebo_plugin::Adm002Protocol;

TEST(Adm002Protocol, AcceptsDocumentedEnableCommand)
{
    const uint8_t command[] {0x01, 0x28, 0x01, 0x01, 0x2B};
    EXPECT_TRUE(Adm002Protocol::IsEnableStreamCommand(command, sizeof(command)));
}

TEST(Adm002Protocol, RejectsBadEnableChecksum)
{
    const uint8_t command[] {0x01, 0x28, 0x01, 0x01, 0x00};
    EXPECT_FALSE(Adm002Protocol::IsEnableStreamCommand(command, sizeof(command)));
}

TEST(Adm002Protocol, EncodesHighSpeedAck)
{
    const auto ack = Adm002Protocol::EncodeEnableAck();
    const Adm002Protocol::EnableAck expected{{0x01, 0x29, 0x2A}};
    EXPECT_EQ(expected, ack);
}

TEST(Adm002Protocol, EncodesPositiveOneNewtonWithGramQuantization)
{
    const auto frame = Adm002Protocol::EncodeForceNewton(1.0);
    EXPECT_EQ(0x03, frame[0]);
    EXPECT_EQ(0x00, frame[1]);
    EXPECT_EQ(0x00, frame[2]);
    EXPECT_EQ(102, frame[3]);
    EXPECT_EQ(Adm002Protocol::AdditiveChecksum(frame.data(), 4), frame[4]);
}

TEST(Adm002Protocol, EncodesNegativeForceSign)
{
    const auto frame = Adm002Protocol::EncodeForceNewton(-1.0);
    EXPECT_EQ(0x02, frame[0]);
    EXPECT_EQ(102, frame[3]);
    EXPECT_EQ(Adm002Protocol::AdditiveChecksum(frame.data(), 4), frame[4]);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
