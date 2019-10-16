#include "../ubxmessage.hpp"
// Bring in gtest
#include <gtest/gtest.h>
#include <cmath>

// Declare a test
TEST(ublox_test, test_read_write_payload_8bit)
{
    UbxMessage msg(UbxMessage::CLASS_CFG, UbxMessage::MSG_VAL_SET, false);

    msg.write_payload<uint8_t>(0, 0xab);
    msg.write_payload<int8_t>(1, -22);

    EXPECT_EQ(msg.get_payload_size(), 2u);
    EXPECT_EQ(0xab, msg.read_payload<uint8_t>(0));
    EXPECT_EQ(-22, msg.read_payload<int8_t>(1));
}

TEST(ublox_test, test_read_write_payload_16bit)
{
    UbxMessage msg(UbxMessage::CLASS_CFG, UbxMessage::MSG_VAL_SET, false);

    msg.write_payload<uint16_t>(0, 0xdefa);
    msg.write_payload<int16_t>(2, -5129);

    EXPECT_EQ(msg.get_payload_size(), 4u);
    EXPECT_EQ(0xdefa, msg.read_payload<uint16_t>(0));
    EXPECT_EQ(-5129, msg.read_payload<int16_t>(2));
}

TEST(ublox_test, test_read_write_payload_32bit)
{
    UbxMessage msg(UbxMessage::CLASS_CFG, UbxMessage::MSG_VAL_SET, false);

    msg.write_payload<uint32_t>(0, 0x0badf00d);
    msg.write_payload<int32_t>(4, -0xbeefee);

    EXPECT_EQ(msg.get_payload_size(), 8u);
    EXPECT_EQ(0x0badf00du, msg.read_payload<uint32_t>(0));
    EXPECT_EQ(-0xbeefee, msg.read_payload<int32_t>(4));
}

TEST(ublox_test, test_read_write_payload_float32)
{
    UbxMessage msg(UbxMessage::CLASS_CFG, UbxMessage::MSG_VAL_SET, false);

    msg.write_payload<float>(0, M_PI);

    EXPECT_EQ(msg.get_payload_size(), 4u);
    EXPECT_FLOAT_EQ(M_PI, msg.read_payload<float>(0));
}

TEST(ublox_test, test_read_write_payload_float64)
{
    UbxMessage msg(UbxMessage::CLASS_CFG, UbxMessage::MSG_VAL_SET, false);

    double expected = M_PI * 0.00001f;

    msg.write_payload<double>(0, expected);

    EXPECT_EQ(msg.get_payload_size(), 8u);
    EXPECT_FLOAT_EQ(expected, msg.read_payload<double>(0));
}

TEST(ublox_test, test_auto_payload_resize)
{
    UbxMessage msg(UbxMessage::CLASS_CFG, UbxMessage::MSG_VAL_SET, false);

    msg.write_payload<uint16_t>(0, 0);
    EXPECT_EQ(msg.get_payload_size(), 2u);

    msg.write_payload<uint32_t>(4, 0);
    EXPECT_EQ(msg.get_payload_size(), 8u);

    msg.write_payload<uint8_t>(12, 0);
    EXPECT_EQ(msg.get_payload_size(), 13u);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
