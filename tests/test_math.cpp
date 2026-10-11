#include <gtest/gtest.h>
#include "surface_controller/math/Vector3.hpp"
#include "surface_controller/math/MathUtils.hpp"
#include "surface_controller/math/Filter.hpp"

using namespace surface_controller::math;

TEST(MathTest, Vector3Operations) {
    Vector3f a{1.0f, 2.0f, 3.0f};
    Vector3f b{4.0f, 5.0f, 6.0f};

    Vector3f sum = a + b;
    EXPECT_FLOAT_EQ(sum.x, 5.0f);
    EXPECT_FLOAT_EQ(sum.y, 7.0f);
    EXPECT_FLOAT_EQ(sum.z, 9.0f);

    Vector3f diff = b - a;
    EXPECT_FLOAT_EQ(diff.x, 3.0f);
    EXPECT_FLOAT_EQ(diff.y, 3.0f);
    EXPECT_FLOAT_EQ(diff.z, 3.0f);

    float dot = a.dot(b);
    EXPECT_FLOAT_EQ(dot, 1.0f*4.0f + 2.0f*5.0f + 3.0f*6.0f); // 32.0f

    Vector3f c = a.cross(b);
    EXPECT_FLOAT_EQ(c.x, -3.0f);
    EXPECT_FLOAT_EQ(c.y, 6.0f);
    EXPECT_FLOAT_EQ(c.z, -3.0f);

    Vector3f v{3.0f, 4.0f, 0.0f};
    EXPECT_FLOAT_EQ(v.norm(), 5.0f);

    Vector3f v_norm = v.normalized();
    EXPECT_FLOAT_EQ(v_norm.x, 0.6f);
    EXPECT_FLOAT_EQ(v_norm.y, 0.8f);
    EXPECT_FLOAT_EQ(v_norm.z, 0.0f);
}

TEST(MathTest, MathUtilsFunctions) {
    EXPECT_EQ(clamp(5, 0, 10), 5);
    EXPECT_EQ(clamp(-2, 0, 10), 0);
    EXPECT_EQ(clamp(15, 0, 10), 10);

    EXPECT_FLOAT_EQ(deadband(0.05f, 0.1f), 0.0f);
    EXPECT_FLOAT_EQ(deadband(0.25f, 0.1f), 0.15f);
    EXPECT_FLOAT_EQ(deadband(-0.25f, 0.1f), -0.15f);

    EXPECT_NEAR(mapRange(0.5f, 0.0f, 1.0f, 1000.0f, 2000.0f), 1500.0f, 1e-4);
    EXPECT_NEAR(degToRad(180.0f), PI, 1e-5);
    EXPECT_NEAR(radToDeg(PI), 180.0f, 1e-4);
}

TEST(FilterTest, LowPassFilterAttenuatesStep) {
    LowPassFilter lpf;
    lpf.configure(5.0f, 100.0f); // 5 Hz cutoff, 100 Hz sample rate

    // First sample initializes
    float out1 = lpf.update(1.0f);
    EXPECT_FLOAT_EQ(out1, 1.0f);

    // Step to 2.0f
    float out2 = lpf.update(2.0f);
    EXPECT_GT(out2, 1.0f);
    EXPECT_LT(out2, 2.0f);
}

TEST(FilterTest, MedianFilterRejectsImpulsiveSpike) {
    MedianFilter3 mf;

    EXPECT_FLOAT_EQ(mf.update(10.0f), 10.0f);
    EXPECT_FLOAT_EQ(mf.update(10.0f), 10.0f);

    // Transient spike of 100.0f injected!
    float filtered = mf.update(100.0f);
    // Median of {10.0, 10.0, 100.0} is 10.0! Spike completely rejected!
    EXPECT_FLOAT_EQ(filtered, 10.0f);
}
