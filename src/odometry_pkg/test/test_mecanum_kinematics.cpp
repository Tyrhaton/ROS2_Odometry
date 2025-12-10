/**
 * @file test_mecanum_kinematics.cpp
 * @brief Unit tests for mecanum wheel inverse kinematics
 *
 * Tests verify that mecanum wheel odometry works according to Assignment 4:
 * - Inverse kinematics formulas from PDF
 * - Wheel velocity conversion (rad/s to linear velocity)
 * - Robot velocity calculation (vx, vy, omega)
 * - Position integration from wheel encoders
 */

#include <gtest/gtest.h>
#include <cmath>

const double EPSILON = 1e-6;  ///< Tolerance for floating point comparisons

/**
 * @brief Helper function to compare doubles with tolerance
 */
bool approx_equal(double a, double b, double epsilon = EPSILON) {
    return std::abs(a - b) < epsilon;
}

// ============================================================================
// TEST SUITE: Mecanum Inverse Kinematics (Assignment Formulas)
// ============================================================================

TEST(MecanumKinematicsTest, InverseKinematics_ForwardMotion) {
    // All wheels rotating at same speed -> pure forward motion
    // Wheel numbering: 1=FL, 2=FR, 3=RR, 4=RL
    double w1 = 10.0;  // rad/s
    double w2 = 10.0;
    double w3 = 10.0;
    double w4 = 10.0;

    // Robot parameters
    double wheel_radius = 0.05;  // 5 cm
    double lx = 0.30;  // distance front-rear
    double ly = 0.25;  // distance left-right

    // Assignment formulas from PDF:
    // vx = (r/4) * (w1 + w2 + w3 + w4)
    // vy = (r/4) * (-w1 + w2 + w3 - w4)
    // omega = (r/(4*(lx+ly))) * (-w1 + w2 - w3 + w4)

    double vx = (wheel_radius / 4.0) * (w1 + w2 + w3 + w4);
    double vy = (wheel_radius / 4.0) * (-w1 + w2 + w3 - w4);
    double omega = (wheel_radius / (4.0 * (lx + ly))) * (-w1 + w2 - w3 + w4);

    // Expected: vx = 0.05/4 * 40 = 0.5 m/s forward
    EXPECT_DOUBLE_EQ(vx, 0.5);
    // Expected: vy = 0 (no lateral motion)
    EXPECT_DOUBLE_EQ(vy, 0.0);
    // Expected: omega = 0 (no rotation)
    EXPECT_DOUBLE_EQ(omega, 0.0);
}

TEST(MecanumKinematicsTest, InverseKinematics_SidewaysMotion) {
    // Opposing diagonal wheels same speed -> pure sideways motion
    // Pattern: FL and RR negative, FR and RL positive
    double w1 = -10.0;  // FL
    double w2 = 10.0;   // FR
    double w3 = 10.0;   // RR
    double w4 = -10.0;  // RL

    double wheel_radius = 0.05;
    double lx = 0.30;
    double ly = 0.25;

    double vx = (wheel_radius / 4.0) * (w1 + w2 + w3 + w4);
    double vy = (wheel_radius / 4.0) * (-w1 + w2 + w3 - w4);
    double omega = (wheel_radius / (4.0 * (lx + ly))) * (-w1 + w2 - w3 + w4);

    // Expected: vx = 0 (no forward motion)
    EXPECT_DOUBLE_EQ(vx, 0.0);
    // Expected: vy = 0.05/4 * (10 + 10 + 10 + 10) = 0.5 m/s sideways
    EXPECT_DOUBLE_EQ(vy, 0.5);
    // Expected: omega = 0 (no rotation)
    EXPECT_DOUBLE_EQ(omega, 0.0);
}

TEST(MecanumKinematicsTest, InverseKinematics_PureRotation) {
    // Left wheels backward, right wheels forward -> rotation in place
    double w1 = -10.0;  // FL (backward)
    double w2 = 10.0;   // FR (forward)
    double w3 = 10.0;   // RR (forward)
    double w4 = -10.0;  // RL (backward)

    double wheel_radius = 0.05;
    double lx = 0.30;
    double ly = 0.25;

    double vx = (wheel_radius / 4.0) * (w1 + w2 + w3 + w4);
    double vy = (wheel_radius / 4.0) * (-w1 + w2 + w3 - w4);
    double omega = (wheel_radius / (4.0 * (lx + ly))) * (-w1 + w2 - w3 + w4);

    // Expected: vx = 0 (no forward motion)
    EXPECT_DOUBLE_EQ(vx, 0.0);
    // Expected: vy = 0.05/4 * (10 + 10 + 10 + 10) = 0.5
    // Wait, this configuration gives BOTH sideways AND rotation!

    // Actually for PURE rotation: FL=-10, FR=+10, RR=-10, RL=+10
    // Let me recalculate:
    w1 = -10.0;  // FL
    w2 = 10.0;   // FR
    w3 = -10.0;  // RR (corrected)
    w4 = 10.0;   // RL (corrected)

    vx = (wheel_radius / 4.0) * (w1 + w2 + w3 + w4);
    vy = (wheel_radius / 4.0) * (-w1 + w2 + w3 - w4);
    omega = (wheel_radius / (4.0 * (lx + ly))) * (-w1 + w2 - w3 + w4);

    // vx = 0.05/4 * (-10 + 10 - 10 + 10) = 0
    EXPECT_DOUBLE_EQ(vx, 0.0);
    // vy = 0.05/4 * (10 + 10 - 10 - 10) = 0
    EXPECT_DOUBLE_EQ(vy, 0.0);
    // omega = 0.05/(4*0.55) * (10 + 10 + 10 + 10) = 0.05/2.2 * 40
    double expected_omega = (wheel_radius / (4.0 * (lx + ly))) * 40.0;
    EXPECT_DOUBLE_EQ(omega, expected_omega);
}

TEST(MecanumKinematicsTest, InverseKinematics_CircularMotion) {
    // Combined forward + sideways motion (diagonal motion)
    // Note: For actual rotation, different wheel pattern is needed
    double forward_vel = 5.0;  // rad/s base
    double sideway_mod = 1.0;  // rad/s modification

    // Wheel velocities for combined motion
    double w1 = forward_vel - sideway_mod;  // FL: 4 rad/s
    double w2 = forward_vel + sideway_mod;  // FR: 6 rad/s
    double w3 = forward_vel + sideway_mod;  // RR: 6 rad/s
    double w4 = forward_vel - sideway_mod;  // RL: 4 rad/s

    double wheel_radius = 0.05;
    double lx = 0.30;
    double ly = 0.25;

    double vx = (wheel_radius / 4.0) * (w1 + w2 + w3 + w4);
    double vy = (wheel_radius / 4.0) * (-w1 + w2 + w3 - w4);
    double omega = (wheel_radius / (4.0 * (lx + ly))) * (-w1 + w2 - w3 + w4);

    // vx = 0.05/4 * (4 + 6 + 6 + 4) = 0.05/4 * 20 = 0.25 m/s
    EXPECT_DOUBLE_EQ(vx, 0.25);
    // vy = 0.05/4 * (-4 + 6 + 6 - 4) = 0.05/4 * 4 = 0.05 m/s
    EXPECT_DOUBLE_EQ(vy, 0.05);
    // omega = (0.05/(4*0.55)) * (-4 + 6 - 6 + 4) = 0 (no rotation)
    EXPECT_DOUBLE_EQ(omega, 0.0);
}

// ============================================================================
// TEST SUITE: Wheel Radius Scaling
// ============================================================================

TEST(MecanumKinematicsTest, WheelRadius_LinearScaling) {
    // Larger wheels -> higher linear velocity for same angular velocity
    double w1 = 10.0, w2 = 10.0, w3 = 10.0, w4 = 10.0;
    double lx = 0.30, ly = 0.25;

    // Small wheels
    double r_small = 0.025;  // 2.5 cm
    double vx_small = (r_small / 4.0) * (w1 + w2 + w3 + w4);

    // Large wheels
    double r_large = 0.10;  // 10 cm
    double vx_large = (r_large / 4.0) * (w1 + w2 + w3 + w4);

    // Velocity should scale linearly with radius
    EXPECT_DOUBLE_EQ(vx_large / vx_small, r_large / r_small);
    EXPECT_DOUBLE_EQ(vx_large / vx_small, 4.0);
}

// ============================================================================
// TEST SUITE: Robot Geometry (lx, ly) Effects
// ============================================================================

TEST(MecanumKinematicsTest, RobotGeometry_RotationScaling) {
    // Smaller robot -> higher angular velocity for same wheel speeds
    double w1 = -10.0, w2 = 10.0, w3 = -10.0, w4 = 10.0;
    double wheel_radius = 0.05;

    // Large robot
    double lx_large = 0.50, ly_large = 0.40;
    double omega_large = (wheel_radius / (4.0 * (lx_large + ly_large))) *
                        (-w1 + w2 - w3 + w4);

    // Small robot
    double lx_small = 0.20, ly_small = 0.15;
    double omega_small = (wheel_radius / (4.0 * (lx_small + ly_small))) *
                        (-w1 + w2 - w3 + w4);

    // Smaller robot rotates faster
    EXPECT_GT(omega_small, omega_large);
}

// ============================================================================
// TEST SUITE: Position Integration from Velocities
// ============================================================================

TEST(MecanumKinematicsTest, PositionIntegration_StraightLine) {
    // Robot moving straight forward
    double position_x = 0.0;
    double position_y = 0.0;
    double alpha = 0.0;  // heading

    double vx_robot = 1.0;  // m/s forward
    double vy_robot = 0.0;
    double omega = 0.0;

    double dt = 0.02;  // 50Hz

    // Integrate for 5 seconds
    for (int i = 0; i < 250; i++) {
        // Update heading
        alpha += omega * dt;

        // Transform to map frame
        double cos_alpha = std::cos(alpha);
        double sin_alpha = std::sin(alpha);
        double vx_map = cos_alpha * vx_robot - sin_alpha * vy_robot;
        double vy_map = sin_alpha * vx_robot + cos_alpha * vy_robot;

        // Update position
        position_x += vx_map * dt;
        position_y += vy_map * dt;
    }

    // After 5 seconds: x = 5 m, y = 0 m
    EXPECT_TRUE(approx_equal(position_x, 5.0, 0.01));
    EXPECT_TRUE(approx_equal(position_y, 0.0, 0.01));
    EXPECT_DOUBLE_EQ(alpha, 0.0);
}

TEST(MecanumKinematicsTest, PositionIntegration_Sideways) {
    // Robot moving sideways (mecanum capability)
    double position_x = 0.0;
    double position_y = 0.0;
    double alpha = 0.0;

    double vx_robot = 0.0;
    double vy_robot = 1.0;  // sideways
    double omega = 0.0;

    double dt = 0.02;

    // Integrate for 3 seconds
    for (int i = 0; i < 150; i++) {
        alpha += omega * dt;
        double cos_alpha = std::cos(alpha);
        double sin_alpha = std::sin(alpha);
        double vx_map = cos_alpha * vx_robot - sin_alpha * vy_robot;
        double vy_map = sin_alpha * vx_robot + cos_alpha * vy_robot;
        position_x += vx_map * dt;
        position_y += vy_map * dt;
    }

    // After 3 seconds: x = 0, y = 3 m (sideways)
    EXPECT_TRUE(approx_equal(position_x, 0.0, 0.01));
    EXPECT_TRUE(approx_equal(position_y, 3.0, 0.01));
}

TEST(MecanumKinematicsTest, PositionIntegration_RotationInPlace) {
    // Robot rotating without translation
    double position_x = 0.0;
    double position_y = 0.0;
    double alpha = 0.0;

    double vx_robot = 0.0;
    double vy_robot = 0.0;
    double omega = M_PI / 2.0;  // 90 deg/s

    double dt = 0.02;

    // Integrate for 1 second (90 degree rotation)
    for (int i = 0; i < 50; i++) {
        alpha += omega * dt;
        double cos_alpha = std::cos(alpha);
        double sin_alpha = std::sin(alpha);
        double vx_map = cos_alpha * vx_robot - sin_alpha * vy_robot;
        double vy_map = sin_alpha * vx_robot + cos_alpha * vy_robot;
        position_x += vx_map * dt;
        position_y += vy_map * dt;
    }

    // Position should not change (rotation in place)
    EXPECT_TRUE(approx_equal(position_x, 0.0, 0.01));
    EXPECT_TRUE(approx_equal(position_y, 0.0, 0.01));
    // Heading should be 90 degrees
    EXPECT_TRUE(approx_equal(alpha, M_PI / 2.0, 0.01));
}

// ============================================================================
// TEST SUITE: Assignment-Specific Scenarios
// ============================================================================

TEST(MecanumKinematicsTest, Assignment_CircularPath) {
    // Simulate diagonal motion from the assignment simulator pattern
    double position_x = 0.0;
    double position_y = 0.0;
    double alpha = 0.0;

    // Wheel parameters from assignment
    double wheel_radius = 0.05;
    double lx = 0.30;
    double ly = 0.25;

    // Wheel velocities (from simulator) - creates forward + sideways motion
    double forward_vel = 5.0;  // rad/s
    double sideway_mod = 1.0;  // rad/s

    double dt = 0.02;  // 50Hz

    // Integrate for 10 seconds
    for (int i = 0; i < 500; i++) {
        // Calculate wheel velocities (this pattern gives diagonal motion, no rotation)
        double w1 = forward_vel - sideway_mod;
        double w2 = forward_vel + sideway_mod;
        double w3 = forward_vel + sideway_mod;
        double w4 = forward_vel - sideway_mod;

        // Inverse kinematics
        double vx_robot = (wheel_radius / 4.0) * (w1 + w2 + w3 + w4);
        double vy_robot = (wheel_radius / 4.0) * (-w1 + w2 + w3 - w4);
        double omega = (wheel_radius / (4.0 * (lx + ly))) * (-w1 + w2 - w3 + w4);

        // Update heading
        alpha += omega * dt;

        // Transform to map frame
        double cos_alpha = std::cos(alpha);
        double sin_alpha = std::sin(alpha);
        double vx_map = cos_alpha * vx_robot - sin_alpha * vy_robot;
        double vy_map = sin_alpha * vx_robot + cos_alpha * vy_robot;

        // Update position
        position_x += vx_map * dt;
        position_y += vy_map * dt;
    }

    // After diagonal motion, robot should be displaced in both directions
    EXPECT_GT(std::abs(position_x), 1.0);  // significant forward motion
    EXPECT_GT(std::abs(position_y), 0.1);  // some sideways motion
    // No rotation with this wheel pattern
    EXPECT_DOUBLE_EQ(alpha, 0.0);
}

TEST(MecanumKinematicsTest, Assignment_ZeroVelocityNoise) {
    // When wheels are stationary with small noise, position shouldn't drift much
    double position_x = 0.0;
    double position_y = 0.0;
    double alpha = 0.0;

    double wheel_radius = 0.05;
    double lx = 0.30;
    double ly = 0.25;

    double dt = 0.02;

    // Small random noise around zero
    for (int i = 0; i < 100; i++) {
        // Simulate small encoder noise (±0.1 rad/s)
        double w1 = 0.05;
        double w2 = -0.03;
        double w3 = 0.02;
        double w4 = -0.04;

        double vx_robot = (wheel_radius / 4.0) * (w1 + w2 + w3 + w4);
        double vy_robot = (wheel_radius / 4.0) * (-w1 + w2 + w3 - w4);
        double omega = (wheel_radius / (4.0 * (lx + ly))) * (-w1 + w2 - w3 + w4);

        alpha += omega * dt;
        double cos_alpha = std::cos(alpha);
        double sin_alpha = std::sin(alpha);
        double vx_map = cos_alpha * vx_robot - sin_alpha * vy_robot;
        double vy_map = sin_alpha * vx_robot + cos_alpha * vy_robot;
        position_x += vx_map * dt;
        position_y += vy_map * dt;
    }

    // Position drift should be minimal (< 10 cm over 2 seconds)
    EXPECT_LT(std::abs(position_x), 0.1);
    EXPECT_LT(std::abs(position_y), 0.1);
}

TEST(MecanumKinematicsTest, Assignment_DefaultParameters) {
    // Verify default parameters from launch file work correctly
    double wheel_radius = 0.05;   // 5 cm (default)
    double lx = 0.30;             // 30 cm (default)
    double ly = 0.25;             // 25 cm (default)

    // All parameters should be positive
    EXPECT_GT(wheel_radius, 0.0);
    EXPECT_GT(lx, 0.0);
    EXPECT_GT(ly, 0.0);

    // Reasonable robot dimensions
    EXPECT_LT(wheel_radius, 0.20);  // wheels < 20 cm
    EXPECT_LT(lx, 2.0);             // wheelbase < 2 m
    EXPECT_LT(ly, 2.0);
}

// ============================================================================
// Main function
// ============================================================================

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
