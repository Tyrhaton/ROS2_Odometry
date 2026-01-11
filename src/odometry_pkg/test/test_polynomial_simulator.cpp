/**
 * @file test_polynomial_simulator.cpp
 * @brief Unit tests for polynomial sensor data simulator
 *
 * Tests verify that the polynomial simulator works according to Assignment 4 specifications:
 * - CONSTANT polynomial: f(t) = c
 * - LINEAR polynomial: f(t) = a*t + b
 * - QUADRATIC polynomial: f(t) = a*t^2 + b*t + c
 * - Default value 0.0 outside defined intervals
 */

#include <gtest/gtest.h>
#include <cmath>

/**
 * @enum PolynomialType
 * @brief Types of polynomial functions for sensor data simulation
 */
enum class PolynomialType {
    CONSTANT,  ///< Constant value: f(t) = c
    LINEAR,    ///< Linear function: f(t) = a*t + b
    QUADRATIC  ///< Quadratic function: f(t) = a*t^2 + b*t + c
};

/**
 * @struct PolynomialInterval
 * @brief Defines a time interval with a polynomial function
 */
struct PolynomialInterval {
    double start_time;      ///< Start time of interval (seconds)
    double end_time;        ///< End time of interval (seconds)
    PolynomialType type;    ///< Type of polynomial
    double a;               ///< Coefficient a (quadratic or linear term)
    double b;               ///< Coefficient b (linear term or constant for LINEAR)
    double c;               ///< Coefficient c (constant term for QUADRATIC)

    /**
     * @brief Evaluate polynomial at time t
     */
    double evaluate(double t) const {
        switch (type) {
            case PolynomialType::CONSTANT:
                return c;
            case PolynomialType::LINEAR:
                return a * t + b;
            case PolynomialType::QUADRATIC:
                return a * t * t + b * t + c;
        }
        return 0.0;
    }
};

/**
 * @class PolynomialFunction
 * @brief Manages multiple polynomial intervals for sensor data simulation
 */
class PolynomialFunction {
private:
    std::vector<PolynomialInterval> intervals;

public:
    void add_interval(const PolynomialInterval& interval) {
        intervals.push_back(interval);
    }

    double get_value(double t) const {
        for (const auto& interval : intervals) {
            if (t >= interval.start_time && t <= interval.end_time) {
                return interval.evaluate(t);
            }
        }
        return 0.0;  // Default value outside defined intervals
    }
};

//
// TEST SUITE: Constant Polynomial
//

TEST(PolynomialSimulatorTest, ConstantPolynomial_InsideInterval) {
    PolynomialFunction func;
    PolynomialInterval interval;
    interval.start_time = 0.0;
    interval.end_time = 10.0;
    interval.type = PolynomialType::CONSTANT;
    interval.c = 5.5;
    func.add_interval(interval);

    // Test at different points inside interval
    EXPECT_DOUBLE_EQ(func.get_value(0.0), 5.5);
    EXPECT_DOUBLE_EQ(func.get_value(5.0), 5.5);
    EXPECT_DOUBLE_EQ(func.get_value(10.0), 5.5);
}

TEST(PolynomialSimulatorTest, ConstantPolynomial_OutsideInterval) {
    PolynomialFunction func;
    PolynomialInterval interval;
    interval.start_time = 5.0;
    interval.end_time = 15.0;
    interval.type = PolynomialType::CONSTANT;
    interval.c = 3.2;
    func.add_interval(interval);

    // Test outside interval - should return 0.0
    EXPECT_DOUBLE_EQ(func.get_value(0.0), 0.0);
    EXPECT_DOUBLE_EQ(func.get_value(4.9), 0.0);
    EXPECT_DOUBLE_EQ(func.get_value(15.1), 0.0);
    EXPECT_DOUBLE_EQ(func.get_value(100.0), 0.0);
}

//
// TEST SUITE: Linear Polynomial
//

TEST(PolynomialSimulatorTest, LinearPolynomial_CorrectValues) {
    PolynomialFunction func;
    PolynomialInterval interval;
    interval.start_time = 0.0;
    interval.end_time = 10.0;
    interval.type = PolynomialType::LINEAR;
    interval.a = 2.0;  // slope
    interval.b = 1.0;  // intercept
    func.add_interval(interval);

    // f(t) = 2t + 1
    EXPECT_DOUBLE_EQ(func.get_value(0.0), 1.0);   // 2*0 + 1 = 1
    EXPECT_DOUBLE_EQ(func.get_value(1.0), 3.0);   // 2*1 + 1 = 3
    EXPECT_DOUBLE_EQ(func.get_value(5.0), 11.0);  // 2*5 + 1 = 11
    EXPECT_DOUBLE_EQ(func.get_value(10.0), 21.0); // 2*10 + 1 = 21
}

TEST(PolynomialSimulatorTest, LinearPolynomial_NegativeSlope) {
    PolynomialFunction func;
    PolynomialInterval interval;
    interval.start_time = 0.0;
    interval.end_time = 10.0;
    interval.type = PolynomialType::LINEAR;
    interval.a = -1.5;  // negative slope
    interval.b = 10.0;  // intercept
    func.add_interval(interval);

    // f(t) = -1.5t + 10
    EXPECT_DOUBLE_EQ(func.get_value(0.0), 10.0);   // -1.5*0 + 10 = 10
    EXPECT_DOUBLE_EQ(func.get_value(2.0), 7.0);    // -1.5*2 + 10 = 7
    EXPECT_DOUBLE_EQ(func.get_value(6.0), 1.0);    // -1.5*6 + 10 = 1
}

TEST(PolynomialSimulatorTest, LinearPolynomial_OutsideInterval) {
    PolynomialFunction func;
    PolynomialInterval interval;
    interval.start_time = 10.0;
    interval.end_time = 20.0;
    interval.type = PolynomialType::LINEAR;
    interval.a = 1.0;
    interval.b = 0.0;
    func.add_interval(interval);

    // Test outside interval - should return 0.0
    EXPECT_DOUBLE_EQ(func.get_value(0.0), 0.0);
    EXPECT_DOUBLE_EQ(func.get_value(9.9), 0.0);
    EXPECT_DOUBLE_EQ(func.get_value(20.1), 0.0);
}

//
// TEST SUITE: Quadratic Polynomial
//

TEST(PolynomialSimulatorTest, QuadraticPolynomial_CorrectValues) {
    PolynomialFunction func;
    PolynomialInterval interval;
    interval.start_time = 0.0;
    interval.end_time = 10.0;
    interval.type = PolynomialType::QUADRATIC;
    interval.a = 1.0;  // quadratic coefficient
    interval.b = 2.0;  // linear coefficient
    interval.c = 3.0;  // constant
    func.add_interval(interval);

    // f(t) = t^2 + 2t + 3
    EXPECT_DOUBLE_EQ(func.get_value(0.0), 3.0);    // 0^2 + 2*0 + 3 = 3
    EXPECT_DOUBLE_EQ(func.get_value(1.0), 6.0);    // 1^2 + 2*1 + 3 = 6
    EXPECT_DOUBLE_EQ(func.get_value(2.0), 11.0);   // 2^2 + 2*2 + 3 = 11
    EXPECT_DOUBLE_EQ(func.get_value(5.0), 38.0);   // 5^2 + 2*5 + 3 = 38
}

TEST(PolynomialSimulatorTest, QuadraticPolynomial_NegativeCoefficients) {
    PolynomialFunction func;
    PolynomialInterval interval;
    interval.start_time = 0.0;
    interval.end_time = 10.0;
    interval.type = PolynomialType::QUADRATIC;
    interval.a = -0.5;  // negative quadratic (parabola opening downward)
    interval.b = 5.0;   // positive linear
    interval.c = 0.0;   // no constant
    func.add_interval(interval);

    // f(t) = -0.5t^2 + 5t
    EXPECT_DOUBLE_EQ(func.get_value(0.0), 0.0);     // -0.5*0^2 + 5*0 = 0
    EXPECT_DOUBLE_EQ(func.get_value(2.0), 8.0);     // -0.5*4 + 10 = 8
    EXPECT_DOUBLE_EQ(func.get_value(5.0), 12.5);    // -0.5*25 + 25 = 12.5
    EXPECT_DOUBLE_EQ(func.get_value(10.0), 0.0);    // -0.5*100 + 50 = 0
}

TEST(PolynomialSimulatorTest, QuadraticPolynomial_OutsideInterval) {
    PolynomialFunction func;
    PolynomialInterval interval;
    interval.start_time = 5.0;
    interval.end_time = 15.0;
    interval.type = PolynomialType::QUADRATIC;
    interval.a = 1.0;
    interval.b = 0.0;
    interval.c = 0.0;
    func.add_interval(interval);

    // Test outside interval - should return 0.0
    EXPECT_DOUBLE_EQ(func.get_value(0.0), 0.0);
    EXPECT_DOUBLE_EQ(func.get_value(4.9), 0.0);
    EXPECT_DOUBLE_EQ(func.get_value(15.1), 0.0);
}

//
// TEST SUITE: Multiple Intervals (Assignment Requirement)
//

TEST(PolynomialSimulatorTest, MultipleIntervals_Sequential) {
    PolynomialFunction func;

    // Interval 1: 0-10s, constant 5.0
    PolynomialInterval interval1;
    interval1.start_time = 0.0;
    interval1.end_time = 10.0;
    interval1.type = PolynomialType::CONSTANT;
    interval1.c = 5.0;
    func.add_interval(interval1);

    // Interval 2: 10-20s, linear 2t + 0
    PolynomialInterval interval2;
    interval2.start_time = 10.0;
    interval2.end_time = 20.0;
    interval2.type = PolynomialType::LINEAR;
    interval2.a = 2.0;
    interval2.b = 0.0;
    func.add_interval(interval2);

    // Interval 3: 20-30s, quadratic t^2
    PolynomialInterval interval3;
    interval3.start_time = 20.0;
    interval3.end_time = 30.0;
    interval3.type = PolynomialType::QUADRATIC;
    interval3.a = 1.0;
    interval3.b = 0.0;
    interval3.c = 0.0;
    func.add_interval(interval3);

    // Test each interval
    EXPECT_DOUBLE_EQ(func.get_value(5.0), 5.0);      // constant
    EXPECT_DOUBLE_EQ(func.get_value(15.0), 30.0);    // linear: 2*15
    EXPECT_DOUBLE_EQ(func.get_value(25.0), 625.0);   // quadratic: 25^2

    // Test boundaries
    EXPECT_DOUBLE_EQ(func.get_value(10.0), 5.0);     // end of interval 1
    EXPECT_DOUBLE_EQ(func.get_value(20.0), 40.0);    // end of interval 2

    // Test outside all intervals
    EXPECT_DOUBLE_EQ(func.get_value(-1.0), 0.0);
    EXPECT_DOUBLE_EQ(func.get_value(35.0), 0.0);
}

TEST(PolynomialSimulatorTest, MultipleIntervals_WithGaps) {
    PolynomialFunction func;

    // Interval 1: 0-5s
    PolynomialInterval interval1;
    interval1.start_time = 0.0;
    interval1.end_time = 5.0;
    interval1.type = PolynomialType::CONSTANT;
    interval1.c = 10.0;
    func.add_interval(interval1);

    // Gap: 5-10s (should return 0.0)

    // Interval 2: 10-15s
    PolynomialInterval interval2;
    interval2.start_time = 10.0;
    interval2.end_time = 15.0;
    interval2.type = PolynomialType::CONSTANT;
    interval2.c = 20.0;
    func.add_interval(interval2);

    // Test inside intervals
    EXPECT_DOUBLE_EQ(func.get_value(2.0), 10.0);
    EXPECT_DOUBLE_EQ(func.get_value(12.0), 20.0);

    // Test in gap - should return 0.0 (assignment requirement)
    EXPECT_DOUBLE_EQ(func.get_value(7.0), 0.0);
    EXPECT_DOUBLE_EQ(func.get_value(9.9), 0.0);
}

//
// TEST SUITE: Assignment-Specific Scenarios
//

TEST(PolynomialSimulatorTest, Assignment_AccelerationProfile) {
    // Simulate the acceleration profile from the assignment PDF
    PolynomialFunction accel_x;

    // Circular motion: 0-50s
    PolynomialInterval circular;
    circular.start_time = 0.0;
    circular.end_time = 50.0;
    circular.type = PolynomialType::LINEAR;
    circular.a = 0.0;
    circular.b = -0.5;  // constant acceleration for circular motion
    accel_x.add_interval(circular);

    // Straight line: 50-100s
    PolynomialInterval straight;
    straight.start_time = 50.0;
    straight.end_time = 100.0;
    straight.type = PolynomialType::CONSTANT;
    straight.c = 0.0;  // no acceleration
    accel_x.add_interval(straight);

    // Verify values
    EXPECT_DOUBLE_EQ(accel_x.get_value(25.0), -0.5);   // during circular motion
    EXPECT_DOUBLE_EQ(accel_x.get_value(75.0), 0.0);    // during straight line
    EXPECT_DOUBLE_EQ(accel_x.get_value(110.0), 0.0);   // after all intervals
}

TEST(PolynomialSimulatorTest, Assignment_BoundaryConditions) {
    PolynomialFunction func;

    PolynomialInterval interval;
    interval.start_time = 10.0;
    interval.end_time = 20.0;
    interval.type = PolynomialType::LINEAR;
    interval.a = 1.0;
    interval.b = 0.0;
    func.add_interval(interval);

    // Test exact boundaries (inclusive per assignment)
    EXPECT_DOUBLE_EQ(func.get_value(10.0), 10.0);  // start_time is inclusive
    EXPECT_DOUBLE_EQ(func.get_value(20.0), 20.0);  // end_time is inclusive

    // Just outside boundaries
    EXPECT_DOUBLE_EQ(func.get_value(9.999), 0.0);
    EXPECT_DOUBLE_EQ(func.get_value(20.001), 0.0);
}

//
// Main function
//

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
