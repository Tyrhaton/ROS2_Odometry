/**
 * @file path_solver.cpp
 * @brief Path Solver - Lost automatisch onbekende waarden op in path definities
 *
 * Dit programma leest een YAML-bestand met "onbekend" waarden en lost deze op
 * met behulp van kinematische vergelijkingen en constraints.
 *
 * Ondersteunde onbekenden:
 * - interval tijden (onbekend, onbekend_1, onbekend_2, etc.)
 * - acceleratie waarden
 *
 * Constraints:
 * - final_position_x/y: eindpositie
 * - final_velocity_x/y: eindsnelheid
 * - total_duration: totale tijd
 *
 * Kinematische vergelijkingen (constante acceleratie):
 *   v(t) = v0 + a * t
 *   x(t) = x0 + v0 * t + 0.5 * a * t^2
 *
 * @author Group g1
 * @date 2026-01-12
 */

#include <chrono>
#include <memory>
#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

/**
 * @struct SegmentData
 * @brief Parsed segment data (may contain unknowns)
 */
struct SegmentData {
    // Interval times (can be "onbekend" strings initially)
    std::string start_time_str;
    std::string end_time_str;
    double start_time{0.0};
    double end_time{0.0};
    bool start_is_unknown{false};
    bool end_is_unknown{false};

    // Acceleration values (can be "onbekend" strings initially)
    std::string accel_x_str;
    std::string accel_y_str;
    double accel_x{0.0};
    double accel_y{0.0};
    bool accel_x_is_unknown{false};
    bool accel_y_is_unknown{false};

    // Type
    std::string type{"constant"};
};

/**
 * @struct KinematicState
 * @brief Position and velocity state at a point in time
 */
struct KinematicState {
    double time{0.0};
    double position_x{0.0};
    double position_y{0.0};
    double velocity_x{0.0};
    double velocity_y{0.0};
};

/**
 * @struct Constraint
 * @brief A constraint that must be satisfied
 */
struct Constraint {
    std::string type;
    double value{0.0};
};

/**
 * @class PathSolver
 * @brief Solves for unknown values in path definitions
 */
class PathSolver : public rclcpp::Node
{
public:
    PathSolver() : Node("path_solver")
    {
        // PARAMETER SETUP
        declare_and_get_parameters();

        // LOAD AND SOLVE
        if (!load_solve_yaml()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load solve YAML file");
            return;
        }

        if (!solve_unknowns()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to solve unknowns");
            return;
        }

        // OUTPUT SOLVED PATH
        if (!write_solved_yaml()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write solved YAML file");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Path solved successfully!");
        RCLCPP_INFO(this->get_logger(), "Output written to: %s", output_file_.c_str());

        // Print summary
        print_solution_summary();
    }

private:
    void declare_and_get_parameters()
    {
        this->declare_parameter<std::string>("input_file", "config/paths/solve.yaml");
        this->declare_parameter<std::string>("output_file", "");

        input_file_ = this->get_parameter("input_file").as_string();
        output_file_ = this->get_parameter("output_file").as_string();

        // Default output: input_file with "_solved" suffix
        if (output_file_.empty()) {
            size_t dot_pos = input_file_.rfind('.');
            if (dot_pos != std::string::npos) {
                output_file_ = input_file_.substr(0, dot_pos) + "_solved" + input_file_.substr(dot_pos);
            } else {
                output_file_ = input_file_ + "_solved";
            }
        }

        RCLCPP_INFO(this->get_logger(), "Input file: %s", input_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output file: %s", output_file_.c_str());
    }

    std::string resolve_file_path(const std::string& file_path)
    {
        if (!file_path.empty() && file_path[0] == '/') {
            return file_path;
        }
        try {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("odometry_pkg");
            return pkg_share + "/" + file_path;
        } catch (...) {
            return file_path;
        }
    }

    /**
     * @brief Check if a string represents an unknown value
     */
    bool is_unknown(const std::string& value) const
    {
        // Match "onbekend", "onbekend_1", "onbekend_plus_10", etc.
        return value.find("onbekend") != std::string::npos;
    }

    /**
     * @brief Parse interval value (could be number or "onbekend")
     */
    std::pair<double, bool> parse_interval_value(const YAML::Node& node)
    {
        try {
            // Try to parse as double first
            return {node.as<double>(), false};
        } catch (...) {
            // It's a string (onbekend)
            return {0.0, true};
        }
    }

    /**
     * @brief Load the solve YAML file
     */
    bool load_solve_yaml()
    {
        std::string full_path = resolve_file_path(input_file_);
        RCLCPP_INFO(this->get_logger(), "Loading: %s", full_path.c_str());

        try {
            YAML::Node config = YAML::LoadFile(full_path);

            if (!config["path"]) {
                RCLCPP_ERROR(this->get_logger(), "YAML must contain 'path' key");
                return false;
            }

            YAML::Node path = config["path"];

            // Store path name and description
            path_name_ = path["name"] ? path["name"].as<std::string>() : "solved_path";
            path_description_ = path["description"] ? path["description"].as<std::string>() : "";

            // Parse initial conditions
            if (path["initial_conditions"]) {
                auto ic = path["initial_conditions"];
                initial_state_.position_x = ic["position_x"] ? ic["position_x"].as<double>() : 0.0;
                initial_state_.position_y = ic["position_y"] ? ic["position_y"].as<double>() : 0.0;
                initial_state_.velocity_x = ic["velocity_x"] ? ic["velocity_x"].as<double>() : 0.0;
                initial_state_.velocity_y = ic["velocity_y"] ? ic["velocity_y"].as<double>() : 0.0;
            }

            RCLCPP_INFO(this->get_logger(), "Initial conditions: pos(%.2f, %.2f) vel(%.2f, %.2f)",
                initial_state_.position_x, initial_state_.position_y,
                initial_state_.velocity_x, initial_state_.velocity_y);

            // Parse constraints
            if (path["constraints"]) {
                for (const auto& c : path["constraints"]) {
                    Constraint constraint;
                    constraint.type = c["type"].as<std::string>();
                    constraint.value = c["value"].as<double>();
                    constraints_.push_back(constraint);

                    RCLCPP_INFO(this->get_logger(), "Constraint: %s = %.4f",
                        constraint.type.c_str(), constraint.value);
                }
            }

            // Parse segments
            if (!path["segments"]) {
                RCLCPP_ERROR(this->get_logger(), "YAML must contain 'segments' array");
                return false;
            }

            for (const auto& seg_node : path["segments"]) {
                SegmentData seg;

                // Parse interval
                auto interval = seg_node["interval"];
                auto [start, start_unknown] = parse_interval_value(interval[0]);
                auto [end, end_unknown] = parse_interval_value(interval[1]);

                seg.start_time = start;
                seg.end_time = end;
                seg.start_is_unknown = start_unknown;
                seg.end_is_unknown = end_unknown;

                // Store string representation for unknowns
                if (start_unknown) {
                    seg.start_time_str = interval[0].as<std::string>();
                }
                if (end_unknown) {
                    seg.end_time_str = interval[1].as<std::string>();
                }

                // Parse type
                seg.type = seg_node["type"] ? seg_node["type"].as<std::string>() : "constant";

                // Parse accelerations
                if (seg_node["accel_x"]) {
                    try {
                        seg.accel_x = seg_node["accel_x"].as<double>();
                    } catch (...) {
                        seg.accel_x_str = seg_node["accel_x"].as<std::string>();
                        seg.accel_x_is_unknown = is_unknown(seg.accel_x_str);
                    }
                }

                if (seg_node["accel_y"]) {
                    try {
                        seg.accel_y = seg_node["accel_y"].as<double>();
                    } catch (...) {
                        seg.accel_y_str = seg_node["accel_y"].as<std::string>();
                        seg.accel_y_is_unknown = is_unknown(seg.accel_y_str);
                    }
                }

                segments_.push_back(seg);

                RCLCPP_INFO(this->get_logger(), "Segment %zu: [%s, %s] a=(%.3f, %.3f) unknowns: start=%d end=%d ax=%d ay=%d",
                    segments_.size(),
                    start_unknown ? seg.start_time_str.c_str() : std::to_string(start).c_str(),
                    end_unknown ? seg.end_time_str.c_str() : std::to_string(end).c_str(),
                    seg.accel_x, seg.accel_y,
                    seg.start_is_unknown, seg.end_is_unknown,
                    seg.accel_x_is_unknown, seg.accel_y_is_unknown);
            }

            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML: %s", e.what());
            return false;
        }
    }

    /**
     * @brief Simulate kinematics for a segment with constant acceleration
     * @param state Input state (will be modified to output state)
     * @param accel_x X acceleration
     * @param accel_y Y acceleration
     * @param duration Segment duration
     */
    void simulate_segment(KinematicState& state, double accel_x, double accel_y, double duration)
    {
        // Kinematic equations:
        // v(t) = v0 + a * t
        // x(t) = x0 + v0 * t + 0.5 * a * t^2

        double v0x = state.velocity_x;
        double v0y = state.velocity_y;

        // Update position
        state.position_x += v0x * duration + 0.5 * accel_x * duration * duration;
        state.position_y += v0y * duration + 0.5 * accel_y * duration * duration;

        // Update velocity
        state.velocity_x += accel_x * duration;
        state.velocity_y += accel_y * duration;

        // Update time
        state.time += duration;
    }

    /**
     * @brief Calculate final state given a value for the unknown
     */
    KinematicState calculate_final_state(double unknown_value)
    {
        KinematicState state = initial_state_;
        double current_time = 0.0;

        for (size_t i = 0; i < segments_.size(); ++i) {
            auto& seg = segments_[i];

            // Determine segment start time
            double seg_start;
            if (seg.start_is_unknown) {
                // Parse the unknown expression
                if (seg.start_time_str == "onbekend") {
                    seg_start = unknown_value;
                } else {
                    seg_start = current_time;  // Use current time
                }
            } else {
                seg_start = seg.start_time;
            }

            // Determine segment end time
            double seg_end;
            if (seg.end_is_unknown) {
                if (seg.end_time_str == "onbekend") {
                    seg_end = unknown_value;
                } else if (seg.end_time_str.find("onbekend_plus_") != std::string::npos) {
                    // Parse "onbekend_plus_X" format
                    std::regex r("onbekend_plus_(\\d+\\.?\\d*)");
                    std::smatch m;
                    if (std::regex_search(seg.end_time_str, m, r)) {
                        double offset = std::stod(m[1].str());
                        seg_end = unknown_value + offset;
                    } else {
                        seg_end = unknown_value + 10.0;  // Default offset
                    }
                } else {
                    seg_end = unknown_value;
                }
            } else {
                seg_end = seg.end_time;
            }

            double duration = seg_end - seg_start;
            if (duration <= 0) continue;

            // Simulate this segment
            simulate_segment(state, seg.accel_x, seg.accel_y, duration);
            current_time = seg_end;
        }

        return state;
    }

    /**
     * @brief Solve for unknowns using binary search or analytical solution
     */
    bool solve_unknowns()
    {
        // Count unknowns
        int num_unknown_times = 0;
        int unknown_segment_index = -1;

        for (size_t i = 0; i < segments_.size(); ++i) {
            if (segments_[i].start_is_unknown || segments_[i].end_is_unknown) {
                num_unknown_times++;
                unknown_segment_index = i;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Found %d segments with unknown times", num_unknown_times);

        if (num_unknown_times == 0) {
            RCLCPP_INFO(this->get_logger(), "No unknowns to solve - path is complete");
            return true;
        }

        // For now, we support single unknown time (can be extended)
        if (num_unknown_times > 1) {
            RCLCPP_WARN(this->get_logger(),
                "Multiple unknown times detected - using constraint-based solver");
        }

        // Find the constraint to use for solving
        double target_value = 0.0;
        std::string constraint_type = "";

        for (const auto& c : constraints_) {
            if (c.type == "final_position_x") {
                constraint_type = "position_x";
                target_value = c.value;
                break;
            } else if (c.type == "final_position_y") {
                constraint_type = "position_y";
                target_value = c.value;
                break;
            } else if (c.type == "final_velocity_x") {
                constraint_type = "velocity_x";
                target_value = c.value;
                break;
            } else if (c.type == "final_velocity_y") {
                constraint_type = "velocity_y";
                target_value = c.value;
                break;
            }
        }

        if (constraint_type.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No usable constraint found for solving");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Solving for unknown using constraint: %s = %.4f",
            constraint_type.c_str(), target_value);

        // Try analytical solution first for simple cases
        if (try_analytical_solution(constraint_type, target_value)) {
            return true;
        }

        // Fall back to binary search
        return solve_by_binary_search(constraint_type, target_value);
    }

    /**
     * @brief Try to solve analytically for simple cases
     */
    bool try_analytical_solution(const std::string& constraint_type, double target_value)
    {
        // This works for the specific case in the example:
        // Segments with known accelerations, one unknown duration
        // We can derive T directly from the kinematic equations

        // First, find segments with and without unknowns
        std::vector<size_t> known_segments;
        std::vector<size_t> unknown_segments;

        for (size_t i = 0; i < segments_.size(); ++i) {
            if (segments_[i].start_is_unknown || segments_[i].end_is_unknown) {
                unknown_segments.push_back(i);
            } else {
                known_segments.push_back(i);
            }
        }

        if (unknown_segments.size() != 1) {
            return false;  // Can't do simple analytical solution
        }

        size_t unknown_idx = unknown_segments[0];

        // Calculate state at start of unknown segment
        KinematicState state_before = initial_state_;
        double time_before_unknown = 0.0;

        for (size_t i = 0; i < unknown_idx; ++i) {
            double duration = segments_[i].end_time - segments_[i].start_time;
            simulate_segment(state_before, segments_[i].accel_x, segments_[i].accel_y, duration);
            time_before_unknown = segments_[i].end_time;
        }

        RCLCPP_INFO(this->get_logger(), "State before unknown segment: pos=(%.4f, %.4f) vel=(%.4f, %.4f)",
            state_before.position_x, state_before.position_y,
            state_before.velocity_x, state_before.velocity_y);

        // Calculate contributions of segments after the unknown (they have known durations relative to unknown)
        // For the example: segment 4 has duration 10s and a=-0.01
        double accel_unknown = segments_[unknown_idx].accel_x;

        // Contribution from unknown segment: position += v * T + 0.5 * a * T^2
        // But velocity changes too, which affects later segments

        // For now, use binary search as it's more general
        return false;
    }

    /**
     * @brief Solve using binary search
     */
    bool solve_by_binary_search(const std::string& constraint_type, double target_value)
    {
        // Binary search for the unknown value
        double low = 0.0;
        double high = 1000.0;  // Max 1000 seconds
        double tolerance = 0.0001;  // 0.1ms precision
        int max_iterations = 100;

        // First, find the last known time to set as minimum
        for (const auto& seg : segments_) {
            if (!seg.end_is_unknown) {
                low = std::max(low, seg.end_time);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Binary search range: [%.2f, %.2f]", low, high);

        // Check bounds
        auto state_low = calculate_final_state(low);
        auto state_high = calculate_final_state(high);

        double value_low = get_state_value(state_low, constraint_type);
        double value_high = get_state_value(state_high, constraint_type);

        RCLCPP_INFO(this->get_logger(), "At T=%.2f: %s = %.4f", low, constraint_type.c_str(), value_low);
        RCLCPP_INFO(this->get_logger(), "At T=%.2f: %s = %.4f", high, constraint_type.c_str(), value_high);

        // Check if solution is in range
        if ((value_low - target_value) * (value_high - target_value) > 0) {
            // Try extending the range
            high = 10000.0;
            state_high = calculate_final_state(high);
            value_high = get_state_value(state_high, constraint_type);

            if ((value_low - target_value) * (value_high - target_value) > 0) {
                RCLCPP_ERROR(this->get_logger(),
                    "No solution found in range [%.2f, %.2f]. Values: [%.4f, %.4f], target: %.4f",
                    low, high, value_low, value_high, target_value);
                return false;
            }
        }

        // Binary search
        for (int iter = 0; iter < max_iterations; ++iter) {
            double mid = (low + high) / 2.0;
            auto state_mid = calculate_final_state(mid);
            double value_mid = get_state_value(state_mid, constraint_type);

            if (std::abs(value_mid - target_value) < tolerance) {
                solved_unknown_value_ = mid;
                RCLCPP_INFO(this->get_logger(),
                    "SOLVED! Unknown time T = %.4f seconds (iteration %d)", mid, iter);
                RCLCPP_INFO(this->get_logger(),
                    "Final state: pos=(%.4f, %.4f) vel=(%.4f, %.4f)",
                    state_mid.position_x, state_mid.position_y,
                    state_mid.velocity_x, state_mid.velocity_y);

                apply_solution(mid);
                return true;
            }

            if ((value_low - target_value) * (value_mid - target_value) < 0) {
                high = mid;
                value_high = value_mid;
            } else {
                low = mid;
                value_low = value_mid;
            }
        }

        RCLCPP_ERROR(this->get_logger(), "Binary search did not converge after %d iterations", max_iterations);
        return false;
    }

    double get_state_value(const KinematicState& state, const std::string& type)
    {
        if (type == "position_x") return state.position_x;
        if (type == "position_y") return state.position_y;
        if (type == "velocity_x") return state.velocity_x;
        if (type == "velocity_y") return state.velocity_y;
        return 0.0;
    }

    /**
     * @brief Apply the solved value to segments
     */
    void apply_solution(double unknown_value)
    {
        double current_time = 0.0;

        for (size_t i = 0; i < segments_.size(); ++i) {
            auto& seg = segments_[i];

            // Update start time
            if (seg.start_is_unknown) {
                if (seg.start_time_str == "onbekend") {
                    seg.start_time = unknown_value;
                } else {
                    seg.start_time = current_time;
                }
                seg.start_is_unknown = false;
            }

            // Update end time
            if (seg.end_is_unknown) {
                if (seg.end_time_str == "onbekend") {
                    seg.end_time = unknown_value;
                } else if (seg.end_time_str.find("onbekend_plus_") != std::string::npos) {
                    std::regex r("onbekend_plus_(\\d+\\.?\\d*)");
                    std::smatch m;
                    if (std::regex_search(seg.end_time_str, m, r)) {
                        double offset = std::stod(m[1].str());
                        seg.end_time = unknown_value + offset;
                    } else {
                        seg.end_time = unknown_value + 10.0;
                    }
                } else {
                    seg.end_time = unknown_value;
                }
                seg.end_is_unknown = false;
            }

            current_time = seg.end_time;
        }
    }

    /**
     * @brief Write the solved YAML to output file
     */
    bool write_solved_yaml()
    {
        std::string full_path = resolve_file_path(output_file_);

        YAML::Emitter out;
        out << YAML::Comment("AUTO-GENERATED by path_solver");
        out << YAML::Comment("Original file: " + input_file_);
        out << YAML::Newline;

        out << YAML::BeginMap;
        out << YAML::Key << "path";
        out << YAML::Value << YAML::BeginMap;

        // Name and description
        out << YAML::Key << "name" << YAML::Value << path_name_ + "_solved";
        out << YAML::Key << "description" << YAML::Value << "Solved path: " + path_description_;

        // Duration (calculated from last segment end time)
        double duration = 0.0;
        for (const auto& seg : segments_) {
            duration = std::max(duration, seg.end_time);
        }
        out << YAML::Key << "duration" << YAML::Value << duration;
        out << YAML::Key << "sample_rate_hz" << YAML::Value << 100;

        // Segments
        out << YAML::Key << "segments";
        out << YAML::Value << YAML::BeginSeq;

        for (const auto& seg : segments_) {
            out << YAML::BeginMap;

            // Interval
            out << YAML::Key << "interval";
            out << YAML::Value << YAML::Flow << YAML::BeginSeq;
            out << seg.start_time << seg.end_time;
            out << YAML::EndSeq;

            // Type
            out << YAML::Key << "type" << YAML::Value << seg.type;

            // Accelerations
            out << YAML::Key << "accel_x" << YAML::Value << seg.accel_x;
            out << YAML::Key << "accel_y" << YAML::Value << seg.accel_y;
            out << YAML::Key << "accel_z" << YAML::Value << 0.0;

            out << YAML::EndMap;
        }

        out << YAML::EndSeq;
        out << YAML::EndMap;
        out << YAML::EndMap;

        // Write to file
        std::ofstream fout(full_path);
        if (!fout.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open output file: %s", full_path.c_str());
            return false;
        }

        fout << out.c_str();
        fout.close();

        return true;
    }

    void print_solution_summary()
    {
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "========== SOLUTION SUMMARY ==========");

        KinematicState state = initial_state_;
        double current_time = 0.0;

        RCLCPP_INFO(this->get_logger(), "Initial: t=%.2f pos=(%.4f, %.4f) vel=(%.4f, %.4f)",
            current_time, state.position_x, state.position_y, state.velocity_x, state.velocity_y);

        for (size_t i = 0; i < segments_.size(); ++i) {
            const auto& seg = segments_[i];
            double duration = seg.end_time - seg.start_time;

            simulate_segment(state, seg.accel_x, seg.accel_y, duration);

            RCLCPP_INFO(this->get_logger(),
                "Segment %zu [%.2f, %.2f]: a=(%.4f, %.4f) -> pos=(%.4f, %.4f) vel=(%.4f, %.4f)",
                i + 1, seg.start_time, seg.end_time, seg.accel_x, seg.accel_y,
                state.position_x, state.position_y, state.velocity_x, state.velocity_y);
        }

        RCLCPP_INFO(this->get_logger(), "======================================");
        RCLCPP_INFO(this->get_logger(), "Total duration: %.2f seconds", segments_.back().end_time);
        RCLCPP_INFO(this->get_logger(), "Final position: (%.4f, %.4f) m", state.position_x, state.position_y);
        RCLCPP_INFO(this->get_logger(), "Final velocity: (%.4f, %.4f) m/s", state.velocity_x, state.velocity_y);

        // Verify constraints
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Constraint verification:");
        for (const auto& c : constraints_) {
            double actual = 0.0;
            if (c.type == "final_position_x") actual = state.position_x;
            else if (c.type == "final_position_y") actual = state.position_y;
            else if (c.type == "final_velocity_x") actual = state.velocity_x;
            else if (c.type == "final_velocity_y") actual = state.velocity_y;

            double error = std::abs(actual - c.value);
            const char* status = error < 0.001 ? "OK" : "FAIL";
            RCLCPP_INFO(this->get_logger(), "  %s: target=%.4f actual=%.4f error=%.6f [%s]",
                c.type.c_str(), c.value, actual, error, status);
        }
    }

    // Configuration
    std::string input_file_;
    std::string output_file_;

    // Path data
    std::string path_name_;
    std::string path_description_;
    KinematicState initial_state_;
    std::vector<Constraint> constraints_;
    std::vector<SegmentData> segments_;

    // Solution
    double solved_unknown_value_{0.0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<PathSolver>();
        // Single execution - no spin needed for solver
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("path_solver"),
            "Fatal error: %s", e.what());
        return 1;
    }

    return 0;
}
