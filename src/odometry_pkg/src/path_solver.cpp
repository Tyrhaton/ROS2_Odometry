/**
 * @file path_solver.cpp
 * @brief Path Solver - Lost automatisch onbekende waarden op via analytische integratie
 *
 * Dit programma leest een YAML-bestand met "onbekend" waarden en lost deze op
 * door de kinematische vergelijkingen symbolisch te integreren.
 *
 * METHODE: Analytische integratie
 * ================================
 *
 * Kinematische vergelijkingen (constante acceleratie):
 *   v(t) = v0 + a * t                    (integraal van a)
 *   x(t) = x0 + v0 * t + 0.5 * a * t^2   (integraal van v)
 *
 * Voor elk segment berekenen we de bijdrage aan positie en snelheid.
 * Als een segment duur T onbekend is, krijgen we een vergelijking in T.
 *
 * Voorbeeld:
 *   Segment met a=0, duur=T, beginsnelheid=v0:
 *   - Δv = a * T = 0
 *   - Δx = v0 * T + 0.5 * a * T^2 = v0 * T
 *
 *   Dit geeft: x_eind = x_begin + v0 * T
 *   Als x_eind bekend is: T = (x_eind - x_begin) / v0
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
    std::string start_time_str;
    std::string end_time_str;
    double start_time{0.0};
    double end_time{0.0};
    bool start_is_unknown{false};
    bool end_is_unknown{false};

    std::string accel_x_str;
    std::string accel_y_str;
    double accel_x{0.0};
    double accel_y{0.0};
    bool accel_x_is_unknown{false};
    bool accel_y_is_unknown{false};

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
 * @struct SymbolicExpression
 * @brief Represents a polynomial expression in T: a0 + a1*T + a2*T^2
 *
 * Dit wordt gebruikt om symbolisch te integreren zonder numerieke waarde voor T
 */
struct SymbolicExpression {
    double a0{0.0};  // Constante term
    double a1{0.0};  // Coëfficiënt van T
    double a2{0.0};  // Coëfficiënt van T^2

    SymbolicExpression() = default;
    SymbolicExpression(double c) : a0(c), a1(0), a2(0) {}
    SymbolicExpression(double c0, double c1, double c2) : a0(c0), a1(c1), a2(c2) {}

    // Evalueer voor gegeven T
    double evaluate(double T) const {
        return a0 + a1 * T + a2 * T * T;
    }

    // Optellen
    SymbolicExpression operator+(const SymbolicExpression& other) const {
        return SymbolicExpression(a0 + other.a0, a1 + other.a1, a2 + other.a2);
    }

    // Vermenigvuldigen met constante
    SymbolicExpression operator*(double c) const {
        return SymbolicExpression(a0 * c, a1 * c, a2 * c);
    }
};

/**
 * @class PathSolver
 * @brief Solves for unknown values in path definitions using analytical integration
 */
class PathSolver : public rclcpp::Node
{
public:
    PathSolver() : Node("path_solver")
    {
        declare_and_get_parameters();

        if (!load_solve_yaml()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load solve YAML file");
            return;
        }

        if (!solve_by_integration()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to solve unknowns");
            return;
        }

        if (!write_solved_yaml()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write solved YAML file");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Path solved successfully!");
        RCLCPP_INFO(this->get_logger(), "Output written to: %s", output_file_.c_str());

        print_solution_summary();
    }

private:
    void declare_and_get_parameters()
    {
        this->declare_parameter<std::string>("input_file", "config/paths/solve.yaml");
        this->declare_parameter<std::string>("output_file", "");

        input_file_ = this->get_parameter("input_file").as_string();
        output_file_ = this->get_parameter("output_file").as_string();

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

    bool is_unknown(const std::string& value) const
    {
        return value.find("onbekend") != std::string::npos;
    }

    std::pair<double, bool> parse_interval_value(const YAML::Node& node)
    {
        try {
            return {node.as<double>(), false};
        } catch (...) {
            return {0.0, true};
        }
    }

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

            path_name_ = path["name"] ? path["name"].as<std::string>() : "solved_path";
            path_description_ = path["description"] ? path["description"].as<std::string>() : "";

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

            if (!path["segments"]) {
                RCLCPP_ERROR(this->get_logger(), "YAML must contain 'segments' array");
                return false;
            }

            for (const auto& seg_node : path["segments"]) {
                SegmentData seg;

                auto interval = seg_node["interval"];
                auto [start, start_unknown] = parse_interval_value(interval[0]);
                auto [end, end_unknown] = parse_interval_value(interval[1]);

                seg.start_time = start;
                seg.end_time = end;
                seg.start_is_unknown = start_unknown;
                seg.end_is_unknown = end_unknown;

                if (start_unknown) {
                    seg.start_time_str = interval[0].as<std::string>();
                }
                if (end_unknown) {
                    seg.end_time_str = interval[1].as<std::string>();
                }

                seg.type = seg_node["type"] ? seg_node["type"].as<std::string>() : "constant";

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

                RCLCPP_INFO(this->get_logger(), "Segment %zu: [%s, %s] a=(%.3f, %.3f)",
                    segments_.size(),
                    start_unknown ? seg.start_time_str.c_str() : std::to_string(start).c_str(),
                    end_unknown ? seg.end_time_str.c_str() : std::to_string(end).c_str(),
                    seg.accel_x, seg.accel_y);
            }

            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML: %s", e.what());
            return false;
        }
    }

    /**
     * @brief Integreer segment met constante acceleratie
     *
     * v(t) = v0 + a * dt        ->  Δv = a * dt
     * x(t) = x0 + v0*dt + 0.5*a*dt^2  ->  Δx = v0*dt + 0.5*a*dt^2
     */
    void integrate_segment(double& pos_x, double& pos_y, double& vel_x, double& vel_y,
                          double accel_x, double accel_y, double duration)
    {
        // Positie update (met huidige snelheid)
        pos_x += vel_x * duration + 0.5 * accel_x * duration * duration;
        pos_y += vel_y * duration + 0.5 * accel_y * duration * duration;

        // Snelheid update
        vel_x += accel_x * duration;
        vel_y += accel_y * duration;
    }

    /**
     * @brief Los op via analytische integratie
     *
     * Strategie:
     * 1. Integreer alle bekende segmenten vóór de onbekende
     * 2. Stel symbolische vergelijking op voor segment met onbekende T
     * 3. Integreer alle segmenten ná de onbekende (hun bijdrage hangt mogelijk af van T)
     * 4. Los de vergelijking op: target = f(T)
     */
    bool solve_by_integration()
    {
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "===== ANALYTISCHE INTEGRATIE =====");

        // Vind segment met onbekende tijd
        int unknown_segment_idx = -1;
        for (size_t i = 0; i < segments_.size(); ++i) {
            if (segments_[i].end_is_unknown) {
                unknown_segment_idx = static_cast<int>(i);
                break;
            }
        }

        if (unknown_segment_idx == -1) {
            RCLCPP_INFO(this->get_logger(), "Geen onbekende gevonden - path is compleet");
            return true;
        }

        // Vind constraint
        double target_position_x = 0.0;
        bool has_position_constraint = false;
        for (const auto& c : constraints_) {
            if (c.type == "final_position_x") {
                target_position_x = c.value;
                has_position_constraint = true;
                break;
            }
        }

        if (!has_position_constraint) {
            RCLCPP_ERROR(this->get_logger(), "Geen final_position_x constraint gevonden");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Target positie: x = %.4f m", target_position_x);
        RCLCPP_INFO(this->get_logger(), "Onbekende in segment %d", unknown_segment_idx + 1);

        // ============================================
        // STAP 1: Integreer segmenten VOOR de onbekende
        // ============================================
        double pos_x = initial_state_.position_x;
        double pos_y = initial_state_.position_y;
        double vel_x = initial_state_.velocity_x;
        double vel_y = initial_state_.velocity_y;
        double time = 0.0;

        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "Stap 1: Integreer bekende segmenten vóór onbekende");
        RCLCPP_INFO(this->get_logger(), "  Start: t=%.2f x=%.4f v=%.4f", time, pos_x, vel_x);

        for (int i = 0; i < unknown_segment_idx; ++i) {
            double dt = segments_[i].end_time - segments_[i].start_time;
            double a = segments_[i].accel_x;

            RCLCPP_INFO(this->get_logger(), " ");
            RCLCPP_INFO(this->get_logger(), "  Segment %d: dt=%.2f a=%.4f", i + 1, dt, a);
            RCLCPP_INFO(this->get_logger(), "    v(t) = v0 + a*t = %.4f + %.4f*%.2f = %.4f",
                vel_x, a, dt, vel_x + a * dt);
            RCLCPP_INFO(this->get_logger(), "    x(t) = x0 + v0*t + 0.5*a*t^2 = %.4f + %.4f*%.2f + 0.5*%.4f*%.2f^2 = %.4f",
                pos_x, vel_x, dt, a, dt, pos_x + vel_x * dt + 0.5 * a * dt * dt);

            integrate_segment(pos_x, pos_y, vel_x, vel_y, a, segments_[i].accel_y, dt);
            time = segments_[i].end_time;

            RCLCPP_INFO(this->get_logger(), "    Na segment: t=%.2f x=%.4f v=%.4f", time, pos_x, vel_x);
        }

        // Sla state op vóór onbekende segment
        double pos_before_unknown = pos_x;
        double vel_before_unknown = vel_x;
        double time_before_unknown = time;

        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "State vóór onbekende segment:");
        RCLCPP_INFO(this->get_logger(), "  t = %.2f s", time_before_unknown);
        RCLCPP_INFO(this->get_logger(), "  x = %.4f m", pos_before_unknown);
        RCLCPP_INFO(this->get_logger(), "  v = %.4f m/s", vel_before_unknown);

        // ============================================
        // STAP 2: Symbolische integratie van onbekende segment
        // ============================================
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "Stap 2: Symbolische integratie (segment met onbekende T)");

        double a_unknown = segments_[unknown_segment_idx].accel_x;
        RCLCPP_INFO(this->get_logger(), "  Segment %d: a = %.4f, duur = T (onbekend)",
            unknown_segment_idx + 1, a_unknown);

        // Na dit segment:
        // v_na = v_voor + a * T
        // x_na = x_voor + v_voor * T + 0.5 * a * T^2
        RCLCPP_INFO(this->get_logger(), "  v(T) = %.4f + %.4f * T", vel_before_unknown, a_unknown);
        RCLCPP_INFO(this->get_logger(), "  x(T) = %.4f + %.4f * T + 0.5 * %.4f * T^2",
            pos_before_unknown, vel_before_unknown, a_unknown);

        // Symbolische snelheid en positie na onbekend segment (als functie van T)
        // vel_after = vel_before + a_unknown * T
        // pos_after = pos_before + vel_before * T + 0.5 * a_unknown * T^2
        SymbolicExpression vel_after_unknown(vel_before_unknown, a_unknown, 0.0);
        SymbolicExpression pos_after_unknown(pos_before_unknown, vel_before_unknown, 0.5 * a_unknown);

        // ============================================
        // STAP 3: Integreer segmenten NA de onbekende (symbolisch)
        // ============================================
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "Stap 3: Integreer segmenten ná onbekende (symbolisch)");

        SymbolicExpression final_pos = pos_after_unknown;
        SymbolicExpression final_vel = vel_after_unknown;

        for (size_t i = unknown_segment_idx + 1; i < segments_.size(); ++i) {
            // Parse de duur van dit segment
            double dt;
            if (segments_[i].start_is_unknown && segments_[i].end_is_unknown) {
                // Beide zijn onbekend, maar relatief bekend (bijv. onbekend tot onbekend+10)
                // Parse de offset
                std::regex r("onbekend_plus_(\\d+\\.?\\d*)");
                std::smatch m;
                if (std::regex_search(segments_[i].end_time_str, m, r)) {
                    dt = std::stod(m[1].str());
                } else {
                    dt = 10.0;  // Default
                }
            } else {
                dt = segments_[i].end_time - segments_[i].start_time;
            }

            double a = segments_[i].accel_x;

            RCLCPP_INFO(this->get_logger(), " ");
            RCLCPP_INFO(this->get_logger(), "  Segment %zu: dt=%.2f a=%.4f", i + 1, dt, a);

            // Symbolische integratie:
            // nieuwe_pos = oude_pos + oude_vel * dt + 0.5 * a * dt^2
            // nieuwe_vel = oude_vel + a * dt

            // pos += vel * dt + 0.5 * a * dt^2
            // Als vel = (v0, v1, v2) dan vel * dt = (v0*dt, v1*dt, v2*dt)
            SymbolicExpression delta_pos(
                final_vel.a0 * dt + 0.5 * a * dt * dt,
                final_vel.a1 * dt,
                final_vel.a2 * dt
            );

            final_pos = final_pos + delta_pos;

            // vel += a * dt
            final_vel.a0 += a * dt;

            RCLCPP_INFO(this->get_logger(), "    Δx = v*dt + 0.5*a*dt^2");
            RCLCPP_INFO(this->get_logger(), "    x(T) = %.4f + %.4f*T + %.4f*T^2",
                final_pos.a0, final_pos.a1, final_pos.a2);
            RCLCPP_INFO(this->get_logger(), "    v(T) = %.4f + %.4f*T",
                final_vel.a0, final_vel.a1);
        }

        // ============================================
        // STAP 4: Los de vergelijking op
        // ============================================
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "Stap 4: Los vergelijking op");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "  Eindpositie als functie van T:");
        RCLCPP_INFO(this->get_logger(), "    x(T) = %.4f + %.4f*T + %.4f*T^2",
            final_pos.a0, final_pos.a1, final_pos.a2);
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "  Constraint: x(T) = %.4f", target_position_x);
        RCLCPP_INFO(this->get_logger(), " ");

        // Los op: a0 + a1*T + a2*T^2 = target
        // => a2*T^2 + a1*T + (a0 - target) = 0
        double A = final_pos.a2;
        double B = final_pos.a1;
        double C = final_pos.a0 - target_position_x;

        RCLCPP_INFO(this->get_logger(), "  Vergelijking: %.4f*T^2 + %.4f*T + %.4f = 0", A, B, C);

        double T_solved;

        if (std::abs(A) < 1e-10) {
            // Lineaire vergelijking: B*T + C = 0
            if (std::abs(B) < 1e-10) {
                RCLCPP_ERROR(this->get_logger(), "Geen oplossing mogelijk (0*T = %.4f)", -C);
                return false;
            }
            T_solved = -C / B;
            RCLCPP_INFO(this->get_logger(), " ");
            RCLCPP_INFO(this->get_logger(), "  Lineaire vergelijking: T = -%.4f / %.4f = %.4f", C, B, T_solved);
        } else {
            // Kwadratische vergelijking: ABC-formule
            double discriminant = B * B - 4 * A * C;

            RCLCPP_INFO(this->get_logger(), " ");
            RCLCPP_INFO(this->get_logger(), "  Discriminant D = B^2 - 4AC = %.4f^2 - 4*%.4f*%.4f = %.4f",
                B, A, C, discriminant);

            if (discriminant < 0) {
                RCLCPP_ERROR(this->get_logger(), "Geen reële oplossing (D < 0)");
                return false;
            }

            double T1 = (-B + std::sqrt(discriminant)) / (2 * A);
            double T2 = (-B - std::sqrt(discriminant)) / (2 * A);

            RCLCPP_INFO(this->get_logger(), "  T1 = (-%.4f + sqrt(%.4f)) / (2*%.4f) = %.4f", B, discriminant, A, T1);
            RCLCPP_INFO(this->get_logger(), "  T2 = (-%.4f - sqrt(%.4f)) / (2*%.4f) = %.4f", B, discriminant, A, T2);

            // Kies de positieve oplossing die fysisch zinvol is
            if (T1 >= 0 && T2 >= 0) {
                T_solved = std::min(T1, T2);  // Neem de kleinste positieve
            } else if (T1 >= 0) {
                T_solved = T1;
            } else if (T2 >= 0) {
                T_solved = T2;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Geen positieve oplossing gevonden");
                return false;
            }
        }

        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "===== OPLOSSING =====");
        RCLCPP_INFO(this->get_logger(), "  T = %.4f seconden", T_solved);
        RCLCPP_INFO(this->get_logger(), " ");

        // Verificatie
        double final_x = final_pos.evaluate(T_solved);
        double final_v = final_vel.evaluate(T_solved);
        RCLCPP_INFO(this->get_logger(), "  Verificatie:");
        RCLCPP_INFO(this->get_logger(), "    x(T=%.4f) = %.4f m (target: %.4f)", T_solved, final_x, target_position_x);
        RCLCPP_INFO(this->get_logger(), "    v(T=%.4f) = %.4f m/s", T_solved, final_v);
        RCLCPP_INFO(this->get_logger(), "    Error: %.6f m", std::abs(final_x - target_position_x));

        // Apply solution
        solved_unknown_value_ = T_solved;
        apply_solution(time_before_unknown + T_solved);

        return true;
    }

    void apply_solution(double unknown_end_time)
    {
        for (size_t i = 0; i < segments_.size(); ++i) {
            auto& seg = segments_[i];

            if (seg.end_is_unknown) {
                if (seg.end_time_str == "onbekend") {
                    seg.end_time = unknown_end_time;
                }
                seg.end_is_unknown = false;
            }

            if (seg.start_is_unknown) {
                if (seg.start_time_str == "onbekend") {
                    seg.start_time = unknown_end_time;
                }
                seg.start_is_unknown = false;
            }

            // Handle onbekend_plus_X format
            if (seg.end_time_str.find("onbekend_plus_") != std::string::npos) {
                std::regex r("onbekend_plus_(\\d+\\.?\\d*)");
                std::smatch m;
                if (std::regex_search(seg.end_time_str, m, r)) {
                    double offset = std::stod(m[1].str());
                    seg.end_time = unknown_end_time + offset;
                }
            }
        }
    }

    bool write_solved_yaml()
    {
        std::string full_path = resolve_file_path(output_file_);

        YAML::Emitter out;
        out << YAML::Comment("AUTO-GENERATED by path_solver (analytische integratie)");
        out << YAML::Comment("Original file: " + input_file_);
        out << YAML::Newline;

        out << YAML::BeginMap;
        out << YAML::Key << "path";
        out << YAML::Value << YAML::BeginMap;

        out << YAML::Key << "name" << YAML::Value << path_name_ + "_solved";
        out << YAML::Key << "description" << YAML::Value << "Solved path: " + path_description_;

        double duration = 0.0;
        for (const auto& seg : segments_) {
            duration = std::max(duration, seg.end_time);
        }
        out << YAML::Key << "duration" << YAML::Value << duration;
        out << YAML::Key << "sample_rate_hz" << YAML::Value << 100;

        out << YAML::Key << "segments";
        out << YAML::Value << YAML::BeginSeq;

        for (const auto& seg : segments_) {
            out << YAML::BeginMap;
            out << YAML::Key << "interval";
            out << YAML::Value << YAML::Flow << YAML::BeginSeq;
            out << seg.start_time << seg.end_time;
            out << YAML::EndSeq;
            out << YAML::Key << "type" << YAML::Value << seg.type;
            out << YAML::Key << "accel_x" << YAML::Value << seg.accel_x;
            out << YAML::Key << "accel_y" << YAML::Value << seg.accel_y;
            out << YAML::Key << "accel_z" << YAML::Value << 0.0;
            out << YAML::EndMap;
        }

        out << YAML::EndSeq;
        out << YAML::EndMap;
        out << YAML::EndMap;

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
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "========== SOLUTION SUMMARY ==========");

        double pos_x = initial_state_.position_x;
        double pos_y = initial_state_.position_y;
        double vel_x = initial_state_.velocity_x;
        double vel_y = initial_state_.velocity_y;

        RCLCPP_INFO(this->get_logger(), "Initial: t=%.2f x=%.4f v=%.4f", 0.0, pos_x, vel_x);

        for (size_t i = 0; i < segments_.size(); ++i) {
            const auto& seg = segments_[i];
            double dt = seg.end_time - seg.start_time;

            integrate_segment(pos_x, pos_y, vel_x, vel_y, seg.accel_x, seg.accel_y, dt);

            RCLCPP_INFO(this->get_logger(),
                "Segment %zu [%.2f, %.2f]: a=%.4f -> x=%.4f v=%.4f",
                i + 1, seg.start_time, seg.end_time, seg.accel_x, pos_x, vel_x);
        }

        RCLCPP_INFO(this->get_logger(), "======================================");
        RCLCPP_INFO(this->get_logger(), "Total duration: %.2f seconds", segments_.back().end_time);
        RCLCPP_INFO(this->get_logger(), "Final position: %.4f m", pos_x);
        RCLCPP_INFO(this->get_logger(), "Final velocity: %.4f m/s", vel_x);

        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "Constraint verification:");
        for (const auto& c : constraints_) {
            double actual = 0.0;
            if (c.type == "final_position_x") actual = pos_x;
            else if (c.type == "final_velocity_x") actual = vel_x;

            double error = std::abs(actual - c.value);
            const char* status = error < 0.001 ? "OK" : "FAIL";
            RCLCPP_INFO(this->get_logger(), "  %s: target=%.4f actual=%.4f error=%.6f [%s]",
                c.type.c_str(), c.value, actual, error, status);
        }
    }

    std::string input_file_;
    std::string output_file_;
    std::string path_name_;
    std::string path_description_;
    KinematicState initial_state_;
    std::vector<Constraint> constraints_;
    std::vector<SegmentData> segments_;
    double solved_unknown_value_{0.0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<PathSolver>();
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("path_solver"),
            "Fatal error: %s", e.what());
        return 1;
    }

    return 0;
}
