/*
 * @Description: Shared AR5 real-time setup helpers aligned with Rokae vendor sample.
 */

#ifndef TELEOP_ADAPTER_AR5_RT_SETUP_HPP
#define TELEOP_ADAPTER_AR5_RT_SETUP_HPP

#include "rokae/robot.h"
#include "rokae/motion_control_rt.h"
#include "nexus_log.hpp"

#include <array>
#include <string>

namespace teleop_adapter {
namespace ar5_rt_setup {

struct Config {
    double filter_joint_hz{10.0};
    double filter_cartesian_hz{10.0};
    double filter_torque_hz{10.0};
    std::array<double, 7> fc_friction{};
    std::array<double, 7> fc_gain{};
    int movej_speed{600};
    bool prefer_robot_toolset{true};

    Config() {
        fc_friction.fill(0.1);
        fc_gain.fill(30.0);
    }
};

inline void readToolsetLoad(rokae::ArRobot& robot,
                            rokae::Load& load,
                            bool& has_load,
                            bool prefer_robot_toolset,
                            const std::string& tag) {
    error_code ec;
    rokae::Toolset toolset = robot.toolset(ec);
    if (ec) {
        NEXUS_COUT << "[" << tag << "] toolset() failed: " << ec.message()
                   << ", using YAML load if configured" << std::endl;
        return;
    }

    NEXUS_COUT << "[" << tag << "] Pre-calibration toolset load: mass=" << toolset.load.mass
               << "kg, cog=[" << toolset.load.cog[0] << "," << toolset.load.cog[1] << ","
               << toolset.load.cog[2] << "], inertia=[" << toolset.load.inertia[0] << ","
               << toolset.load.inertia[1] << "," << toolset.load.inertia[2] << "]" << std::endl;

    if (prefer_robot_toolset) {
        if (toolset.load.mass > 0.0) {
            load = toolset.load;
            has_load = true;
        }
    } else if (toolset.load.mass > 0.0) {
        load = toolset.load;
        has_load = true;
    }
}

inline void applyRtForceControlSetup(rokae::ArRobot& robot,
                                     rokae::RtMotionControlCobot<7>& rt_controller,
                                     const Config& cfg,
                                     const rokae::Load& load,
                                     bool has_load,
                                     const std::string& tag) {
    error_code ec;

    if (has_load) {
        rt_controller.setLoad(load, ec);
        if (ec) {
            NEXUS_CERR << "[" << tag << "] setLoad failed: " << ec.message() << std::endl;
        } else {
            NEXUS_COUT << "[" << tag << "] RT load set: mass=" << load.mass
                       << "kg, cog=[" << load.cog[0] << "," << load.cog[1] << ","
                       << load.cog[2] << "], inertia=[" << load.inertia[0] << ","
                       << load.inertia[1] << "," << load.inertia[2] << "]" << std::endl;
        }
    }

    rt_controller.setFilterFrequency(
        cfg.filter_joint_hz, cfg.filter_cartesian_hz, cfg.filter_torque_hz, ec);
    if (ec) {
        NEXUS_CERR << "[" << tag << "] setFilterFrequency failed: " << ec.message() << std::endl;
    } else {
        NEXUS_COUT << "[" << tag << "] Filter frequency set: "
                   << cfg.filter_joint_hz << "/" << cfg.filter_cartesian_hz << "/"
                   << cfg.filter_torque_hz << " Hz" << std::endl;
    }

    auto fc = robot.forceControl();
    fc.setFriction(cfg.fc_friction, ec);
    if (ec) {
        NEXUS_CERR << "[" << tag << "] setFriction failed: " << ec.message() << std::endl;
    }

    fc.setFcGain(cfg.fc_gain, ec);
    if (ec) {
        NEXUS_CERR << "[" << tag << "] setFcGain failed: " << ec.message() << std::endl;
    } else {
        NEXUS_COUT << "[" << tag << "] Force control friction/gain configured." << std::endl;
    }
}

}  // namespace ar5_rt_setup
}  // namespace teleop_adapter

#endif  // TELEOP_ADAPTER_AR5_RT_SETUP_HPP
