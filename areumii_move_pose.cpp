#include "Sharemem.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

constexpr int MOTOR_NUM = 16;
constexpr int LEFT_BEGIN = 0;
constexpr int RIGHT_BEGIN = 8;
constexpr int ARM_DOF = 7;
constexpr key_t SHM_KEY = 13563267;

// URDF/model joint order:
// 0 shoulder_pitch
// 1 shoulder_roll
// 2 shoulder_yaw
// 3 elbow
// 4 wrist_roll
// 5 wrist_yaw
// 6 wrist_pitch
//
// Real wrist_yaw direction is reversed.
constexpr std::array<double, ARM_DOF> LEFT_SIGN = {
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
};

constexpr std::array<double, ARM_DOF> RIGHT_SIGN = {
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
};

constexpr std::array<double, ARM_DOF> READY_Q = {
    0.5, 0.1, 0.1, 1.5, 0.0, 0.0, 0.7
};

// 테이블 얹은 자세
constexpr std::array<double, ARM_DOF> DOWN_Q = {
   -0.1, 0.0, 0.0, 0.8, 0.0, 0.0, 0.8
};
// constexpr std::array<double, ARM_DOF> DOWN_Q = {
//    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
// };

constexpr double MOVE_TIME_SEC = 3.0;
constexpr double LOOP_DT_SEC = 0.01;  // 100 Hz

struct ArmConfig {
    const char* name;
    int begin;
    std::array<double, ARM_DOF> sign;
};

constexpr ArmConfig LEFT_ARM = {"left", LEFT_BEGIN, LEFT_SIGN};
constexpr ArmConfig RIGHT_ARM = {"right", RIGHT_BEGIN, RIGHT_SIGN};

std::array<double, ARM_DOF> read_start_q(
    const Feedback_Param* fb,
    const ArmConfig& arm
) {
    std::array<double, ARM_DOF> start_q{};

    for (int i = 0; i < ARM_DOF; ++i) {
        // Convert hardware feedback -> URDF/model convention.
        start_q[i] = arm.sign[i] * static_cast<double>(fb[arm.begin + i].pos);
    }

    return start_q;
}

void write_arm_q(
    Control_param* ctrl,
    const ArmConfig& arm,
    const std::array<double, ARM_DOF>& q
) {
    for (int i = 0; i < ARM_DOF; ++i) {
        // Convert URDF/model convention -> hardware convention.
        ctrl[arm.begin + i].pos = arm.sign[i] * q[i];
        ctrl[arm.begin + i].vel = 0.0;
    }
}

std::array<double, ARM_DOF> blend_q(
    const std::array<double, ARM_DOF>& start_q,
    const std::array<double, ARM_DOF>& target,
    double s
) {
    std::array<double, ARM_DOF> q{};

    for (int i = 0; i < ARM_DOF; ++i) {
        q[i] = start_q[i] + s * (target[i] - start_q[i]);
    }

    return q;
}

void zero_arm(Control_param* ctrl, const ArmConfig& arm) {
    for (int i = 0; i < ARM_DOF; ++i) {
        ctrl[arm.begin + i] = {0};
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0]
                  << " ready|down\n";
        return 1;
    }

    std::array<double, ARM_DOF> target{};

    const std::string mode = argv[1];
    if (mode == "ready") {
        target = READY_Q;
    } else if (mode == "down") {
        target = DOWN_Q;
    } else {
        std::cerr << "Use 'ready' or 'down'.\n";
        return 1;
    }

    Control_Shm<MOTOR_NUM> shared(SHM_KEY);
    auto* shm = shared.get();

    Control_param ctrl[MOTOR_NUM]{};
    Feedback_Param fb[MOTOR_NUM]{};

    // Read current feedback.
    if (!shm->try_read_fb(fb)) {
        std::cerr << "Failed to read feedback from shared memory.\n";
        return 1;
    }

    if (!shm->try_read_ctrl(ctrl)) {
        std::cerr << "Failed to read control data from shared memory.\n";
        return 1;
    }

    std::cout << "[move_pose] "
              << (mode == "ready" ? "READY" : "DOWN")
              << " both"
              << " : current pose -> target in "
              << MOVE_TIME_SEC << " s\n";

    const int steps = static_cast<int>(MOVE_TIME_SEC / LOOP_DT_SEC);
    const std::array<double, ARM_DOF> left_start_q =
        read_start_q(fb, LEFT_ARM);
    const std::array<double, ARM_DOF> right_start_q =
        read_start_q(fb, RIGHT_ARM);

    ctrl[0].Kp =50;    
    ctrl[1].Kp =30;
    ctrl[2].Kp =30;
    ctrl[3].Kp =50;

    ctrl[4].Kp =3;
    ctrl[5].Kp =3;
    ctrl[6].Kp =10;
    ctrl[7].Kp =3;

    ctrl[8].Kp =50;    
    ctrl[9].Kp =30;
    ctrl[10].Kp =50;
    ctrl[11].Kp =30;

    ctrl[12].Kp =3;
    ctrl[13].Kp =3;
    ctrl[14].Kp =10;
    ctrl[15].Kp =3;


    ctrl[0].Kd =2;    
    ctrl[1].Kd =2;
    ctrl[2].Kd =2;
    ctrl[3].Kd =2;

    ctrl[4].Kd =1;
    ctrl[5].Kd =1;
    ctrl[6].Kd =1;
    ctrl[7].Kd =1;

    ctrl[8].Kd =2;    
    ctrl[9].Kd =2;
    ctrl[10].Kd =2;
    ctrl[11].Kd =2;

    ctrl[12].Kd =1;
    ctrl[13].Kd =1;
    ctrl[14].Kd =1;
    ctrl[15].Kd =1;


    for (int step = 1; step <= steps; ++step) {
        const double t = static_cast<double>(step) / steps;

        // Smoothstep: still simple, but starts/stops less abruptly than linear t.
        const double s = t * t * (3.0 - 2.0 * t);

        write_arm_q(ctrl, LEFT_ARM, blend_q(left_start_q, target, s));
        write_arm_q(ctrl, RIGHT_ARM, blend_q(right_start_q, target, s));

        shm->write_ctrl(ctrl);
        std::this_thread::sleep_for(
            std::chrono::duration<double>(LOOP_DT_SEC));
    }

    // Write exact final target once more.
    write_arm_q(ctrl, LEFT_ARM, target);
    write_arm_q(ctrl, RIGHT_ARM, target);
    shm->write_ctrl(ctrl);

    std::this_thread::sleep_for(
        std::chrono::duration<double>(LOOP_DT_SEC));
    if(mode == "down"){
        zero_arm(ctrl, LEFT_ARM);
        zero_arm(ctrl, RIGHT_ARM);
    }
    shm->write_ctrl(ctrl);

    std::cout << "[move_pose] done\n";
    return 0;
}
