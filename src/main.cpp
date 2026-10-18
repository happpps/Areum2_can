#include "RobstrideMotor.hpp"
#include "Rx_handler.hpp"
#include "Sharemem.hpp"
#include <signal.h>
#include <vector>
#include <iostream>

#include <fcntl.h>
#include <pthread.h>
#include <chrono>
#include <array>


const char* CAN_INTERFACE_0 = "can0";
const char* CAN_INTERFACE_1 = "can1";
bool running = true;
constexpr long CONTROL_PERIOD = 3'000'000;
constexpr int MaxID = 34;
constexpr int MOTOR_NUM = 16;

Motor_con AReuMii;


void signal_handler(int signum) { 
    running = false;
    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) {
                motor.write_operation_frame(0, 0, 0);
                motor.control_param.pos = 0;
                motor.control_param.Kp = 0;
                motor.control_param.Kd = 0;
                motor.stop_motor();//motor.clear_fault();usleep(100);
        }
        }(vecs));
    }, AReuMii);
}


void* print_thread_func(void*) {
    while (running) {
        usleep(500000);
        std::apply([](auto&... vecs) {
            (..., [](auto& vec) {
                for (const auto& motor : vec) {
                    std::cout
                        << "[ID 0x" << std::hex << motor.can_id << std::dec << "] "
                        << "pos="   << motor.Feedback_param.pos.load(std::memory_order_relaxed)
                        << " corr=" << motor.Feedback_param.pos.load(std::memory_order_relaxed) + motor.pos_offset
                        << " vel="  << motor.Feedback_param.vel.load(std::memory_order_relaxed)
                        << " torq=" << motor.Feedback_param.torque.load(std::memory_order_relaxed)
                        << " temp=" << motor.Feedback_param.temp.load(std::memory_order_relaxed)
                        << " off="  << motor.pos_offset
                        << "\n";
                }
            }(vecs));
        }, AReuMii);
        std::cout << "---\n";
    }
    return nullptr;
}

void* CAN_Comm_thread(void* arg) {  // TODO : 캘리브레이션을 위한 로직을 따로 뺄 것 그리고 뭐 캔 인터페이스 갯수 맞출 수 있게 탬플릿을 넣거나 
    struct sched_param param;
    param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    Rx_handler<MaxID> hRx(AReuMii);
    can_frame cf;
    auto& can_interface_vec = *static_cast<std::vector<int>*>(arg);
    if (can_interface_vec.size() != 2) {
        std::cerr << "Expected two CAN interfaces.\n";
        return nullptr;
    }

    for(int s : can_interface_vec){
        while (readframe(s, &cf));
    }

    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.write_operation_frame(0, 0, 0);
        }(vecs));
    }, AReuMii);

    // 첫 사이클: 피드백 수신 후 모든 모터 offset 캘리브레이션
    RealTimeClock RTC;
    RTC.wait_next(CONTROL_PERIOD);

    for(int s : can_interface_vec){
        while (readframe(s, &cf)) {
            auto [id, err, p, v, t, tem] = hRx.parse_Rx_frame(&cf);
            if (!err) hRx.Write_Fb(id, p, v, t, tem);
        }
    }

    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.calibrate();
        }(vecs));
    }, AReuMii);

    // 그리퍼는 매 주기가 아니라 명령이 바뀐 순간에만 프레임을 날린다.
    // struct GripperCmd { double pos, vel, Kp, Kd, ffTorque; };
    // constexpr int GRIPPER_VEC_IDX[2] = {VEC_INDEX_LEFT_GRIPPER, VEC_INDEX_RIGHT_GRIPPER};
    // GripperCmd last_gripper[2] = { {NAN,NAN,NAN,NAN,NAN}, {NAN,NAN,NAN,NAN,NAN} };  // 첫 사이클은 무조건 1회 전송

    // Track sustained queue saturation per channel, rather than per motor.
    using Clock = std::chrono::steady_clock;
    std::array<Clock::time_point, 2> buffer_full_since{};
    std::array<unsigned int, 2> restart_attempts{};
    RTC.reset();
    while (running) {
        RTC.wait_next(CONTROL_PERIOD);

        for(int s : can_interface_vec){
            while (readframe(s, &cf)) {
                auto [id, err, p, v, t, tem] = hRx.parse_Rx_frame(&cf);
                if (err) [[unlikely]] {
                    std::cerr << id << " errorcode: " << err;
                    return nullptr;
                }
                hRx.Write_Fb(id, p, v, t, tem);
            }
        }


        // 그리퍼를 뺀 나머지 축은 매 주기 전송
        std::array<bool, 2> buffer_full{};
        std::apply([&](auto&... vecs) {
            (..., [&](auto& vec) {
                for (auto& motor : vec) {
                    //if (motor.can_id == CAN_ID_LEFT_EE || motor.can_id == CAN_ID_RIGHT_EE) continue;
                    const auto it = std::find(can_interface_vec.begin(), can_interface_vec.end(),
                                              motor.get_can_interface());
                    if (it == can_interface_vec.end()) continue;
                    const auto channel = it - can_interface_vec.begin();
                    if (buffer_full[channel]) continue;
                    if (!motor.write_updated_operation_frame() && errno == ENOBUFS) {
                        buffer_full[channel] = true;
                    }
                }
            }(vecs));
        }, AReuMii);

        for (size_t channel = 0; channel < can_interface_vec.size(); ++channel) {
            if (!buffer_full[channel]) {
                buffer_full_since[channel] = Clock::time_point{};
                restart_attempts[channel] = 0;
                continue;
            }
            const auto now = Clock::now();
            if (buffer_full_since[channel] == Clock::time_point{}) {
                buffer_full_since[channel] = now;
            }
            if (now - buffer_full_since[channel] < std::chrono::milliseconds(100)) continue;

            if (restart_attempts[channel] >= 3) {
                std::cerr << "CAN channel " << channel << ": recovery failed after 3 attempts; stopping CAN loop.\n";
                return nullptr;
            }
            ++restart_attempts[channel];
            std::cerr << "CAN channel " << channel << ": ENOBUFS persisted; restarting interface.\n";
            if (!restart_can(can_interface_vec[channel])) {
                std::cerr << "CAN restart failed; stopping CAN loop.\n";
                return nullptr;
            }
            buffer_full_since[channel] = Clock::time_point{};
            RTC.reset();
        }

        // // 그리퍼: 직전 전송값과 다를 때만 전송
        // bool gripper_sent = false;
        // for (int i = 0; i < 2; ++i) {
        //     auto& g = std::get<EL05_Vec>(AReuMii)[GRIPPER_VEC_IDX[i]];
        //     GripperCmd cur{
        //         g.control_param.pos.load(std::memory_order_relaxed),
        //         g.control_param.vel.load(std::memory_order_relaxed),
        //         g.control_param.Kp.load(std::memory_order_relaxed),
        //         g.control_param.Kd.load(std::memory_order_relaxed),
        //         g.control_param.ffTorque.load(std::memory_order_relaxed)
        //     };
        //     if (cur.pos == last_gripper[i].pos && cur.vel == last_gripper[i].vel &&
        //         cur.Kp  == last_gripper[i].Kp  && cur.Kd  == last_gripper[i].Kd  &&
        //         cur.ffTorque == last_gripper[i].ffTorque) continue;

        //     g.write_updated_operation_frame();
        //     last_gripper[i] = cur;
        //     gripper_sent = true;
        // }

        // 그리퍼 프레임이 나간 만큼 주기가 밀리므로 기준 시각을 다시 잡고 다음 사이클로
        // if (gripper_sent) RTC.reset();
    }

    return NULL;
}

void* update_Control_params(void* args){

    Control_Shm<MOTOR_NUM> ctrl_shm(13563267);
    auto* shm_ptr = ctrl_shm.get();

    Control_param ctrl_buf[MOTOR_NUM];
    Feedback_Param fb_buf[MOTOR_NUM];

    // 왼팔 Kp
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.Kp.store(10, std::memory_order_relaxed);
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.Kp.store(10, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.Kp.store(10, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.Kp.store(10, std::memory_order_relaxed);

    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.Kp.store(3, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.Kp.store(3, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.Kp.store(3, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].control_param.Kp.store(3, std::memory_order_relaxed);

    // // 오른팔 Kp
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.Kp.store(10, std::memory_order_relaxed);
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.Kp.store(10, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.Kp.store(10, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.Kp.store(10, std::memory_order_relaxed);

    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].control_param.Kp.store(3, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].control_param.Kp.store(3, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].control_param.Kp.store(3, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].control_param.Kp.store(3, std::memory_order_relaxed);


    // // 왼팔 Kd
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.Kd.store(4, std::memory_order_relaxed);
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.Kd.store(4, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.Kd.store(4, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.Kd.store(4, std::memory_order_relaxed);

    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.Kd.store(1, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.Kd.store(1, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.Kd.store(1, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].control_param.Kd.store(1, std::memory_order_relaxed);

    // // 오른팔 Kd
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.Kd.store(4, std::memory_order_relaxed);
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.Kd.store(4, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.Kd.store(4, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.Kd.store(4, std::memory_order_relaxed);

    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].control_param.Kd.store(1, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].control_param.Kd.store(1, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].control_param.Kd.store(1, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].control_param.Kd.store(1, std::memory_order_relaxed);


    while(running){
        while (!shm_ptr->try_read_ctrl(ctrl_buf));  // torn read면 재시도

        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.pos.store(std::clamp(-ctrl_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);  // 라디안임.
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].pos, -2.0, 2.0), std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.pos.store(std::clamp(-ctrl_buf[SHM_INDEX_LEFT_SHOULDER_YAW].pos, -2.0, 2.0), std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.pos.store(std::clamp(-ctrl_buf[SHM_INDEX_LEFT_ELBOW_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.pos.store(std::clamp(-ctrl_buf[SHM_INDEX_LEFT_WRIST_ROLL].pos, -2.0, 2.0), std::memory_order_relaxed);  // 라디안임.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_WRIST_YAW].pos, -2.0, 2.0), std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_WRIST_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_GRIPPER].pos, -2.0, 2.0), std::memory_order_relaxed);


        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.vel.store(-ctrl_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].vel, std::memory_order_relaxed);  // 라디안임.
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].vel,  std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.vel.store(-ctrl_buf[SHM_INDEX_LEFT_SHOULDER_YAW].vel,  std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.vel.store(-ctrl_buf[SHM_INDEX_LEFT_ELBOW_PITCH].vel, std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.vel.store(-ctrl_buf[SHM_INDEX_LEFT_WRIST_ROLL].vel, std::memory_order_relaxed);  // 라디안임.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_YAW].vel, std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_PITCH].vel, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_GRIPPER].vel, std::memory_order_relaxed);


        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);  // 라디안임.
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.pos.store(-std::clamp(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].pos, -2.0, 0.3), std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].pos, -1.5, 1.5), std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_WRIST_ROLL].pos, -2.0, 2.0), std::memory_order_relaxed);  // 라디안임.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].control_param.pos.store(-std::clamp(ctrl_buf[SHM_INDEX_RIGHT_WRIST_YAW].pos, -2.0, 2.0), std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].control_param.pos.store(-std::clamp(ctrl_buf[SHM_INDEX_RIGHT_WRIST_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_GRIPPER].pos, -2.0, 2.0), std::memory_order_relaxed);


        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].vel, std::memory_order_relaxed);  // 라디안임.
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.vel.store(-ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].vel,  std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].vel,  std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].vel, std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_ROLL].vel, std::memory_order_relaxed);  // 라디안임.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].control_param.vel.store(-ctrl_buf[SHM_INDEX_RIGHT_WRIST_YAW].vel, std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].control_param.vel.store(-ctrl_buf[SHM_INDEX_RIGHT_WRIST_PITCH].vel, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_GRIPPER].vel, std::memory_order_relaxed);


        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].Kp, std::memory_order_relaxed);
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].Kp, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_YAW].Kp, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_ELBOW_PITCH].Kp, std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_ROLL].Kp, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_YAW].Kp, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_PITCH].Kp, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_GRIPPER].Kp, std::memory_order_relaxed);

        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].Kp, std::memory_order_relaxed);
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].Kp, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].Kp, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].Kp, std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_ROLL].Kp, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_YAW].Kp, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_PITCH].Kp, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_GRIPPER].Kp, std::memory_order_relaxed);


        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].Kd, std::memory_order_relaxed);
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].Kd, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_YAW].Kd, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_ELBOW_PITCH].Kd, std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_ROLL].Kd, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_YAW].Kd, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_PITCH].Kd, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_GRIPPER].Kd, std::memory_order_relaxed);

        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].Kd, std::memory_order_relaxed);
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].Kd, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].Kd, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].Kd, std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_ROLL].Kd, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_YAW].Kd, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_PITCH].Kd, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_GRIPPER].Kd, std::memory_order_relaxed);




        fb_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].pos = -std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].pos_offset;
        fb_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].pos = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].pos_offset;
        fb_buf[SHM_INDEX_LEFT_SHOULDER_YAW].pos = -std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].pos_offset;
        fb_buf[SHM_INDEX_LEFT_ELBOW_PITCH].pos = -std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].pos_offset;
        
        fb_buf[SHM_INDEX_LEFT_WRIST_ROLL].pos = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].pos_offset;
        fb_buf[SHM_INDEX_LEFT_WRIST_YAW].pos = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].pos_offset;
        fb_buf[SHM_INDEX_LEFT_WRIST_PITCH].pos = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].pos_offset;
        fb_buf[SHM_INDEX_LEFT_GRIPPER].pos = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].pos_offset;


        fb_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].vel = -std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].vel = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_SHOULDER_YAW].vel = -std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_ELBOW_PITCH].vel = -std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        
        fb_buf[SHM_INDEX_LEFT_WRIST_ROLL].vel = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_WRIST_YAW].vel = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_WRIST_PITCH].vel = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_GRIPPER].vel = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].Feedback_param.vel.load(std::memory_order_relaxed);


        fb_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].torque = -std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].torque = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_SHOULDER_YAW].torque = -std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_ELBOW_PITCH].torque = -std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[SHM_INDEX_LEFT_WRIST_ROLL].torque = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_WRIST_YAW].torque = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_WRIST_PITCH].torque = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_GRIPPER].torque = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].Feedback_param.torque.load(std::memory_order_relaxed);


        fb_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].pos = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].pos = -std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].pos = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].pos = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].pos_offset;
        
        fb_buf[SHM_INDEX_RIGHT_WRIST_ROLL].pos = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_WRIST_YAW].pos = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_WRIST_PITCH].pos = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_GRIPPER].pos = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].pos_offset;


        fb_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].vel = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].vel = -std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].vel = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].vel = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        
        fb_buf[SHM_INDEX_RIGHT_WRIST_ROLL].vel = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_WRIST_YAW].vel = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_WRIST_PITCH].vel = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_GRIPPER].vel = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].Feedback_param.vel.load(std::memory_order_relaxed);


        fb_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].torque = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].torque = -std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].torque = -std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].torque = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[SHM_INDEX_RIGHT_WRIST_ROLL].torque = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_WRIST_YAW].torque = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_WRIST_PITCH].torque = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_GRIPPER].torque = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].Feedback_param.torque.load(std::memory_order_relaxed);


        // fb_buf[0].temp = std::get<RS02_Vec>(AReuMii)[0].Feedback_param.temp.load(std::memory_order_relaxed);
        // fb_buf[1].temp = std::get<RS02_Vec>(AReuMii)[1].Feedback_param.temp.load(std::memory_order_relaxed);
        // fb_buf[2].temp = std::get<RS00_Vec>(AReuMii)[0].Feedback_param.temp.load(std::memory_order_relaxed);
        // fb_buf[3].temp = std::get<RS00_Vec>(AReuMii)[1].Feedback_param.temp.load(std::memory_order_relaxed);
        shm_ptr->write_fb(fb_buf);

       usleep(1000);
    }
    return nullptr;
}


int main() {

    signal(SIGINT, signal_handler);



    int s1 = init_can(CAN_INTERFACE_0);
    if (s1 < 0) return -1;

    int s2 = init_can(CAN_INTERFACE_1);
    if (s2 < 0) return -1;

    std::vector<int> can_interface = {s1 , s2};

    std::get<RS02_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_SHOULDER_PITCH);
    std::get<RS02_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_SHOULDER_ROLL);
    std::get<RS00_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_SHOULDER_YAW);
    std::get<RS00_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_ELBOW);

    std::get<EL05_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_WRIST_ROLL);
    std::get<EL05_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_WRIST_YAW);
    std::get<EL05_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_WRIST_PITCH);
    std::get<EL05_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_EE);


    std::get<RS02_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_SHOULDER_PITCH);
    std::get<RS02_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_SHOULDER_ROLL);
    std::get<RS00_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_SHOULDER_YAW);
    std::get<RS00_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_ELBOW);

    std::get<EL05_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_WRIST_ROLL);
    std::get<EL05_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_WRIST_YAW);
    std::get<EL05_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_WRIST_PITCH);
    std::get<EL05_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_EE);


    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec){
                motor.init_motor_MIT(10, 10);
                motor.stop_motor();
                usleep(10000);
            }
        }(vecs));
    }, AReuMii);

    usleep(100000);

    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec){
                motor.init_motor_MIT(10, 10);
            }
        }(vecs));
    }, AReuMii);


    //여기서 스레드 생성
    pthread_t rt_t, print_t, shm_t;
    pthread_create(&rt_t,    NULL, CAN_Comm_thread,   &can_interface);
    pthread_create(&print_t, NULL, print_thread_func,  nullptr);  // 초반 확인용이라 나중에는 안쓰는 스레드임. 
    pthread_create(&shm_t, NULL, update_Control_params,  nullptr);



    pthread_join(rt_t,    nullptr);
    pthread_join(print_t, nullptr);
    pthread_join(shm_t , nullptr);

    return 0;
}
