/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:17:52
 */

#pragma once
#include <cstring>
// projects
#include "internal/actuator_base.h"

namespace xyber_omni_picker {

class OmniPicker : public xyber::Actuator {
 public:

  explicit OmniPicker(std::string name, uint8_t id, xyber::CtrlChannel ctrl_ch)
      : Actuator(name, xyber::ActuatorType::OMNI_PICKER, id, ctrl_ch) {}
  ~OmniPicker() {}

 public:
  virtual float GetEffort() override { return state_data_->cur / 255.0f; }
  virtual float GetVelocity() override { return state_data_->vel / 255.0f; }
  virtual float GetPosition() override { return state_data_->pos / 255.0f; }
  virtual void SetMitCmd(float pos, float, float cur, float, float) override {
    // // ================= [诊断代码开始] =================
    // // 逻辑：每隔 1 秒打印一次夹爪发回来的数据
    // static auto last_print_time = std::chrono::steady_clock::now();
    // auto now = std::chrono::steady_clock::now();

    // if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count() >= 1) {
    //     if (state_data_ != nullptr) {
    //         // 这里打印的是【夹爪发给你的数据】(Feedback/RX)
    //         // ID: 你的代码认为的ID
    //         // Err: 错误码 (0x00 为正常)
    //         // State: 状态位 (0x00 通常是没通信上或没使能)
    //         // ActPos: 实际位置 (如果你掰动夹爪，这个数应该会变)
    //         printf("[CHECK] ID=%d | Feedback: Err=0x%02X, State=0x%02X, ActPos=%d\n", 
    //                id_, 
    //                state_data_->error_code, 
    //                state_data_->state, 
    //                state_data_->pos);
    //     } else {
    //         printf("[CHECK] ID=%d | Feedback is NULL (No Data)\n", id_);
    //     }
    //     last_print_time = now;
    // }
    // // ================= [诊断代码结束] =================
    if (pos > 1) {
      pos = 1;
    }
    if (cur > 1) {
      cur = 1;
    }
    cmd_data_->pos = 0xFF * pos;
    cmd_data_->cur = 0xFF * cur;
    cmd_data_->speed = 0xFF;  // TODO: hardcoding for claw, cause it` using different protocol
    cmd_data_->acc = 0xFF;
    cmd_data_->dcc = 0xFF;
  }

 private:
  virtual void SetDataFiled(uint8_t* send, uint8_t* recv) override {
    send_buf_ = send + ACTUATOR_FRAME_SIZE * (id_ - 1);
    recv_buf_ = recv + ACTUATOR_FRAME_SIZE * (id_ - 1);

    cmd_data_ = (CmdData*)send_buf_;
    state_data_ = (StateData*)recv_buf_;
  }

 private:
  struct CmdData {
    uint8_t reserved_1 = 0;
    uint8_t pos = 0;
    uint8_t cur = 0;
    uint8_t speed = 0;
    uint8_t acc = 0;
    uint8_t dcc = 0;
    uint16_t reserved_2 = 0;
  };

  struct StateData {
    uint8_t error_code = 0;
    uint8_t state = 0;
    uint8_t pos = 0;
    uint8_t vel = 0;
    uint8_t cur = 0;
    uint8_t reserved_1 = 0;
    uint16_t reserved_2 = 0;
  };

  CmdData* cmd_data_ = nullptr;
  StateData* state_data_ = nullptr;
};

}  // namespace xyber_omni_picker
