//
// Created by ye on 2026/10/30.
//

#ifndef THREE_FRICTION_HAL_CAN_HPP
#define THREE_FRICTION_HAL_CAN_HPP

#include <etl/vector.h>
#include <stdint.h>

// pi-lens-ignore: unused-include
#include "can.h"

class CanNode;

namespace canCfg {
// STM32F407 双 bxCAN 共享 28 个过滤器组：CAN1 用 0~13，CAN2 用 14~27
inline constexpr uint32_t kTotalFilterBanks = 28U;
inline constexpr uint32_t kCan1FilterBank = 0U;
inline constexpr uint32_t kCan2FilterBank = kTotalFilterBanks / 2U;  // 14

inline constexpr uint32_t kStdIdMask = 0x7FFU;
inline constexpr uint32_t kExtIdMask = 0x1FFFFFFFU;

// 掩码预设
inline constexpr uint32_t kIdMaskNone = 0x7FF;    // 标准帧11位全匹配
inline constexpr uint32_t kIdMaskLow8 = 0x780;    // 只比较高4位(ID 分组)
inline constexpr uint32_t kExtIdMaskFull = 0x1FFFFFFFU;  // 扩展帧29位全匹配（VESC 等）

inline constexpr uint8_t kCanNodeSizeMax = 32;

// 累计收到的 CAN 帧数（含未匹配任何节点的帧），用于诊断
inline volatile uint32_t g_rx_frame_count = 0U;
}  // namespace canCfg

class CanNode {
   public:
    // 构造即注册：把自身加入全局节点表供中断分发
    CanNode(CAN_HandleTypeDef* hcan, uint32_t id, uint32_t feedback_id, uint32_t id_mask)
        : hcan_(hcan), id_(id), feedback_id_(feedback_id), id_mask_(id_mask) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (node_tab_.size() < canCfg::kCanNodeSizeMax)
            node_tab_.push_back(this);
        else
            __BKPT(0);
        __set_PRIMASK(primask);
    }

    // 析构即注销：从全局节点表移除
    virtual ~CanNode() {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        node_tab_.erase(etl::remove(node_tab_.begin(), node_tab_.end(), this), node_tab_.end());
        __set_PRIMASK(primask);
    }

    // 会自我注册进全局表的对象不能被拷贝：
    // 拷贝构造不执行注册逻辑（对象不在表里），但析构却会去表里删自己。
    CanNode(const CanNode&) = delete;
    CanNode& operator=(const CanNode&) = delete;

    // 只读遍历接口（供中断分发使用），增删仍由构造/析构负责
    static const etl::ivector<CanNode*>& Nodes() { return node_tab_; }

    // 只读访问器：句柄/发送 ID/反馈 ID/过滤掩码
    CAN_HandleTypeDef* Hcan() const { return hcan_; }
    uint32_t Id() const { return id_; }
    uint32_t FeedbackId() const { return feedback_id_; }
    uint32_t IdMask() const { return id_mask_; }

    // 运行期修改绑定参数
    void SetHcan(CAN_HandleTypeDef* hcan) { hcan_ = hcan; }
    void SetId(uint32_t id) { id_ = id; }
    void SetFeedbackId(uint32_t feedback_id) { feedback_id_ = feedback_id; }
    void SetIdMask(uint32_t id_mask) { id_mask_ = id_mask; }

    // 反馈帧回调，由中断按 ID 分发进入
    virtual HAL_StatusTypeDef OnFrame(const uint8_t* data, uint16_t len);
    // 配置过滤器与中断：全部放行，由软件按 ID 分发
    static HAL_StatusTypeDef FilterInit(CAN_HandleTypeDef* hcan);

   protected:
    // 发标准帧（最长 8 字节）
    HAL_StatusTypeDef SendStandard(const uint8_t* data, uint16_t len);
    // 发扩展帧（最长 8 字节）
    HAL_StatusTypeDef SendExtended(const uint8_t* data, uint16_t len);
    // 按显式扩展 ID 发送：用于每条命令高字节不同的协议（如 VESC）
    HAL_StatusTypeDef SendExtendedWithId(uint32_t ext_id, const uint8_t* data, uint16_t len);

   private:
    static etl::vector<CanNode*, canCfg::kCanNodeSizeMax> node_tab_;

    CAN_HandleTypeDef* hcan_ = nullptr;
    uint32_t id_ = 0U;
    uint32_t feedback_id_ = 0U;
    uint32_t id_mask_ = 0U;
};

#endif  // THREE_FRICTION_HAL_CAN_HPP
