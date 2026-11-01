//
// Created by ye on 2026/10/30.
//

#ifndef THREE_FRICTION_HAL_CAN_HPP
#define THREE_FRICTION_HAL_CAN_HPP

#include <stdint.h>
#include "can.h"
#include <etl/vector.h>

class CanNode;

namespace canCfg
{
        // STM32F407 双 bxCAN 共享 28 个过滤器组：CAN1 用 0~13，CAN2 用 14~27
        inline constexpr uint32_t kTotalFilterBanks = 28U;
        inline constexpr uint32_t kCan1FilterBank   = 0U;
        inline constexpr uint32_t kCan2FilterBank   = kTotalFilterBanks / 2U; // 14

        inline constexpr uint32_t kStdIdMask = 0x7FFU;
        inline constexpr uint32_t kExtIdMask = 0x1FFFFFFFU;

        //掩码预设
        inline constexpr uint32_t kIdMaskNone = 0x7FF; //标准帧11位全匹配
        inline constexpr uint32_t kIdMaskLow8 = 0x780; //只比较高4位(ID 分组)

        inline constexpr uint8_t kCanNodeSizeMax = 32;
}

class CanNode
{
        public:
                CanNode(
                        CAN_HandleTypeDef *hcan,
                        uint32_t           id,
                        uint32_t           feedback_id,
                        uint32_t           id_mask
                )
                        : hcan_(hcan),
                          id_(id),
                          feedback_id_(feedback_id),
                          id_mask_(id_mask)
                {
                        uint32_t primask = __get_PRIMASK();
                        __disable_irq();
                        if (node_tab.size() < canCfg::kCanNodeSizeMax)
                                node_tab.push_back(this);
                        else
                                __BKPT(0);
                        __set_PRIMASK(primask);
                }

                virtual ~CanNode()
                {
                        uint32_t primask = __get_PRIMASK();
                        __disable_irq();
                        node_tab.erase(
                                       etl::remove(
                                                   node_tab.begin(),
                                                   node_tab.end(), this
                                                  ),
                                       node_tab.end()
                                      );
                        __set_PRIMASK(primask);
                }

                // 会自我注册进全局表的对象不能被拷贝：
                // 拷贝构造不执行注册逻辑（对象不在表里），但析构却会去表里删自己。
                CanNode(const CanNode &)            = delete;
                CanNode &operator=(const CanNode &) = delete;



                // 只读遍历接口（供中断分发使用），增删仍由构造/析构负责
                static const etl::ivector<CanNode *> &nodes() { return node_tab; }

                CAN_HandleTypeDef *getHcan() const { return hcan_; }
                uint32_t           getId() const { return id_; }
                uint32_t           getFeedbackId() const { return feedback_id_; }
                uint32_t           getIdMask() const { return id_mask_; }

                void setHcan(CAN_HandleTypeDef *hcan) { hcan_ = hcan; };
                void setId(uint32_t id) { id_ = id; }
                void setFeedbackId(uint32_t feedback_id) { feedback_id_ = feedback_id; }
                void setIdMask(uint32_t id_mask) { id_mask_ = id_mask; }

                virtual HAL_StatusTypeDef onFrame(const uint8_t *data, uint16_t len);

                //CAN过滤器初始化: CAN1→bank0, CAN2→bank14, 全部放行→FIFO0
                static HAL_StatusTypeDef filterInit(CAN_HandleTypeDef *hcan);


        protected:
                HAL_StatusTypeDef send(const uint8_t *data, uint16_t len);

                HAL_StatusTypeDef sendEx(const uint8_t *data, uint16_t len);

        private:
                static etl::vector<CanNode *, canCfg::kCanNodeSizeMax> node_tab;

                CAN_HandleTypeDef *hcan_        = nullptr;
                uint32_t           id_          = 0x00;
                uint32_t           feedback_id_ = 0x00;
                uint32_t           id_mask_     = 0x00;
};

#endif //THREE_FRICTION_HAL_CAN_HPP
