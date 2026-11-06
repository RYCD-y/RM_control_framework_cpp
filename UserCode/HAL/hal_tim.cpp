#include "hal_tim.hpp"

namespace tim_util {

// 取定时器计数时钟：按挂载总线选 APB1/APB2，分频不为 1 时翻倍
uint32_t ClockFreqHz(const TIM_TypeDef* tim) {
    if (tim == nullptr) return 0U;
    RCC_ClkInitTypeDef clk_config = {};
    uint32_t latency = 0U;
    HAL_RCC_GetClockConfig(&clk_config, &latency);

    const bool on_apb2 = tim == TIM1 || tim == TIM8 || tim == TIM9 || tim == TIM10 || tim == TIM11;
    const uint32_t pclk = on_apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    const uint32_t divider = on_apb2 ? clk_config.APB2CLKDivider : clk_config.APB1CLKDivider;
    return divider == RCC_HCLK_DIV1 ? pclk : pclk * 2U;
}

// 1us 对应的计数值 = 计数时钟 / (PSC+1) / 1e6
float TicksPerUs(const TIM_HandleTypeDef* htim) {
    if (htim == nullptr) return 0.0f;
    const uint32_t psc = static_cast<uint32_t>(htim->Instance->PSC) + 1U;
    if (psc == 0U) return 0.0f;
    return static_cast<float>(ClockFreqHz(htim->Instance)) / (static_cast<float>(psc) * 1000000.0f);
}

}  // namespace tim_util
