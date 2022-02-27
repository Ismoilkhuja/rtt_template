#define MAIN_C

#include "stm32f4xx.h"
#include "SEGGER_RTT.h"

__IO uint32_t tick = 0;

void delay_ms(const uint32_t ms){
    uint32_t start = tick;
    while ((tick - start)<ms){}
}

void GPIO_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
    MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE13_1, GPIO_MODER_MODE13_0);
    CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT13);

}

void SetSysClkTo84(void){

    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN);
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2WS);
    while (READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY)!= FLASH_ACR_LATENCY_2WS){}

    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM);
    SET_BIT(RCC->PLLCFGR, 25 << RCC_PLLCFGR_PLLM_Pos);

    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN);
    SET_BIT(RCC->PLLCFGR, 168 << RCC_PLLCFGR_PLLN_Pos);

    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC);

    CLEAR_BIT(RCC->CFGR, RCC_CFGR_HPRE_3);    // AHB prescaler - not divided

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_1, RCC_CFGR_PPRE1_2);   // APB1 prescaler - divided by 2

    CLEAR_BIT(RCC->CFGR, RCC_CFGR_PPRE2_2);    // APB2 prescaler - not divided

    SET_BIT(RCC->CR, RCC_CR_HSEON);
    while(!READ_BIT(RCC->CR, RCC_CR_HSERDY)){}

    SET_BIT(RCC->CR, RCC_CR_PLLON);
    while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY)){}

    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_0, RCC_CFGR_SW_1);  // System clock - PLL

    while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL){}

}



int main(void) {
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    SetSysClkTo84();
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000);
    GPIO_Init();

    while (1) {
        SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
        SEGGER_RTT_WriteString(0, "LED turn ON\n");
        delay_ms(1000);
        SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
        SEGGER_RTT_WriteString(0, "LED turn OFF\n");
        delay_ms(300);
    }

    return 0;
}

void SysTick_Handler(void){
    tick++;
}



