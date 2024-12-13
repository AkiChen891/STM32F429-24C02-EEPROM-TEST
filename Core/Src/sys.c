/**
 ****************************************************************************************************
 * @file        sys.c
 * @author      ALIENTEK
 * @version     V1.0
 * @date        2022-4-20
 * @brief       系统底层头文件
 ****************************************************************************************************
 */

#include "sys.h"


/**
 * @brief       设置中断向量表偏移地址
 * @param       baseaddr: 基址
 * @param       offset: 偏移量
 * @retval      NULL
 */
void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset)
{
    /* 设置NVIC的向量表偏移寄存器，VTOR低9位保留，即[8:0]保留 */
    SCB->VTOR = baseaddr | (offset & (uint32_t)0xFFFFFE00);
}

/**
 * @brief       执行WFI指令（执行完后进入低功耗状态，等待中断唤醒）
 * @param       NULL
 * @retval      NULL
 */
void sys_wfi_set(void)
{
    __ASM volatile("wfi");
}

/**
 * @brief       关闭所有中断（不包括fault和NMI中断）
 * @param       NULL
 * @retval      NULL
 */
void sys_intx_disable(void)
{
    __ASM volatile("cpsid i");
}

/**
 * @brief       开启所有中断
 * @param       NULL
 * @retval      NULL
 */
void sys_intx_enable(void)
{
    __ASM volatile("cpsie i");
}

/**
 * @brief       设置栈顶地址
 * @note        若有X属于IDE误报
 * @param       addr: 栈顶地址
 * @retval      NULL
 */
void sys_msr_msp(uint32_t addr)
{
    __set_MSP(addr);    /* ַ设置栈顶地址 */
}

/**
 * @brief       进入待机模式
 * @param       NULL
 * @retval      NULL
 */
void sys_standby(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();    /* 使能电源时钟 */
    SET_BIT(PWR->CR, PWR_CR_PDDS); /* 进入待机 */
}

/**
 * @brief       系统软复位
 * @param       NULL
 * @retval      NULL
 */
void sys_soft_reset(void)
{
    NVIC_SystemReset();
}

/**
 * @brief       ʱ�����ú���
 * @param       plln: PLL1��Ƶϵ��(PLL��Ƶ), ȡֵ��Χ: 64~432.
 * @param       pllm: PLL1Ԥ��Ƶϵ��(��PLL֮ǰ�ķ�Ƶ), ȡֵ��Χ: 2~63.
 * @param       pllp: PLL1��p��Ƶϵ��(PLL֮��ķ�Ƶ), ��Ƶ����Ϊϵͳʱ��, ȡֵ��Χ: 2,4,6,8.(������4��ֵ!)
 * @param       pllq: PLL1��q��Ƶϵ��(PLL֮��ķ�Ƶ), ȡֵ��Χ: 2~15.
 * @note
 *
 *              Fvco: VCOƵ��
 *              Fsys: ϵͳʱ��Ƶ��, Ҳ��PLL1��p��Ƶ���ʱ��Ƶ��
 *              Fq:   PLL1��q��Ƶ���ʱ��Ƶ��
 *              Fs:   PLL����ʱ��Ƶ��, ������HSI, HSE��.
 *              Fvco = Fs * (plln / pllm);
 *              Fsys = Fvco / pllp = Fs * (plln / (pllm * pllp));
 *              Fq   = Fvco / pllq = Fs * (plln / (pllm * pllq));
 *
 *              �ⲿ����Ϊ25M��ʱ��, �Ƽ�ֵ: plln = 360, pllm = 25, pllp = 2, pllq = 8.
 *              �õ�:Fvco = 25 * (360 / 25) = 360Mhz
 *                   Fsys = pll1_p_ck = 360 / 2 = 180Mhz
 *                   Fq   = pll1_q_ck = 360 / 8 = 45(ʹ��USBʱ��������plln=384�����ɵõ�48MhzƵ��, ��ʱ��ƵΪ192M)
 *
 *              F429Ĭ����Ҫ���õ�Ƶ������:
 *              CPUƵ��(HCLK) = pll_p_ck = 180Mhz
 *              AHB1/2/3(rcc_hclk1/2/3) = 180Mhz
 *              APB1(rcc_pclk1) = pll_p_ck / 4 = 45Mhz
 *              APB2(rcc_pclk2) = pll_p_ck / 2 = 90Mhz
 *
 * @retval      �������: 0, �ɹ�; 1, ����;
 */
uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq)
{
    HAL_StatusTypeDef ret = HAL_OK;
    RCC_ClkInitTypeDef rcc_clk_init = {0};
    RCC_OscInitTypeDef rcc_osc_init = {0};
    
    __HAL_RCC_PWR_CLK_ENABLE();                                     /* ʹ��PWRʱ�� */
    
    /* ������������������õ�ѹ�������ѹ�����Ա�������δ�����Ƶ�ʹ���ʱʹ�����빦��ʵ��ƽ�� */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  /* ��ѹ�������ѹ����ѡ�񣺼���1ģʽ */

    /* ʹ��HSE����ѡ��HSE��ΪPLLʱ��Դ������PLL1������USBʱ�� */
    rcc_osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;           /* ʱ��ԴΪHSE */
    rcc_osc_init.HSEState = RCC_HSE_ON;                             /* ��HSE */
    rcc_osc_init.PLL.PLLState = RCC_PLL_ON;                         /* ��PLL */
    rcc_osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;                 /* PLLʱ��Դѡ��HSE */
    rcc_osc_init.PLL.PLLN = plln;
    rcc_osc_init.PLL.PLLM = pllm;
    rcc_osc_init.PLL.PLLP = pllp;
    rcc_osc_init.PLL.PLLQ = pllq;
    ret = HAL_RCC_OscConfig(&rcc_osc_init);                         /* ��ʼ��RCC */
    if (ret != HAL_OK)
    {
        return 1;                                                   /* ʱ�ӳ�ʼ��ʧ�ܣ���������������Լ��Ĵ��� */
    }

    ret = HAL_PWREx_EnableOverDrive();                              /* ����Over-Driver���� */
    if (ret != HAL_OK)
    {
        return 1;
    }

    /* ѡ��PLL��Ϊϵͳʱ��Դ��������HCLK,PCLK1��PCLK2*/
    rcc_clk_init.ClockType = ( RCC_CLOCKTYPE_SYSCLK \
                                    | RCC_CLOCKTYPE_HCLK \
                                    | RCC_CLOCKTYPE_PCLK1 \
                                    | RCC_CLOCKTYPE_PCLK2);

    rcc_clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;            /* ����ϵͳʱ��ʱ��ԴΪPLL */
    rcc_clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;                   /* AHB��Ƶϵ��Ϊ1 */
    rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV4;                    /* APB1��Ƶϵ��Ϊ4 */
    rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV2;                    /* APB2��Ƶϵ��Ϊ2 */
    ret = HAL_RCC_ClockConfig(&rcc_clk_init, FLASH_LATENCY_5);      /* ͬʱ����FLASH��ʱ����Ϊ5WS��Ҳ����6��CPU���� */
    if (ret != HAL_OK)
    {
        return 1;                                                   /* ʱ�ӳ�ʼ��ʧ�� */
    }

    return 0;
}


#ifdef  USE_FULL_ASSERT

/**
 * @brief       ��������ʾ������ʱ��˺����������������ļ���������
 * @param       file��ָ��Դ�ļ�
 * @param       line��ָ�����ļ��е�����
 * @retval      ��
 */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    while (1)
    {
    }
}

#endif




