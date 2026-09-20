/* ==================================================================
 * target_port.h — 目标芯片移植层（从 jtag_tool.c 剥离的厂商特有配置）
 *
 * Cortex-M3 核心调试功能（SWD/JTAG、halt/step、寄存器、内存、FPB/DWT、
 * ITM/TPIU）是 ARM 标准，任何 CM3 都能工作。以下部分是芯片厂商定义的，
 * 移植到非 STM32 芯片时只需改这个文件。
 *
 * 当前支持：STM32F1（已验证）
 * 兼容芯片（寄存器克隆）：GD32F1、CH32F1、CS32F1 等 — 零修改
 * 需适配：STM32F4/L4/H7、NXP LPC、TI Tiva 等
 * ================================================================== */
#ifndef TARGET_PORT_H
#define TARGET_PORT_H

/* ---- 调试使能寄存器（睡眠/停止模式下保持内核时钟） ----
 * 作用：固件 WFI 睡眠时内核时钟不停，AP 访问才不会全部 WAIT
 *
 * STM32F1: DBGMCU_CR @ 0xE0042004, 写 0x27 = DBG_SLEEP|STOP|STANDBY|TRACE_IOEN
 * STM32F4: DBGMCU_CR @ 0xE0042004, 位定义相同
 * GD32F1:  兼容 STM32F1
 * NXP LPC: 无此寄存器（不需要，设 TGT_HAS_DBGMCU=0）
 */
#define TGT_HAS_DBGMCU       1
#define TGT_DBGMCU_ADDR      0xE0042004u
#define TGT_DBGMCU_VALUE     0x27u

/* ---- SWO 引脚复用（把 TPIU 输出到物理引脚） ----
 * 作用：不配置则 TPIU 信号出不了芯片引脚，线上只见 SWD 轮询漏流
 *
 * STM32F1: AFIO_MAPR @ 0x40010004, SWJ_CFG bits[26:24]
 *   写 010 = 释放 PB3 给 TPIU（SWO 复用 JTDO 引脚）
 *   必须先经过 000（full SWJ）再到 010——直接写 010 会静默卡死
 *
 * STM32F4/L4/H7: GPIO AFR 寄存器配置 AF0（完全不同的机制）
 * GD32F1/CH32F1: 兼容 STM32F1
 * NXP LPC: SWO 引脚默认可用（设 TGT_HAS_AFIO=0）
 */
#define TGT_HAS_AFIO         1
#define TGT_AFIO_MAPR_ADDR   0x40010004u
#define TGT_SWJ_CFG_SHIFT    24u
#define TGT_SWJ_CFG_SWO      2u    /* 010 = SWO on PB3 */

#endif /* TARGET_PORT_H */
