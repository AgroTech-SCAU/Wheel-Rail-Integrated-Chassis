#ifndef _entry_h_
#define _entry_h_

/**
 * @file entry.h
 * @brief CubeMX `main.c` 的应用入口替代层
 */

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化底盘应用
 */
void entry_init(void);

/**
 * @brief 执行一次底盘应用轮询
 */
void entry_loop(void);

#endif
