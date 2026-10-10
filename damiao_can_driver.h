#ifndef DAMIAO_CAN_DRIVER_H
#define DAMIAO_CAN_DRIVER_H

/*
 * 达妙电机 CAN 驱动，面向 STM32 HAL FDCAN。
 *
 * 约定：
 * - 使用经典 CAN，标准 11 位 ID，数据长度最多 8 字节；
 * - 电机反馈帧默认使用 CAN ID 0（master_id 可在初始化时修改）；
 * - 反馈帧中的电机 ID 占 D[0] 的低 4 位，所以电机 ID 建议为 0~15；
 * - STM32 为小端平台，位置/速度/浮点数据按协议以小端发送。
 */

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DM_CAN_BROADCAST_ID       0x7FFU
#define DM_CAN_DEFAULT_MASTER_ID  0x000U
#define DM_CAN_MAX_MOTOR_ID       0x0FU

typedef enum {
    DM_MODE_MIT     = 0x000U,
    DM_MODE_POS_VEL = 0x100U,
    DM_MODE_SPEED   = 0x200U,
    DM_MODE_POS_CUR = 0x300U
} dm_mode_t;

typedef enum {
    DM_STATE_DISABLED       = 0x0,
    DM_STATE_ENABLED        = 0x1,
    DM_STATE_NOT_CALIBRATED = 0x2,
    DM_STATE_OVER_VOLT      = 0x8,
    DM_STATE_UNDER_VOLT     = 0x9,
    DM_STATE_OVER_CURRENT   = 0xA,
    DM_STATE_MOS_OVER_TEMP  = 0xB,
    DM_STATE_COIL_OVER_TEMP = 0xC,
    DM_STATE_COMM_LOST      = 0xD,
    DM_STATE_OVERLOAD       = 0xE
} dm_state_t;

typedef struct {
    uint8_t  id;
    uint8_t  state;
    uint16_t pos_raw;
    uint16_t vel_raw;
    uint16_t torque_raw;
    float    position_rad;
    float    velocity_rad_s;
    float    torque_nm;
    uint8_t  mos_temperature_c;
    uint8_t  coil_temperature_c;
} dm_feedback_t;

typedef void (*dm_feedback_callback_t)(const dm_feedback_t *feedback,
                                       void *user);
typedef void (*dm_raw_frame_callback_t)(uint32_t can_id,
                                        const uint8_t *data,
                                        uint8_t length,
                                        void *user);

typedef struct {
    FDCAN_HandleTypeDef       *hfdcan;
    uint16_t                   feedback_can_id;
    float                      feedback_pmax;
    float                      feedback_vmax;
    float                      feedback_tmax;
    dm_feedback_callback_t     feedback_callback;
    dm_raw_frame_callback_t    raw_frame_callback;
    void                      *callback_user;
} dm_can_bus_t;

typedef struct {
    dm_can_bus_t  *bus;
    uint8_t        id;
    dm_mode_t      mode;
    float          pmax;
    float          vmax;
    float          tmax;
    dm_feedback_t feedback;
} dm_motor_t;

/* 配置标准帧过滤器、启动 FDCAN，并打开 FIFO0 接收中断。 */
HAL_StatusTypeDef dm_can_init(dm_can_bus_t *bus,
                              FDCAN_HandleTypeDef *hfdcan,
                              uint16_t feedback_can_id,
                              dm_feedback_callback_t feedback_callback,
                              dm_raw_frame_callback_t raw_frame_callback,
                              void *callback_user);

/* 设置反馈帧的线性映射量程，默认是 12.5/30/10。 */
void dm_can_set_feedback_scale(dm_can_bus_t *bus,
                               float pmax,
                               float vmax,
                               float tmax);

/* 在 HAL_FDCAN_RxFifo0Callback 中调用；函数会排空 FIFO0。 */
void dm_can_rx_fifo0_callback(dm_can_bus_t *bus,
                              FDCAN_HandleTypeDef *hfdcan);

/* 初始化一台电机。pmax/vmax/tmax 必须与电机内部参数一致。 */
HAL_StatusTypeDef dm_motor_init(dm_motor_t *motor,
                                dm_can_bus_t *bus,
                                uint8_t motor_id,
                                float pmax,
                                float vmax,
                                float tmax,
                                dm_mode_t mode);

HAL_StatusTypeDef dm_motor_set_mode(dm_motor_t *motor, dm_mode_t mode);
HAL_StatusTypeDef dm_motor_enable(dm_motor_t *motor);
HAL_StatusTypeDef dm_motor_disable(dm_motor_t *motor);
HAL_StatusTypeDef dm_motor_clear_error(dm_motor_t *motor);
HAL_StatusTypeDef dm_motor_save_zero(dm_motor_t *motor);

HAL_StatusTypeDef dm_motor_mit(dm_motor_t *motor,
                               float position_rad,
                               float velocity_rad_s,
                               float kp,
                               float kd,
                               float torque_nm);
HAL_StatusTypeDef dm_motor_pos_vel(dm_motor_t *motor,
                                   float position_rad,
                                   float velocity_rad_s);
HAL_StatusTypeDef dm_motor_speed(dm_motor_t *motor,
                                 float velocity_rad_s);
HAL_StatusTypeDef dm_motor_pos_current(dm_motor_t *motor,
                                       float position_rad,
                                       float velocity_limit_rad_s,
                                       float current_pu);

/* 寄存器协议：读取/写入/保存。读回数据通过 raw_frame_callback 提供。 */
HAL_StatusTypeDef dm_motor_read_reg(dm_motor_t *motor, uint8_t rid);
HAL_StatusTypeDef dm_motor_write_reg(dm_motor_t *motor,
                                     uint8_t rid,
                                     const uint8_t value[4]);
HAL_StatusTypeDef dm_motor_save_params(dm_motor_t *motor);

/* 解析 8 字节反馈帧；成功返回 true。 */
bool dm_motor_decode_feedback(const uint8_t data[8],
                              float pmax,
                              float vmax,
                              float tmax,
                              dm_feedback_t *feedback);

/* 解析寄存器读回帧（data[2] == 0x33）。 */
bool dm_motor_decode_reg_read_response(const uint8_t data[8],
                                       uint8_t *rid,
                                       uint32_t *value);

#ifdef __cplusplus
}
#endif

#endif /* DAMIAO_CAN_DRIVER_H */
