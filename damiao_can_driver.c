#include "damiao_can_driver.h"

#include <string.h>

#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f

static uint8_t dm_dlc_to_length(uint32_t dlc)
{
    switch (dlc) {
    case FDCAN_DLC_BYTES_0: return 0U;
    case FDCAN_DLC_BYTES_1: return 1U;
    case FDCAN_DLC_BYTES_2: return 2U;
    case FDCAN_DLC_BYTES_3: return 3U;
    case FDCAN_DLC_BYTES_4: return 4U;
    case FDCAN_DLC_BYTES_5: return 5U;
    case FDCAN_DLC_BYTES_6: return 6U;
    case FDCAN_DLC_BYTES_7: return 7U;
    case FDCAN_DLC_BYTES_8: return 8U;
    default: return 0U;
    }
}

static bool dm_valid_mode(dm_mode_t mode)
{
    return (mode == DM_MODE_MIT) ||
           (mode == DM_MODE_POS_VEL) ||
           (mode == DM_MODE_SPEED) ||
           (mode == DM_MODE_POS_CUR);
}

static uint32_t dm_length_to_dlc(uint8_t length)
{
    switch (length) {
    case 0U: return FDCAN_DLC_BYTES_0;
    case 1U: return FDCAN_DLC_BYTES_1;
    case 2U: return FDCAN_DLC_BYTES_2;
    case 3U: return FDCAN_DLC_BYTES_3;
    case 4U: return FDCAN_DLC_BYTES_4;
    case 5U: return FDCAN_DLC_BYTES_5;
    case 6U: return FDCAN_DLC_BYTES_6;
    case 7U: return FDCAN_DLC_BYTES_7;
    case 8U: return FDCAN_DLC_BYTES_8;
    default: return 0U;
    }
}

static uint32_t dm_float_to_uint(float value,
                                 float min_value,
                                 float max_value,
                                 uint8_t bits)
{
    const float span = max_value - min_value;
    const uint32_t max_int = (1UL << bits) - 1UL;
    float normalized;

    if (span <= 0.0f) {
        return 0U;
    }
    if (value < min_value) {
        value = min_value;
    } else if (value > max_value) {
        value = max_value;
    }
    normalized = (value - min_value) / span;
    return (uint32_t)(normalized * (float)max_int);
}

static float dm_uint_to_float(uint32_t value,
                              float min_value,
                              float max_value,
                              uint8_t bits)
{
    const uint32_t max_int = (1UL << bits) - 1UL;
    return ((float)value * (max_value - min_value) / (float)max_int)
           + min_value;
}

static void dm_put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)(value >> 8);
}

static uint16_t dm_get_u16_le(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static void dm_put_f32_le(uint8_t *dst, float value)
{
    /* STM32 Cortex-M 是小端；memcpy 避免违反 strict-aliasing。 */
    memcpy(dst, &value, sizeof(value));
}

static HAL_StatusTypeDef dm_can_send(dm_can_bus_t *bus,
                                     uint16_t can_id,
                                     const uint8_t *data,
                                     uint8_t length)
{
    FDCAN_TxHeaderTypeDef header;

    if ((bus == NULL) || (bus->hfdcan == NULL) ||
        (data == NULL) || (length > 8U) || (can_id > 0x7FFU)) {
        return HAL_ERROR;
    }

    memset(&header, 0, sizeof(header));
    header.Identifier = can_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = dm_length_to_dlc(length);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(bus->hfdcan, &header,
                                         (uint8_t *)data);
}

HAL_StatusTypeDef dm_can_init(dm_can_bus_t *bus,
                              FDCAN_HandleTypeDef *hfdcan,
                              uint16_t feedback_can_id,
                              dm_feedback_callback_t feedback_callback,
                              dm_raw_frame_callback_t raw_frame_callback,
                              void *callback_user)
{
    FDCAN_FilterTypeDef filter;
    HAL_StatusTypeDef status;

    if ((bus == NULL) || (hfdcan == NULL) || (feedback_can_id > 0x7FFU)) {
        return HAL_ERROR;
    }

    memset(bus, 0, sizeof(*bus));
    bus->hfdcan = hfdcan;
    bus->feedback_can_id = feedback_can_id;
    bus->feedback_pmax = 12.5f;
    bus->feedback_vmax = 30.0f;
    bus->feedback_tmax = 10.0f;
    bus->feedback_callback = feedback_callback;
    bus->raw_frame_callback = raw_frame_callback;
    bus->callback_user = callback_user;

    memset(&filter, 0, sizeof(filter));
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U; /* mask=0：接收所有标准 ID，便于同时接收 0x7FF 寄存器响应。 */

    status = HAL_FDCAN_ConfigFilter(hfdcan, &filter);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                          FDCAN_REJECT,
                                          FDCAN_REJECT,
                                          FDCAN_REJECT_REMOTE,
                                          FDCAN_REJECT_REMOTE);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_FDCAN_Start(hfdcan);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_FDCAN_ActivateNotification(hfdcan,
                                           FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                           0U);
}

void dm_can_set_feedback_scale(dm_can_bus_t *bus,
                               float pmax,
                               float vmax,
                               float tmax)
{
    if ((bus == NULL) || (pmax <= 0.0f) ||
        (vmax <= 0.0f) || (tmax <= 0.0f)) {
        return;
    }
    bus->feedback_pmax = pmax;
    bus->feedback_vmax = vmax;
    bus->feedback_tmax = tmax;
}

void dm_can_rx_fifo0_callback(dm_can_bus_t *bus,
                              FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
    uint8_t length;

    if ((bus == NULL) || (hfdcan == NULL) || (hfdcan != bus->hfdcan)) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        memset(&header, 0, sizeof(header));
        memset(data, 0, sizeof(data));
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &header, data) != HAL_OK) {
            break;
        }

        length = dm_dlc_to_length(header.DataLength);
        if (bus->raw_frame_callback != NULL) {
            bus->raw_frame_callback(header.Identifier, data, length,
                                    bus->callback_user);
        }

        if ((header.Identifier == bus->feedback_can_id) && (length == 8U) &&
            (bus->feedback_callback != NULL)) {
            dm_feedback_t feedback;
            if (dm_motor_decode_feedback(data,
                                         bus->feedback_pmax,
                                         bus->feedback_vmax,
                                         bus->feedback_tmax,
                                         &feedback)) {
                bus->feedback_callback(&feedback, bus->callback_user);
            }
        }
    }
}

HAL_StatusTypeDef dm_motor_init(dm_motor_t *motor,
                                dm_can_bus_t *bus,
                                uint8_t motor_id,
                                float pmax,
                                float vmax,
                                float tmax,
                                dm_mode_t mode)
{
    if ((motor == NULL) || (bus == NULL) ||
        (motor_id > DM_CAN_MAX_MOTOR_ID) ||
        !dm_valid_mode(mode) ||
        (pmax <= 0.0f) || (vmax <= 0.0f) || (tmax <= 0.0f)) {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->bus = bus;
    motor->id = motor_id;
    motor->mode = mode;
    motor->pmax = pmax;
    motor->vmax = vmax;
    motor->tmax = tmax;
    return HAL_OK;
}

HAL_StatusTypeDef dm_motor_set_mode(dm_motor_t *motor, dm_mode_t mode)
{
    if ((motor == NULL) || !dm_valid_mode(mode)) {
        return HAL_ERROR;
    }
    motor->mode = mode;
    return HAL_OK;
}

static HAL_StatusTypeDef dm_motor_send_special(dm_motor_t *motor,
                                               uint8_t command)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                       0xFFU, 0xFFU, 0xFFU, command};
    return dm_can_send(motor->bus,
                       (uint16_t)(motor->id + (uint16_t)motor->mode),
                       data, 8U);
}

HAL_StatusTypeDef dm_motor_enable(dm_motor_t *motor)
{
    return (motor == NULL) ? HAL_ERROR : dm_motor_send_special(motor, 0xFCU);
}

HAL_StatusTypeDef dm_motor_disable(dm_motor_t *motor)
{
    return (motor == NULL) ? HAL_ERROR : dm_motor_send_special(motor, 0xFDU);
}

HAL_StatusTypeDef dm_motor_clear_error(dm_motor_t *motor)
{
    return (motor == NULL) ? HAL_ERROR : dm_motor_send_special(motor, 0xFBU);
}

HAL_StatusTypeDef dm_motor_save_zero(dm_motor_t *motor)
{
    return (motor == NULL) ? HAL_ERROR : dm_motor_send_special(motor, 0xFEU);
}

HAL_StatusTypeDef dm_motor_mit(dm_motor_t *motor,
                               float position_rad,
                               float velocity_rad_s,
                               float kp,
                               float kd,
                               float torque_nm)
{
    uint8_t data[8];
    uint16_t p, v, kp_u, kd_u, t;

    if (motor == NULL) {
        return HAL_ERROR;
    }
    p = (uint16_t)dm_float_to_uint(position_rad, -motor->pmax,
                                   motor->pmax, 16U);
    v = (uint16_t)dm_float_to_uint(velocity_rad_s, -motor->vmax,
                                   motor->vmax, 12U);
    kp_u = (uint16_t)dm_float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12U);
    kd_u = (uint16_t)dm_float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12U);
    t = (uint16_t)dm_float_to_uint(torque_nm, -motor->tmax,
                                   motor->tmax, 12U);

    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)(((v & 0x0FU) << 4) | (kp_u >> 8));
    data[4] = (uint8_t)kp_u;
    data[5] = (uint8_t)(kd_u >> 4);
    data[6] = (uint8_t)(((kd_u & 0x0FU) << 4) | (t >> 8));
    data[7] = (uint8_t)t;

    return dm_can_send(motor->bus, motor->id, data, 8U);
}

HAL_StatusTypeDef dm_motor_pos_vel(dm_motor_t *motor,
                                   float position_rad,
                                   float velocity_rad_s)
{
    uint8_t data[8];

    if (motor == NULL) {
        return HAL_ERROR;
    }
    dm_put_f32_le(&data[0], position_rad);
    dm_put_f32_le(&data[4], velocity_rad_s);
    return dm_can_send(motor->bus,
                       (uint16_t)(motor->id + DM_MODE_POS_VEL), data, 8U);
}

HAL_StatusTypeDef dm_motor_speed(dm_motor_t *motor,
                                 float velocity_rad_s)
{
    uint8_t data[4];

    if (motor == NULL) {
        return HAL_ERROR;
    }
    dm_put_f32_le(data, velocity_rad_s);
    return dm_can_send(motor->bus,
                       (uint16_t)(motor->id + DM_MODE_SPEED), data, 4U);
}

HAL_StatusTypeDef dm_motor_pos_current(dm_motor_t *motor,
                                       float position_rad,
                                       float velocity_limit_rad_s,
                                       float current_pu)
{
    uint8_t data[8];
    uint16_t velocity_u16;
    uint16_t current_u16;

    if (motor == NULL) {
        return HAL_ERROR;
    }
    if (velocity_limit_rad_s < 0.0f) {
        velocity_limit_rad_s = 0.0f;
    } else if (velocity_limit_rad_s > 100.0f) {
        velocity_limit_rad_s = 100.0f;
    }
    if (current_pu < 0.0f) {
        current_pu = 0.0f;
    } else if (current_pu > 1.0f) {
        current_pu = 1.0f;
    }
    velocity_u16 = (uint16_t)(velocity_limit_rad_s * 100.0f);
    current_u16 = (uint16_t)(current_pu * 10000.0f);

    dm_put_f32_le(&data[0], position_rad);
    dm_put_u16_le(&data[4], velocity_u16);
    dm_put_u16_le(&data[6], current_u16);
    return dm_can_send(motor->bus,
                       (uint16_t)(motor->id + DM_MODE_POS_CUR), data, 8U);
}

HAL_StatusTypeDef dm_motor_read_reg(dm_motor_t *motor, uint8_t rid)
{
    uint8_t data[4];

    if (motor == NULL) {
        return HAL_ERROR;
    }
    data[0] = motor->id;
    data[1] = 0U;
    data[2] = 0x33U;
    data[3] = rid;
    return dm_can_send(motor->bus, DM_CAN_BROADCAST_ID, data, 4U);
}

HAL_StatusTypeDef dm_motor_write_reg(dm_motor_t *motor,
                                     uint8_t rid,
                                     const uint8_t value[4])
{
    uint8_t data[8];

    if ((motor == NULL) || (value == NULL)) {
        return HAL_ERROR;
    }
    data[0] = motor->id;
    data[1] = 0U;
    data[2] = 0x55U;
    data[3] = rid;
    memcpy(&data[4], value, 4U);
    return dm_can_send(motor->bus, DM_CAN_BROADCAST_ID, data, 8U);
}

HAL_StatusTypeDef dm_motor_save_params(dm_motor_t *motor)
{
    uint8_t data[4] = {0U, 0U, 0xAAU, 0x01U};

    if (motor == NULL) {
        return HAL_ERROR;
    }
    data[0] = motor->id;
    return dm_can_send(motor->bus, DM_CAN_BROADCAST_ID, data, 4U);
}

bool dm_motor_decode_feedback(const uint8_t data[8],
                              float pmax,
                              float vmax,
                              float tmax,
                              dm_feedback_t *feedback)
{
    if ((data == NULL) || (feedback == NULL) ||
        (pmax <= 0.0f) || (vmax <= 0.0f) || (tmax <= 0.0f)) {
        return false;
    }

    memset(feedback, 0, sizeof(*feedback));
    feedback->id = data[0] & 0x0FU;
    feedback->state = data[0] >> 4;
    feedback->pos_raw = ((uint16_t)data[1] << 8) | data[2];
    feedback->vel_raw = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    feedback->torque_raw = ((uint16_t)(data[4] & 0x0FU) << 8) | data[5];
    feedback->position_rad = dm_uint_to_float(feedback->pos_raw,
                                              -pmax, pmax, 16U);
    feedback->velocity_rad_s = dm_uint_to_float(feedback->vel_raw,
                                                -vmax, vmax, 12U);
    feedback->torque_nm = dm_uint_to_float(feedback->torque_raw,
                                           -tmax, tmax, 12U);
    feedback->mos_temperature_c = data[6];
    feedback->coil_temperature_c = data[7];
    return true;
}

bool dm_motor_decode_reg_read_response(const uint8_t data[8],
                                       uint8_t *rid,
                                       uint32_t *value)
{
    if ((data == NULL) || (rid == NULL) || (value == NULL) ||
        (data[2] != 0x33U)) {
        return false;
    }
    *rid = data[3];
    *value = ((uint32_t)data[4]) |
             ((uint32_t)data[5] << 8) |
             ((uint32_t)data[6] << 16) |
             ((uint32_t)data[7] << 24);
    return true;
}
