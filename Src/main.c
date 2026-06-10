/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Remote chassis + CAN2 DM4340 + CAN2 ZDT57 + CAN2 ZDT42 + Raspberry Pi vision UART control
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "i6x_receive.h"
#include "i6x.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
    float kp;
    float ki;
    float kd;

    float err;
    float last_err;
    float integral;

    float max_out;
    float max_integral;
} motor_pid_t;

typedef enum
{
    CONTROL_SOURCE_REMOTE = 0,
    CONTROL_SOURCE_VISION = 1
} control_source_t;

typedef struct
{
    uint8_t online;
    uint8_t err;
    uint8_t id;

    uint16_t pos_raw;
    uint16_t vel_raw;
    uint16_t torque_raw;

    float pos_rad;
    float vel_rad_s;
    float torque_nm;

    uint8_t t_mos;
    uint8_t t_rotor;

    uint32_t last_update_tick;
    uint32_t rx_count;
} dm4340_feedback_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ========================= M3508 Chassis Definition ========================= */

#define LF_DIR   1
#define RF_DIR  -1
#define LR_DIR   1
#define RR_DIR  -1

#define RC_FORWARD_CH       1
#define RC_TURN_CH          0

#define RC_FORWARD_SIGN     1.0f
#define RC_TURN_SIGN        1.0f

#define RC_CH_MAX_VALUE     660.0f
#define RC_DEADBAND         35

#define MAX_FORWARD_RPM     220.0f
#define MAX_TURN_RPM        180.0f

#define PID_MAX_OUT         10000.0f
#define PID_MAX_INTEGRAL    30000.0f

#define CHASSIS_CONTROL_PERIOD_MS   2U

/* ========================= Raspberry Pi Vision UART Definition ========================= */

#define VISION_DEFAULT_SPEED_RPM       120.0f
#define VISION_DEFAULT_TURN_RPM        120.0f

#define VISION_UART_LINE_MAX_LEN       64U

/*
 * Telemetry output uses a second UART to avoid conflict with Raspberry Pi
 * gesture-control serial port.
 * Keep USART1 for gesture commands. Use USART6 TX -> second USB-TTL RX for feedback.
 * If your CubeMX enables another UART such as USART2, change huart6 to huart2.
 */
#define TELEMETRY_UART_HANDLE          huart6

/*
 * Arbitration rule required by user:
 *   1) If Raspberry Pi sends a valid hand gesture command, vision has priority.
 *   2) If the remote controller sends an active command, remote control can take back control.
 *   3) If the remote controller has no active command for 10 seconds, return to vision control.
 *
 * Note:
 *   STOP_ALL from Raspberry Pi is treated as a stop state, but it does not take control
 *   away from an actively used remote controller. This prevents NO_HAND -> STOP_ALL
 *   from blocking manual remote control.
 */
#define RC_IDLE_TO_VISION_MS           10000U
#define VISION_GESTURE_HOLD_MS         1200U
#define VISION_COMMAND_TIMEOUT_MS      1200U

/* ========================= DM4340 CAN2 Definition ========================= */

#define DM_LEFT_UP_ID              0x01
#define DM_LEFT_DOWN_ID            0x02
#define DM_RIGHT_ROTATE_ID         0x03

/*
 * i6X switch mapping:
 * s[0] = SwA
 * s[1] = SwB
 * s[2] = SwC
 * s[3] = SwD
 */
#define DM_DISC_SWITCH_INDEX       0
#define DM_RIGHT_SWITCH_INDEX      1

#define DM_DISC_SPEED_RAD_S        0.5f
#define DM_LEFT_UP_DIR             1.0f
#define DM_LEFT_DOWN_DIR          -1.0f

#define RIGHT_POS_0_RAD            0.0f
#define RIGHT_POS_90_RAD           1.5707963f

/*
 * Manual self-rescue control for CAN2 DM4340 ID3.
 *
 * Original SwB logic is kept unchanged:
 *   SwB down -> 90 deg
 *   SwB up   -> 0 deg
 *
 * New left-stick horizontal incremental control:
 *   left  -> negative rotation
 *   right -> positive rotation
 *   center -> stop at and hold the last commanded position
 *
 * This is NOT a one-shot limit-position command. The target position is
 * integrated gradually according to stick deflection and is clamped to
 * [-45 deg, +180 deg] to protect the cable.
 *
 * Current project convention:
 *   ch[0]/ch[1] are used for chassis drive.
 *   ch[3] is used here as the left-stick horizontal channel.
 *
 * If the physical stick direction is opposite after testing, change
 * DM_RIGHT_MANUAL_SIGN from 1.0f to -1.0f.
 */
#define DM_RIGHT_MANUAL_CH             3U
#define DM_RIGHT_MANUAL_DEADBAND       120
#define DM_RIGHT_MANUAL_SIGN           1.0f
#define DM_RIGHT_MANUAL_STICK_MAX      660.0f
#define RIGHT_POS_MANUAL_MIN_RAD      -0.7853982f   /* -45 deg */
#define RIGHT_POS_MANUAL_MAX_RAD       3.1415927f   /* +180 deg */

#define RIGHT_POS_SPEED_RAD_S      0.25f
/*
 * Manual control strategy:
 * While the left stick is held left/right, send a stable far target
 * (-45 deg or +180 deg) and let DM4340 move smoothly with the same
 * velocity limit as SwB.  Meanwhile, the code estimates the current
 * arm angle according to RIGHT_POS_SPEED_RAD_S.  When the stick returns
 * to center, the estimated current angle is immediately sent as the new
 * target, so the arm stops and holds its current position.
 */
#define DM_SEND_PERIOD_MS          5U

/*
 * DM4340 feedback decoding and telemetry output.
 *
 * DM4340 feedback frame format from the manual:
 *   STD ID = Master ID, default 0
 *   D0 = ID | (ERR << 4)
 *   D1-D2 = POS, 16-bit
 *   D3-D4 = VEL, 12-bit
 *   D4-D5 = T,   12-bit
 *   D6 = MOS temperature
 *   D7 = rotor temperature
 *
 * PMAX/VMAX/TMAX must match the values configured in Damiao Assistant.
 * These default values are used for first-stage logging. If the angle scale
 * is obviously wrong, read PMAX/VMAX/TMAX from the assistant and update here.
 */
#define DM_FEEDBACK_P_MAX_RAD       12.5f
#define DM_FEEDBACK_V_MAX_RAD_S     45.0f
#define DM_FEEDBACK_T_MAX_NM        40.0f
#define DM_FEEDBACK_OFFLINE_MS      500U

#define TELEMETRY_SEND_PERIOD_MS    50U
#define TELEMETRY_UART_TIMEOUT_MS   5U
#define TELEMETRY_LINE_MAX_LEN      512U

#define DM_REG_CTRL_MODE           0x0A

#define DM_MODE_POS_VEL            2
#define DM_MODE_VELOCITY           3

/* ========================= ZDT57 CAN2 Definition ========================= */

/*
 * ZDT57 open-loop CAN control on CAN2.
 *
 * 57 driver board settings:
 *   FWType    = FW_X
 *   CtrlMode  = CR_OPEN
 *   P_Serial  = CAN1_MAP / CAN
 *   ID_Addr   = 4
 *   CAN_Baud  = same as CAN2 / DM4340
 *   Checksum  = 0x6B
 *   En        = Hold
 *   Ma        = 2000mA first
 */
#define ZDT57_ID                  4U

/*
 * Required by user:
 *   SwC up     -> stop
 *   SwC middle -> reverse
 *   SwC down   -> forward
 *
 * In current i6x.c:
 *   rc->s[2] is already mapped to -1 / 0 / 1.
 */
#define ZDT57_SWITCH_INDEX        2U

/*
 * X firmware speed command uses 0.1 rpm unit.
 * 600 = 60.0 rpm.
 */
#define ZDT57_SPEED_RPM_X10       600U
#define ZDT57_ACC_RPM_S           300U

/*
 * If actual mechanical direction is opposite, swap these two values.
 */
#define ZDT57_DIR_FORWARD         0U
#define ZDT57_DIR_REVERSE         1U

#define ZDT57_SEND_PERIOD_MS      50U

/* ========================= ZDT42 CAN2 Definition ========================= */

/*
 * ZDT42 closed-loop CAN control on CAN2.
 *
 * 42 driver board settings:
 *   FWType    = FW_X
 *   CtrlMode  = CR_VFOC
 *   P_Serial  = CAN1_MAP / CAN
 *   ID_Addr   = 5
 *   CAN_Baud  = same as CAN2 / DM4340
 *   Checksum  = 0x6B
 *   En        = Hold
 *   Response  = None or Receive
 *
 * Required by user:
 *   SwD up   -> OFF / stop
 *   SwD down -> ON / run
 *   VRB clockwise -> speed increases
 */
#define ZDT42_ID                  5U
#define ZDT42_SWITCH_INDEX        3U

/*
 * In current i6x.c:
 *   ch[0]~ch[5] are mapped to -660 ~ 660.
 *
 * Usually:
 *   VRA -> ch[4]
 *   VRB -> ch[5]
 */
#define ZDT42_VRB_CH              5U

/*
 * Unit: 0.1 rpm.
 * 1000 = 100.0 rpm.
 * Increase this later if the drill needs higher speed.
 */
#define ZDT42_MIN_SPEED_RPM_X10   0U
#define ZDT42_MAX_SPEED_RPM_X10   1000U

#define ZDT42_ACC_RPM_S           200U

/*
 * Direction when SwD is down.
 * If the drill rotation direction is wrong, change 0U to 1U.
 */
#define ZDT42_DIR_RUN             0U

#define ZDT42_SEND_PERIOD_MS      50U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static motor_pid_t pid_lf = {12.0f, 0.08f, 0.0f, 0.0f, 0.0f, 0.0f, PID_MAX_OUT, PID_MAX_INTEGRAL};
static motor_pid_t pid_rf = {12.0f, 0.08f, 0.0f, 0.0f, 0.0f, 0.0f, PID_MAX_OUT, PID_MAX_INTEGRAL};
static motor_pid_t pid_lr = {12.0f, 0.08f, 0.0f, 0.0f, 0.0f, 0.0f, PID_MAX_OUT, PID_MAX_INTEGRAL};
static motor_pid_t pid_rr = {12.0f, 0.08f, 0.0f, 0.0f, 0.0f, 0.0f, PID_MAX_OUT, PID_MAX_INTEGRAL};

static CAN_TxHeaderTypeDef dm_tx_header;
static uint32_t dm_tx_mailbox;

static CAN_TxHeaderTypeDef zdt_tx_header;
static uint32_t zdt_tx_mailbox;

/* Raspberry Pi vision UART receive */
static uint8_t vision_uart_rx_byte = 0;
static char vision_uart_line[VISION_UART_LINE_MAX_LEN];
static uint8_t vision_uart_line_len = 0;

static volatile uint8_t vision_mode_active = 0;
static volatile uint32_t vision_last_cmd_tick = 0U;
static volatile uint32_t vision_last_gesture_tick = 0U;

static volatile float vision_left_rpm = 0.0f;
static volatile float vision_right_rpm = 0.0f;

/*
 * In this project, Python DRILL_ON / DRILL_OFF are intentionally used
 * to control the collecting disc: CAN2 DM4340 ID1 and ID2.
 * The real drill motor ZDT42 is always controlled by remote controller SwD + VRB only.
 */
static volatile uint8_t vision_disc_enabled = 0;
static volatile float vision_right_target_pos = RIGHT_POS_0_RAD;

static volatile control_source_t current_control_source = CONTROL_SOURCE_REMOTE;
static uint32_t rc_last_active_tick = 0U;

/*
 * ID3 manual self-rescue state.
 * dm_right_manual_hold_active keeps the arm holding the last manual target
 * after the left stick returns to center.
 */
static float dm_right_last_target_pos = RIGHT_POS_0_RAD;
static float dm_right_manual_target_pos = RIGHT_POS_0_RAD;
static uint8_t dm_right_manual_hold_active = 0U;
static uint32_t dm_right_manual_last_tick = 0U;

/* DM4340 ID3 feedback for Raspberry Pi logging */
static volatile dm4340_feedback_t dm3_feedback;

/* Integrated rover telemetry state for paper experiments */
static volatile float log_lf_target_rpm = 0.0f;
static volatile float log_rf_target_rpm = 0.0f;
static volatile float log_lr_target_rpm = 0.0f;
static volatile float log_rr_target_rpm = 0.0f;

static volatile uint8_t log_disc_cmd_enabled = 0U;
static volatile uint8_t log_zdt42_cmd_enabled = 0U;
static volatile uint16_t log_zdt42_cmd_speed_x10 = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* Chassis functions */
static float limit_float(float input, float min, float max);
static void pid_clear(motor_pid_t *pid);
static int16_t pid_calc(motor_pid_t *pid, float target, float feedback);
static int16_t rc_deadband(int16_t value);
static float rc_to_rpm(int16_t value, float max_rpm);
static void chassis_stop(void);
static void chassis_speed_set(float lf_rpm, float rf_rpm, float lr_rpm, float rr_rpm);
static void chassis_remote_control(void);

/* Raspberry Pi vision UART functions */
static void Vision_UART_Start(void);
static void Vision_UART_Parse_Byte(uint8_t byte);
static void Vision_UART_Execute_Line(const char *line);
static float Vision_Parse_Speed(const char *str, float default_speed, float max_speed);
static void Vision_Mark_Valid_Gesture(void);
static uint8_t Vision_Gesture_Is_Recent(void);
static uint8_t Vision_Command_Is_Timed_Out(void);
static void Vision_Set_Stop_All(void);
static void Vision_Set_Drive(float left_rpm, float right_rpm);
static uint8_t Remote_Command_Is_Active(void);
static void Control_Source_Update(void);
static void chassis_vision_control(void);
static void DM_Vision_Control(void);

/* DM4340 functions */
static void DM_CAN2_Start_If_Needed(void);
static uint8_t DM_CAN2_Send(uint32_t std_id, uint8_t *data, uint8_t len);

static void DM_Write_Uint32_Register(uint16_t can_id, uint8_t reg, uint32_t value);
static void DM_Set_Control_Mode(uint16_t can_id, uint32_t mode);

static void DM_Velocity_Clear_Error(uint16_t can_id);
static void DM_Velocity_Enable(uint16_t can_id);
static void DM_Velocity_Disable(uint16_t can_id);
static void DM_Set_Velocity(uint16_t can_id, float vel_rad_s);

static void DM_PosVel_Clear_Error(uint16_t can_id);
static void DM_PosVel_Enable(uint16_t can_id);
static void DM_PosVel_Disable(uint16_t can_id);
static void DM_PosVel_Set_Zero(uint16_t can_id);
static void DM_Set_Position_Velocity(uint16_t can_id, float pos_rad, float vel_rad_s);

static void DM_Init_Collecting_Disc(uint16_t can_id);
static void DM_Init_Right_Rotate_Motor(uint16_t can_id);
static void DM_Stop_All(void);

static void DM_CAN2_Feedback_Filter_Init(void);
static void DM_CAN2_Feedback_Start(void);
static float DM_Uint_To_Float(uint32_t x_int, float x_min, float x_max, uint8_t bits);
static void DM_Parse_Feedback_Frame(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

static float DM_Clamp_Right_Manual_Target(float target_pos);
static int16_t DM_Get_Right_Manual_Stick(i6x_ctrl_t *rc);
static uint8_t DM_Right_Manual_Is_Active(i6x_ctrl_t *rc);
static void DM_Right_Send_Position(float target_pos, float speed_rad_s);
static void DM_Remote_Control(void);

/* ZDT CAN2 functions */
static uint8_t ZDT_CAN2_Send_Ext(uint8_t addr, uint8_t packet, uint8_t *data, uint8_t len);
static void ZDT_CAN2_Send_X_Speed(uint8_t addr, uint8_t dir, uint16_t acc_rpm_s, uint16_t speed_rpm_x10);

static void ZDT57_Stop(void);
static void ZDT57_Remote_Control(void);

static void ZDT42_Stop(void);
static uint16_t ZDT42_Get_VRB_Speed_X10(i6x_ctrl_t *rc);
static void ZDT42_Remote_Control(void);

/* Raspberry Pi telemetry */
static void Telemetry_Send_DM3(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ========================= Chassis Control ========================= */

static float limit_float(float input, float min, float max)
{
    if (input > max)
    {
        return max;
    }
    else if (input < min)
    {
        return min;
    }
    else
    {
        return input;
    }
}

static void pid_clear(motor_pid_t *pid)
{
    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->integral = 0.0f;
}

static int16_t pid_calc(motor_pid_t *pid, float target, float feedback)
{
    float out;

    pid->err = target - feedback;

    pid->integral += pid->err;
    pid->integral = limit_float(pid->integral,
                                -pid->max_integral,
                                pid->max_integral);

    out = pid->kp * pid->err
        + pid->ki * pid->integral
        + pid->kd * (pid->err - pid->last_err);

    pid->last_err = pid->err;

    out = limit_float(out, -pid->max_out, pid->max_out);

    return (int16_t)out;
}

static int16_t rc_deadband(int16_t value)
{
    if (value > -RC_DEADBAND && value < RC_DEADBAND)
    {
        return 0;
    }

    return value;
}

static float rc_to_rpm(int16_t value, float max_rpm)
{
    float rpm;

    value = rc_deadband(value);

    rpm = ((float)value / RC_CH_MAX_VALUE) * max_rpm;
    rpm = limit_float(rpm, -max_rpm, max_rpm);

    return rpm;
}

static void chassis_stop(void)
{
    pid_clear(&pid_lf);
    pid_clear(&pid_rf);
    pid_clear(&pid_lr);
    pid_clear(&pid_rr);

    log_lf_target_rpm = 0.0f;
    log_rf_target_rpm = 0.0f;
    log_lr_target_rpm = 0.0f;
    log_rr_target_rpm = 0.0f;

    CAN_cmd_chassis(0, 0, 0, 0);
}

static void chassis_speed_set(float lf_rpm, float rf_rpm, float lr_rpm, float rr_rpm)
{
    const motor_measure_t *lf_motor;
    const motor_measure_t *rf_motor;
    const motor_measure_t *lr_motor;
    const motor_measure_t *rr_motor;

    float lf_target_raw;
    float rf_target_raw;
    float lr_target_raw;
    float rr_target_raw;

    int16_t lf_current;
    int16_t rf_current;
    int16_t lr_current;
    int16_t rr_current;

    log_lf_target_rpm = lf_rpm;
    log_rf_target_rpm = rf_rpm;
    log_lr_target_rpm = lr_rpm;
    log_rr_target_rpm = rr_rpm;

    lf_motor = get_chassis_motor_measure_point(0);
    rf_motor = get_chassis_motor_measure_point(1);
    lr_motor = get_chassis_motor_measure_point(2);
    rr_motor = get_chassis_motor_measure_point(3);

    lf_target_raw = LF_DIR * lf_rpm;
    rf_target_raw = RF_DIR * rf_rpm;
    lr_target_raw = LR_DIR * lr_rpm;
    rr_target_raw = RR_DIR * rr_rpm;

    lf_current = pid_calc(&pid_lf, lf_target_raw, (float)lf_motor->speed_rpm);
    rf_current = pid_calc(&pid_rf, rf_target_raw, (float)rf_motor->speed_rpm);
    lr_current = pid_calc(&pid_lr, lr_target_raw, (float)lr_motor->speed_rpm);
    rr_current = pid_calc(&pid_rr, rr_target_raw, (float)rr_motor->speed_rpm);

    CAN_cmd_chassis(lf_current, rf_current, lr_current, rr_current);
}

static void chassis_remote_control(void)
{
    i6x_ctrl_t *rc;
    float forward_rpm;
    float turn_rpm;

    float left_rpm;
    float right_rpm;

    rc = get_i6x_point();

    if (rc->failsafe)
    {
        chassis_stop();
        return;
    }

    forward_rpm = RC_FORWARD_SIGN * rc_to_rpm(rc->ch[RC_FORWARD_CH], MAX_FORWARD_RPM);
    turn_rpm    = RC_TURN_SIGN    * rc_to_rpm(rc->ch[RC_TURN_CH],    MAX_TURN_RPM);

    left_rpm  = forward_rpm + turn_rpm;
    right_rpm = forward_rpm - turn_rpm;

    left_rpm  = limit_float(left_rpm,  -MAX_FORWARD_RPM, MAX_FORWARD_RPM);
    right_rpm = limit_float(right_rpm, -MAX_FORWARD_RPM, MAX_FORWARD_RPM);

    chassis_speed_set(left_rpm, right_rpm, left_rpm, right_rpm);
}

/* ========================= Raspberry Pi Vision UART Control ========================= */

static void Vision_UART_Start(void)
{
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    vision_uart_line_len = 0;
    memset(vision_uart_line, 0, sizeof(vision_uart_line));

    HAL_UART_Receive_IT(&huart1, &vision_uart_rx_byte, 1);
}

static float Vision_Parse_Speed(const char *str, float default_speed, float max_speed)
{
    int value;

    if (str == NULL || *str == '\0')
    {
        return default_speed;
    }

    value = atoi(str);

    if (value <= 0)
    {
        return default_speed;
    }

    return limit_float((float)value, 0.0f, max_speed);
}

static void Vision_Mark_Valid_Gesture(void)
{
    uint32_t now;

    now = HAL_GetTick();

    vision_last_cmd_tick = now;
    vision_last_gesture_tick = now;
    vision_mode_active = 1;
    current_control_source = CONTROL_SOURCE_VISION;
}

static uint8_t Vision_Gesture_Is_Recent(void)
{
    uint32_t now;

    now = HAL_GetTick();

    if (vision_last_gesture_tick == 0U)
    {
        return 0;
    }

    if ((now - vision_last_gesture_tick) <= VISION_GESTURE_HOLD_MS)
    {
        return 1;
    }

    return 0;
}

static uint8_t Vision_Command_Is_Timed_Out(void)
{
    uint32_t now;

    now = HAL_GetTick();

    if (vision_last_cmd_tick == 0U)
    {
        return 1;
    }

    if ((now - vision_last_cmd_tick) > VISION_COMMAND_TIMEOUT_MS)
    {
        return 1;
    }

    return 0;
}

static void Vision_Set_Stop_All(void)
{
    vision_left_rpm = 0.0f;
    vision_right_rpm = 0.0f;

    vision_disc_enabled = 0;
    vision_right_target_pos = RIGHT_POS_0_RAD;

    /*
     * Do not force current_control_source to vision here.
     * Python may send STOP_ALL when it sees NO_HAND. If STOP_ALL forced vision mode,
     * the remote controller could never take back control while the camera sees no hand.
     */
    vision_last_cmd_tick = HAL_GetTick();
}

static void Vision_Set_Drive(float left_rpm, float right_rpm)
{
    vision_left_rpm = limit_float(left_rpm, -MAX_FORWARD_RPM, MAX_FORWARD_RPM);
    vision_right_rpm = limit_float(right_rpm, -MAX_FORWARD_RPM, MAX_FORWARD_RPM);

    Vision_Mark_Valid_Gesture();
}

static void Vision_UART_Execute_Line(const char *line)
{
    float speed;

    if (line == NULL || line[0] == '\0')
    {
        return;
    }

    if (line[0] == 'F')
    {
        speed = Vision_Parse_Speed(&line[1], VISION_DEFAULT_SPEED_RPM, MAX_FORWARD_RPM);
        Vision_Set_Drive(speed, speed);
        return;
    }

    if (line[0] == 'B')
    {
        speed = Vision_Parse_Speed(&line[1], VISION_DEFAULT_SPEED_RPM, MAX_FORWARD_RPM);
        Vision_Set_Drive(-speed, -speed);
        return;
    }

    if (line[0] == 'L')
    {
        speed = Vision_Parse_Speed(&line[1], VISION_DEFAULT_TURN_RPM, MAX_TURN_RPM);
        Vision_Set_Drive(-speed, speed);
        return;
    }

    if (line[0] == 'R')
    {
        speed = Vision_Parse_Speed(&line[1], VISION_DEFAULT_TURN_RPM, MAX_TURN_RPM);
        Vision_Set_Drive(speed, -speed);
        return;
    }

    if (line[0] == 'S')
    {
        Vision_Set_Stop_All();
        return;
    }

    if (strcmp(line, "STOP") == 0 || strcmp(line, "STOP_ALL") == 0)
    {
        Vision_Set_Stop_All();
        return;
    }

    if (strcmp(line, "DRIVE_FORWARD") == 0)
    {
        Vision_Set_Drive(VISION_DEFAULT_SPEED_RPM, VISION_DEFAULT_SPEED_RPM);
        return;
    }

    if (strcmp(line, "DRIVE_BACKWARD") == 0)
    {
        Vision_Set_Drive(-VISION_DEFAULT_SPEED_RPM, -VISION_DEFAULT_SPEED_RPM);
        return;
    }

    if (strcmp(line, "DRIVE_LEFT") == 0)
    {
        Vision_Set_Drive(-VISION_DEFAULT_TURN_RPM, VISION_DEFAULT_TURN_RPM);
        return;
    }

    if (strcmp(line, "DRIVE_RIGHT") == 0)
    {
        Vision_Set_Drive(VISION_DEFAULT_TURN_RPM, -VISION_DEFAULT_TURN_RPM);
        return;
    }

    if (strcmp(line, "BUCKET_UP") == 0)
    {
        /* Same as remote SwB down: CAN2 DM4340 ID3 goes to 90 degrees. */
        vision_right_target_pos = RIGHT_POS_90_RAD;
        Vision_Mark_Valid_Gesture();
        return;
    }

    if (strcmp(line, "BUCKET_DOWN") == 0)
    {
        /* Same as remote SwB up / not down: CAN2 DM4340 ID3 returns to 0 degree. */
        vision_right_target_pos = RIGHT_POS_0_RAD;
        Vision_Mark_Valid_Gesture();
        return;
    }

    if (strcmp(line, "DRILL_ON") == 0)
    {
        /* In this project this command means collecting disc ON: DM4340 ID1 and ID2. */
        vision_disc_enabled = 1;
        Vision_Mark_Valid_Gesture();
        return;
    }

    if (strcmp(line, "DRILL_OFF") == 0)
    {
        /* In this project this command means collecting disc OFF: DM4340 ID1 and ID2. */
        vision_disc_enabled = 0;
        Vision_Mark_Valid_Gesture();
        return;
    }
}

static void Vision_UART_Parse_Byte(uint8_t byte)
{
    if (byte == '\r' || byte == '\n')
    {
        if (vision_uart_line_len > 0)
        {
            vision_uart_line[vision_uart_line_len] = '\0';
            Vision_UART_Execute_Line(vision_uart_line);

            vision_uart_line_len = 0;
            memset(vision_uart_line, 0, sizeof(vision_uart_line));
        }

        return;
    }

    if (vision_uart_line_len < (VISION_UART_LINE_MAX_LEN - 1U))
    {
        vision_uart_line[vision_uart_line_len] = (char)byte;
        vision_uart_line_len++;
    }
    else
    {
        vision_uart_line_len = 0;
        memset(vision_uart_line, 0, sizeof(vision_uart_line));
    }
}

static uint8_t Remote_Command_Is_Active(void)
{
    i6x_ctrl_t *rc;
    int16_t forward;
    int16_t turn;

    rc = get_i6x_point();

    if (rc->failsafe)
    {
        return 0;
    }

    forward = rc->ch[RC_FORWARD_CH];
    turn = rc->ch[RC_TURN_CH];

    if (forward > RC_DEADBAND || forward < -RC_DEADBAND)
    {
        return 1;
    }

    if (turn > RC_DEADBAND || turn < -RC_DEADBAND)
    {
        return 1;
    }

    /* Left stick horizontal: incremental self-rescue command for DM4340 ID3. */
    if (DM_Right_Manual_Is_Active(rc) || dm_right_manual_hold_active)
    {
        return 1;
    }

    /* SwA: collecting disc ID1/ID2 remote command. */
    if (i6x_switch_is_down(rc->s[DM_DISC_SWITCH_INDEX]))
    {
        return 1;
    }

    /* SwB: DM4340 ID3 remote command, same meaning as BUCKET_UP/DOWN. */
    if (i6x_switch_is_down(rc->s[DM_RIGHT_SWITCH_INDEX]))
    {
        return 1;
    }

    /* SwC: ZDT57 lift motor remote command. */
    if (i6x_switch_is_down(rc->s[ZDT57_SWITCH_INDEX]) ||
        rc->s[ZDT57_SWITCH_INDEX] == I6X_SW_MID)
    {
        return 1;
    }

    /* SwD: ZDT42 drill motor remote command. */
    if (i6x_switch_is_down(rc->s[ZDT42_SWITCH_INDEX]))
    {
        return 1;
    }

    return 0;
}

static void Control_Source_Update(void)
{
    uint32_t now;
    uint8_t rc_active;

    now = HAL_GetTick();
    rc_active = Remote_Command_Is_Active();

    if (rc_active)
    {
        rc_last_active_tick = now;
    }

    /* Highest priority: a recent valid human hand gesture from Raspberry Pi. */
    if (Vision_Gesture_Is_Recent())
    {
        current_control_source = CONTROL_SOURCE_VISION;
        vision_mode_active = 1;
        return;
    }

    /* Remote can take back control when it actively commands something. */
    if (rc_active)
    {
        current_control_source = CONTROL_SOURCE_REMOTE;
        vision_mode_active = 0;
        return;
    }

    /* If remote is idle for 10 seconds, return to Raspberry Pi vision control. */
    if ((now - rc_last_active_tick) >= RC_IDLE_TO_VISION_MS)
    {
        current_control_source = CONTROL_SOURCE_VISION;
        vision_mode_active = 1;
        return;
    }

    /* During the 10-second idle window after remote use, keep remote settings. */
    current_control_source = CONTROL_SOURCE_REMOTE;
    vision_mode_active = 0;
}

static void chassis_vision_control(void)
{
    if (Vision_Command_Is_Timed_Out())
    {
        chassis_stop();
        return;
    }

    if (vision_left_rpm == 0.0f && vision_right_rpm == 0.0f)
    {
        chassis_stop();
    }
    else
    {
        chassis_speed_set(vision_left_rpm,
                          vision_right_rpm,
                          vision_left_rpm,
                          vision_right_rpm);
    }
}

/* ========================= DM4340 CAN2 Control ========================= */

static void DM_CAN2_Start_If_Needed(void)
{
    HAL_CAN_StateTypeDef state;

    state = HAL_CAN_GetState(&hcan2);

    if (state == HAL_CAN_STATE_READY)
    {
        if (HAL_CAN_Start(&hcan2) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

static uint8_t DM_CAN2_Send(uint32_t std_id, uint8_t *data, uint8_t len)
{
    uint32_t tick_start;

    dm_tx_header.StdId = std_id;
    dm_tx_header.ExtId = 0x0000;
    dm_tx_header.IDE = CAN_ID_STD;
    dm_tx_header.RTR = CAN_RTR_DATA;
    dm_tx_header.DLC = len;
    dm_tx_header.TransmitGlobalTime = DISABLE;

    tick_start = HAL_GetTick();

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
    {
        if ((HAL_GetTick() - tick_start) > 10U)
        {
            return 0;
        }
    }

    if (HAL_CAN_AddTxMessage(&hcan2, &dm_tx_header, data, &dm_tx_mailbox) != HAL_OK)
    {
        return 0;
    }

    return 1;
}

static void DM_Write_Uint32_Register(uint16_t can_id, uint8_t reg, uint32_t value)
{
    uint8_t data[8];

    data[0] = (uint8_t)(can_id & 0xFF);
    data[1] = (uint8_t)((can_id >> 8) & 0xFF);
    data[2] = 0x55;
    data[3] = reg;

    data[4] = (uint8_t)(value & 0xFF);
    data[5] = (uint8_t)((value >> 8) & 0xFF);
    data[6] = (uint8_t)((value >> 16) & 0xFF);
    data[7] = (uint8_t)((value >> 24) & 0xFF);

    DM_CAN2_Send(0x7FF, data, 8);
}

static void DM_Set_Control_Mode(uint16_t can_id, uint32_t mode)
{
    DM_Write_Uint32_Register(can_id, DM_REG_CTRL_MODE, mode);
}

static void DM_Velocity_Clear_Error(uint16_t can_id)
{
    uint8_t data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFB
    };

    DM_CAN2_Send(0x200 + can_id, data, 8);
}

static void DM_Velocity_Enable(uint16_t can_id)
{
    uint8_t data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFC
    };

    DM_CAN2_Send(0x200 + can_id, data, 8);
}

static void DM_Velocity_Disable(uint16_t can_id)
{
    uint8_t data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFD
    };

    DM_CAN2_Send(0x200 + can_id, data, 8);
}

static void DM_Set_Velocity(uint16_t can_id, float vel_rad_s)
{
    uint8_t data[4];

    memcpy(data, &vel_rad_s, 4);

    DM_CAN2_Send(0x200 + can_id, data, 4);
}

static void DM_PosVel_Clear_Error(uint16_t can_id)
{
    uint8_t data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFB
    };

    DM_CAN2_Send(0x100 + can_id, data, 8);
}

static void DM_PosVel_Enable(uint16_t can_id)
{
    uint8_t data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFC
    };

    DM_CAN2_Send(0x100 + can_id, data, 8);
}

static void DM_PosVel_Disable(uint16_t can_id)
{
    uint8_t data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFD
    };

    DM_CAN2_Send(0x100 + can_id, data, 8);
}

static void DM_PosVel_Set_Zero(uint16_t can_id)
{
    uint8_t data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFE
    };

    DM_CAN2_Send(0x100 + can_id, data, 8);
}

static void DM_Set_Position_Velocity(uint16_t can_id, float pos_rad, float vel_rad_s)
{
    uint8_t data[8];

    memcpy(&data[0], &pos_rad, 4);
    memcpy(&data[4], &vel_rad_s, 4);

    DM_CAN2_Send(0x100 + can_id, data, 8);
}

static void DM_Init_Collecting_Disc(uint16_t can_id)
{
    DM_Set_Control_Mode(can_id, DM_MODE_VELOCITY);
    HAL_Delay(300);

    DM_Velocity_Clear_Error(can_id);
    HAL_Delay(100);

    DM_Velocity_Enable(can_id);
    HAL_Delay(300);

    DM_Set_Velocity(can_id, 0.0f);
    HAL_Delay(50);
}

static void DM_Init_Right_Rotate_Motor(uint16_t can_id)
{
    uint8_t i;

    DM_Set_Control_Mode(can_id, DM_MODE_POS_VEL);
    HAL_Delay(500);

    DM_PosVel_Clear_Error(can_id);
    HAL_Delay(100);

    DM_PosVel_Set_Zero(can_id);
    HAL_Delay(500);

    DM_PosVel_Enable(can_id);
    HAL_Delay(300);

    for (i = 0; i < 50; i++)
    {
        DM_Set_Position_Velocity(can_id, RIGHT_POS_0_RAD, RIGHT_POS_SPEED_RAD_S);
        HAL_Delay(10);
    }

    dm_right_last_target_pos = RIGHT_POS_0_RAD;
    dm_right_manual_target_pos = RIGHT_POS_0_RAD;
    dm_right_manual_hold_active = 0U;
    dm_right_manual_last_tick = 0U;
}

static void DM_Stop_All(void)
{
    DM_Set_Velocity(DM_LEFT_UP_ID, 0.0f);
    DM_Set_Velocity(DM_LEFT_DOWN_ID, 0.0f);

    dm_right_manual_hold_active = 0U;
    dm_right_manual_last_tick = 0U;
    DM_Right_Send_Position(RIGHT_POS_0_RAD, RIGHT_POS_SPEED_RAD_S);
}

static void DM_CAN2_Feedback_Filter_Init(void)
{
    CAN_FilterTypeDef can_filter;

    /*
     * can_filter_init() in the RoboMaster project usually starts CAN2 and
     * assigns CAN2 messages to FIFO0.  For DM4340 logging we stop CAN2,
     * overwrite CAN2 filter bank 14 to FIFO1, then restart CAN2.
     *
     * CAN1 chassis feedback is not affected.
     */
    if (HAL_CAN_GetState(&hcan2) != HAL_CAN_STATE_READY)
    {
        (void)HAL_CAN_Stop(&hcan2);
    }

    can_filter.FilterBank = 14;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO1;
    can_filter.FilterActivation = ENABLE;
    can_filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan2, &can_filter) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
}

static void DM_CAN2_Feedback_Start(void)
{
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

static float DM_Uint_To_Float(uint32_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span;
    float offset;
    uint32_t max_int;

    max_int = (1UL << bits) - 1UL;
    span = x_max - x_min;
    offset = x_min;

    return ((float)x_int) * span / ((float)max_int) + offset;
}

static void DM_Parse_Feedback_Frame(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    uint8_t id;
    uint8_t err;
    uint16_t pos_raw;
    uint16_t vel_raw;
    uint16_t torque_raw;

    if (rx_header == 0 || rx_data == 0)
    {
        return;
    }

    if (rx_header->IDE != CAN_ID_STD || rx_header->RTR != CAN_RTR_DATA || rx_header->DLC < 8U)
    {
        return;
    }

    /*
     * DM4340 normal feedback:
     *   D0 low 4 bits  = motor ID
     *   D0 high 4 bits = ERR/state
     *
     * The project uses ID3 as the self-rescue arm.
     */
    id = rx_data[0] & 0x0FU;
    err = (rx_data[0] >> 4) & 0x0FU;

    if (id != DM_RIGHT_ROTATE_ID)
    {
        return;
    }

    pos_raw = ((uint16_t)rx_data[1] << 8) | rx_data[2];
    vel_raw = ((uint16_t)rx_data[3] << 4) | (rx_data[4] >> 4);
    torque_raw = (((uint16_t)rx_data[4] & 0x0FU) << 8) | rx_data[5];

    dm3_feedback.id = id;
    dm3_feedback.err = err;
    dm3_feedback.pos_raw = pos_raw;
    dm3_feedback.vel_raw = vel_raw;
    dm3_feedback.torque_raw = torque_raw;

    dm3_feedback.pos_rad = DM_Uint_To_Float(pos_raw,
                                            -DM_FEEDBACK_P_MAX_RAD,
                                             DM_FEEDBACK_P_MAX_RAD,
                                             16U);

    dm3_feedback.vel_rad_s = DM_Uint_To_Float(vel_raw,
                                              -DM_FEEDBACK_V_MAX_RAD_S,
                                               DM_FEEDBACK_V_MAX_RAD_S,
                                               12U);

    dm3_feedback.torque_nm = DM_Uint_To_Float(torque_raw,
                                              -DM_FEEDBACK_T_MAX_NM,
                                               DM_FEEDBACK_T_MAX_NM,
                                               12U);

    dm3_feedback.t_mos = rx_data[6];
    dm3_feedback.t_rotor = rx_data[7];

    dm3_feedback.last_update_tick = HAL_GetTick();
    dm3_feedback.rx_count++;
    dm3_feedback.online = 1U;
}

static float DM_Clamp_Right_Manual_Target(float target_pos)
{
    return limit_float(target_pos,
                       RIGHT_POS_MANUAL_MIN_RAD,
                       RIGHT_POS_MANUAL_MAX_RAD);
}

static int16_t DM_Get_Right_Manual_Stick(i6x_ctrl_t *rc)
{
    int16_t stick;

    if (rc == 0)
    {
        return 0;
    }

    stick = rc->ch[DM_RIGHT_MANUAL_CH];

    if (DM_RIGHT_MANUAL_SIGN < 0.0f)
    {
        stick = (int16_t)(-stick);
    }

    if (stick > -DM_RIGHT_MANUAL_DEADBAND && stick < DM_RIGHT_MANUAL_DEADBAND)
    {
        return 0;
    }

    return stick;
}

static uint8_t DM_Right_Manual_Is_Active(i6x_ctrl_t *rc)
{
    if (DM_Get_Right_Manual_Stick(rc) != 0)
    {
        return 1;
    }

    return 0;
}

static void DM_Right_Send_Position(float target_pos, float speed_rad_s)
{
    dm_right_last_target_pos = target_pos;

    DM_Set_Position_Velocity(DM_RIGHT_ROTATE_ID,
                             target_pos,
                             speed_rad_s);
}

static void DM_Remote_Control(void)
{
    static uint32_t last_dm_tick = 0U;

    i6x_ctrl_t *rc;
    uint32_t now;
    float disc_speed_up;
    float disc_speed_down;
    float right_target_pos;
    float right_target_speed;
    float dt_s;
    int16_t manual_stick;

    now = HAL_GetTick();

    if ((now - last_dm_tick) < DM_SEND_PERIOD_MS)
    {
        return;
    }

    last_dm_tick = now;

    rc = get_i6x_point();

    if (rc->failsafe)
    {
        DM_Stop_All();
        return;
    }

    if (i6x_switch_is_down(rc->s[DM_DISC_SWITCH_INDEX]))
    {
        disc_speed_up   = DM_LEFT_UP_DIR   * DM_DISC_SPEED_RAD_S;
        disc_speed_down = DM_LEFT_DOWN_DIR * DM_DISC_SPEED_RAD_S;
        log_disc_cmd_enabled = 1U;
    }
    else
    {
        disc_speed_up = 0.0f;
        disc_speed_down = 0.0f;
        log_disc_cmd_enabled = 0U;
    }

    DM_Set_Velocity(DM_LEFT_UP_ID, disc_speed_up);
    DM_Set_Velocity(DM_LEFT_DOWN_ID, disc_speed_down);

    manual_stick = DM_Get_Right_Manual_Stick(rc);

    if (manual_stick != 0)
    {
        float manual_dir;

        /*
         * Left-stick horizontal self-rescue control.
         * The stick is used as a direction switch:
         *   left  -> negative angle direction, limited to -45 deg
         *   right -> positive angle direction, limited to +180 deg
         *
         * Do NOT send tiny incremental targets to DM4340.  That causes
         * stuttering in position-velocity mode.  While the stick is held,
         * send a stable far target and let DM4340 move with the same velocity
         * limit as SwB.  At the same time, estimate the current target angle.
         * When the stick returns to center, immediately send this estimated
         * current angle as the hold target.
         */
        manual_dir = (manual_stick > 0) ? 1.0f : -1.0f;

        if (dm_right_manual_last_tick == 0U)
        {
            dm_right_manual_last_tick = now;
        }

        if (!dm_right_manual_hold_active)
        {
            dm_right_manual_target_pos = DM_Clamp_Right_Manual_Target(dm_right_last_target_pos);
        }

        dt_s = (float)(now - dm_right_manual_last_tick) * 0.001f;
        dm_right_manual_last_tick = now;

        if (dt_s < 0.0f)
        {
            dt_s = 0.0f;
        }
        else if (dt_s > 0.1f)
        {
            dt_s = 0.1f;
        }

        dm_right_manual_target_pos += manual_dir * RIGHT_POS_SPEED_RAD_S * dt_s;
        dm_right_manual_target_pos = DM_Clamp_Right_Manual_Target(dm_right_manual_target_pos);

        dm_right_manual_hold_active = 1U;

        if (manual_dir > 0.0f)
        {
            right_target_pos = RIGHT_POS_MANUAL_MAX_RAD;
        }
        else
        {
            right_target_pos = RIGHT_POS_MANUAL_MIN_RAD;
        }

        right_target_speed = RIGHT_POS_SPEED_RAD_S;
    }
    else if (dm_right_manual_hold_active)
    {
        /* Stick centered: hold the current feedback angle if available. */
        dm_right_manual_last_tick = now;

        if (dm3_feedback.last_update_tick != 0U &&
            (now - dm3_feedback.last_update_tick) <= DM_FEEDBACK_OFFLINE_MS)
        {
            dm_right_manual_target_pos =
                DM_Clamp_Right_Manual_Target(dm3_feedback.pos_rad);
        }

        right_target_pos = dm_right_manual_target_pos;
        right_target_speed = 0.0f;
    }
    else if (i6x_switch_is_down(rc->s[DM_RIGHT_SWITCH_INDEX]))
    {
        right_target_pos = RIGHT_POS_90_RAD;
        right_target_speed = RIGHT_POS_SPEED_RAD_S;
    }
    else
    {
        right_target_pos = RIGHT_POS_0_RAD;
        right_target_speed = RIGHT_POS_SPEED_RAD_S;
    }

    /* Operating SwB deliberately exits the manual hold mode without changing SwB angles. */
    if (manual_stick == 0 && dm_right_manual_hold_active &&
        i6x_switch_is_down(rc->s[DM_RIGHT_SWITCH_INDEX]))
    {
        dm_right_manual_hold_active = 0U;
        dm_right_manual_last_tick = now;
        right_target_pos = RIGHT_POS_90_RAD;
        right_target_speed = RIGHT_POS_SPEED_RAD_S;
    }

    if (manual_stick == 0 && !dm_right_manual_hold_active &&
        !i6x_switch_is_down(rc->s[DM_RIGHT_SWITCH_INDEX]))
    {
        dm_right_manual_last_tick = now;
    }

    DM_Right_Send_Position(right_target_pos, right_target_speed);
}

static void DM_Vision_Control(void)
{
    static uint32_t last_dm_tick = 0U;

    uint32_t now;
    float disc_speed_up;
    float disc_speed_down;

    now = HAL_GetTick();

    if ((now - last_dm_tick) < DM_SEND_PERIOD_MS)
    {
        return;
    }

    last_dm_tick = now;

    if (Vision_Command_Is_Timed_Out())
    {
        DM_Stop_All();
        return;
    }

    if (vision_disc_enabled)
    {
        disc_speed_up   = DM_LEFT_UP_DIR   * DM_DISC_SPEED_RAD_S;
        disc_speed_down = DM_LEFT_DOWN_DIR * DM_DISC_SPEED_RAD_S;
        log_disc_cmd_enabled = 1U;
    }
    else
    {
        disc_speed_up = 0.0f;
        disc_speed_down = 0.0f;
        log_disc_cmd_enabled = 0U;
    }

    DM_Set_Velocity(DM_LEFT_UP_ID, disc_speed_up);
    DM_Set_Velocity(DM_LEFT_DOWN_ID, disc_speed_down);

    dm_right_manual_hold_active = 0U;
    DM_Right_Send_Position(vision_right_target_pos, RIGHT_POS_SPEED_RAD_S);
}

/* ========================= ZDT CAN2 Control ========================= */

static uint8_t ZDT_CAN2_Send_Ext(uint8_t addr, uint8_t packet, uint8_t *data, uint8_t len)
{
    uint32_t tick_start;

    zdt_tx_header.StdId = 0x000;
    zdt_tx_header.ExtId = ((uint32_t)addr << 8) | packet;
    zdt_tx_header.IDE = CAN_ID_EXT;
    zdt_tx_header.RTR = CAN_RTR_DATA;
    zdt_tx_header.DLC = len;
    zdt_tx_header.TransmitGlobalTime = DISABLE;

    tick_start = HAL_GetTick();

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
    {
        if ((HAL_GetTick() - tick_start) > 10U)
        {
            return 0;
        }
    }

    if (HAL_CAN_AddTxMessage(&hcan2,
                             &zdt_tx_header,
                             data,
                             &zdt_tx_mailbox) != HAL_OK)
    {
        return 0;
    }

    return 1;
}

static void ZDT_CAN2_Send_X_Speed(uint8_t addr, uint8_t dir, uint16_t acc_rpm_s, uint16_t speed_rpm_x10)
{
    uint8_t data[8];

    data[0] = 0xF6;
    data[1] = dir;
    data[2] = (uint8_t)(acc_rpm_s >> 8);
    data[3] = (uint8_t)(acc_rpm_s & 0xFF);
    data[4] = (uint8_t)(speed_rpm_x10 >> 8);
    data[5] = (uint8_t)(speed_rpm_x10 & 0xFF);
    data[6] = 0x00;
    data[7] = 0x6B;

    ZDT_CAN2_Send_Ext(addr, 0, data, 8);
}

static void ZDT57_Stop(void)
{
    ZDT_CAN2_Send_X_Speed(ZDT57_ID, ZDT57_DIR_FORWARD, ZDT57_ACC_RPM_S, 0U);
}

static void ZDT57_Remote_Control(void)
{
    static uint32_t last_zdt_tick = 0U;

    i6x_ctrl_t *rc;
    uint32_t now;
    int8_t swc;

    now = HAL_GetTick();

    if ((now - last_zdt_tick) < ZDT57_SEND_PERIOD_MS)
    {
        return;
    }

    last_zdt_tick = now;

    rc = get_i6x_point();

    if (rc->failsafe)
    {
        ZDT57_Stop();
        return;
    }

    swc = rc->s[ZDT57_SWITCH_INDEX];

    /*
     * Current i6x.c maps switch value to -1 / 0 / 1.
     *
     * Required mapping:
     *   SwC up     -> stop
     *   SwC middle -> reverse
     *   SwC down   -> forward
     */
    if (i6x_switch_is_down(swc))
    {
        ZDT_CAN2_Send_X_Speed(ZDT57_ID,
                              ZDT57_DIR_FORWARD,
                              ZDT57_ACC_RPM_S,
                              ZDT57_SPEED_RPM_X10);
    }
    else if (swc == I6X_SW_MID)
    {
        ZDT_CAN2_Send_X_Speed(ZDT57_ID,
                              ZDT57_DIR_REVERSE,
                              ZDT57_ACC_RPM_S,
                              ZDT57_SPEED_RPM_X10);
    }
    else
    {
        ZDT57_Stop();
    }
}

static void ZDT42_Stop(void)
{
    log_zdt42_cmd_enabled = 0U;
    log_zdt42_cmd_speed_x10 = 0U;

    ZDT_CAN2_Send_X_Speed(ZDT42_ID, ZDT42_DIR_RUN, ZDT42_ACC_RPM_S, 0U);
}

static uint16_t ZDT42_Get_VRB_Speed_X10(i6x_ctrl_t *rc)
{
    int16_t vrb;
    int32_t speed;

    /*
     * Current i6x.c maps ch[0]~ch[5] to -660 ~ 660.
     *
     * VRB channel:
     *   -660 -> minimum speed
     *    660 -> maximum speed
     *
     * This realizes:
     *   clockwise -> speed increases
     *
     * If physical VRB direction is opposite after testing,
     * change (vrb + 660) to (660 - vrb).
     */
    vrb = rc->ch[ZDT42_VRB_CH];

    if (vrb < -660)
    {
        vrb = -660;
    }
    else if (vrb > 660)
    {
        vrb = 660;
    }

    speed = ((int32_t)(vrb + 660) *
             (int32_t)(ZDT42_MAX_SPEED_RPM_X10 - ZDT42_MIN_SPEED_RPM_X10)) / 1320;
    speed += ZDT42_MIN_SPEED_RPM_X10;

    if (speed < ZDT42_MIN_SPEED_RPM_X10)
    {
        speed = ZDT42_MIN_SPEED_RPM_X10;
    }

    if (speed > ZDT42_MAX_SPEED_RPM_X10)
    {
        speed = ZDT42_MAX_SPEED_RPM_X10;
    }

    return (uint16_t)speed;
}

static void ZDT42_Remote_Control(void)
{
    static uint32_t last_zdt42_tick = 0U;

    i6x_ctrl_t *rc;
    uint32_t now;
    int8_t swd;
    uint16_t speed_x10;

    now = HAL_GetTick();

    if ((now - last_zdt42_tick) < ZDT42_SEND_PERIOD_MS)
    {
        return;
    }

    last_zdt42_tick = now;

    rc = get_i6x_point();

    if (rc->failsafe)
    {
        ZDT42_Stop();
        return;
    }

    swd = rc->s[ZDT42_SWITCH_INDEX];

    /*
     * Required mapping:
     *   SwD up   -> OFF / stop
     *   SwD down -> ON / run
     *
     * SwD is used as a two-position switch.
     * If it is not down, treat it as OFF.
     */
    if (i6x_switch_is_down(swd))
    {
        speed_x10 = ZDT42_Get_VRB_Speed_X10(rc);

        log_zdt42_cmd_enabled = 1U;
        log_zdt42_cmd_speed_x10 = speed_x10;

        ZDT_CAN2_Send_X_Speed(ZDT42_ID,
                              ZDT42_DIR_RUN,
                              ZDT42_ACC_RPM_S,
                              speed_x10);
    }
    else
    {
        ZDT42_Stop();
    }
}

/* ========================= Raspberry Pi Telemetry ========================= */

static void Telemetry_Send_DM3(void)
{
    static uint32_t last_tx_tick = 0U;

    uint32_t now;
    char line[TELEMETRY_LINE_MAX_LEN];
    int len;

    const motor_measure_t *lf_motor;
    const motor_measure_t *rf_motor;
    const motor_measure_t *lr_motor;
    const motor_measure_t *rr_motor;
    i6x_ctrl_t *rc;

    int32_t lf_target_x10;
    int32_t rf_target_x10;
    int32_t lr_target_x10;
    int32_t rr_target_x10;
    int32_t lf_feedback_x10;
    int32_t rf_feedback_x10;
    int32_t lr_feedback_x10;
    int32_t rr_feedback_x10;

    int32_t dm_pos_deg_x100;
    int32_t dm_vel_mrad_s;
    int32_t dm_torque_x100;
    int32_t dm_target_deg_x100;
    int16_t dm_stick_x;
    uint8_t dm_online;

    int16_t rc_ch0;
    int16_t rc_ch1;
    int16_t rc_ch3;
    int16_t rc_ch5;
    int8_t rc_s0;
    int8_t rc_s1;
    int8_t rc_s2;
    int8_t rc_s3;

    now = HAL_GetTick();

    if ((now - last_tx_tick) < TELEMETRY_SEND_PERIOD_MS)
    {
        return;
    }

    last_tx_tick = now;

    if (dm3_feedback.last_update_tick == 0U ||
        (now - dm3_feedback.last_update_tick) > DM_FEEDBACK_OFFLINE_MS)
    {
        dm_online = 0U;
    }
    else
    {
        dm_online = 1U;
    }

    dm3_feedback.online = dm_online;

    lf_motor = get_chassis_motor_measure_point(0);
    rf_motor = get_chassis_motor_measure_point(1);
    lr_motor = get_chassis_motor_measure_point(2);
    rr_motor = get_chassis_motor_measure_point(3);

    rc = get_i6x_point();

    rc_ch0 = rc->ch[0];
    rc_ch1 = rc->ch[1];
    rc_ch3 = rc->ch[3];
    rc_ch5 = rc->ch[5];
    rc_s0 = rc->s[0];
    rc_s1 = rc->s[1];
    rc_s2 = rc->s[2];
    rc_s3 = rc->s[3];

    lf_target_x10 = (int32_t)(log_lf_target_rpm * 10.0f);
    rf_target_x10 = (int32_t)(log_rf_target_rpm * 10.0f);
    lr_target_x10 = (int32_t)(log_lr_target_rpm * 10.0f);
    rr_target_x10 = (int32_t)(log_rr_target_rpm * 10.0f);

    /* Convert raw motor feedback into physical wheel direction for comparison with target rpm. */
    lf_feedback_x10 = (int32_t)((float)(LF_DIR * lf_motor->speed_rpm) * 10.0f);
    rf_feedback_x10 = (int32_t)((float)(RF_DIR * rf_motor->speed_rpm) * 10.0f);
    lr_feedback_x10 = (int32_t)((float)(LR_DIR * lr_motor->speed_rpm) * 10.0f);
    rr_feedback_x10 = (int32_t)((float)(RR_DIR * rr_motor->speed_rpm) * 10.0f);

    dm_pos_deg_x100 = (int32_t)(dm3_feedback.pos_rad * 5729.57795f);
    dm_vel_mrad_s = (int32_t)(dm3_feedback.vel_rad_s * 1000.0f);
    dm_torque_x100 = (int32_t)(dm3_feedback.torque_nm * 100.0f);
    dm_target_deg_x100 = (int32_t)(dm_right_manual_target_pos * 5729.57795f);
    dm_stick_x = DM_Get_Right_Manual_Stick(rc);

    /*
     * Integrated output line for Raspberry Pi:
     *
     * LOG,time_ms,
     * lf_t_x10,lf_r_x10,rf_t_x10,rf_r_x10,lr_t_x10,lr_r_x10,rr_t_x10,rr_r_x10,
     * dm_online,dm_err,dm_rx_count,dm_pos_deg_x100,dm_vel_mrad_s,dm_torque_x100,
     * dm_t_mos,dm_t_rotor,dm_target_deg_x100,dm_stick_x,
     * zdt42_enabled,zdt42_cmd_x10,disc_cmd,
     * rc_ch0,rc_ch1,rc_ch3,rc_ch5,rc_s0,rc_s1,rc_s2,rc_s3,control_source
     *
     * x10/x100 scaling avoids floating-point printf problems in Keil.
     */
    len = snprintf(line,
                   sizeof(line),
                   "LOG,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%lu,%ld,%ld,%ld,%u,%u,%ld,%d,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%u\r\n",
                   (unsigned long)now,
                   (long)lf_target_x10,
                   (long)lf_feedback_x10,
                   (long)rf_target_x10,
                   (long)rf_feedback_x10,
                   (long)lr_target_x10,
                   (long)lr_feedback_x10,
                   (long)rr_target_x10,
                   (long)rr_feedback_x10,
                   (unsigned int)dm_online,
                   (unsigned int)dm3_feedback.err,
                   (unsigned long)dm3_feedback.rx_count,
                   (long)dm_pos_deg_x100,
                   (long)dm_vel_mrad_s,
                   (long)dm_torque_x100,
                   (unsigned int)dm3_feedback.t_mos,
                   (unsigned int)dm3_feedback.t_rotor,
                   (long)dm_target_deg_x100,
                   (int)dm_stick_x,
                   (unsigned int)log_zdt42_cmd_enabled,
                   (unsigned int)log_zdt42_cmd_speed_x10,
                   (unsigned int)log_disc_cmd_enabled,
                   (int)rc_ch0,
                   (int)rc_ch1,
                   (int)rc_ch3,
                   (int)rc_ch5,
                   (int)rc_s0,
                   (int)rc_s1,
                   (int)rc_s2,
                   (int)rc_s3,
                   (unsigned int)current_control_source);

    if (len <= 0)
    {
        return;
    }

    if (len > (int)(sizeof(line) - 1U))
    {
        len = (int)(sizeof(line) - 1U);
    }

    (void)HAL_UART_Transmit(&TELEMETRY_UART_HANDLE,
                            (uint8_t *)line,
                            (uint16_t)len,
                            TELEMETRY_UART_TIMEOUT_MS);
}

/* ========================= CAN2 DM4340 Feedback Callback ========================= */

void CAN2_RX1_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan2);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan->Instance == CAN2)
    {
        while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO1) > 0U)
        {
            if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK)
            {
                DM_Parse_Feedback_Frame(&rx_header, rx_data);
            }
        }
    }
}

/* ========================= UART Callback ========================= */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        Vision_UART_Parse_Byte(vision_uart_rx_byte);
        HAL_UART_Receive_IT(&huart1, &vision_uart_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);

        vision_uart_line_len = 0;
        memset(vision_uart_line, 0, sizeof(vision_uart_line));

        HAL_UART_Receive_IT(&huart1, &vision_uart_rx_byte, 1);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  can_filter_init();

  /*
   * Reconfigure CAN2 reception to FIFO1 for DM4340 feedback logging.
   * CAN1 chassis feedback remains handled by the original RoboMaster CAN code.
   */
  DM_CAN2_Feedback_Filter_Init();
  DM_CAN2_Start_If_Needed();
  DM_CAN2_Feedback_Start();

  i6x_remote_init();

  Vision_UART_Start();

  current_control_source = CONTROL_SOURCE_REMOTE;
  vision_mode_active = 0;
  rc_last_active_tick = HAL_GetTick();

  chassis_stop();

  ZDT57_Stop();
  ZDT42_Stop();

  HAL_Delay(1000);

  DM_Init_Collecting_Disc(DM_LEFT_UP_ID);
  DM_Init_Collecting_Disc(DM_LEFT_DOWN_ID);
  DM_Init_Right_Rotate_Motor(DM_RIGHT_ROTATE_ID);

  DM_Stop_All();
  ZDT57_Stop();
  ZDT42_Stop();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Control_Source_Update();

    if (current_control_source == CONTROL_SOURCE_VISION)
    {
        chassis_vision_control();
        DM_Vision_Control();
    }
    else
    {
        chassis_remote_control();
        DM_Remote_Control();
    }

    /*
     * SwC controls ZDT57 lift motor:
     *   up     -> stop
     *   middle -> reverse
     *   down   -> forward
     *
     * This motor is still remote-controlled only.
     */
    ZDT57_Remote_Control();

    /*
     * SwD controls ZDT42 drill motor:
     *   up   -> stop
     *   down -> run
     *   VRB  -> speed control
     *
     * The real drill is always remote-controlled only.
     * Raspberry Pi DRILL_ON/OFF does NOT control ZDT42 here.
     */
    ZDT42_Remote_Control();

    /*
     * Send DM4340 ID3 feedback to Raspberry Pi through USART6.
     */
    Telemetry_Send_DM3();

    HAL_Delay(CHASSIS_CONTROL_PERIOD_MS);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
