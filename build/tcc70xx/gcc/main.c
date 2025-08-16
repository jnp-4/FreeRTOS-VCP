// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : main.c
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*
*
***************************************************************************************************
*/

#include <main.h>

#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "string.h"
#include "can_demo.h"
#include "pdm.h"
#include "timer.h"
#include "gpio.h"
#include "math.h"
//jam
#include "queue.h"
// #include "task.h"   // xTaskGetCurrentTaskHandle, 

/*
***************************************************************************************************
*                                         GLOBAL VARIABLES
***************************************************************************************************
*/
uint32                                  gALiveMsgOnOff;
extern uint32 gDebugOption;             // 디버깅 활성화
static volatile uint32 gReceiveFlag[3];
// extern CANMessage_t    main_sRxMsg;
// extern volatile uint8 gMainRxReady;
/* === CAN 수신 큐 === */
#define CAN_RXQ_LEN  32
static QueueHandle_t gCanRxQ;

#define AEB_TASK_STK_SIZE                (256U)
#define CAN_TASK_STK_SIZE                (256U)
#define AUTO_PARKING_TASK_STK_SIZE       (256U)
#define MAIN_UINT_MAX_NUM               (4294967295U)
#define SAL_PRIO_AUTO_PARKING (6)   //jam sal_com.h 말고 임시로 여기에 정의함
SemaphoreHandle_t aebStartSem;
SemaphoreHandle_t aebClearSem;
SemaphoreHandle_t parkStartSem; //jam 자율 주차 모드 실행 시작 위해 semaphore 정의함
volatile BaseType_t gAEBActive = pdFALSE;
volatile BaseType_t gparkActive = pdFALSE;   // jam 오토파킹 실행 중 플래그
volatile BaseType_t gAebAcceptClear = pdFALSE;  // 5초 뒤에만 0x4B('K') 입력 허용 위해 선언

// 5초 뒤에만 0x4B('K') 입력 허용 위해 선언
extern void CAN_DemoFlushRxAll(void);
extern void CAN_DemoSetRxQueue(QueueHandle_t q);
//jam
typedef enum {
    MODE_IDLE = 0,
    MODE_AUTOPARK,
    MODE_AEB_HOLD
} SystemMode_t;
volatile SystemMode_t gMode = MODE_IDLE;

/* 자율주차 태스크 핸들: suspend/resume 대상 */
static TaskHandle_t gApTaskHandle = NULL;
// AutoParking_StartTask create위한 변수를 전역 변수로 설정
static uint32           uiAutoParkingTaskID;
static uint32           uiAutoParkingTaskStk[AUTO_PARKING_TASK_STK_SIZE];

//ultrasonic
#define SOUND_SPEED_CM_PER_US 0.0343    
#define ECHO_RISE_TIMEOUT_US 2318   // burst 전송 시간 = 200us, 여유 있게 줌
#define ECHO_FALL_TIMEOUT_US 12000   // 200cm 이내에만 탐지할 예정 
                                    // (200 cm × 2) / 0.034 cm/μs ≈ 11764.7 μs
                                    
//micros() 구현
#define TMR_CLK_RATE      (12UL * 1000UL * 1000UL) // 12MHz 예시
#define TMR_PRESCALE      11UL
#define MICRO_TIMER_CH    TIMER_CH_1
#define TMR_BASE_ADDR     (MCU_BSP_TIMER_BASE + 0x100UL * (MICRO_TIMER_CH))
#define TMR_MAIN_CNT_REG  (*(volatile uint32 *)(TMR_BASE_ADDR + 0x14UL))
extern volatile uint32 t_start;
volatile uint32 t_end = 0;

#define TRIG_1 (GPIO_GPB(21))
#define ECHO_1 (GPIO_GPB(22))

#define TRIG_2 (GPIO_GPB(19))
#define ECHO_2 (GPIO_GPB(20))

#define TRIG_3 (GPIO_GPA(27))
#define ECHO_3 (GPIO_GPA(30))

#define TRIG_4 (GPIO_GPA(24))
#define ECHO_4 (GPIO_GPA(26))

#define TRIG_5 (GPIO_GPA(23))
#define ECHO_5 (GPIO_GPA(25))

#define TRIG_6 (GPIO_GPA(21))
#define ECHO_6 (GPIO_GPA(22))

// 서보모터 제어
PDMModeConfig_t pwm_cfg_servo;          //전역 변수 선언 (servo motor)
#define SERVO_CH 6                      // 6은 16을 의미
#define PWM_PERIOD_NS_SERVO 20000000UL  // 20ms (servo motor pwm 최대 주기)
#define KP_STEER 3.0
#define SERVO_CENTER_DEG 90
#define SERVO_LEFT_DEG 45
#define SERVO_RIGHT_DEG 135

//DC 모터 제어
PDMModeConfig_t pwm_cfg_dc;    //전역 변수 선언 (dc motor)
#define DC_CH 0 
#define IN_1 (GPIO_GPB(23))
#define IN_2 (GPIO_GPB(24))
#define PWM_PERIOD_NS_DC 1000000UL  // 1ms (pwm 최대 주기(사용자가 그저 설정))
/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

static void AEB_StartTask(void *pArg);
static void CAN_StartTask(void *pArg);
static void AutoParking_StartTask(void *pArg);

unsigned long micros(void);
unsigned long millis(void);
void delayMicroseconds(unsigned int us);
void Init_Ultrasonic(void);
double run_ultrasonic(const int trigPin, const int echoPin);
void print_double(double value);

PDMModeConfig_t init_pwm_on_GPIO_GPA(uint32 period_ns, uint32 duty_ns);
void set_servo_angle(uint8 angle);
static inline int clamp_int(int value, int min_val, int max_val);

void dc_motor_forward(uint16 pwm_value);
void dc_motor_backward(uint16 pwm_value);
void dc_motor_stop(void);

void send_vcp_cmd(uint8 code);
static void AutoPark_Restart(void);
static inline void CanRxQueueDrain(void);
/*
***************************************************************************************************
*                                         FUNCTIONS
***************************************************************************************************
*/

unsigned long micros(void)
{
    return TMR_MAIN_CNT_REG;  // 1 tick = 1 µs
}

unsigned long millis(void)
{
    return micros() / 1000UL;  // 단순히 나눠서 1ms 구현
}

void delayMicroseconds(unsigned int us) {
    unsigned long start = micros();
    while (micros() - start < us) {
        __asm volatile ("nop");
    }
}

void Init_Ultrasonic(void){
    
    GPIO_Config(TRIG_1, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(ECHO_1, (GPIO_FUNC(0) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLDN));

    GPIO_Config(TRIG_2, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(ECHO_2, (GPIO_FUNC(0) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLDN));

    GPIO_Config(TRIG_3, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(ECHO_3, (GPIO_FUNC(0) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLDN));

    GPIO_Config(TRIG_4, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(ECHO_4, (GPIO_FUNC(0) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLDN));

    GPIO_Config(TRIG_5, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(ECHO_5, (GPIO_FUNC(0) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLDN));

    GPIO_Config(TRIG_6, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(ECHO_6, (GPIO_FUNC(0) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLDN));

    //micros()에서 사용할 timer enable
    TIMERConfig_t cfg = {
    .ctChannel       = MICRO_TIMER_CH,
    .ctStartMode     = TIMER_START_ZERO,
    .ctOpMode        = TIMER_OP_FREERUN,
    .ctCounterMode   = TIMER_COUNTER_MAIN,  // 주의: COMP0 대신 MAIN
    .ctMainValueUsec = 0,  // Free-run
    .ctCmp0ValueUsec = 0,
    .ctCmp1ValueUsec = 0,
    .fnHandler       = NULL,
    .pArgs           = NULL
    };
    TIMER_EnableWithCfg(&cfg);

    GPIO_Set(TRIG_1, 0);
    GPIO_Set(TRIG_2, 0);
    GPIO_Set(TRIG_3, 0);
    GPIO_Set(TRIG_4, 0);
    GPIO_Set(TRIG_5, 0);
    GPIO_Set(TRIG_6, 0);
}

double run_ultrasonic(const int trigPin, const int echoPin)
{   
    long start_wait;
    long start;
    long end;
    long duration;
    double distance;

    GPIO_Set(trigPin, 1); 
    delayMicroseconds(20);  //datasheet에서는 10us면, 1->0으로 되어야한다고 나와있음
    GPIO_Set(trigPin, 0); 

    start_wait = micros();
    while (GPIO_Get(echoPin) == 0) 
    {
        if (micros() - start_wait > ECHO_RISE_TIMEOUT_US) 
        {
            mcu_printf("Timeout: ECHO did not go HIGH\n");
            return -1;
        }
    }
    start = micros();

    while (GPIO_Get(echoPin) == 1) 
    {
        if (micros() - start_wait > ECHO_FALL_TIMEOUT_US) 
        {
            mcu_printf("(echoPin : %d)Timeout: ECHO did not go LOW\n",echoPin);
            return -1;
        }
    }
    end = micros();

    // Distance calculation
    duration = end - start;
    distance = duration * SOUND_SPEED_CM_PER_US / 2.0;

    return distance;
}

void print_double(double value)
{
    int int_part;
    int frac_part;

    int_part = (int)value;
    frac_part = (int)((value - int_part) * 10000);
    mcu_printf("value = %d.%04d\n", int_part, frac_part);
    return;
}

PDMModeConfig_t init_pwm_on_GPIO_GPA(uint32 period_ns, uint32 duty_ns)
{
    PDMModeConfig_t cfg;
    cfg.mcPortNumber      = GPIO_PERICH_CH0;
    cfg.mcOperationMode   = PDM_OUTPUT_MODE_PHASE_1;
    cfg.mcInversedSignal  = 0;
    cfg.mcOutSignalInIdle = 0;
    cfg.mcLoopCount       = 0;
    cfg.mcOutputCtrl      = 0;
    cfg.mcPeriodNanoSec1  = period_ns; 
    cfg.mcDutyNanoSec1    = duty_ns;
    cfg.mcPeriodNanoSec2  = 0;
    cfg.mcDutyNanoSec2    = 0;
    return cfg;
}

void set_servo_angle(uint8 angle)
{
    // pwm_cfg는 전역 변수로 받아 옴(duty_ns만 바꾸기 위함)
    // PDMModeConfig_t pwm_cfg;
    uint32 duty_ns;
    uint32 wait_cnt = 0;
    // uint32 channel = SERVO_CH; //8은 A18번 pin을 의미 

    if (angle > 180) angle = 180;
    //if (angle < 0) angle = 0;

    // 0도 → 500000ns, 180도 → 2500000ns
    duty_ns = 500000 + ((uint32)angle * 2000000UL) / 180;

    // PDM 종료
    PDM_Disable(SERVO_CH, PMM_ON);
    while (PDM_GetChannelStatus(SERVO_CH)) {
        //SAL_TaskSleep(1);
        if (++wait_cnt > 100) {
            mcu_printf("Timeout disabling PDM\n");
            return;
        }
    }

    // 설정 구성
    pwm_cfg_servo.mcDutyNanoSec1    = duty_ns;

    // 설정 적용
    if (PDM_SetConfig(SERVO_CH, &pwm_cfg_servo) != SAL_RET_SUCCESS) {
        mcu_printf("PDM_SetConfig failed\n");
        return;
    }

    if (PDM_Enable(SERVO_CH, PMM_ON) != SAL_RET_SUCCESS) {
        mcu_printf("PDM_Enable failed\n");
        return;
    }


    mcu_printf("[Servo] angle = %d deg → duty = %d ns ",
               angle, duty_ns, duty_ns / 1000000.0);

    int int_part = (int)(duty_ns / 1000000.0);
    int frac_part = (int)(((duty_ns / 1000000.0) - int_part) * 10000);
    mcu_printf("(%d.%04d ms) \n", int_part, frac_part);
}

static inline int clamp_int(int value, int min_val, int max_val)
{
    if (value < min_val) {
        return min_val;
    } else if (value > max_val) {
        return max_val;
    } else {
        return value;
    }
}

void dc_motor_forward(uint16 pwm_value)
{

    if (pwm_value > 1000) pwm_value = 1000;

    GPIO_Set(IN_1, 1); // 전진 방향
    GPIO_Set(IN_2, 0);

    
    uint32 duty_ns = ((PWM_PERIOD_NS_DC) * pwm_value) / 1000;
    mcu_printf("duty_ns : %d\n",duty_ns);
    pwm_cfg_dc.mcDutyNanoSec1 = duty_ns;
    
    PDM_Disable(DC_CH, PMM_ON);
    while (PDM_GetChannelStatus(DC_CH)) SAL_TaskSleep(1);

    if (PDM_SetConfig(DC_CH, &pwm_cfg_dc) != SAL_RET_SUCCESS) {
        mcu_printf("PDM_SetConfig failed\n");
        return;
    }

    if (PDM_Enable(DC_CH, PMM_ON) != SAL_RET_SUCCESS) {
        mcu_printf("PDM_Enable failed\n");
        return;
    }
    
    // mcu_printf("[Forward] PWM = %u → duty = %u ns\n", pwm_value, duty_ns);
}

void dc_motor_backward(uint16 pwm_value)
{
    
    if (pwm_value > 1000) pwm_value = 1000;

    GPIO_Set(IN_1, 0);
    GPIO_Set(IN_2, 1); // 후진 방향

    uint32 duty_ns = (PWM_PERIOD_NS_DC * pwm_value) / 1000;
    pwm_cfg_dc.mcDutyNanoSec1 = duty_ns;

    PDM_Disable(DC_CH, PMM_ON);
    while (PDM_GetChannelStatus(DC_CH)) SAL_TaskSleep(1);

    if (PDM_SetConfig(DC_CH, &pwm_cfg_dc) != SAL_RET_SUCCESS) {
        mcu_printf("PDM_SetConfig failed\n");
        return;
    }

    if (PDM_Enable(DC_CH, PMM_ON) != SAL_RET_SUCCESS) {
        mcu_printf("PDM_Enable failed\n");
        return;
    }

    // mcu_printf("[Backward] PWM = %u → duty = %u ns\n", pwm_value, duty_ns);
}

void dc_motor_stop()
{
    pwm_cfg_dc.mcDutyNanoSec1 = 0;

    PDM_Disable(DC_CH, PMM_ON);
    while (PDM_GetChannelStatus(DC_CH)) SAL_TaskSleep(1);

    if (PDM_SetConfig(DC_CH, &pwm_cfg_dc) != SAL_RET_SUCCESS) {
        mcu_printf("PDM_SetConfig failed\n");
        return;
    }

    if (PDM_Enable(DC_CH, PMM_ON) != SAL_RET_SUCCESS) {
        mcu_printf("PDM_Enable failed\n");
        return;
    }
}

void send_vcp_cmd(uint8 code)
{
    CANMessage_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.mBufferType          = CAN_TX_BUFFER_TYPE_DBUFFER;
    tx.mBufferIndex         = 0;
    tx.mExtendedId          = 0;
    tx.mRemoteTransmitRequest = 0;
    tx.mFDFormat            = 0;   /* Classic CAN */
    tx.mBitRateSwitching    = 0;
    tx.mId                  = 0x201;  /* VCP -> Target */
    tx.mDataLength          = 1;
    tx.mData[0]             = code;

    uint8 txIdx = 0;
    CANErrorType_t r = CAN_SendMessage(0, &tx, &txIdx);

    if (r == CAN_ERROR_NONE)
    {
        mcu_printf("[VCP->Target] CAN 0x%03X [%d] %02X\n", tx.mId, tx.mDataLength, tx.mData[0]);
        mcu_printf("[Main] Autonomous parking complete\n");
    }
    else
    {
        mcu_printf("[VCP->Target] send fail %d\n", r);
    }
}


static void AutoPark_Restart(void) {
    taskENTER_CRITICAL();
    if (gApTaskHandle) {
        vTaskDelete(gApTaskHandle);
        gApTaskHandle = NULL;
    }
    // parkStartSem 잔여 토큰 비우기(선택)
    while (xSemaphoreTake(parkStartSem, 0) == pdTRUE) { /* drain */ }

    SALRetCode_t rc = SAL_TaskCreate(&uiAutoParkingTaskID, (const uint8*)"Auto Parking Task",
                                      (SALTaskFunc)&AutoParking_StartTask,
                                      &uiAutoParkingTaskStk[0], AUTO_PARKING_TASK_STK_SIZE,
                                      SAL_PRIO_AUTO_PARKING, NULL_PTR);

    if (rc != SAL_RET_SUCCESS) {
        mcu_printf("TaskCreate failed: AutoParking=%d\n", (int)rc);
    }
    taskEXIT_CRITICAL();
}

static inline void CanRxQueueDrain(void)
{
    CANMessage_t dump;
    while (xQueueReceive(gCanRxQ /* 또는 sCanRxQ */, &dump, 0) == pdTRUE) {
        /* drop */
    }
}

/*
***************************************************************************************************
*                                          cmain
*
* This is the standard entry point for C code.
*
* Notes
*   It is assumed that your code will call main() once you have performed all necessary
*   initialization.
*
***************************************************************************************************
*/
void cmain (void)
{
    static uint32           uiAEBTaskID = 0;
    static uint32           uiAEBTaskStk[AEB_TASK_STK_SIZE];
    static uint32           uiCanTaskID;
    static uint32           uiCanTaskStk[CAN_TASK_STK_SIZE];
    // static uint32           uiAutoParkingTaskID;
    // static uint32           uiAutoParkingTaskStk[AUTO_PARKING_TASK_STK_SIZE];
    SALMcuVersionInfo_t     versionInfo = {0,0,0,0};
    gDebugOption = DBG_LOG_ENABLEALL;

    (void)SAL_Init();

    BSP_PreInit(); /* Initialize basic BSP functions */

    BSP_Init(); /* Initialize BSP functions */

    TIMERConfig_t cfg = {
        .ctChannel       = MICRO_TIMER_CH,
        .ctStartMode     = TIMER_START_ZERO,
        .ctOpMode        = TIMER_OP_FREERUN,
        .ctCounterMode   = TIMER_COUNTER_MAIN,  // 주의: COMP0 대신 MAIN
        .ctMainValueUsec = 0,  // Free-run
        .ctCmp0ValueUsec = 0,
        .ctCmp1ValueUsec = 0,
        .fnHandler       = NULL,
        .pArgs           = NULL
    };
    TIMER_EnableWithCfg(&cfg);

    // 초음파센서 초기화
    Init_Ultrasonic();

    // PWM 초기화
    PDM_Init();

    //servo motor 초기화
    pwm_cfg_servo = init_pwm_on_GPIO_GPA(PWM_PERIOD_NS_SERVO, 500000);
    set_servo_angle(90);

    //dc motor 초기화
    pwm_cfg_dc = init_pwm_on_GPIO_GPA(PWM_PERIOD_NS_DC, 0);
    GPIO_Config(IN_1, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(IN_2, (GPIO_FUNC(0) | GPIO_OUTPUT));

    mcu_printf("init finished\n");

    (void)SAL_GetVersion(&versionInfo);
    mcu_printf("\n===============================\n");
    mcu_printf("    MCU BSP Version: V%d.%d.%d\n",
           versionInfo.viMajorVersion,
           versionInfo.viMinorVersion,
           versionInfo.viPatchVersion);
    mcu_printf("-------------------------------\n");
    //DisplayOTPInfo();
    mcu_printf("===============================\n\n");


    /* 큐 생성 */
    gCanRxQ = xQueueCreate(CAN_RXQ_LEN, sizeof(CANMessage_t));
    configASSERT(gCanRxQ);
    /* can_demo.c에서 main.c에서의 큐에 push할 수 있도록 큐를 등록 */
    CAN_DemoSetRxQueue(gCanRxQ);

    aebStartSem  = xSemaphoreCreateBinary();
    aebClearSem  = xSemaphoreCreateBinary();
    parkStartSem = xSemaphoreCreateBinary();    //jam 자율 주차
    configASSERT(aebStartSem && aebClearSem && parkStartSem);

    // create the first app task...
    // 긴급 제동 태스크 (우선순위 첫 번째)
    SALRetCode_t rc1 = SAL_TaskCreate(&uiAEBTaskID, (const uint8*)"AEB Task",
                                      (SALTaskFunc)&AEB_StartTask,
                                      &uiAEBTaskStk[0], AEB_TASK_STK_SIZE,
                                      SAL_PRIO_APP_CFG, NULL_PTR);
    
    // 일반 주행(CAN받는) 태스크 (우선순위 두 번째)
    SALRetCode_t rc2 = SAL_TaskCreate(&uiCanTaskID, (const uint8*)"Can Task",
                                      (SALTaskFunc)&CAN_StartTask,
                                      &uiCanTaskStk[0], CAN_TASK_STK_SIZE,
                                      SAL_PRIO_CAN_DEMO, NULL_PTR);

    // 자율주차 태스크 (우선순위 세 번째)
    SALRetCode_t rc3 = SAL_TaskCreate(&uiAutoParkingTaskID, (const uint8*)"Auto Parking Task",
                                      (SALTaskFunc)&AutoParking_StartTask,
                                      &uiAutoParkingTaskStk[0], AUTO_PARKING_TASK_STK_SIZE,
                                      SAL_PRIO_AUTO_PARKING, NULL_PTR);
    
    if (rc1 == SAL_RET_SUCCESS && rc2 == SAL_RET_SUCCESS && rc3 == SAL_RET_SUCCESS) {
        SAL_OsInitFuncs();
        SAL_OsStart();
    } else {
        mcu_printf("TaskCreate failed: AEB=%d, CAN=%d, AutoParking=%d\n", (int)rc1, (int)rc2, (int)rc3);
    }
}

/*
***************************************************************************************************
*                                          Main_StartTask
*
* This is an example of a startup task.
*
* Notes
*   As mentioned in the book's text, you MUST initialize the ticker only once multitasking has
*   started.
*
*   1) The first line of code is used to prevent a compiler warning because 'pArg' is not used.
*      The compiler should not generate any code for this statement.
*
***************************************************************************************************
*/
static void AEB_StartTask(void *pArg)
{
    (void)pArg;
    
    for(;;)
    {
        mcu_printf("[AEB_StartTask] Task start!!\n");
        if(xSemaphoreTake(aebStartSem, portMAX_DELAY) == pdTRUE)
        {
            gAebAcceptClear = pdFALSE;   // 5초 동안 0x4B 무시
            mcu_printf("WAIT 5sec\n");
            delayMicroseconds(5000000);
            mcu_printf("5sec is gone\n");
            // SAL_TaskSleep(1);

            // 여기서 드라이버 RX + 앱 큐 모두 비움
            CAN_DemoFlushRxAll();
            while (xSemaphoreTake(aebClearSem, 0) == pdTRUE) { /* drain stale clear */ }
        }        

        // 이제부터 들어오는 0x4B만 유효
        gAebAcceptClear = pdTRUE;

        if(xSemaphoreTake(aebClearSem, portMAX_DELAY) == pdTRUE) {
            // gAEBActive = pdFALSE;
            mcu_printf("[AEB_StartTask] AEB_over!!\n");
            delayMicroseconds(5000000);
        }

        // SAL_TaskSleep(1);

        mcu_printf("[AEB_StartTask] Task End!!\n");
    }
}

static void CAN_StartTask(void *pArg)
{
    (void)pArg;
    CANMessage_t rx;
    int pre_gMode=0;  // gMode 갱신 전, 모드값 저장 위한 변수
    mcu_printf("[Can Task] start\n");

    /* CAN 초기화 */
    if (CAN_DemoInitialize() != 0) {
        mcu_printf("[MAIN] CAN init failed!\n");
        return;
    }

    CAN_DemoCreateApp();

    /* 수신 태스크(start) */
    const uint8 *argv_recv[] = { (const uint8 *)"receive", (const uint8 *)"start" };
    CAN_DemoTest(2, (void *)argv_recv);

    for(;;)
    {
        if(xQueueReceive(gCanRxQ, &rx, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }
             
        switch (gMode)
        {
            case MODE_IDLE:
                pre_gMode = MODE_IDLE;
                switch (rx.mData[0])
                {
                    case 0x53 : dc_motor_stop();
                            gMode = MODE_AEB_HOLD;
                            //delay를 주지 않는다.

                            // /* 만약 오토파킹이 지금 vTaskDelay 중이면 바로 깨워서 suspend */
                            // if (gApTaskHandle) 
                            // {
                            //     xTaskAbortDelay(gApTaskHandle);     //긴 delay 상태여도 바로 깨우도록 함
                            //     vTaskSuspend(gApTaskHandle);    //깨운 후, 바로 suspend
                            // }

                            // AEB 시작 직전에 과거에 쌓인 메시지 제거
                            CanRxQueueDrain();
                            xSemaphoreGive(aebStartSem);
                            break;

                    case 0x50 : mcu_printf("[CAN_StartTask] AutoParking START!!!\n");
                            xSemaphoreGive(parkStartSem);      // 자율 주차 태스크 깨우기
                            gMode = MODE_AUTOPARK;
                            SAL_TaskSleep(5);
                            break;
                    
                    case 0x46 : mcu_printf("GOGOGO\n");
                            dc_motor_forward(1000);
                            SAL_TaskSleep(5);
                            break;

                    case 0x42 : mcu_printf("BACKBACK\n");
                            dc_motor_backward(1000);
                            SAL_TaskSleep(5);
                            break;

                    case 0x4B : mcu_printf("BRAKE\n");
                            dc_motor_stop();
                            SAL_TaskSleep(5);
                            break;

                    case 0x4C : mcu_printf("LEFT\n");
                            set_servo_angle(60);
                            SAL_TaskSleep(5);
                            break;

                    case 0x52 : mcu_printf("RIGHT\n");
                            set_servo_angle(120);
                            SAL_TaskSleep(5);
                            break;

                    case 0x4E : mcu_printf("NEUTRAL\n");
                            set_servo_angle(90);
                            SAL_TaskSleep(5);
                            break;

                    case 0x44 : mcu_printf("Normal Driving mode!!\n");
                            gMode = MODE_IDLE;  // 일반 주행 모드 유지
                            SAL_TaskSleep(5);
                            break;
                }
                break;
            case MODE_AUTOPARK:
                pre_gMode = MODE_AUTOPARK;
                switch (rx.mData[0])
                {
                    case 0x53 : dc_motor_stop();
                            gMode = MODE_AEB_HOLD;
                            //delay를 주지 않는다.

                            /* 만약 오토파킹이 지금 vTaskDelay 중이면 바로 깨워서 suspend */
                            if (gApTaskHandle) 
                            {
                                xTaskAbortDelay(gApTaskHandle);     //긴 delay 상태여도 바로 깨우도록 함
                                vTaskSuspend(gApTaskHandle);    //깨운 후, 바로 suspend
                            }

                            // AEB 시작 직전에 과거에 쌓인 메시지 제거
                            CanRxQueueDrain();
                            xSemaphoreGive(aebStartSem);
                            break;
                    case 0x44 : mcu_printf("Normal Driving mode!!\n");
                            gMode = MODE_IDLE;  // 일반 주행 모드로 변경
                            
                            AutoPark_Restart();              // 자율 주차 태스크 완전 리셋
                            SAL_TaskSleep(5);
                            break;
                    default : break;
                }
                break;
            case MODE_AEB_HOLD:
                switch (rx.mData[0])
                {
                    case 0x4B : mcu_printf("BRAKE\n");
                        if(gAebAcceptClear)
                        {
                            if (pre_gMode == MODE_AUTOPARK)
                            {
                                mcu_printf("[CAN_StartTask] AutoParking RESUME\n");
                                gMode = MODE_AUTOPARK;  // 모드 복귀
                                if (gApTaskHandle) vTaskResume(gApTaskHandle);
                                xSemaphoreGive(aebClearSem);
                                                
                            }
                            else if(pre_gMode == MODE_IDLE)
                            {
                                mcu_printf("[CAN_StartTask] return to IDLE\n");
                                gMode = MODE_IDLE;  // 모드 복귀
                                xSemaphoreGive(aebClearSem);
                            }
                        }
                        else
                        {
                            //5초 홀드 중에는 무시
                            mcu_printf("[CAN_StartTask] 0x4B ignored during AEB hold\n");
                        }
                            
                            
                        SAL_TaskSleep(5);
                        break;
                    default : break;
                }
            break;
        }
        
        mcu_printf( "[CAN_StartTask] hi\n" );
    }
        
}


static void AutoParking_StartTask(void *pArg)
{
    (void)pArg;

    /* 내 핸들을 전역에 보관 — CAN에서 suspend/resume 할 대상 */
    gApTaskHandle = xTaskGetCurrentTaskHandle();

    for (;;)
    {
        if(xSemaphoreTake(parkStartSem, portMAX_DELAY)==pdTRUE)
        {
            gparkActive = pdTRUE;
            gMode = MODE_AUTOPARK;  // 시작 시, 모드 설정
        }
        mcu_printf("[AutoParking_StartTask] START\n");

        for (;;)
        {
            /* === 오토파킹 로직 === */
            for (int i = 0; i < 10; ++i) {
                mcu_printf("[AutoParking] %d%d%d%d\n", i, i, i, i);
            }
            SAL_TaskSleep(100);  // 100ms 주기 로직
        }

        mcu_printf("[AutoParking] END, waiting next start\n");
    }
}