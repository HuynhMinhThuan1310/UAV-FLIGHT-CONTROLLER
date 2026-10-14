/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   GY-87 Step 2 — Pitch / Roll / Yaw / Altitude
  *          Blue Pill STM32F103C8T6
  *          I2C1 : PB6=SCL  PB7=SDA
  *          UART1: PA9=TX   PA10=RX  9600
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart1;

#define MPU6050_ADDR   (0x68 << 1)
#define HMC5883L_ADDR  (0x1E << 1)
#define BMP180_ADDR    (0x77 << 1)

#define MPU_PWR_MGMT_1   0x6B
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_INT_PIN_CFG  0x37

#define HMC_CONFIG_A 0x00
#define HMC_CONFIG_B 0x01
#define HMC_MODE     0x02
#define HMC_DATA_XH  0x03

#define BMP_REG_CALIB 0xAA
#define BMP_REG_CTRL  0xF4
#define BMP_REG_DATA  0xF6
#define BMP_CMD_TEMP  0x2E
#define BMP_CMD_PRES  0x34
#define BMP_OSS       0

/* Complementary filter: 98% gyro, 2% accel */
#define CF_ALPHA   0.98f
#define LOOP_DT    0.01f   /* 10 ms loop */

/* Adjust to your local sea-level pressure */
#define SEA_LEVEL_HPA 1013.25f

typedef struct {
    int16_t  AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t  B1, B2, MB, MC, MD;
} BMP180_Calib;

static BMP180_Calib bmp_calib;
static float cf_pitch = 0.0f;
static float cf_roll  = 0.0f;

/* printf → UART1 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* ---- I2C helpers ---- */
static HAL_StatusTypeDef I2C_WriteReg(uint16_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return HAL_I2C_Master_Transmit(&hi2c1, dev, buf, 2, HAL_MAX_DELAY);
}
static HAL_StatusTypeDef I2C_ReadRegs(uint16_t dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, dev, &reg, 1, HAL_MAX_DELAY);
    if (st != HAL_OK) return st;
    return HAL_I2C_Master_Receive(&hi2c1, dev, buf, len, HAL_MAX_DELAY);
}

/* ---- MPU6050 ---- */
static void MPU6050_Init(void)
{
    I2C_WriteReg(MPU6050_ADDR, MPU_PWR_MGMT_1, 0x00);
    HAL_Delay(200);
    I2C_WriteReg(MPU6050_ADDR, MPU_INT_PIN_CFG, 0x02); /* bypass */
}
static void MPU6050_Read(float *ax, float *ay, float *az,
                          float *gx, float *gy, float *gz)
{
    uint8_t buf[14];
    if (I2C_ReadRegs(MPU6050_ADDR, MPU_ACCEL_XOUT_H, buf, 14) != HAL_OK) {
        *ax=*ay=*az=*gx=*gy=*gz=0; return;
    }
    *ax = (int16_t)(buf[0]<<8|buf[1])  / 16384.0f;
    *ay = (int16_t)(buf[2]<<8|buf[3])  / 16384.0f;
    *az = (int16_t)(buf[4]<<8|buf[5])  / 16384.0f;
    *gx = (int16_t)(buf[8]<<8|buf[9])  / 131.0f;
    *gy = (int16_t)(buf[10]<<8|buf[11])/ 131.0f;
    *gz = (int16_t)(buf[12]<<8|buf[13])/ 131.0f;
}

/* ---- Complementary filter ---- */
static void CF_Update(float ax, float ay, float az, float gx, float gy)
{
    float ap = atan2f(ay, az)               * (180.0f / 3.14159265f);
    float ar = atan2f(-ax, sqrtf(ay*ay+az*az)) * (180.0f / 3.14159265f);
    cf_pitch = CF_ALPHA * (cf_pitch + gy * LOOP_DT) + (1.0f - CF_ALPHA) * ap;
    cf_roll  = CF_ALPHA * (cf_roll  + gx * LOOP_DT) + (1.0f - CF_ALPHA) * ar;
}

/* ---- HMC5883L ---- */
static void HMC5883L_Init(void)
{
    I2C_WriteReg(HMC5883L_ADDR, HMC_CONFIG_A, 0x70);
    I2C_WriteReg(HMC5883L_ADDR, HMC_CONFIG_B, 0x20);
    I2C_WriteReg(HMC5883L_ADDR, HMC_MODE,     0x00);
    HAL_Delay(10);
}
static void HMC5883L_Read(float *mx, float *my, float *mz)
{
    uint8_t buf[6];
    if (I2C_ReadRegs(HMC5883L_ADDR, HMC_DATA_XH, buf, 6) != HAL_OK) {
        *mx=*my=*mz=0; return;
    }
    /* Order: X Z Y */
    *mx = (int16_t)(buf[0]<<8|buf[1]) / 1090.0f;
    *mz = (int16_t)(buf[2]<<8|buf[3]) / 1090.0f;
    *my = (int16_t)(buf[4]<<8|buf[5]) / 1090.0f;
}

/* Tilt-compensated yaw 0-360 */
static float ComputeYaw(float mx, float my, float mz, float pitch_d, float roll_d)
{
    float p = pitch_d * (3.14159265f / 180.0f);
    float r = roll_d  * (3.14159265f / 180.0f);
    float mx2 = mx*cosf(p) + mz*sinf(p);
    float my2 = mx*sinf(r)*sinf(p) + my*cosf(r) - mz*sinf(r)*cosf(p);
    float yaw = atan2f(-my2, mx2) * (180.0f / 3.14159265f);
    if (yaw < 0.0f) yaw += 360.0f;
    return yaw;
}

/* ---- BMP180 ---- */
static void BMP180_ReadCalib(void)
{
    uint8_t buf[22];
    I2C_ReadRegs(BMP180_ADDR, BMP_REG_CALIB, buf, 22);
    bmp_calib.AC1=(int16_t)(buf[0]<<8|buf[1]);  bmp_calib.AC2=(int16_t)(buf[2]<<8|buf[3]);
    bmp_calib.AC3=(int16_t)(buf[4]<<8|buf[5]);  bmp_calib.AC4=(uint16_t)(buf[6]<<8|buf[7]);
    bmp_calib.AC5=(uint16_t)(buf[8]<<8|buf[9]); bmp_calib.AC6=(uint16_t)(buf[10]<<8|buf[11]);
    bmp_calib.B1=(int16_t)(buf[12]<<8|buf[13]); bmp_calib.B2=(int16_t)(buf[14]<<8|buf[15]);
    bmp_calib.MB=(int16_t)(buf[16]<<8|buf[17]); bmp_calib.MC=(int16_t)(buf[18]<<8|buf[19]);
    bmp_calib.MD=(int16_t)(buf[20]<<8|buf[21]);
}
static int32_t BMP180_ReadUT(void)
{
    I2C_WriteReg(BMP180_ADDR, BMP_REG_CTRL, BMP_CMD_TEMP); HAL_Delay(5);
    uint8_t buf[2]; I2C_ReadRegs(BMP180_ADDR, BMP_REG_DATA, buf, 2);
    return (int32_t)(buf[0]<<8|buf[1]);
}
static int32_t BMP180_ReadUP(void)
{
    I2C_WriteReg(BMP180_ADDR, BMP_REG_CTRL, BMP_CMD_PRES+(BMP_OSS<<6)); HAL_Delay(5);
    uint8_t buf[3]; I2C_ReadRegs(BMP180_ADDR, BMP_REG_DATA, buf, 3);
    return (int32_t)((buf[0]<<16|buf[1]<<8|buf[2])>>(8-BMP_OSS));
}
static void BMP180_Read(float *tc, float *hpa)
{
    int32_t UT=BMP180_ReadUT();
    int32_t X1=((UT-(int32_t)bmp_calib.AC6)*(int32_t)bmp_calib.AC5)>>15;
    int32_t X2=((int32_t)bmp_calib.MC<<11)/(X1+(int32_t)bmp_calib.MD);
    int32_t B5=X1+X2;
    *tc=((B5+8)>>4)/10.0f;

    int32_t UP=BMP180_ReadUP();
    int32_t B6=B5-4000;
    X1=((int32_t)bmp_calib.B2*((B6*B6)>>12))>>11;
    X2=((int32_t)bmp_calib.AC2*B6)>>11;
    int32_t X3=X1+X2;
    int32_t B3=((((int32_t)bmp_calib.AC1*4+X3)<<BMP_OSS)+2)/4;
    X1=((int32_t)bmp_calib.AC3*B6)>>13;
    X2=((int32_t)bmp_calib.B1*((B6*B6)>>12))>>16;
    X3=((X1+X2)+2)>>2;
    uint32_t B4=((uint32_t)bmp_calib.AC4*(uint32_t)(X3+32768))>>15;
    uint32_t B7=((uint32_t)(UP-B3)*(50000UL>>BMP_OSS));
    int32_t p=(B7<0x80000000UL)?(int32_t)((B7*2)/B4):(int32_t)((B7/B4)*2);
    X1=(p>>8)*(p>>8); X1=(X1*3038)>>16; X2=(-7357*p)>>16;
    p=p+((X1+X2+3791)>>4);
    *hpa=p/100.0f;
}
static float ComputeAltitude(float hpa)
{
    return 44330.0f*(1.0f-powf(hpa/SEA_LEVEL_HPA,0.1903f));
}

/* ---- CubeMX prototypes ---- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* ================================================================
 *  main
 * ================================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    HAL_Delay(200);
    MPU6050_Init();
    HMC5883L_Init();
    BMP180_ReadCalib();

    /* 1-second warm-up for complementary filter */
    for (int i = 0; i < 100; i++) {
        float ax,ay,az,gx,gy,gz;
        MPU6050_Read(&ax,&ay,&az,&gx,&gy,&gz);
        CF_Update(ax,ay,az,gx,gy);
        HAL_Delay(10);
    }

    printf("\r\n=== GY-87 Step 2: Pitch / Roll / Yaw / Altitude ===\r\n\r\n");

    static float altitude = 0.0f;
    static float temp_bmp = 0.0f;
    uint8_t  bmp_cnt   = 0;
    uint32_t print_cnt = 0;

    while (1)
    {
        /* Read IMU */
        float ax,ay,az,gx,gy,gz;
        MPU6050_Read(&ax,&ay,&az,&gx,&gy,&gz);
        CF_Update(ax,ay,az,gx,gy);

        /* Read magnetometer */
        float mx,my,mz;
        HMC5883L_Read(&mx,&my,&mz);
        float yaw = ComputeYaw(mx,my,mz,cf_pitch,cf_roll);

        /* BMP180 every 10 loops = ~100 ms */
        if (++bmp_cnt >= 10) {
            bmp_cnt = 0;
            float pres;
            BMP180_Read(&temp_bmp, &pres);
            altitude = ComputeAltitude(pres);
        }

        /* Print every 20 loops = ~200 ms */
        if (++print_cnt >= 20) {
            print_cnt = 0;

            /* ── One-line summary (easy to parse for ESC controller later) ── */
            printf("P:%6.1f R:%6.1f Y:%6.1f A:%6.1f T:%4.1f\r\n",
                   cf_pitch, cf_roll, yaw, altitude, temp_bmp);

            /* ── Detail block ── */
            printf("  Accel  X:%6.3f Y:%6.3f Z:%6.3f g\r\n", ax, ay, az);
            printf("  Gyro   X:%6.1f Y:%6.1f Z:%6.1f d/s\r\n", gx, gy, gz);
            printf("  Mag    X:%6.3f Y:%6.3f Z:%6.3f G\r\n\r\n", mx, my, mz);
        }

        HAL_Delay(10); /* 10 ms = 100 Hz loop */
    }
}

/* ---- CubeMX generated ---- */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef o={0}; RCC_ClkInitTypeDef c={0};
    o.OscillatorType=RCC_OSCILLATORTYPE_HSI; o.HSIState=RCC_HSI_ON;
    o.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT; o.PLL.PLLState=RCC_PLL_NONE;
    if(HAL_RCC_OscConfig(&o)!=HAL_OK) Error_Handler();
    c.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource=RCC_SYSCLKSOURCE_HSI; c.AHBCLKDivider=RCC_SYSCLK_DIV1;
    c.APB1CLKDivider=RCC_HCLK_DIV1; c.APB2CLKDivider=RCC_HCLK_DIV1;
    if(HAL_RCC_ClockConfig(&c,FLASH_LATENCY_0)!=HAL_OK) Error_Handler();
}
static void MX_I2C1_Init(void)
{
    hi2c1.Instance=I2C1; hi2c1.Init.ClockSpeed=100000; hi2c1.Init.DutyCycle=I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1=0; hi2c1.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE; hi2c1.Init.OwnAddress2=0;
    hi2c1.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE; hi2c1.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(&hi2c1)!=HAL_OK) Error_Handler();
}
static void MX_USART1_UART_Init(void)
{
    huart1.Instance=USART1; huart1.Init.BaudRate=9600; huart1.Init.WordLength=UART_WORDLENGTH_8B;
    huart1.Init.StopBits=UART_STOPBITS_1; huart1.Init.Parity=UART_PARITY_NONE;
    huart1.Init.Mode=UART_MODE_TX_RX; huart1.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    huart1.Init.OverSampling=UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart1)!=HAL_OK) Error_Handler();
}
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}
void Error_Handler(void) { __disable_irq(); while(1){} }
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
