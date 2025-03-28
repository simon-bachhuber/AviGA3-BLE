/*
 * TU Berlin - Control Systems - AviGA3
 *
 * IMU:
 * The "Integrated Measurement Unit" BMI270 is used in this project.
 * The communication is using SPI.
 * In bmi270.c, the initialization, setting,... for the IMU are performed.
 */

/*********************************************************************/
#ifndef BMI270_CUSTOM_H
#define BMI270_CUSTOM_H

/*********************************************************************/
/*Includes*/
#include <bmi270_common.h>

/*       Pins              */
/******************************************************************************/
#define BMI_INT1_Pin GPIO_PIN_4
#define BMI_INT1_GPIO_Port GPIOA
#define BMI_INT1_EXTI_IRQn EXTI4_IRQn
#define SPI_SCK_Pin GPIO_PIN_5
#define SPI_SCK_GPIO_Port GPIOA
#define SPI_MISO_Pin GPIO_PIN_6
#define SPI_MISO_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_7
#define SPI_MOSI_GPIO_Port GPIOA
#define SPI_CS_BMI_Pin GPIO_PIN_8
#define SPI_CS_BMI_GPIO_Port GPIOA
#define BMI_INT2_Pin GPIO_PIN_9
#define BMI_INT2_GPIO_Port GPIOA


/*       Buffer              */
/******************************************************************************/
//SPI-TX-RX-Buffersize (Min. Size - Size of bmi270_config: 8192)
#define SPI_BUFF_SZ (8192+1)

/*       FIFO              */
/******************************************************************************/
/*! Buffer size allocated to store raw FIFO data (BMI270 FIFO-Size: 2 kB ) */
#define BMI2_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(2048)

/*! Length of data to be read from FIFO (BMI270 FIFO-Size: 2 kB ) */
#define BMI2_FIFO_RAW_DATA_USER_LENGTH  UINT16_C(2048)

/*! Number of accel frames to be extracted from FIFO */

/*!
 * Calculation:
 * Complete FIFO = 2048, accel_frame_len = 6, gyro_frame_len = 6.
 * fifo_accel_frame_count = (2048 / (6 + 6 )) = 170 frames
 */
#define BMI2_FIFO_ACCEL_FRAME_COUNT     UINT8_C(170)

/*! Number of gyro frames to be extracted from FIFO */
#define BMI2_FIFO_GYRO_FRAME_COUNT      UINT8_C(170)

/*! Setting a watermark level in FIFO (2048 * 90% = 1843. (2 bytes * 3 axes * 2 sensors = 12 bytes. round(1843 bytes / 12 bytes) = 153 sets. 153 * 12 bytes = 1836 bytes) */
#define BMI2_FIFO_WATERMARK_LEVEL       UINT16_C(1836)

/*       Structs              */
/******************************************************************************/


/*       Functions              */
/******************************************************************************/
uint8_t imu_init(struct bmi2_dev *bmi2_dev);
int8_t set_accel_gyro_config(struct bmi2_dev *dev, uint8_t imu_odr, uint8_t acc_range, uint8_t gyr_range);
int8_t read_imu_data_direct(struct bmi2_dev *dev, struct bmi2_sens_axes_data *accel, struct bmi2_sens_axes_data *gyro);

#endif /* BMI270_CUSTOM_H */
