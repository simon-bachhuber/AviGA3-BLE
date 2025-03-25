/*
 * TU Berlin - Control Systems - AviGA3
 *
 * IMU:
 * The "Integrated Measurement Unit" BMI270 is used in this project.
 * The communication is using SPI.
 * In bmi270.c, the initialization, setting,... for the IMU are performed.
 */

/*********************************************************************/
/*Includes*/

//COMMON
#include <common.h>

//BMI270
#include <bmi270_custom.h>

/*********************************************************************/
/*Private variables*/

/*********************************************************************/
/*Functions*/

//Init IMU
uint8_t imu_init(struct bmi2_dev *bmi2_dev)
{

  /* Status of api are returned to this variable. */
     int8_t rslt;

     /* Interface reference is given as a parameter
      * For I2C : BMI2_I2C_INTF
      * For SPI : BMI2_SPI_INTF
      */

     rslt = bmi2_interface_init(bmi2_dev, BMI2_SPI_INTF);

     /* Initialize bmi270. */
     rslt = bmi270_init(bmi2_dev);

     return rslt;
}

//Configuration ACCEL & GYRO
int8_t set_accel_gyro_config(struct bmi2_dev *dev, uint8_t imu_odr, uint8_t acc_range, uint8_t gyr_range)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accel and gyro configurations. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_get_sensor_config(config, 2, dev);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameter according to their requirement. */
        /* Accel configuration settings. */
        /* Set Output Data Rate */
        config[0].cfg.acc.odr = imu_odr;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[0].cfg.acc.range = acc_range;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* Gyro configuration settings. */
        /* Set Output Data Rate */
        config[1].cfg.gyr.odr = imu_odr;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[1].cfg.gyr.range = gyr_range;

        /* Gyroscope Bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set new configurations. */
        rslt = bmi270_set_sensor_config(config, 2, dev);
    }

    return rslt;
}


