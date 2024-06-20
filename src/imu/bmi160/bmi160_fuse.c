/*
 * Copyright (c) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/drivers/i2c.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "bmi160.h"
#include "../../../Fusion/Fusion/Fusion.h"
#include "bmi160_fuse.h"

/*! @brief This structure containing relevant bmi160 info */
struct bmi160_dev bmi160dev;

/*! @brief variable to hold the bmi160 accel data */
struct bmi160_sensor_data bmi160_accel;

/*! @brief variable to hold the bmi160 gyro data */
struct bmi160_sensor_data bmi160_gyro;

#define BMI160_DEV_ADDR       0x68
#define MAIN_IMU_NODE DT_NODELABEL(bmi_160_imu)
const struct i2c_dt_spec main_imu = I2C_DT_SPEC_GET(MAIN_IMU_NODE);

static void init_sensor_interface(void);

static void init_bmi160(void);

static void init_bmi160_sensor_driver_interface(void);



static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  return i2c_burst_write_dt(&main_imu, reg, bufp, len);
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  return i2c_burst_read_dt(&main_imu, reg, bufp, len);
}


void delays(uint32_t time) {
	k_sleep(K_MSEC(time));
}

static void init_bmi160_sensor_driver_interface(void)
{
	bmi160dev.write = platform_write;
    bmi160dev.read = platform_read;
    bmi160dev.delay_ms = delays;

    /* set correct i2c address */
    bmi160dev.id = BMI160_DEV_ADDR;
    bmi160dev.intf = BMI160_I2C_INTF;
}

static void init_bmi160(void)
{
    int8_t rslt;

    rslt = bmi160_init(&bmi160dev);

    if (rslt == BMI160_OK)
    {
        printf("BMI160 initialization success !\n");
        printf("Chip ID 0x%X\n", bmi160dev.chip_id);
    }
    else
    {
        printf("BMI160 initialization failure !\n");
    }

    /* Select the Output data rate, range of accelerometer sensor */
    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_8G;
    bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi160dev);
}

void setup_bmi160()
{
	init_bmi160_sensor_driver_interface();
	init_bmi160();
}







#define SAMPLE_RATE (100) // replace this with actual sample rate
// Define calibration (replace with actual calibration data if available)
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

// Initialise algorithms
    FusionOffset offset;
    FusionAhrs ahrs;

void bmi_160_fusion_init() {



	setup_bmi160();

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    // This loop should repeat each time new gyroscope data is available
}
void bmi_160_fusion_loop(float *quat[4]) {
	bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &bmi160_accel, &bmi160_gyro, &bmi160dev);
    
    // Acquire latest sensor data
    const clock_t timestamp = k_uptime_get(); // replace this with actual gyroscope timestamp
    FusionVector accelerometer = {bmi160_accel.x/4096.0f, bmi160_accel.y/4096.0f, bmi160_accel.z/4096.0f}; // replace this with actual gyroscope data in degrees/s
    FusionVector gyroscope = {bmi160_gyro.x/16.384f, bmi160_gyro.y/16.384f, bmi160_gyro.z/16.384f}; // replace this with actual accelerometer data in g

    // Apply calibration
    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
    
    // Update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    static clock_t previousTimestamp;
    const float deltaTime = (float) (timestamp - previousTimestamp) / 1000;
    previousTimestamp = timestamp;

    // Update gyroscope AHRS algorithm
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

    // Print algorithm outputs
    const FusionQuaternion bmiquat = FusionAhrsGetQuaternion(&ahrs);
    const FusionEuler euler = FusionQuaternionToEuler(bmiquat);
    const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

    // quat = [quat]

    printf("Roll %4.1lf, Pitch %4.1lf, Yaw %4.1lf, X %4.1lf, Y %4.1lf, Z %4.1lf, dT %4.6lf\n",
            euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
            earth.axis.x, earth.axis.y, earth.axis.z,
            deltaTime);
}