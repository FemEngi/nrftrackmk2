

#include "bmi160/bmi160.h"
void read_fused_IMU(uint8_t *quat[4]) {
    #if IMU == BMI160
        bmi_160_fusion_loop(&quat);
    #endif
}
void init_fused_IMU() {
    #if IMU == BMI160
        bmi_160_fusion_init();
    #endif
}