/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdbool.h>
#include "imu/sensor.h"



int main(void)
{
	init_fused_IMU();
	while (1) {
		read_fused_IMU();
	}
}
