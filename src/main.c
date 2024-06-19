/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdbool.h>
#include "imu/bmi160/bmi160_fuse.h"

int main(void)
{
	bmi_160_fusion_init();
	while (1) {
		bmi_160_fusion_loop();

	}
}
