/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdbool.h>
#include "imu/bmi160/bmi160.h"

int main(void)
{
	printf("haii");
	bmi160init();
	while(true) {
		bmi160loop();
	}
}
