/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

void main(void)
{
    while (1)
    {
        k_msleep(10);
    }
}
