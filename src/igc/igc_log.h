
/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c)  2018 Intel Corporation */

#ifndef _IGC_LOG_H_

#include <stdarg.h>

#define IGC_LOG_INF 1
#define IGC_LOG_ERR 2

void igc_logger(int msg_log_level, char * msg, ...);

#endif