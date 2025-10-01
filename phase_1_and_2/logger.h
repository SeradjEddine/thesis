#ifndef LOGGER_H
#define LOGGER_H
#include <stdio.h>
#include "ekf.h"
int open_state_log(const char *filename);
void close_state_log(void);
void log_state(const struct ekf_state *x, const double P[P_DIM]);
#endif

