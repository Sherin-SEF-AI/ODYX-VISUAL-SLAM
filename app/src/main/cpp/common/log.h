// ODYX :: native logging shim (routes to logcat with per-module tags).
#pragma once
#include <android/log.h>

#define ODYX_LOG_TAG "ODYX"

#define ODYXI(tag, ...) __android_log_print(ANDROID_LOG_INFO,  "ODYX." tag, __VA_ARGS__)
#define ODYXW(tag, ...) __android_log_print(ANDROID_LOG_WARN,  "ODYX." tag, __VA_ARGS__)
#define ODYXE(tag, ...) __android_log_print(ANDROID_LOG_ERROR, "ODYX." tag, __VA_ARGS__)
#define ODYXD(tag, ...) __android_log_print(ANDROID_LOG_DEBUG, "ODYX." tag, __VA_ARGS__)
