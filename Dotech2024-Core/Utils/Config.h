/**
 * @file Config.h
 */

#pragma once
#include "main.h"
// #include "appConfig.hpp"

/**
 * @brief The application name
 */
#ifndef APP_NAME
#warning "APP_NAME is not defined"  // This macro is defined in the makefile as TARGET
#define APP_NAME Dotech_2023
#endif  // APP_NAME

/**
 * @brief Useful macro for changing a macro to a static string
 */
#define _MACRO_TO_STRING(x) #x
#define MACRO_TO_STRING(x) _MACRO_TO_STRING(x)
