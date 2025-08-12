#ifndef CONFIG_H
#define CONFIG_H

#if JOINT == 1
#pragma message("Building for Joint 1")
#include "joint_1.h"
#elif JOINT == 2
#pragma message("Building for Joint 2")
#include "joint_2.h"
#elif JOINT == 3
#pragma message("Building for Joint 3")
#include "joint_3.h"
#elif JOINT == 4
#pragma message("Building for Joint 4")
#include "joint_4.h"
#else
#error "No configuration selected"
#endif

#endif // CONFIG_H