#ifndef MACROS_H
#define MACROS_H

#define RCCHECK(fn)                                                                           \
    {                                                                                         \
        rcl_ret_t temp_rc = fn;                                                               \
        if ((temp_rc != RCL_RET_OK))                                                          \
        {                                                                                     \
            ESP_LOGE(TAG, "Failed status on line %d: %d. Retrying.", __LINE__, (int)temp_rc); \
            taskYIELD();                                                                      \
            goto try_uros_task;                                                               \
        }                                                                                     \
    }
#define RCSOFTCHECK(fn)                                                                         \
    {                                                                                           \
        rcl_ret_t temp_rc = fn;                                                                 \
        if ((temp_rc != RCL_RET_OK))                                                            \
        {                                                                                       \
            ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.", __LINE__, (int)temp_rc); \
        }                                                                                       \
    }

#endif // MACROS_H