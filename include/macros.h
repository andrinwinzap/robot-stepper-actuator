#ifndef MACROS_H
#define MACROS_H

#define RCCHECK(fn)                                                                             \
    {                                                                                           \
        rcl_ret_t temp_rc = fn;                                                                 \
        if ((temp_rc != RCL_RET_OK))                                                            \
        {                                                                                       \
            ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                                  \
        }                                                                                       \
    }
#define RCSOFTCHECK(fn)                                                                           \
    {                                                                                             \
        rcl_ret_t temp_rc = fn;                                                                   \
        if ((temp_rc != RCL_RET_OK))                                                              \
        {                                                                                         \
            ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                         \
    }

#endif // MACROS_H