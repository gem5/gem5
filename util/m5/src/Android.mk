LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE := m5
LOCAL_SRC_FILES := \
    m5.c

ifeq ($(TARGET_ARCH),x86)
    LOCAL_SRC_FILES += \
        m5op_x86.S
else ifeq ($(TARGET_ARCH),arm)
    LOCAL_SRC_FILES += \
        m5op_arm.S
else ifeq ($(TARGET_ARCH),arm64)
    LOCAL_SRC_FILES += \
        m5op_arm_A64.S
else
    $(error "Unsupported TARGET_ARCH $(TARGET_ARCH)")
endif

include $(BUILD_EXECUTABLE)
