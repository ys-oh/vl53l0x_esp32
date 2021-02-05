/*
 * File : vl53l0x.c
 * Created: Thursday, 04 February 2021
 * Author: yunsik oh (oyster90@naver.com)
 * 
 * Modified: Thursday, 04 February 2021
 * 
 */
#include "vl53l0x.h"
#include "vl53l0x_platform_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#define VERSION_REQUIRED_MAJOR 1
#define VERSION_REQUIRED_MINOR 0
#define VERSION_REQUIRED_BUILD 2

#define IO_NUM          GPIO_NUM_5
#define IO_SEL(num)     (1ULL << (num))

#define msec(t) ((t) / portTICK_PERIOD_MS) // millisecond convert
#define sec(t) ((t) * 100U)
#define minute(t) (sec(t * 60));

static const char* TAG = "vl53l0x";
#define VL53L0X_TAG TAG

#ifdef VL53L0X_LOG_ENABLE
#define VL53L0X_Log(level, fmt, ...) \
    ESP_LOG_LEVEL_LOCAL(level, VL53L0X_TAG, fmt, ##__VA_ARGS__)

#define VL53L0X_ErrLog(fmt, ...) \
    VL53L0X_Log(ESP_LOG_ERROR, "VL53L0X_ErrLog %s" fmt, __func__, ##__VA_ARGS__)
#else
    #define VL53L0X_Log(level, fmt, ...) (void)0
    #define VL53L0X_ErrLog(fmt, ...) (void)0
#endif

static void print_pal_error(VL53L0X_Error Status)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    VL53L0X_ErrLog("API Status: %i : %s\n", Status, buf);
}

static VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady = 0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE)
    {
        LoopNb = 0;
        do
        {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE)
            {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP)
        {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

static VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted = 0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE)
    {
        LoopNb = 0;
        do
        {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE)
            {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP)
        {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

VL53L0X_Error VL53L0X_Device_init(VL53L0X_Dev_t *device)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t *pMyDevice = device;
    VL53L0X_Version_t Version;
    VL53L0X_Version_t *pVersion = &Version;
    VL53L0X_DeviceInfo_t DeviceInfo;

    pMyDevice->comms_type = 1;
    pMyDevice->comms_speed_khz = 400;
    pMyDevice->I2cDevAddr = 0x29;           ///< Default sensor I2C address
    

    Status = VL53L0X_comms_initialise(0, 400); // VL53L0X_i2c_init(I2C_NUM_0, 400000);
    if (Status != VL53L0X_ERROR_NONE)
    {
        VL53L0X_ErrLog("i2c init failed!");
        return Status;
    }

    /*
     *  Get the version of the VL53L0X API running in the firmware
     */

    int32_t status_int;
    status_int = VL53L0X_GetVersion(pVersion);
    if (status_int != 0)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    /*
     *  Verify the version of the VL53L0X API running in the firmrware
     */

    if (Status == VL53L0X_ERROR_NONE)
    {
        if (pVersion->major != VERSION_REQUIRED_MAJOR ||
            pVersion->minor != VERSION_REQUIRED_MINOR ||
            pVersion->build != VERSION_REQUIRED_BUILD)
        {
            VL53L0X_Log(ESP_LOG_DEBUG, "VL53L0X API Version Error: Your firmware has %d.%d.%d (revision %d). This example requires %d.%d.%d.\n",
                     pVersion->major, pVersion->minor, pVersion->build, pVersion->revision,
                     VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
        }
    }

    // End of implementation specific
    VL53L0X_Log(ESP_LOG_DEBUG, "Call of VL53L0X_DataInit\n");
    Status = VL53L0X_DataInit(pMyDevice); // Data initialization
    if (Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    Status = VL53L0X_GetDeviceInfo(pMyDevice, &DeviceInfo);
    if (Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    VL53L0X_Log(ESP_LOG_DEBUG, "VL53L0X_GetDeviceInfo:\n");
    VL53L0X_Log(ESP_LOG_DEBUG, "Device Name : %s\n", DeviceInfo.Name);
    VL53L0X_Log(ESP_LOG_DEBUG, "Device Type : %s\n", DeviceInfo.Type);
    VL53L0X_Log(ESP_LOG_DEBUG, "Device ID : %s\n", DeviceInfo.ProductId);
    VL53L0X_Log(ESP_LOG_DEBUG, "ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
    VL53L0X_Log(ESP_LOG_DEBUG, "ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);

    if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1))
    {
        VL53L0X_Log(ESP_LOG_DEBUG, "Error expected cut 1.1 but found cut %d.%d\n",
                    DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
        Status = VL53L0X_ERROR_NOT_SUPPORTED;
        return Status;
    }

    // StaticInit will set interrupt by default
    VL53L0X_Log(ESP_LOG_DEBUG, "Call of VL53L0X_StaticInit\n");
    Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization

    if (Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    uint8_t VhvSettings;
    uint8_t PhaseCal;

    VL53L0X_Log(ESP_LOG_DEBUG, "Call of VL53L0X_PerformRefCalibration\n");
    Status = VL53L0X_PerformRefCalibration(pMyDevice,
                                            &VhvSettings, &PhaseCal); // Device Initialization
    if (Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    //================================
    // TODO: RefCalibration Data Handling
    //================================

    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    VL53L0X_Log(ESP_LOG_DEBUG, "Call of VL53L0X_PerformRefSpadManagement\n");
    Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
                                                &refSpadCount, &isApertureSpads); // Device Initialization
    if (Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    //================================
    // TODO: RefSpadManagement Data Handling
    //================================

    VL53L0X_Log(ESP_LOG_DEBUG, "Call of VL53L0X_SetDeviceMode\n");
    VL53L0X_DeviceModes default_device_mode = VL53L0X_DEVICEMODE_CONTINUOUS_RANGING;
    Status = VL53L0X_SetDeviceMode(pMyDevice, default_device_mode); // Setup in single ranging mode
    if (Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    VL53L0X_Log(ESP_LOG_DEBUG, "Call of VL53L0X_StartMeasurement\n");
    Status = VL53L0X_StartMeasurement(pMyDevice);
    if (Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    VL53L0X_PollingDelay(pMyDevice);

    return Status;
}

VL53L0X_Error VL53L0X_Device_deinit(VL53L0X_Dev_t *device)
{
    VL53L0X_Error Status;

    VL53L0X_Log(ESP_LOG_DEBUG, "Call of VL53L0X_StopMeasurement\n");
    Status = VL53L0X_StopMeasurement(device);
    if (Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    VL53L0X_Log(ESP_LOG_DEBUG, "Wait Stop to be competed\n");
    Status = WaitStopCompleted(device);
    if (Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    Status = VL53L0X_ClearInterruptMask(device,
                                        VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
    if (Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status);
        return Status;
    }

    return Status;
}

VL53L0X_Error VL53L0X_Device_getMeasurement(VL53L0X_Dev_t *device, uint16_t* data)
{
    VL53L0X_Error Status;
    Status = WaitMeasurementDataReady(device);
    if (Status != VL53L0X_ERROR_NONE)
    {
        VL53L0X_ErrLog("WaitMeasurementDataReady error (%d)", Status);
        return Status;
    }

    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    Status = VL53L0X_GetRangingMeasurementData(device, &RangingMeasurementData);
    if (Status != VL53L0X_ERROR_NONE)
    {
        VL53L0X_ErrLog("VL53L0X_GetRangingMeasurementData error (%d)", Status);
        return Status;
    }

    *data = RangingMeasurementData.RangeMilliMeter;
    // Clear the interrupt
    VL53L0X_ClearInterruptMask(device, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

    return Status;
}
