#ifndef __SHAREDFIRMWARETYPES_H__
#define __SHAREDFIRMWARETYPES_H__

enum class AnalogSensorStatus_e
{
    ANALOG_SENSOR_GOOD = 0,
    ANALOG_SENSOR_CLAMPED = 1,
};

struct AnalogConversion_s
{
    int raw;
    float conversion;
    AnalogSensorStatus_e status;
};

template <int N>
struct AnalogConversionPacket_s
{
    AnalogConversion_s conversions[N];
};





/**
 * Struct containing ALL of the data from the VCF Interfaces. An instance of this struct will be
 * passed into each of VCF's systems as an input.
 */
struct VCFInterfaceData_s
{
    VCRState_s vcr_state; // Received over ethernet. Only Dashboard signals are strictly necessary.
    PedalsUnfiltered_s pedals_unfiltered;
    FrontLoadCellsUnfiltered_s front_loadcells_unfiltered;
    FrontSusPotsUnfiltered_s front_suspots_unfiltered;
    SteeringUnfiltered_s steering_unfiltered;
    DashInputState_s dash_input_state; // Direct button signals from the dashboard IOExpander
};

struct PedalsUnfiltered_s
{
    float accel1_unfiltered_percent;
    float accel2_unfiltered_percent;
    float brake1_unfiltered_percent;
    float brake2_unfiltered_percent;
};

struct FrontLoadCellsUnfiltered_s
{
    float FL_loadcell_unfiltered_pounds;
    float FR_loadcell_unfiltered_pounds;
};

struct FrontSusPotsUnfiltered_s
{
    float FL_sus_pot_unfiltered_pounds;
    float FR_sus_pot_unfiltered_pounds;
};

struct SteeringUnfiltered_s
{
    float analog_steering_unfiltered_degrees;
    float digital_steering_unfiltered_degrees;
};

struct DashInputState_s
{
    bool dim_btn_is_pressed : 1;
    bool preset_btn_is_pressed : 1;
    bool mc_reset_btn_is_pressed : 1; // Resets the motor controller errors
    bool mode_btn_is_pressed : 1; // Currently un-used (dial state instead)
    bool start_btn_is_pressed : 1; // The start button is the READY_TO_DRIVE button
    bool data_btn_is_pressed : 1;
    bool left_paddle_is_pressed : 1; // Paddle shifters were not on HT08, but the code is compatible with them.
    bool right_paddle_is_pressed : 1; // Paddle shifters were not on HT08, but the code is compatible with them.
    ControllerMode_e dial_state;
};

/**
 * Enum for the modes on the dial, corresponds directly to dial index position.
 */
enum class ControllerMode_e
{
    MODE_0,
    MODE_1,
    MODE_2,
    MODE_3,
    MODE_4,
    MODE_5,
};





/**
 * Struct containing ALL of the VCF systems' data. An instance of this struct, along with an instance
 * of the interface data struct, will be passed into the VCF interfaces so they can send the data
 * out towards other microcontrollers.
 */
struct VCFSystemData_s
{
    PedalsSystemData_s pedals_system_data;
    FrontLoadCellsFiltered_s front_loadcells_filtered;
    FrontSusPotsFiltered_s front_suspots_filtered;
    SteeringFiltered_s steering_filtered;
    DashDisplayState_s dash_display;
};

/**
 * Copied from HT08 MCU, but with clearer names.
 */
struct PedalsSystemData_s
{
    bool accel_is_implausible : 1; // Checks if either accel pedal is out of range OR they disagree by more than 10%
    bool brake_is_implausible : 1; // Checks if brake sensor is out of range.
    bool brake_is_pressed : 1; // True if brake pedal is pressed beyond the specified activationPercentage.
    bool accel_is_pressed : 1; // True if the accel pedal is pressed beyond the specified activationPercentage.
    bool mech_brake_is_active : 1; // True if the brake pedal is pressed beyond mechanical_activation_percentage.
    bool brake_and_accel_pressed_implausibility_high : 1; // If accel is pressed at all while mech_brake_is_active.
    bool implausibility_has_exceeded_max_duration : 1; // True if implausibility lasts more than 100ms
    float accel_percent;
    float brake_percent;
    float regen_percent; // When brake pedal is 0%, regen_percent is 0.0. When brakes are at mechanical_activation_percentage,
                         // regen_percent is at 1.0. For instance, if mech activation percentage was 60%, then when brake
                         // travel is at 40%, regen_percent would be 0.667. Beyond that, regen_percent is clamped to 1.0.
};

struct FrontLoadCellsFiltered_s
{
    float FL_loadcell_filtered_pounds;
    float FR_loadcell_filtered_pounds;
    bool front_loadcell_FIR_is_saturated : 1;
};

struct FrontSusPotsFiltered_s
{
    float FL_sus_pot_filtered_pounds;
    float FR_sus_pot_filtered_pounds;
    bool front_loadcell_FIR_is_saturated : 1;
};

struct SteeringFiltered_s
{
    float steering_filtered_degrees;
    bool steering_FIR_is_saturated : 1;
};

/**
 * Struct for sending additional data to the Dash regarding what information to display. This may or may not be
 * necessary because the Dash will already probably be receiving a lot of redundant information.
 */
struct DashDisplayState_s
{
    int dash_data = -1;
};

#endif // __SHAREDFIRMWARETYPES_H__