#ifndef __SHAREDFIRMWARETYPES_H__
#define __SHAREDFIRMWARETYPES_H__
#include <stdint.h>
#include <array>

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
 * Generic data vector type that can be used for tires, load cells, or anything that has to do with
 * the four corners of the car.
 */
template <typename T>
struct veh_vec
{
public:
    T FL;
    T FR;
    T RL;
    T RR;

public:
    veh_vec() = default;
    veh_vec(T _FL, T _FR, T _RL, T _RR)
    {
        FL = _FL;
        FR = _FR;
        RL = _RL;
        RR = _RR;
    }

    /// @brief copy values to array in FL, FR, RL, RR order
    void copy_to_arr(T (&arr_out)[4])
    {
        arr_out[0] = FL;
        arr_out[1] = FR;
        arr_out[2] = RL;
        arr_out[3] = RR;
    }
};

template <typename T>
struct xyz_vec
{
    T x;
    T y;
    T z;
};

template <typename T>
struct xy_vec
{
    T x;
    T y;
};





/**
 * Struct containing ALL of the data from the VCF Interfaces. An instance of this struct will be
 * passed into each of VCF's systems as an input.
 */
struct VCFInterfaceData_s
{
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

/**
 * Since suspension potentiometers are only used for validation, it's OK to keep them in units of "analog".
 */
struct FrontSusPotsUnfiltered_s
{
    float FL_sus_pot_unfiltered_analog;
    float FR_sus_pot_unfiltered_analog;
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

/**
 * Since suspension potentiometers are only used for validation, it's OK to keep them in units of "analog".
 */
struct FrontSusPotsFiltered_s
{
    float FL_sus_pot_filtered_analog;
    float FR_sus_pot_filtered_analog;
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





/**
 * Struct containing ALL of the VCR interfaces' data. An instance of this struct  will be passed into the VCR
 * systems before calling their tick() methods.
 */
struct VCRInterfaceData_s
{
    RearLoadCellsUnfiltered_s rear_loadcells_unfiltered;
    RearSusPotsUnfiltered_s rear_suspots_unfiltered;
    VectorNavData_s vectornav_data;
    CurrentSensorData_s current_sensor_data;
    ShutdownSensingData_s shutdown_sensing_data;
    EthernetLinkData_s ethernet_is_linked;
    veh_vec<InverterData_s> inverter_data;
    PedalsSystemData_s pedals_system_data;
    FrontLoadCellsFiltered_s front_loadcells_filtered;
    FrontSusPotsFiltered_s front_suspots_filtered;
    SteeringFiltered_s steering_filtered;
    DashInputState_s dash_input_state;

};

struct RearLoadCellsUnfiltered_s
{
    float RL_loadcell_unfiltered_pounds;
    float RR_loadcell_unfiltered_pounds;
};

/**
 * Since suspension potentiometers are only used for validation, it's OK to keep them in units of "analog".
 */
struct RearSusPotsUnfiltered_s
{
    float RL_sus_pot_unfiltered_analog;
    float RR_sus_pot_unfiltered_analog;
};

/**
 * Directly copied from HT08 MCU SharedDataTypes.h.
 */
struct VectorNavData_s
{
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float linear_accel_x;
    float linear_accel_y;
    float linear_accel_z;
    float uncompLinear_accel[3]; // 3D uncompensated linear acceleration
    float yaw;
    float pitch;
    float roll;
    double latitude;
    double longitude;
    double ecef_coords[3]; // x,y,z
    uint64_t gps_time;     // gps time
    uint8_t vn_status;     // status
    xyz_vec<float> angular_rates;
};

struct CurrentSensorData_s
{
    float twentyfour_volt_sensor; // Senses the 24V power line
    float current_sensor_unfiltered;
    float current_refererence_unfiltered;
};

/**
 * The signals beginning with a letter prefix are according to this page (https://wiki.hytechracing.org/books/ht09-design/page/shutdown-circuit-order),
 * and are on the shutdown line. Signals without the letter prefix are the inputs to those shutdown relays that determine whether or not they close
 * when the latch button is pressed. VCR has FOUR relays. Since each shutdown "letter" is a node between shutdown components, that means VCR has five
 * nodes to probe and four relay inputs to probe (total of 9 booleans).
 */
struct ShutdownSensingData_s
{
    bool i_shutdown_in : 1;
    bool j_bspd_relay : 1;
    bool k_watchdog_relay : 1;
    bool l_bms_relay : 1;
    bool m_imd_relay : 1;

    bool bspd_is_ok : 1;
    bool watchdog_is_ok : 1;
    bool bms_is_ok : 1;
    bool imd_is_ok : 1;
};

/**
 * The 'link' lights from the ethernet switch to indicate whether or not each item is connected.
 */
struct EthernetLinkData_s
{
    bool acu_link : 1;
    bool drivebrain_link : 1;
    bool vcf_link : 1;
    bool teensy_link : 1;
    bool debug_link : 1;
    bool ubiquiti_link : 1;
};

/**
 * A collection of all the data InverterInterface.tpp used to send individually over CAN. Now,
 * using ethernet, we can bundle all of these values together for better organization.
 */
struct InverterData_s
{
    bool system_ready : 1;
    bool error : 1;
    bool warning : 1;
    bool quit_dc_on : 1;
    bool dc_on : 1;
    bool quit_inverter_on : 1;
    bool inverter_on : 1;
    bool derating_on : 1;
    int speed_rpm;
    int actual_motor_torque;
    int commanded_torque;
    int motor_temp;
    int inverter_temp;
    int diagnostic_number;
    int igbt_temp;
    int dc_bus_voltage;
    int actual_power;
    int feedback_torque;
};

struct BMSData_s
{
    float voltages[126];
    float temperatures[126];
    long last_acu_message_timestamp; // Used for detecting "heartbeat"    
};

/**
 * Forwarded directly from CAN with no additional transformations.
 */
struct EnergyMeterData_s
{
    float em_current; // Current, in amps, from the EM
    float em_voltage; // Voltage, in volts, from the EM.
};




/**
 * Struct containing ALL of the VCR systems' data. An instance of this struct, along with an instance
 * of the interface data struct, will be passed into the VCR interfaces so they can send the data
 * out towards other microcontrollers.
 */
struct VCRSystemData_s
{
    RearLoadCellsFiltered_s rear_loadcells_filtered;
    RearSusPotsFiltered_s rear_suspots_filtered;
    SafetySystemData_s safety_system_data;
};

struct RearLoadCellsFiltered_s
{
    float RL_loadcell_filtered_pounds;
    float RR_loadcell_filtered_pounds;
    bool rear_loadcell_FIR_is_saturated : 1;
};

/**
 * Since suspension potentiometers are only used for validation, it's OK to keep them in units of "analog".
 */
struct RearSusPotsFiltered_s
{
    float RL_sus_pot_filtered_analog;
    float RR_sus_pot_filtered_analog;
    bool rear_loadcell_FIR_is_saturated : 1;
};

struct SafetySystemData_s
{
    bool software_is_ok : 1;
};

#endif // __SHAREDFIRMWARETYPES_H__