#include "Blimp.h"
#include <SRV_Channel/SRV_Channel.h>

#define PROP_SCALE_MAX 1000

const AP_Param::GroupInfo Propellers::var_info[] = {

    // @Param: THRUST_MAX
    // @DisplayName: Maximum thrust
    // @Description: This is the maximum thrust of the propellers
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("THRUST_MAX", 1, Propellers, thrust_max, 100),

    // @Param: TURBO_MODE
    // @DisplayName: Enable turbo mode
    // @Description: Enables increased thrust in turbo mode
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("TURBO_MODE", 2, Propellers, turbo_mode, 0),

    AP_GROUPEND
};

// Constructor
Propellers::Propellers(uint16_t loop_rate) :
    _loop_rate(loop_rate)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void Propellers::setup_propellers()
{
    // Initialize the propellers, assuming propeller IDs are 0, 1, and 2
    add_propeller(0, 0, 1.0, 0);  // Horizontal right propeller (forward and right yaw)
    add_propeller(1, 0, 1.0, 0);  // Horizontal left propeller (forward and left yaw)
    add_propeller(2, 0, 0, 1.0);  // Vertical propeller (vertical thrust)

    // Set the initial angle for the propellers
    SRV_Channels::set_angle(SRV_Channel::k_motor1, PROP_SCALE_MAX);
    SRV_Channels::set_angle(SRV_Channel::k_motor2, PROP_SCALE_MAX);
    SRV_Channels::set_angle(SRV_Channel::k_motor3, PROP_SCALE_MAX);
}

void Propellers::add_propeller(int8_t prop_num, float right_thrust_fac, float front_thrust_fac, float down_thrust_fac)
{
    // Ensure valid propeller number is provided
    if (prop_num >= 0 && prop_num < NUM_PROPS) {
        // Set thrust factors
        _right_thrust_factor[prop_num] = right_thrust_fac;
        _front_thrust_factor[prop_num] = front_thrust_fac;
        _down_thrust_factor[prop_num] = down_thrust_fac;
    }
}

void Propellers::output()
{
    if (!_armed) {
        // Set everything to zero so propellers stop moving
        right_out = 0;
        front_out = 0;
        down_out  = 0;
        yaw_out   = 0;
    }

#if HAL_LOGGING_ENABLED
    blimp.Write_PROP(right_out, front_out, down_out, yaw_out);
#endif

    // Constrain outputs
    right_out = constrain_float(right_out, -1, 1);
    front_out = constrain_float(front_out, -1, 1);
    down_out = constrain_float(down_out, -1, 1);
    yaw_out = constrain_float(yaw_out, -1, 1);

    _time = AP_HAL::micros() * 1.0e-6;

    for (int8_t i = 0; i < NUM_PROPS; i++) {
        _thrust[i] = _right_thrust_factor[i] * right_out + 
                     _front_thrust_factor[i] * front_out +
                     _down_thrust_factor[i] * down_out +
                     _yaw_thrust_factor[i] * yaw_out;

        if (turbo_mode) {
            // Increase thrust in turbo mode
            _thrust[i] *= 1.5;
        }

        // Constrain thrust
        _thrust[i] = constrain_float(_thrust[i], 0, thrust_max);

        // Output thrust to propeller
        SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), _thrust[i] * PROP_SCALE_MAX);
    }

#if HAL_LOGGING_ENABLED
    blimp.Write_PROPO(_thrust);
#endif
}

void Propellers::output_min()
{
    right_out = 0;
    front_out = 0;
    down_out  = 0;
    yaw_out   = 0;
    Propellers::output();
}
