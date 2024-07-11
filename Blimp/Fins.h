//This class converts horizontal acceleration commands to fin flapping commands.
#ifndef PROPELLERS_H
#define PROPELLERS_H

#include "Blimp.h"
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define NUM_PROPS 3 // Current number of propellers
#define PROP_SCALE_MAX 1000

class Propellers
{
public:
    friend class Blimp;
    friend class Loiter;

    enum motor_frame_class {
        MOTOR_FRAME_UNDEFINED = 0,
        MOTOR_FRAME_AIRFISH = 1,
    };
    enum motor_frame_type {
        MOTOR_FRAME_TYPE_AIRFISH = 1,
    };

    // Constructor
    Propellers(uint16_t loop_rate);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    bool initialised_ok() const
    {
        return true;
    }

    void armed(bool arm)
    {
        if (arm != _armed) {
            _armed = arm;
            AP_Notify::flags.armed = arm;
        }
    }

    bool armed() const
    {
        return _armed;
    }

protected:
    // Internal variables
    const uint16_t _loop_rate;          // Rate in Hz at which output() function is called (normally 400Hz)
    uint16_t _speed_hz;                 // Speed in Hz to send updates to motors
    float _throttle_avg_max;            // Last throttle input from set_throttle_avg_max
    float _time;                        // Current timestamp
    bool _armed;                        // 0 if disarmed, 1 if armed

    float _thrust[NUM_PROPS];           // Thrust values
    float _right_thrust_factor[NUM_PROPS];
    float _front_thrust_factor[NUM_PROPS];
    float _down_thrust_factor[NUM_PROPS];

    int8_t _num_added;

public:
    float right_out;                    // Input right movement, negative for left, +1 to -1
    float front_out;                    // Input forward movement, negative for backward, +1 to -1
    float yaw_out;                      // Input yaw, +1 to -1
    float down_out;                     // Input height control, +1 to -1

    AP_Float thrust_max;
    AP_Int8 turbo_mode;

    bool _interlock;                    // 1 if the motor interlock is enabled (i.e., motors run), 0 if disabled (motors don't run)
    bool _initialised_ok;               // 1 if initialization was successful

    void output_min();

    void add_propeller(int8_t prop_num, float right_thrust_fac, float front_thrust_fac, float down_thrust_fac);

    void setup_propellers();

    void output();

    float get_throttle()
    {
        // Only for Mavlink - essentially just an indicator of how hard the propellers are working.
        // Note that this is the unconstrained version, so if the higher level control gives too high input,
        // throttle will be displayed as more than 100.
        return fmaxf(fmaxf(fabsf(down_out), fabsf(front_out)), fmaxf(fabsf(right_out), fabsf(yaw_out)));
    }

    void rc_write(uint8_t chan, uint16_t pwm);
};

#endif // PROPELLERS_H
