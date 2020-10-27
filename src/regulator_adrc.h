#ifndef __SPEED_CONTROLLER__
#define __SPEED_CONTROLLER__


#include "app.h"
#include "config_map.h"
#include "math/fix16_math.h"

// ADRC iteration frequency, Hz. To fit math in fix16 without overflow.
#define APP_ADRC_FREQUENCY 40

constexpr int freq_divisor = APP_TICK_FREQUENCY / APP_ADRC_FREQUENCY;
// Coefficient used by ADRC observers integrators
constexpr fix16_t integr_coeff = F16(1.0 / APP_ADRC_FREQUENCY);

class Regulator
{
public:
    // Output power [0..1] for triac control
    fix16_t out_power = 0;

    // ADRC coefficients

    fix16_t cfg_adrc_Kp;
    fix16_t cfg_adrc_Kobservers;
    fix16_t cfg_adrc_b0_inv;

    // 1-st order ADRC system inside by this article
    //   https://arxiv.org/pdf/1908.04596.pdf
    //   
    void tick(fix16_t knob, fix16_t speed)
    {
        // Downscale input frequency to avoid fixed poind overflow.
        // 40000Hz => 40Hz

        if (tick_freq_divide_counter >= freq_divisor) tick_freq_divide_counter = 0;

        if (tick_freq_divide_counter > 0)
        {
            tick_freq_divide_counter++;
            return;
        }

        tick_freq_divide_counter++;

        knob_normalized = normalize_knob(knob);

        regulator_speed_out = speed_adrc_tick(speed);
        out_power = apply_rpm_voltage_compensation(regulator_speed_out);
    }

    // Load config from emulated EEPROM
    void configure()
    {
        cfg_dead_zone_width_norm = fix16_from_float(eeprom_float_read(CFG_DEAD_ZONE_WIDTH_ADDR,
            CFG_DEAD_ZONE_WIDTH_DEFAULT) / 100.0f);

        float _rpm_max = eeprom_float_read(CFG_RPM_MAX_ADDR, CFG_RPM_MAX_DEFAULT);

        cfg_rpm_max_limit_norm = fix16_from_float(
            eeprom_float_read(CFG_RPM_MAX_LIMIT_ADDR, CFG_RPM_MAX_LIMIT_DEFAULT) / _rpm_max
        );

        float _rpm_min_limit = eeprom_float_read(CFG_RPM_MIN_LIMIT_ADDR, CFG_RPM_MIN_LIMIT_DEFAULT);
        // Don't allow too small low limit
        // ~ 3000 for 35000 max limit
        if (_rpm_min_limit < _rpm_max * 0.085f) _rpm_min_limit = _rpm_max * 0.085f;

        cfg_rpm_min_limit_norm = fix16_from_float(_rpm_min_limit / _rpm_max);

        knob_norm_coeff =  fix16_div(
            cfg_rpm_max_limit_norm - cfg_rpm_min_limit_norm,
            fix16_one - cfg_dead_zone_width_norm
        );

        // Read motor RPM <-> volts correction table (interpolation points)
        // Note, real table has (CFG_RPM_INTERP_TABLE_LENGTH + 2) points,
        // but first and last are always 0.0 and 1.0.
        correction_table_len = 0;
        cfg_rpm_volts_correction_table[correction_table_len++] = 0;

        for (int i = 0; i < CFG_RPM_INTERP_TABLE_LENGTH; i++)
        {
            cfg_rpm_volts_correction_table[correction_table_len++] = fix16_from_float(
                eeprom_float_read(
                    i + CFG_RPM_INTERP_TABLE_START_ADDR,
                    ((float)i + 1.0f) / (CFG_RPM_INTERP_TABLE_LENGTH + 1)
                )
            );
        }

        cfg_rpm_volts_correction_table[correction_table_len++] = fix16_one;

        cfg_adrc_Kp = fix16_from_float(eeprom_float_read(CFG_ADRC_KP_ADDR,
            CFG_ADRC_KP_DEFAULT));
        cfg_adrc_Kobservers = fix16_from_float(eeprom_float_read(CFG_ADRC_KOBSERVERS_ADDR,
            CFG_ADRC_KOBSERVERS_DEFAULT));

        cfg_adrc_b0_inv = fix16_from_float(1.0f
                                           / eeprom_float_read(CFG_ADRC_B0_ADDR,
                                                               CFG_ADRC_B0_DEFAULT));

        adrc_update_observers_parameters();
        reset_state();
    }

    // Calculate internal observers parameters L1, L2
    // based on current cfg_adrc_Kobservers value
    void adrc_update_observers_parameters()
    {
        adrc_L1 = 2 * fix16_mul(cfg_adrc_Kobservers, cfg_adrc_Kp);
        adrc_L2 = fix16_mul(fix16_mul(cfg_adrc_Kobservers, cfg_adrc_Kp),
                            fix16_mul(cfg_adrc_Kobservers, cfg_adrc_Kp));
    }

    // Reset internal regulator state
    void reset_state()
    {
        adrc_speed_estimated = 0;
        adrc_correction = 0;

        regulator_speed_out = 0;
        // Skip iteration to allow meter resync
        tick_freq_divide_counter = 1;
    }

private:
    bool pid_i_enabled;

    // Control dead zone width near 0, when motor should not run.
    fix16_t cfg_dead_zone_width_norm;

    // Config limits are now in normalized [0.0..1.0] form of max motor RPM.
    fix16_t cfg_rpm_max_limit_norm;
    fix16_t cfg_rpm_min_limit_norm;

    // Cache for knob normalization, calculated on config load
    fix16_t knob_norm_coeff = F16(1);
    // knob value normalized to range (cfg_rpm_min_limit..cfg_rpm_max_limit)
    fix16_t knob_normalized;

    // Motor speed/volts characteristics is not linear.
    // This compensation table is filled in calibration step.
    fix16_t cfg_rpm_volts_correction_table[CFG_RPM_INTERP_TABLE_LENGTH + 2];

    int correction_table_len = 0;

    fix16_t adrc_correction;
    fix16_t adrc_speed_estimated;

    fix16_t adrc_L1;
    fix16_t adrc_L2;

    fix16_t regulator_speed_out = 0;

    uint32_t tick_freq_divide_counter = 0;

    // Apply min/max limits to knob output
    fix16_t normalize_knob(fix16_t knob)
    {
        if (knob < cfg_dead_zone_width_norm) return 0;

        return fix16_mul(
            (knob - cfg_dead_zone_width_norm),
            knob_norm_coeff
        ) + cfg_rpm_min_limit_norm;
    }

    // Motor speed/volts characteristics is not linear. But PID expects "linear"
    // reaction for best result. Apply compensation to "linear" input, using
    // interpolation data from calibration step.
    fix16_t apply_rpm_voltage_compensation(fix16_t linear_speed)
    {
        // Bounds check
        if (linear_speed <= 0) return 0;
        if (linear_speed >= fix16_one) return fix16_one;

        int range_idx = (linear_speed * (correction_table_len - 1)) >> 16;
        fix16_t scale = (linear_speed * (correction_table_len - 1)) & 0x0000FFFF;

        fix16_t range_start = cfg_rpm_volts_correction_table[range_idx - 1];
        fix16_t range_end = cfg_rpm_volts_correction_table[range_idx];

        return fix16_mul(range_start, fix16_one - scale) +
            fix16_mul(range_end, scale);
    }

    fix16_t speed_adrc_tick(fix16_t speed)
    {
        // 1-st order ADRC by https://arxiv.org/pdf/1908.04596.pdf

        // u0 - output of linear proportional controller
        fix16_t u0 = fix16_mul((knob_normalized - adrc_speed_estimated), cfg_adrc_Kp);
        // 2 state observers:
        //   - speed observer (adrc_speed_estimated)
        //   - generalized disturbance observer (adrc_correction)
        adrc_correction += fix16_mul(fix16_mul((speed - adrc_speed_estimated), adrc_L2), integr_coeff);
        adrc_speed_estimated += fix16_mul(u0 + fix16_mul(adrc_L1, (speed - adrc_speed_estimated)),
         integr_coeff);

        // b0 = K/T, where K=1 due to speed and triac setpoint normalization,
        // T - motor time constant, estimated by calibration (see calibrator_adrc.h)
        // cfg_adrc_b0_inv = 1/b0
        fix16_t output = fix16_mul((u0 - adrc_correction), cfg_adrc_b0_inv);

        // Anti-Windup
        // 0 <= output <= 1
        //
        // output = (u0 - adrc_correction)/b0,
        // so if output = 0 -> adrc_correction = u0
        if (output < 0)
        {
            output = 0;
            adrc_correction = u0;
        }

        // output = (u0 - adrc_correction)/b0,
        // so if output = 1 -> adrc_correction = u0 - b0 * 1
        if (output > F16(1.0))
        {
            output = F16(1.0);
            adrc_correction = u0 - fix16_div(fix16_one, cfg_adrc_b0_inv);
        }

        output = fix16_clamp(
            output,
            cfg_rpm_min_limit_norm,
            cfg_rpm_max_limit_norm
        );

        return output;
    }
};


#endif
