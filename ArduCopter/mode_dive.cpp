#include "Copter.h"

#if MODE_DIVE_ENABLED

bool ModeDive::allows_arming(AP_Arming::Method method) const
{
    return true;
}

// ---------------------------------------------------------------
// init() — called once when entering DIVE mode
// ---------------------------------------------------------------
bool ModeDive::init(bool ignore_checks)
{
    _state = DiveState::CLIMB_TO_ALT;

    start_pos_valid =
        AP::ahrs().get_relative_position_NED_origin(
            dive_start_pos_ned_m);

    if (start_pos_valid) {

        // Save target position
        dive_target_pos_ned_m = dive_start_pos_ned_m;

        // Move target north by DIVE_DIST meters
        dive_target_pos_ned_m.x += g.dive_target_distance_m_x;
        dive_target_pos_ned_m.y += g.dive_target_distance_m_y;
        gcs().send_text(
            MAV_SEVERITY_INFO,
            "DIVE START SAVED");
    }

    return true;
}

// ---------------------------------------------------------------
// run() — called continuously while in DIVE mode
// ---------------------------------------------------------------
void ModeDive::run()
{
    static uint32_t last_msg_ms = 0;

    if ((AP_HAL::millis() - last_msg_ms) < 500) {
        return;
    }

    last_msg_ms = AP_HAL::millis();

    // -----------------------------------------------------------
    // STEP 1: Get current NED position
    // -----------------------------------------------------------
    Vector3p pos_ned_m;

    if (!start_pos_valid ||
        !AP::ahrs().get_relative_position_NED_origin(pos_ned_m)) {
        return;
    }

    // -----------------------------------------------------------
    // STEP 2: Get current yaw
    // -----------------------------------------------------------
    const float current_yaw_deg =
        degrees(AP::ahrs().get_yaw());

    // -----------------------------------------------------------
    // STEP 3: Calculate target vector
    // -----------------------------------------------------------
    const double target_dx =
        dive_target_pos_ned_m.x - pos_ned_m.x;

    const double target_dy =
        dive_target_pos_ned_m.y - pos_ned_m.y;

    // -----------------------------------------------------------
    // STEP 4: Distance to target
    // -----------------------------------------------------------
    const double target_dist_m =
        sqrt(target_dx * target_dx +
             target_dy * target_dy);

    // -----------------------------------------------------------
    // STEP 5: Desired heading
    // -----------------------------------------------------------
    const double heading_rad =
        atan2(target_dy, target_dx);

    const double heading_deg =
        degrees(heading_rad);

    // -----------------------------------------------------------
    // STEP 6: Heading error
    // -----------------------------------------------------------
    float yaw_error_deg =
        wrap_180(heading_deg - current_yaw_deg);

    // -----------------------------------------------------------
    // STEP 7: P Controller
    // -----------------------------------------------------------
    float yaw_rate_cmd =
        -radians(yaw_error_deg * 0.5f);

    // -----------------------------------------------------------
    // STEP 8: Limit yaw rate
    // -----------------------------------------------------------
    yaw_rate_cmd =
        constrain_float(
            yaw_rate_cmd,
            -0.5f,
             0.5f);

    // -----------------------------------------------------------
    // STEP 9: Send yaw-rate command
    // -----------------------------------------------------------
    attitude_control->input_rate_bf_roll_pitch_yaw_rads(
        0.0f,
        0.0f,
        yaw_rate_cmd);

    // -----------------------------------------------------------
    // STEP 10: Debug output
    // -----------------------------------------------------------
    gcs().send_text(
        MAV_SEVERITY_INFO,
        "DIST=%.1f HDG=%.1f YAW=%.1f ERR=%.1f CMD=%.2f",
        (double)target_dist_m,
        (double)heading_deg,
        (double)current_yaw_deg,
        (double)yaw_error_deg,
        (double)yaw_rate_cmd);
}

#endif // MODE_DIVE_ENABLED