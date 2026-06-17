// #include "Copter.h"

// #if MODE_DIVE_ENABLED

// bool ModeDive::allows_arming(AP_Arming::Method method)const
// {
//     return true;
// }
// // ---------------------------------------------------------------
// // init() — called once when entering DIVE mode
// // ---------------------------------------------------------------
// bool ModeDive::init(bool ignore_checks)
// {
//     // Must be armed
//     // if (!motors->armed()) {
//     //     gcs().send_text(MAV_SEVERITY_WARNING,
//     //                     "DIVE: Vehicle1212 not armed");
//     //     return false;
//     // }

//     _state = DiveState::CLIMB_TO_ALT;

//     start_pos_valid =
//         AP::ahrs().get_relative_position_NED_origin(
//                               dive_start_pos_ned_m);

//     if (start_pos_valid) {
//         dive_target_pos_ned_m = dive_start_pos_ned_m;
//         dive_target_pos_ned_m.x += 20.0;

//         gcs().send_text(MAV_SEVERITY_INFO,
//                     "DIVE START SAVED ");
//     }



//     return true;
// }

// // ---------------------------------------------------------------
// // run() — called continuously while in DIVE mode
// // ---------------------------------------------------------------
// void ModeDive::run()
// {
//     static uint32_t last_msg_ms = 0;

//     if ((AP_HAL::millis() - last_msg_ms) > 500) {

//         last_msg_ms = AP_HAL::millis();

//         gcs().send_text(
//             MAV_SEVERITY_INFO,
//             "DIVE MODE ACTIVE");

//         Vector3p pos_ned_m;

//         if (start_pos_valid &&
//             AP::ahrs().get_relative_position_NED_origin(pos_ned_m)) {

//             double dx =
//                 pos_ned_m.x - dive_start_pos_ned_m.x;

//             double dy =
//                 pos_ned_m.y - dive_start_pos_ned_m.y;

//             double dist_from_start_m =
//                 sqrt(dx * dx + dy * dy);

//             double target_dx =
//                 dive_target_pos_ned_m.x - pos_ned_m.x;

//             double target_dy =
//                 dive_target_pos_ned_m.y - pos_ned_m.y;

//                  sqrt(target_dx * target_dx +
//                      target_dy * target_dy);

//             // gcs().send_text(
//             //     MAV_SEVERITY_INFO,
//             //     "CUR N=%.1f E=%.1f",
//             //     (double)pos_ned_m.x,
//             //     (double)pos_ned_m.y);

//             // gcs().send_text(
//             //     MAV_SEVERITY_INFO,
//             //     "TARGET N=%.1f E=%.1f",
//             //     (double)dive_target_pos_ned_m.x,
//             //     (double)dive_target_pos_ned_m.y);

//             gcs().send_text(
//                 MAV_SEVERITY_INFO,
//                 "FROM_START=%.1f m",
//                 dist_from_start_m);

//             gcs().send_text(
//                 MAV_SEVERITY_INFO,
//                 "TO_TARGET=%.1f m",
//                 dist_to_target_m);

//             if (dist_to_target_m < 1.0) {
//                 gcs().send_text(
//                     MAV_SEVERITY_INFO,
//                     "TARGET REACHED");
//             }
//         }
//     }
// }

// #endif // MODE_DIVE_ENABLED
























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
        AP::ahrs().get_relative_position_NED_origin(dive_start_pos_ned_m);
    if (start_pos_valid){
        dive_target_pos_ned_m = dive_start_pos_ned_m;
        dive_target_pos_ned_m.x += g.dive_target_distance_m;
        // dive_target_pos_ned_m.y += 20.0;
        
        gcs().send_text(MAV_SEVERITY_INFO,
                            "DIVE STRAT SAVED ");
    }    



    return true;
}

// ---------------------------------------------------------------
// run() — called continuously while in DIVE mode
// ---------------------------------------------------------------
void ModeDive::run()
{
    static uint32_t last_msg_ms = 0;

    if ((AP_HAL::millis() - last_msg_ms) > 500) {

        last_msg_ms = AP_HAL::millis();

        gcs().send_text(
            MAV_SEVERITY_INFO,
            "DIVE MODE ACTIVE");
        float current_yaw_deg = 
                degrees(AP::ahrs().get_yaw());

        Vector3p pos_ned_m;
         if (start_pos_valid &&
            AP::ahrs().get_relative_position_NED_origin(pos_ned_m)){
            double dx =
                pos_ned_m.x - dive_start_pos_ned_m.x;

            double dy =
                pos_ned_m.y - dive_start_pos_ned_m.y;

            double dist_from_start_m =
                sqrt(dx*dx + dy*dy);

            double target_dx =
                dive_target_pos_ned_m.x - pos_ned_m.x;

            double target_dy =
                dive_target_pos_ned_m.y - pos_ned_m.y;

            double target_dist_m =
                sqrt(target_dx*target_dx +
                    target_dy*target_dy);

            double heading_rad =
                atan2(target_dy, target_dx);

            double heading_deg =
                degrees(heading_rad);
            
            float yaw_error_deg =
                heading_deg - current_yaw_deg;
            

            gcs().send_text(
                MAV_SEVERITY_INFO,
                "DT_start=%.1f CUR N=%.1f E=%.1f TARGET N=%.1f E=%.1f target_dist=%.1f HDG=%.1f YAW=%.1f ERR=%.1f",
                (double)dist_from_start_m,
                (double)pos_ned_m.x,
                (double)pos_ned_m.y,
                (double)dive_target_pos_ned_m.x,
                (double)dive_target_pos_ned_m.y,
                (double)target_dist_m,
                (double)heading_deg,
                (double)current_yaw_deg,
                (double)yaw_error_deg);
            }
   

    }
}

#endif // MODE_DIVE_ENABLED