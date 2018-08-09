#include <px4_config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>

#include <uORB/topics/vehicle_attitude_setpoint.h>

__EXPORT int pos_mavlink_send_main(int argc, char *argv[]);

int pos_mavlink_send_main(int argc, char *argv[])
{
        printf("Hello Debug!\n");

        /* advertise named debug value */
        struct  debug_key_value_s  dbg_key = { .key = "roll_body", .value = 0.0f };
        orb_advert_t pub_dbg_key = orb_advertise(ORB_ID(debug_key_value), &dbg_key);

        int _vehicle_attitude_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
        //orb_set_interval(_vehicle_attitude_sp_sub, 200);
        warnx("1");
        struct vehicle_attitude_setpoint_s _att_sp;
        warnx("2");

        int value_counter = 0;
#if 1
        bool updated = true;
        while (value_counter < 100) {
            warnx("3");

            orb_check(_vehicle_attitude_sp_sub, &updated);
            if (updated) {
                warnx("4");
                orb_copy(ORB_ID(vehicle_attitude_setpoint), _vehicle_attitude_sp_sub, &_att_sp);

                uint64_t timestamp_us = hrt_absolute_time();
                uint32_t timestamp_ms = timestamp_us / 1000;

                /* send one named value */
                dbg_key.value = _att_sp.roll_body ;
                dbg_key.timestamp_ms = timestamp_ms ;
                orb_publish(ORB_ID(debug_key_value), pub_dbg_key, &dbg_key);

                warnx("sent one more value..");

            }
            value_counter++;
            usleep(500000);
        }
#endif
        return 0;
}
