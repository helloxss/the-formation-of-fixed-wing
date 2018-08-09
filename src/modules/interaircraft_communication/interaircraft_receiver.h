#pragma once

#include "interaircraft_communication.h"
#include <pthread.h>
#include <uORB/topics/master_slave_control.h>



class InteraircraftReceiver
{
public:
        /**
         * Constructor
         */
        InteraircraftReceiver(InteraircraftCommuication *parent);

        /**
         * Destructor, also kills the mavlinks task.
         */
        ~InteraircraftReceiver();

        /**
         * Display the mavlink status.
         */
        void	print_status();

        /**
         * Start the receiver thread
         */
        static void receive_start(pthread_t *thread, InteraircraftCommuication *parent);
        static void *start_helper(void *context);

        void *receive_thread(void *arg);

        void myhandel_msg(char bufi, int size,orb_advert_t _communication_pub);

private:
        InteraircraftCommuication * _InteraircraftCommuication;
        master_slave_control_s     _receive_msg{};
     //   orb_advert_t              _communication_pub{nullptr};
};
























