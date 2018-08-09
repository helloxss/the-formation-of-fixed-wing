#include <px4_config.h>
#include <px4_time.h>
#include <px4_tasks.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_range_finder.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_tone_alarm.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#ifndef __PX4_POSIX
#include <termios.h>
#endif
#include <errno.h>
#include <stdlib.h>
#include <poll.h>

#include <mathlib/mathlib.h>

#include <conversion/rotation.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <systemlib/airspeed.h>
#include <commander/px4_custom_mode.h>
#include <lib/ecl/geo/geo.h>

#include <uORB/uORB.h>
#include "interaircraft_receiver.h"




InteraircraftReceiver::InteraircraftReceiver(InteraircraftCommuication * parent) :
        _InteraircraftCommuication(parent)
{}

InteraircraftReceiver::~InteraircraftReceiver()
{
}

/**
 * Receive data from UART.
 */
void *
InteraircraftReceiver::receive_thread(void *arg)
{

        /* set thread name */
        {
                char thread_name[24];
                sprintf(thread_name, "InteraircraftReceiver");
                px4_prctl(PR_SET_NAME, thread_name, px4_getpid());
        }

       orb_advert_t  _communication_pub = orb_advertise(ORB_ID(master_slave_control), &_receive_msg);

        // poll timeout in ms. Also defines the max update frequency of the mission & param manager, etc.
        const int timeout = 10;


        /* the serial port buffers internally as well, we just need to fit a small chunk */
        uint8_t buf[64]={0};

        struct pollfd fds[1] = {};

        fds[0].fd = _InteraircraftCommuication->get_uart_fd();
        fds[0].events = POLLIN;

        ssize_t nread = 0;
        //hrt_abstime last_send_update = 0;
        int receivenum = sizeof(_receive_msg);
        //int offset = 0;
        while (!_InteraircraftCommuication->_task_should_exit) {
            if (poll(&fds[0], 1, timeout) > 0) {

                /*
                 * to avoid reading very small chunks wait for data before reading
                 * this is designed to target one message, so >20 bytes at a time
                 */
                const unsigned character_count = 20;

                /* non-blocking read. read may return negative values */
                if ((nread = ::read(fds[0].fd, buf, sizeof(buf))) < (ssize_t)character_count) {
                        unsigned sleeptime = (1.0f / (_InteraircraftCommuication->get_baudrate() / 10)) * character_count * 1000000;
                        usleep(sleeptime);
                }
                for(ssize_t i=0 ;i<nread;i++){
                        myhandel_msg(buf[i] , receivenum,_communication_pub);
                }
            }
        }
        return nullptr;
}

void InteraircraftReceiver::myhandel_msg(char bufi , int size,orb_advert_t _communication_pub)
{
    static bool head_true = false;
    //static bool tail_true = false;
    static char head[6] = {0};
    static char tail[6] = {0};
    static int offset = 0;
    if(false == head_true){
        head[0] = head[1];
        head[1] = head[2];
        head[2] = head[3];
        head[3] = head[4];
        head[4] = head[5];
        head[5] = bufi;
        if(head[0] == 24 && head[1] == 37 && head[2] == 53 && head[3] == 44 && head[4] == 17 && head[5] == 112 ){
            head_true =true;
            memset(head,0,sizeof(head));
            return;
        }else{
            return;
        }
    }
    *((char*)&_receive_msg + offset) =  bufi;
    ++offset;

    if(offset > size  && offset < size+7)
    {
            tail[0] = tail[1];
            tail[1] = tail[2];
            tail[2] = tail[3];
            tail[3] = tail[4];
            tail[4] = tail[5];
            tail[5] = bufi;
            if(offset==size+6 && tail[0] == 112 && tail[1] == 17 && tail[2] == 44 && tail[3] == 53 && tail[4] == 37 && tail[5] == 24 ){
                offset = 0;
                head_true = false;
                orb_publish(ORB_ID(master_slave_control), _communication_pub, &_receive_msg);
                memset(tail,0,sizeof(tail));
            /*    printf("receive package\n");
                printf("start The lon of privious is %f\n",_receive_msg.send_triplet.previous.lon);
                printf("start The lat of privious is %f\n",_receive_msg.send_triplet.previous.lat);
                printf("start The lon of current is %f\n",_receive_msg.send_triplet.current.lon);
                printf("start The lat of current is %f\n",_receive_msg.send_triplet.current.lat);
                printf("start The lon of next is %f\n",_receive_msg.send_triplet.next.lon);
                printf("start The lat of next is %f\n",_receive_msg.send_triplet.next.lat);
                */

            }else if(offset==size+6){
                offset = 0;
                head_true = false;
            }else{
                return;
            }
    }

}

void *
InteraircraftReceiver::start_helper(void *context)
{

        InteraircraftReceiver *rcv = new InteraircraftReceiver((InteraircraftCommuication *)context);

        if (!rcv) {
                PX4_ERR("alloc failed");
                return nullptr;
        }

        void *ret = rcv->receive_thread(nullptr);

        delete rcv;

        return ret;
}

void InteraircraftReceiver::receive_start(pthread_t *thread, InteraircraftCommuication *parent)
{
    pthread_attr_t receiveloop_attr;
    pthread_attr_init(&receiveloop_attr);

    struct sched_param param;
    (void)pthread_attr_getschedparam(&receiveloop_attr, &param);
    param.sched_priority = SCHED_PRIORITY_MAX - 80;
    (void)pthread_attr_setschedparam(&receiveloop_attr, &param);

    pthread_attr_setstacksize(&receiveloop_attr, PX4_STACK_ADJUSTED(2840));
    pthread_create(thread, &receiveloop_attr, InteraircraftReceiver::start_helper, (void *)parent);

    pthread_attr_destroy(&receiveloop_attr);
}

