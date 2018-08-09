#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_module.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <math.h>
#include <poll.h>
#include <termios.h>
#include <time.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/mavlink_log.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/master_slave_control.h>

#include "interaircraft_communication.h"
#include "interaircraft_receiver.h"

bool InteraircraftCommuication::_task_should_exit = false;
bool InteraircraftCommuication::_task_running = false;

extern "C" __EXPORT int interaircraft_communication_main(int argc, char *argv[]);

InteraircraftCommuication::InteraircraftCommuication() :
    _receive_thread{},
    _baudrate(57600),
    _uart_fd (-1),
    _is_usb_uart(false),
    _device_name ("/dev/ttyS6")
{
    param_get(param_find("FORMATION_NUM"),&_formation_num);
}

InteraircraftCommuication::~InteraircraftCommuication()
{}

int InteraircraftCommuication::open_uart(int baud, const char *uart_name)
{
#ifndef B460800
#define B460800 460800
#endif

#ifndef B500000
#define B500000 500000
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
#endif

        /* process baud rate */
        int speed;

        switch (baud) {
        case 0:      speed = B0;      break;

        case 50:     speed = B50;     break;

        case 75:     speed = B75;     break;

        case 110:    speed = B110;    break;

        case 134:    speed = B134;    break;

        case 150:    speed = B150;    break;

        case 200:    speed = B200;    break;

        case 300:    speed = B300;    break;

        case 600:    speed = B600;    break;

        case 1200:   speed = B1200;   break;

        case 1800:   speed = B1800;   break;

        case 2400:   speed = B2400;   break;

        case 4800:   speed = B4800;   break;

        case 9600:   speed = B9600;   break;

        case 19200:  speed = B19200;  break;

        case 38400:  speed = B38400;  break;

        case 57600:  speed = B57600;  break;

        case 115200: speed = B115200; break;

        case 230400: speed = B230400; break;

        case 460800: speed = B460800; break;

        case 500000: speed = B500000; break;

        case 921600: speed = B921600; break;

        case 1000000: speed = B1000000; break;

#ifdef B1500000

        case 1500000: speed = B1500000; break;
#endif

#ifdef B3000000

        case 3000000: speed = B3000000; break;
#endif

        default:
                PX4_ERR("Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600\t\n115200\n230400\n460800\n500000\n921600\n1000000\n",
                        baud);
                return -EINVAL;
        }

        /* open uart */
        _uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY);

        if (_uart_fd < 0) {
                return _uart_fd;
        }

        /* Try to set baud rate */
        struct termios uart_config;
        int termios_state;
        _is_usb_uart = false;

        /* Initialize the uart config */
        if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0) {
                PX4_ERR("ERR GET CONF %s: %d\n", uart_name, termios_state);
                ::close(_uart_fd);
                return -1;
        }

        /* Clear ONLCR flag (which appends a CR for every LF) */
        uart_config.c_oflag &= ~ONLCR;

        /* USB serial is indicated by /dev/ttyACM0*/
        if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

                /* Set baud rate */
                if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
                        PX4_ERR("ERR SET BAUD %s: %d\n", uart_name, termios_state);
                        ::close(_uart_fd);
                        return -1;
                }

        } else {
                _is_usb_uart = true;
                /* USB has no baudrate, but use a magic number for 'fast' */
                //_baudrate = 2000000;
                //_rstatus.type = telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_USB;
        }

#if defined(__PX4_LINUX) || defined(__PX4_DARWIN) || defined(__PX4_CYGWIN)
        /* Put in raw mode */
        cfmakeraw(&uart_config);
#endif

        if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
                PX4_WARN("ERR SET CONF %s\n", uart_name);
                ::close(_uart_fd);
                return -1;
        }

        /*
         * Setup hardware flow control. If the port has no RTS pin this call will fail,
         * which is not an issue, but requires a separate call so we can fail silently.
        */

        /* setup output flow control */
        //if (enable_flow_control(force_flow_control ? FLOW_CONTROL_ON : FLOW_CONTROL_AUTO)) {
        //        PX4_WARN("hardware flow control not supported");
        //}

        return _uart_fd;
}

void InteraircraftCommuication::send_bytes(const char *buf, unsigned packet_len)
{
        size_t ret = -1;

        /* send message to UART */
        ret = ::write(_uart_fd, buf, packet_len);

        if (ret != (size_t) packet_len) {
                //count_txerr();
                //count_txerrbytes(packet_len);
                warnx("send data fault!");

        } else {
                //_last_write_success_time = _last_write_try_time;
                //count_txbytes(packet_len);
        }
}

int InteraircraftCommuication::start(int argc, char *argv[])
{
    InteraircraftCommuication *instance = new InteraircraftCommuication();

    int res;

    if(!instance){
        res = -ENOMEM;
        PX4_ERR("instance InteraircraftCommuication err!!!");
    }
    else
    {
        res = instance->run(argc, argv);
        instance->_task_running = false;
        delete instance;
    }
    return res;
}

int InteraircraftCommuication::run(int argc, char *argv[])
{
    char msg_head[6] = {24,37,53,44,17,112};
    char msg_tail[6] = {112,17,44,53,37,24};

    _pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
    _global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    //orb_set_interval(_pos_sp_triplet_sub, 100);
    /* wakeup source(s) */
    //px4_pollfd_struct_t fds[1];

    /* Setup of loop */
    //fds[0].fd = _pos_sp_triplet_sub;
    //fds[0].events = POLLIN;

    warnx("1");
    _uart_fd = open_uart(_baudrate, _device_name);
    warnx("2");
    if (_uart_fd < 0 ) {
            PX4_ERR("could not open %s", _device_name);
            return PX4_ERROR;
    }

  //  warnx("3");
    if(_formation_num>0)
        InteraircraftReceiver::receive_start(&_receive_thread,this);
    _task_running = true;
    while(!_task_should_exit){
#if 0
        /* wait for up to 50ms for data */
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

        /* timed out - periodic check for _task_should_exit, etc. */
        if (pret == 0) {
                continue;
        }

        /* this is undesirable but not much we can do - might want to flag unhappy status */
        if (pret < 0) {
                PX4_WARN("poll error %d, %d", pret, errno);
                continue;
        }
#endif
        /* check if there is a new setpoint */
        bool updated = false;
        orb_check(_pos_sp_triplet_sub, &updated);
        if (updated) {

            if(_formation_num == 0){
                orb_copy(ORB_ID(position_setpoint_triplet),_pos_sp_triplet_sub, &_send_msg.send_triplet);
                orb_copy(ORB_ID(vehicle_global_position),_global_pos_sub, &_global_pos);
                _send_msg.lat = _global_pos.lat;
                _send_msg.lon = _global_pos.lon;
            /*    printf("The lon of current\n");
                _send_msg.send_triplet.previous.lon = 1.1;
                _send_msg.send_triplet.previous.lat = 2.2;
                _send_msg.send_triplet.current.lon = 3.3;
                _send_msg.send_triplet.current.lat = 4.4;
                _send_msg.send_triplet.next.lon = 5.5;
                _send_msg.send_triplet.next.lat = 6.6;
                */

             /*   printf("The lon of previous is %f\n",_send_msg.send_triplet.previous.lon);
                printf("The lat of previous is %f\n",_send_msg.send_triplet.previous.lat);
                printf("The lon of current is %f\n",_send_msg.send_triplet.current.lon);
                printf("The lat of current is %f\n",_send_msg.send_triplet.current.lat);
                printf("The lon of next is %f\n",_send_msg.send_triplet.next.lon);
                printf("The lat of next is %f\n",_send_msg.send_triplet.next.lat);
*/

               send_bytes((const char *)msg_head,(unsigned int)sizeof(msg_head));
               send_bytes((const char *)&_send_msg,(unsigned int)sizeof(_send_msg));

               send_bytes((const char *)msg_tail,(unsigned int)sizeof(msg_tail));
            }
        }
        usleep(100000);


    }
    /* first wait for threads to complete before tearing down anything */
    pthread_join(_receive_thread, nullptr);
    warnx("interaircraft_communication stoped!!");
    if (_uart_fd >= 0 && !_is_usb_uart) {
           /* close UART */
           ::close(_uart_fd);
    }
    return OK;
}



int interaircraft_communication_main(int argc, char *argv[]){

    if (argc < 2) {
            printf("interaircraft_communication start/stop/status");
            return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (InteraircraftCommuication::_task_running) {
            warnx("already running\n");
            return 0;
        }
        InteraircraftCommuication::_task_should_exit = false; //初始化这个进程
        px4_task_spawn_cmd("interaircraft_communication",
                           SCHED_DEFAULT,
                           SCHED_PRIORITY_DEFAULT,
                           2500,
                           (px4_main_t)&InteraircraftCommuication::start,
                           (char *const *)argv);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        InteraircraftCommuication::_task_should_exit = true;
        return 0;
    }
    if (!strcmp(argv[1], "status")) {
        if (InteraircraftCommuication::_task_running) {
            warnx("running");
        } else {
            warnx("stopped");
            }
        return 0;
    }
    return 0;
}

