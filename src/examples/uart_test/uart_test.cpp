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


int _baudrate=57600;
int _uart_fd = -1;
bool _force_flow_control = false;
bool _is_usb_uart = false;
const char* _device_name = "/dev/ttyS6";

extern "C" __EXPORT int uart_test_main(int argc, char *argv[]);

int open_uart(int baud, const char *uart_name, bool force_flow_control);
void send_bytes(const char *buf, unsigned packet_len);


int open_uart(int baud, const char *uart_name, bool force_flow_control)
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

        /* back off 1800 ms to avoid running into the USB setup timing */
//        while (_mode == MAVLINK_MODE_CONFIG &&
//               hrt_absolute_time() < 1800U * 1000U) {
//                usleep(50000);
//        }

        /* open uart */
        _uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY);
#if 0
        /* if this is a config link, stay here and wait for it to open */
        if (_uart_fd < 0 && _mode == MAVLINK_MODE_CONFIG) {

                int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
                struct actuator_armed_s armed;

                /* get the system arming state and abort on arming */
                while (_uart_fd < 0) {

                        /* abort if an arming topic is published and system is armed */
                        bool updated = false;
                        orb_check(armed_sub, &updated);

                        if (updated) {
                                /* the system is now providing arming status feedback.
                                 * instead of timing out, we resort to abort bringing
                                 * up the terminal.
                                 */
                                orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);

                                if (armed.armed) {
                                        /* this is not an error, but we are done */
                                        orb_unsubscribe(armed_sub);
                                        return -1;
                                }
                        }

                        usleep(100000);
                        _uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY);
                }

                orb_unsubscribe(armed_sub);
        }
#endif
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



void send_bytes(const char *buf, unsigned packet_len)
{
        /* If the wait until transmit flag is on, only transmit after we've received messages.
           Otherwise, transmit all the time. */
        //if (!should_transmit()) {
        //        return;
        //}

        //_last_write_try_time = hrt_absolute_time();

#if 0
        if (get_protocol() == SERIAL) {
                /* check if there is space in the buffer, let it overflow else */
                unsigned buf_free = get_free_tx_buf();

                if (buf_free < packet_len) {
                        /* not enough space in buffer to send */
                        count_txerr();
                        count_txerrbytes(packet_len);
                        return;
                }
        }
#endif
        size_t ret = -1;

        /* send message to UART */
        //if (get_protocol() == SERIAL) {
                ret = ::write(_uart_fd, buf, packet_len);
       //}

        if (ret != (size_t) packet_len) {
                //count_txerr();
                //count_txerrbytes(packet_len);

        } else {
                //_last_write_success_time = _last_write_try_time;
                //count_txbytes(packet_len);
        }

}


int uart_test_main(int argc, char *argv[]){
    char a[12] = {1,2,3,4,5,6,7,8,9,10,11,12};
    //const char *buff = a;
    warnx("1");
    _uart_fd = open_uart(_baudrate, _device_name, _force_flow_control);
    warnx("2");
    if (_uart_fd < 0 ) {
            PX4_ERR("could not open %s", _device_name);
            return PX4_ERROR;
    }
    warnx("3");
    /* initialize send mutex */
    //pthread_mutex_init(&_send_mutex, nullptr);
    warnx("4");
   while(1){
       warnx("5");
       usleep(500000);
       send_bytes(a,12);
   }
}


