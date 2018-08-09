#pragma once

#include <pthread.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/master_slave_control.h>
#include <uORB/topics/position_setpoint_triplet.h>

//struct send_msg_struct{
//position_setpoint_triplet_s send_triplet;
//};

class InteraircraftCommuication
{

public:
        /**
         * Constructor
         */
        InteraircraftCommuication();

        /**
         * Destructor, also kills the mavlinks task.
         */
        ~InteraircraftCommuication();

        /**
        * Start the mavlink task.
         *
         * @return		OK on success.
         */
        static int start(int argc, char *argv[]);

        int open_uart(int baud, const char *uart_name);

        void send_bytes(const char *buf, unsigned packet_len);

        int get_uart_fd()
        {
            return _uart_fd;
        }
        int get_baudrate()
        {
            return _baudrate;
        }

        static bool _task_should_exit;	/**< if true, mavlink task should exit */
        static bool _task_running;
private:
        int run(int argc, char *argv[]);

        pthread_t	_receive_thread;

        int _baudrate;
        int _uart_fd;

        bool _is_usb_uart;
        const char* _device_name;
        int _formation_num;

        vehicle_global_position_s	_global_pos {};

        master_slave_control_s          _send_msg{};

        int _global_pos_sub{-1};
        int _target_pos_sub{-1};
        int _pos_sp_triplet_sub{-1};




        //int _alt_sub;
        //int _yaw_sub;
        //int _speed_sub;

};



