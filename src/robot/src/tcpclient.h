//
// Created by Terry on 15/12/12.
//

#ifndef IOTROBOTCTRL_TCPCLIENT_H
#define IOTROBOTCTRL_TCPCLIENT_H

#define PORT 7777 /* server port */
#define MAXDATASIZE 1024

#define CMD_DATA_CONTENT 0x80
#define CMD_DATA_HEADER 0x81
#define CMD_DATA_FOOTER 0x82
#define MAX_REV_CONTENT_LEN 1024

#define MAX_SEND_DATA_LEN 1024

class tcpclient {
    int sock_fd = -1;

    pthread_t thread_id;
    int thread_running;

    unsigned char rev_content[MAX_REV_CONTENT_LEN];
    int rev_content_index;
    int rev_is_cmd = 1;
    unsigned char rev_current_cmd;

    unsigned char send_data[MAX_SEND_DATA_LEN];

    float rotate_a = 1.0;
    float rotate_x = 0.0;
    float rotate_y = 0.0;
    float rotate_z = 0.0;

    float accel_x = 0.0;
    float accel_y = 1.0;
    float accel_z = 0.0;

    float magnet_x = 0.0;
    float magnet_y = 0.0;
    float magnet_z = 1.0;

    float left_angle = 0.0;
    float right_angle = 0.0;
    float left_power = 0.0;
    float right_power = 0.0;

    int tcpclient_init(void);

    void tcpclient_release(void);

    void tcpclient_run(void);

    void tcpclient_data_decode(unsigned char *buf, size_t len);

    void tcpclient_content_decode(unsigned char *buf, size_t len);

    int tcpclient_send(unsigned char *buf, size_t len);
};

#endif //IOTROBOTCTRL_TCPCLIENT_H
