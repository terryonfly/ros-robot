#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>

#include <ros/ros.h>
#include "std_msgs/String.h"

#define PORT 7777 /* server port */
#define MAXDATASIZE 1024

#define CMD_DATA_CONTENT 0x80
#define CMD_DATA_HEADER 0x81
#define CMD_DATA_FOOTER 0x82
#define MAX_REV_CONTENT_LEN 1024

#define MAX_SEND_DATA_LEN 1024

int running = 1;

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

ros::Publisher pid_pub;

void tcpclient_data_decode(unsigned char *buf, size_t len) {
    int i, k;
    for (i = 0; i < len; i ++) {
        if (rev_is_cmd) {
            if (buf[i] == CMD_DATA_CONTENT || buf[i] == CMD_DATA_HEADER || buf[i] == CMD_DATA_FOOTER) {
                rev_current_cmd = buf[i];
                rev_is_cmd = 0;
            }
        } else {
            switch (rev_current_cmd) {
                case CMD_DATA_CONTENT:
                    rev_content[rev_content_index] = buf[i];
                    rev_content_index ++;
                    if (rev_content_index > MAX_REV_CONTENT_LEN) rev_content_index = 0;
                    break;
                case CMD_DATA_HEADER:
                    // buf[i] is header data
                    rev_content_index = 0;
                    break;
                case CMD_DATA_FOOTER:
                    // buf[i] is footer data
//                    for (k = 0; k < rev_content_index; k ++)
//                        printf("%02x ", rev_content[k]);
//                    printf("\n");
                    tcpclient_content_decode(rev_content, rev_content_index);

                    rev_content_index = 0;
                    break;

                default:
                    break;
            }
            rev_is_cmd = 1;
        }
    }
}

void tcpclient_content_decode(unsigned char *buf, size_t len)
{
    unsigned char i;
    unsigned char* px = buf;
    void *pf;

    pf = &rotate_a;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &rotate_x;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &rotate_y;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &rotate_z;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &accel_x;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &accel_y;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &accel_z;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &magnet_x;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &magnet_y;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &magnet_z;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &left_angle;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &right_angle;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &left_power;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &right_power;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    printf("good data = %.2f\n", left_angle);
}

int tcpclient_send(unsigned char *buf, size_t len)
{
    if (sock_fd == -1) return -1;
    if (len > MAX_SEND_DATA_LEN / 2) return -1;
    int set_i = 0;
    int i, send_len;
    // Header
    send_data[set_i] = CMD_DATA_HEADER;
    set_i ++;
    send_data[set_i] = 0x01;
    set_i ++;
    // Content
    for (i = 0; i < len; i ++) {
        send_data[set_i] = CMD_DATA_CONTENT;
        set_i ++;
        send_data[set_i] = buf[i];
        set_i ++;
    }
    // Footer
    send_data[set_i] = CMD_DATA_FOOTER;
    set_i ++;
    send_data[set_i] = 0x82;
    set_i ++;
    if ((send_len = send(sock_fd, send_data, set_i, 0)) == -1) {
        printf("send_len = %d\n", send_len);
        perror("send() failure ");
        return -1;
    }
    return 0;
}

int main(int argc, char *argv[]) {
    printf("== begin ==\n");
    ros::init(argc, argv, "robot");
    ros::NodeHandle n;
    pid_pub = n.advertise<std_msgs::String>("robot_pid", 0.5);
    while (running) {
        int num; /* files descriptors */
        unsigned char buf[MAXDATASIZE]; /* buf will store received text */
        struct hostent *he; /* structure that will get information about remote host */
        struct sockaddr_in server;

        if ((he = gethostbyname("192.168.102.64")) == NULL) {
            perror("gethostbyname() error ");
            return;
        }

        if ((sock_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            perror("socket() error ");
            return;
        }
        bzero(&server, sizeof(server));
        server.sin_family = AF_INET;
        server.sin_port = htons(PORT);
        server.sin_addr = *((struct in_addr *)he->h_addr);
        if (connect(sock_fd, (struct sockaddr *)&server, sizeof(server)) == -1) {
            perror("try connect() error ");
        } else {
            printf("connect() successes\n");
            while (thread_running) {
                if ((num = recv(sock_fd, buf, MAXDATASIZE, 0)) == -1) {
                    perror("recv() error \n");
                }
                if (num == 0) break;
                tcpclient_data_decode(buf, num);
            }
        }
        close(sock_fd);
        sock_fd = -1;
        sleep(1);
    }
    printf("=== end ===\n");
    return 0;
}