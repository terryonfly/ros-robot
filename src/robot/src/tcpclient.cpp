//
// Created by Terry on 15/12/12.
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h> /* netdb is necessary for struct hostent */
#include <pthread.h>

#include "tcpclient.h"

tcpclient(void) {
    int ret;
    thread_running = 1;
    ret = pthread_create(&thread_id, NULL, (void *)tcpclient_run, NULL);
    if (ret != 0) {
        perror("Create pthread error!\n");
        return -1;
    }
    return 0;
}

~tcpclient(void) {
    printf("Release pthread\n");
    thread_running = 0;
    pthread_join(thread_id, NULL);
}

void tcpclient::tcpclient_run(void) {
    while (thread_running) {
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
}

void tcpclient::tcpclient_data_decode(unsigned char *buf, size_t len) {
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

void tcpclient::tcpclient_content_decode(unsigned char *buf, size_t len)
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
}

int tcpclient::tcpclient_send(unsigned char *buf, size_t len)
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