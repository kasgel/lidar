/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/** 
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the low-level I2C peripheral driver
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Kevin Weiss <kevin.weiss@haw-hamburg.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <math.h>
#include <inttypes.h>
#include <xtimer.h>

#include "periph_conf.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "shell.h"
#include "main.h"
#include "fmt.h"
#include "msg.h"
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netif/hdr.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/pktdump.h"
#include "net/gnrc/pkt.h"
#include "net/gnrc/pktbuf.h"
#include "timex.h"
#include "utlist.h"
#include "od.h"


#ifndef I2C_ACK
#define I2C_ACK         (0)
#endif

#define BUFSIZE         (128U)

#define MEASUREMENT_FREQ (100) // 100 Hz
#define MEASUREMENT_SLEEP (1000000*0.97/MEASUREMENT_FREQ) // Sleep interval in microseconds (modified to account for code execution)
#define UPDATE_FREQ     (100)

// For sending UDP messages
#define SERVER_MSG_QUEUE_SIZE   (8U)
#define SERVER_PRIO             (THREAD_PRIORITY_MAIN - 1)
#define SERVER_STACKSIZE        (THREAD_STACKSIZE_MAIN)
#define SERVER_RESET            (0x8fae)

// Message types
#define MSG_STATE_ONE       (0x01)
#define MSG_STATE_TWO       (0x02)
#define MSG_STATE_THREE     (0x03)
#define MSG_STATE_FOUR      (0x04)


static char* recipient = "fe80::fec2:3d00:1:7f97";

static gnrc_netreg_entry_t server = GNRC_NETREG_ENTRY_INIT_PID(0, KERNEL_PID_UNDEF);

static char server_stack[SERVER_STACKSIZE];
static msg_t server_queue[SERVER_MSG_QUEUE_SIZE];
static kernel_pid_t server_pid = KERNEL_PID_UNDEF;
//static uint8_t send_count = 0;
static gnrc_pktsnip_t *payload;


/* i2c_buf is global to reduce stack memory consumtion */
static uint8_t i2c_buf[BUFSIZE];

static double alpha;
static uint16_t track_gauge;
static uint16_t distance_from_rail;
static uint16_t theoretical_shortest_d;
static uint16_t theoretical_largest_d;
static int16_t rolling_avg_buff[MEASUREMENT_FREQ];

static void *_eventloop(void *arg)
{
    (void)arg;
    msg_t msg, reply;
    unsigned int rcv_count = 0;

    /* setup the message queue */
    msg_init_queue(server_queue, SERVER_MSG_QUEUE_SIZE);

    reply.content.value = (uint32_t)(-ENOTSUP);
    reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
    
    gnrc_pktsnip_t *pkt = NULL;

    while (1) {
        msg_receive(&msg);

        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV:
                printf("Packets received: %u\n", ++rcv_count);
                pkt = msg.content.ptr;
                pkt = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);
                //printf("Packet size: %d\n", pkt->size);
                od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
                //printf("Content of message: %" PRIu32 "\n", msg.content.value);
                //printf("Content of message char: %c\n", (char*)pkt->data);
                float vel = 0;
                switch (*(uint8_t*)pkt->data)
                {
                    case MSG_STATE_ONE:
                        printf("State 1: No object detected.\n");
                        break;
                    case MSG_STATE_TWO:
                        printf("State 2: Object in range.\n");
                        break;
                    case MSG_STATE_THREE:
                        printf("State 3: Object entering valid region.\n");
                        break;
                    case MSG_STATE_FOUR:
                        memcpy(&vel, (uint8_t*)pkt->data + 1, sizeof(vel));
                        //vel = *((uint8_t*)(pkt->data) + 1);
                        printf("State 4: Object velocity: ");
                        print_float(vel, 2);
                        printf("\n");
                        break;
                    default:
                        printf("Unknown message received: %d", *(uint8_t*)pkt->data);
                        break;
                }
                //print_byte_hex(*(uint8_t*)pkt->data);
                //printf("\n");
                //printf("Old printf statement %u\n", *(uint8_t*)pkt->data);
                gnrc_pktbuf_release(msg.content.ptr);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
            case GNRC_NETAPI_MSG_TYPE_SET:
                msg_reply(&msg, &reply);
                break;
            case SERVER_RESET:
                rcv_count = 0;
                break;
            default:
                break;
        }
    }

    /* never reached */
    return NULL;
}

static void send(char *addr_str, char *port_str, uint8_t *data, unsigned int len, unsigned int num) {
    gnrc_netif_t *netif = NULL;
    char *iface;
    uint16_t port;
    ipv6_addr_t addr;

    iface = ipv6_addr_split_iface(addr_str);
    if ((!iface) && (gnrc_netif_numof() == 1)) {
        netif = gnrc_netif_iter(NULL);
    }
    else if (iface) {
        netif = gnrc_netif_get_by_pid(atoi(iface));
    }

    /* parse destination address */
    if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
        puts("Error: unable to parse destination address");
        return;
    }
    /* parse port */
    port = atoi(port_str);
    if (port == 0) {
        puts("Error: unable to parse destination port");
        return;
    }
    
    // Wait until packet buffer is empty
    /*printf("1: %d\n", gnrc_pktbuf_is_empty());
    while (!gnrc_pktbuf_is_empty()) {
        //printf("sleepytime\n");
        xtimer_usleep(1000);
    }*/

    for (unsigned int i = 0; i < num; i++) {
        gnrc_pktsnip_t *udp, *ip;
        unsigned payload_size;

        /* allocate payload */
        payload = gnrc_pktbuf_add(NULL, data, len, GNRC_NETTYPE_UNDEF);
        //printf("2: %d\n", gnrc_pktbuf_is_empty());
        if (payload == NULL) {
            puts("Error: unable to copy data to packet buffer");
            return;
        }
        /* store size for output */
        payload_size = (unsigned)payload->size;
        /* allocate UDP header, set source port := destination port */
        udp = gnrc_udp_hdr_build(payload, port, port);
        if (udp == NULL) {
            puts("Error: unable to allocate UDP header");
            gnrc_pktbuf_release(payload);
            return;
        }
        /* allocate IPv6 header */
        ip = gnrc_ipv6_hdr_build(udp, NULL, &addr);
        if (ip == NULL) {
            puts("Error: unable to allocate IPv6 header");
            gnrc_pktbuf_release(udp);
            return;
        }
        /* add netif header, if interface was given */
        if (netif != NULL) {
            gnrc_pktsnip_t *netif_hdr = gnrc_netif_hdr_build(NULL, 0, NULL, 0);

            gnrc_netif_hdr_set_netif(netif_hdr->data, netif);
            LL_PREPEND(ip, netif_hdr);
        }
        //printf("3: %d\n", gnrc_pktbuf_is_empty());
        /* send packet */
        if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
            puts("Error: unable to locate UDP thread");
            gnrc_pktbuf_release(ip);
            return;
        }
        /* access to `payload` was implicitly given up with the send operation above
         * => use temporary variable for output */
        printf("Success: sent %u byte(s) to [%s]:%u\n", payload_size, addr_str,
               port);
               //printf("4: %d\n", gnrc_pktbuf_is_empty());
        //xtimer_usleep(20000);
    }
}

static void start_server(char *port_str)
{
    uint16_t port;

    /* check if server is already running */
    if (server.target.pid != KERNEL_PID_UNDEF) {
        printf("Error: server already running on port %" PRIu32 "\n",
               server.demux_ctx);
        return;
    }
    /* parse port */
    port = atoi(port_str);
    if (port == 0) {
        puts("Error: invalid port specified");
        return;
    }
    if (server_pid <= KERNEL_PID_UNDEF) {
        /* start server */
        server_pid = thread_create(server_stack, sizeof(server_stack), SERVER_PRIO,
                                   THREAD_CREATE_STACKTEST, _eventloop, NULL, "UDP server");
        if (server_pid <= KERNEL_PID_UNDEF) {
            puts("Error: can not start server thread");
            return;
        }
    }
    /* register server to receive messages from given port */
    gnrc_netreg_entry_init_pid(&server, port, server_pid);
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);
    printf("Success: started UDP server on port %" PRIu16 "\n", port);
}

// static void stop_server(void)
// {
//     msg_t msg = { .type = SERVER_RESET };
//     /* check if server is running at all */
//     if (server.target.pid == KERNEL_PID_UNDEF) {
//         printf("Error: server was not running\n");
//         return;
//     }
//     /* reset server state */
//     msg_send(&msg, server.target.pid);
//     /* stop server */
//     gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &server);
//     gnrc_netreg_entry_init_pid(&server, 0, KERNEL_PID_UNDEF);
//     puts("Success: stopped UDP server");
// }

static int _print_i2c_error(int res)
{
    if (res == -EOPNOTSUPP) {
        printf("Error: EOPNOTSUPP [%d]\n", -res);
        return 0;
    }
    else if (res == -EINVAL) {
        printf("Error: EINVAL [%d]\n", -res);
        return 0;
    }
    else if (res == -EAGAIN) {
        printf("Error: EAGAIN [%d]\n", -res);
        return 0;
    }
    else if (res == -ENXIO) {
        printf("Error: ENXIO [%d]\n", -res);
        return 0;
    }
    else if (res == -EIO) {
        printf("Error: EIO [%d]\n", -res);
        return 0;
    }
    else if (res == -ETIMEDOUT) {
        printf("Error: ETIMEDOUT [%d]\n", -res);
        return 0;
    }
    else if (res == I2C_ACK) {
        printf("Success: I2C_ACK [%d]\n", res);
        return 0;
    }
    printf("Error: Unknown error [%d]\n", res);
    return 0;
}


uint16_t lidar_distance(void) {
    const int addr = 0x10; // I2C ADDRESS of our lidar sensor
    const int dev = 0; // I2C device

    // We want to send following frame to get a data frame back.
    // 5A 05 00 01 60
    // This will return a data frame (9 bytes) of following format:
    // 59 59 [DIST_L] [DIST_H] [STRENGTH_L] [STRENGTH_H] [TEMP_L] [TEMP_H] [CHECKSUM]
    //uint8_t data[5] = {0x5A, 0x05, 0x00, 0x01, 0x60}; // Data frame for cm output
    uint8_t data[5] = {0x5A, 0x05, 0x00, 0x06, 0xBA}; // Data frame for millimeter (mm) output
    int res = i2c_write_bytes(dev, addr, data, sizeof(data), 0);

    if (res != I2C_ACK) {
        return _print_i2c_error(res);
    }

    // Read data frame which we get back.
    res = i2c_read_bytes(dev, addr, i2c_buf, 9, 0);

    if (res != I2C_ACK) {
        return _print_i2c_error(res);
    }
    uint16_t d = ((i2c_buf[3] << 8) | i2c_buf[2]);
    //uint16_t strength = ((i2c_buf[5] << 8) | i2c_buf[4]);
    //printf("Dist: %d mm\n", d);
    
    // Our distance measurement should now exist in the 3rd and 4th indices of i2c_buf
    return d;
}

int cmd_dist(int argc, char **argv) {
    (void) argv;
    (void) argc;
    
    while (true) {
        printf("Dist: %d mm\n", lidar_distance());
        xtimer_usleep(1000);
    }
    
    return 0;
}

int init_lidar(int argc, char **argv) {
    // This method is not used atm.
    (void)argv;
    (void)argc;
    const int addr = 0x10; // I2C ADDRESS of our lidar sensor
    const int dev = 0; // I2C device

    // Set update rate to 1000 Hz
    uint8_t data[6] = { 0x5A, 0x06, 0x03, 0x00, 0x01, 0x64};
    int res = i2c_write_bytes(dev, addr, data, sizeof(data), 0);
    if (res != I2C_ACK) {
        return _print_i2c_error(res);
    }
    xtimer_usleep(100000);

    res = i2c_read_bytes(dev, addr, i2c_buf, 5, 0);
    if (res != I2C_ACK) {
        return _print_i2c_error(res);
    } else {
        //_print_i2c_read(dev, NULL, i2c_buf, 9);
    }
    printf("OK\n");
    return 0;

    // Set measurement unit to millimeters
    /*uint8_t data[5] = { 0x5A, 0x05, 0x05, 0x06, 0x6A }; // Checksum 0x6A
    int res = i2c_write_bytes(dev, addr, data, sizeof(data), 0);
    if (res != I2C_ACK) {
        return _print_i2c_error(res);
    }
    xtimer_usleep(100000);

    res = i2c_read_bytes(dev, addr, i2c_buf, 5, 0);
    if (res != I2C_ACK) {
        return _print_i2c_error(res);
    } else {
        _print_i2c_read(dev, NULL, i2c_buf, 9);
    }*/

    // Enable output
    //uint8_t data2[5] = { 0x5A, 0x05, 0x07, 0x01, 0x67 }; // Checksum 0x67
    /*uint8_t data2[5] = { 0x5A, 0x05, 0x07, 0x00, 0x66 }; // Checksum 0x67
    res = i2c_write_bytes(dev, addr, data2, sizeof(data2), 0);
    if (res != I2C_ACK) {
        return _print_i2c_error(res);
    }
    xtimer_usleep(100000);

    res = i2c_read_bytes(dev, addr, i2c_buf, 5, 0);
    if (res != I2C_ACK) {
        return _print_i2c_error(res);
    } else {
        _print_i2c_read(dev, NULL, i2c_buf, 5);
    }*/

    printf("OK\n");

    return 0;
}

int print_distance(int argc, char **argv) {
    (void)argv;
    (void)argc;
    uint16_t dist = lidar_distance();

    if (dist > 0) {
        printf("Distance: %i mm\n", dist);
    }

    return 0;
}

void initialize(void) {
    distance_from_rail = 500; // 100 cm / 1000 mm
    //track_gauge = 1435;
    track_gauge = 6000;
    alpha = M_PI/2;
    printf("Cos(alpha): %f\n", cos(alpha));

    if (alpha == M_PI/2) {
        theoretical_largest_d = 1160;
        theoretical_shortest_d = 300;
    } else {
        //theoretical_largest_d = (distance_from_rail + track_gauge) / cos(alpha);
        theoretical_largest_d = 12000;
        theoretical_shortest_d = 3500;
        //theoretical_shortest_d = distance_from_rail / cos(alpha);
    }
}

int cmd_start(int argc, char **argv) {
    (void)argv;
    (void)argc;
    initialize();
    printf("The valid region: %u mm - %u mm\n", theoretical_shortest_d, theoretical_largest_d);
    printf("Measurement freq: %u\n", MEASUREMENT_FREQ);
    printf("Measurement sleep (us): %u\n", MEASUREMENT_SLEEP);
    return come_here_my_train();
}

int cmd_start_server(int argc, char **argv)
{
    (void)argv;
    (void)argc;
    start_server("8888");
    return 0;
}

// State 1 of state diagram: Loop until there is an object within distance bounds
int come_here_my_train(void) {
    printf("State 1\n");
    //xtimer_usleep(600000);
   // uint8_t msg[1] = {MSG_STATE_ONE};
    //send(recipient, "8888", msg, 1, 1);

    while (true) {
         uint16_t dist = lidar_distance();
         if (dist > 0 && dist < theoretical_largest_d) {
            printf("Train arrived at sensor: Distance: %u mm\n", dist);
            return check_valid_region();
         }
         xtimer_usleep(1000);
    }
    return 0;
}

// State 2: Check if train is in valid region
int check_valid_region(void) {
    printf("State 2\n");
    //uint8_t msg[1] = {MSG_STATE_TWO};
    //send(recipient, "8888", msg, 1, 1);
    while (true) {
        uint16_t dist = lidar_distance();
        if (dist > theoretical_shortest_d && dist < theoretical_largest_d) {
            printf("Entered valid region: Distance: %i mm. Moving to state 3.\n", dist);
            return init_diff_buff();
        } else if (dist == 0 || dist > theoretical_largest_d) {
            printf("No object detected. Moving back to state 1.\n");
            return come_here_my_train();
        }
    }
    return 0;
}

// State 3: Initialise difference buffer
int init_diff_buff(void)
{
    printf("State 3\n");
    //uint8_t msg[1] = {MSG_STATE_THREE};
    //send(recipient, "8888", msg, 1, 1);

    uint16_t dist = lidar_distance();
    xtimer_usleep(MEASUREMENT_SLEEP);
    uint16_t dist2 = lidar_distance();

    if (dist == 0 || dist > theoretical_largest_d || dist2 == 0 || dist2 > theoretical_largest_d) {
        printf("No object detected. Moving back to state 1.\n");
        return come_here_my_train();
    } else if ((dist > 0 && dist < theoretical_shortest_d) || (dist2 > 0 && dist2 < theoretical_shortest_d)) {
        printf("Object out of valid region. Moving back to state 2.\n");
        return check_valid_region();
    }

    int16_t difference = dist2 - dist;
    printf("First diff: %d\n", difference);
    for (int i = 0; i < MEASUREMENT_FREQ; i ++)
    {
        rolling_avg_buff[i] = difference;
    }

    printf("State 4\n");
    return cont_velocity(dist2, 1, false);
}

int print_flt_avg(uint16_t buff_index, bool buff_initialised) {
    //printf("Hello\n");

    // compute sum of differences
    int16_t sum = 0;
    if (buff_initialised) {
        buff_index = MEASUREMENT_FREQ;
    }

    for (uint16_t i = 0; i < buff_index; i++) {
        sum += rolling_avg_buff[i];
    }

    if (!buff_initialised)
    {
        sum = sum * (MEASUREMENT_FREQ / buff_index);
        //xtimer_usleep(300000);
    }
    
    
    printf("Sum: %d\n", sum);

    // use sum to compute velocity
    float velocity2 = sum * sin(alpha);  // mm/s
    velocity2 = velocity2 / 277.778;  // Convert from mm/s to km/h
    printf("Velocity in km/h: ");
    print_float(velocity2, 2);
    printf("\n");

    //char* msg = malloc(sizeof(uint8_t) + sizeof(float) + 1);
    uint8_t msg[5] = {MSG_STATE_FOUR, 0xFF, 0xFF, 0xFF, 0xFF};

    memcpy(msg+1, &velocity2, sizeof(velocity2));
    send(recipient, "8888", msg, 5, 1);

    return 0;
}

// State 4: Continuous velocity measurements
int cont_velocity(uint16_t prev_dist, uint16_t buff_index, bool buff_initialised) {
    xtimer_usleep(MEASUREMENT_SLEEP);

    //printf("State 4, i: %i\n", buff_index);
    uint16_t dist = lidar_distance();

    if (dist == 0 || dist > theoretical_largest_d) {
        print_flt_avg(buff_index, buff_initialised);
        printf("Moving from state 4 to state 1. Dist: %d mm\n", dist);
        return come_here_my_train();
    } else if (dist > 0 && dist < theoretical_shortest_d) {
        print_flt_avg(buff_index, buff_initialised);
        printf("Moving from state 4 to state 2. Dist: %d mm\n", dist);
        return check_valid_region();
    }

    int16_t difference = dist - prev_dist;
    rolling_avg_buff[buff_index] = difference;

    //printf("Should equal zero for velocity reading: %d\n", (buff_index + 1) % UPDATE_FREQ);
    if ((buff_index + 1) % UPDATE_FREQ == 0)
    {
        print_flt_avg(MEASUREMENT_FREQ, buff_initialised);
    }
    
    if (buff_index == (MEASUREMENT_FREQ - 1)) {
        return cont_velocity(dist, 0, true);
    } else {
        return cont_velocity(dist, buff_index + 1, buff_initialised);
    }
}


static const shell_command_t shell_commands[] = {
    { "print_distance", "Take a distance measurement from lidar", print_distance },
    { "start", "Start state machine", cmd_start },
    { "dist", "Continuous distance measurements", cmd_dist },
    { "rate", "Update frame rate", init_lidar },
    { "start_server", "Starts the UDP server", cmd_start_server},
    { NULL, NULL, NULL }
};

int main(void)
{
    puts("Start: Lidar testing\n");

    char line_buf[SHELL_DEFAULT_BUFSIZE];

    puts("Attempting to acquire I2C bus\n");
    int acq_req;
    // We use 0 as the parameter to acquire the I2C bus, for use with the Lidar sensor.
    acq_req = i2c_acquire(0);
    if (acq_req != 0)
    {
        puts("Failed to acquire bus. Terminating program.\n");
        return 1;
    }
    puts("I2C bus acquired. Issue command to Lidar.\n");

    // Initialise packet buffer
    gnrc_pktbuf_init();
    
    // Start the server
    //start_server("8888");

    puts("Server started.\n");

    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
