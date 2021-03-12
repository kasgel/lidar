/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/** trolololo
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
#include <errno.h>
#include <math.h>

#include "periph_conf.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "shell.h"
#include "main.h"
#include "fmt.h"

#include <xtimer.h>

#ifndef I2C_ACK
#define I2C_ACK         (0)
#endif

#define INVALID_ARGS    puts("Error: Invalid number of arguments");

#define BUFSIZE         (128U)

#define CONVERT_ERROR   (-32768)

#define ARG_ERROR       (-1)

#define MEASUREMENT_FREQ (100) // 100 Hz
#define MEASUREMENT_SLEEP (1000000/MEASUREMENT_FREQ) // Sleep interval in microseconds
#define UPDATE_FREQ     (100)

/* i2c_buf is global to reduce stack memory consumtion */
static uint8_t i2c_buf[BUFSIZE];

static double alpha;
static uint16_t track_gauge;
static uint16_t distance_from_rail;
static uint16_t theoretical_shortest_d;
static uint16_t theoretical_largest_d;
static int16_t rolling_avg_buff[MEASUREMENT_FREQ];

static inline void _print_i2c_read(i2c_t dev, uint16_t *reg, uint8_t *buf,
    int len)
{
    printf("Success: i2c_%i read %i byte(s) ", dev, len);
    if (reg != NULL) {
        printf("from reg 0x%02x ", *reg);
    }
    printf(": [");
    for (int i = 0; i < len; i++) {
        if (i != 0) {
            printf(", ");
        }
        printf("0x%02x", buf[i]);
    }
    printf("]\n");
}

static inline int _get_num(const char *str)
{
    errno = 0;
    char *temp;
    long val = strtol(str, &temp, 0);

    if (temp == str || *temp != '\0' ||
        ((val == LONG_MIN || val == LONG_MAX) && errno == ERANGE)) {
        val = CONVERT_ERROR;
    }
    return (int)val;
}


static int _check_param(int argc, char **argv, int c_min, int c_max, char *use)
{
    int dev;

    if (argc - 1 < c_min || argc - 1 > c_max) {
        printf("Usage: %s %s\n", argv[0], use);
        INVALID_ARGS;
        return ARG_ERROR;
    }

    dev = _get_num(argv[1]);
    if (dev < 0 || dev >= (int)I2C_NUMOF) {
        printf("Error: No device, only %d supported\n", (int)I2C_NUMOF);
        return ARG_ERROR;
    }
    return dev;
}

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

int cmd_i2c_acquire(int argc, char **argv)
{
    int res;
    int dev;

    dev = _check_param(argc, argv, 1, 1, "DEV");
    if (dev == ARG_ERROR) {
        return 1;
    }

    printf("Command: i2c_acquire(%i)\n", dev);
    res = i2c_acquire(dev);
    if (res == I2C_ACK) {
        printf("Success: i2c_%i acquired\n", dev);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_release(int argc, char **argv)
{
    int dev;

    dev = _check_param(argc, argv, 1, 1, "DEV");
    if (dev == ARG_ERROR) {
        return 1;
    }

    printf("Command: i2c_release(%i)\n", dev);
    i2c_release(dev);

    printf("Success: i2c_%i released\n", dev);
    return 0;
}

#ifdef MODULE_PERIPH_I2C_RECONFIGURE
int cmd_i2c_gpio(int argc, char **argv)
{
    int dev;

    dev = _check_param(argc, argv, 1, 1, "DEV");
    if (dev == ARG_ERROR) {
        return 1;
    }

    gpio_t sda_pin = i2c_pin_sda(dev);
    gpio_t scl_pin = i2c_pin_scl(dev);

    printf("Command: i2c_deinit_pins(%i)\n", dev);
    i2c_deinit_pins(dev);

    gpio_init(sda_pin, GPIO_OUT);
    gpio_init(scl_pin, GPIO_OUT);

    xtimer_sleep(1);

    printf("Command: gpio_set()\n");
    gpio_set(sda_pin);
    gpio_set(scl_pin);

    xtimer_sleep(1);

    printf("Command: gpio_clear()\n");
    gpio_clear(sda_pin);
    gpio_clear(scl_pin);

    xtimer_sleep(1);

    printf("Command: i2c_init_pins(%i)\n", dev);
    i2c_init_pins(dev);

    printf("Success: i2c_%i re-init\n", dev);
    return 0;
}
#endif

int cmd_i2c_read_reg(int argc, char **argv)
{
    int res;
    uint16_t addr;
    uint16_t reg;
    uint8_t flags = 0;
    uint8_t data;
    int dev;

    dev = _check_param(argc, argv, 4, 4, "DEV ADDR REG FLAG");
    if (dev == ARG_ERROR) {
        return 1;
    }

    addr = _get_num(argv[2]);
    reg = _get_num(argv[3]);
    flags = _get_num(argv[4]);

    printf("Command: i2c_read_reg(%i, 0x%02x, 0x%02x, 0x%02x)\n", dev, addr,
           reg, flags);
    res = i2c_read_reg(dev, addr, reg, &data, flags);

    if (res == I2C_ACK) {
        _print_i2c_read(dev, &reg, &data, 1);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_read_regs(int argc, char **argv)
{
    int res;
    uint16_t addr;
    uint16_t reg;
    uint8_t flags = 0;
    int len;
    int dev;

    dev = _check_param(argc, argv, 5, 5, "DEV ADDR REG LEN FLAG");
    if (dev == ARG_ERROR) {
        return 1;
    }

    addr = _get_num(argv[2]);
    reg = _get_num(argv[3]);
    len = _get_num(argv[4]);
    flags = _get_num(argv[5]);

    if (len < 1 || len > (int)BUFSIZE) {
        puts("Error: invalid LENGTH parameter given");
        return 1;
    }
    else {
        printf("Command: i2c_read_regs(%i, 0x%02x, 0x%02x, %i, 0x%02x)\n", dev,
            addr, reg, len, flags);
        res = i2c_read_regs(dev, addr, reg, i2c_buf, len, flags);
    }

    if (res == I2C_ACK) {
        _print_i2c_read(dev, &reg, i2c_buf, len);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_read_byte(int argc, char **argv)
{
    int res;
    uint16_t addr;
    uint8_t flags = 0;
    uint8_t data;
    int dev;

    dev = _check_param(argc, argv, 3, 3, "DEV ADDR FLAG");
    if (dev == ARG_ERROR) {
        return 1;
    }

    addr = _get_num(argv[2]);
    flags = _get_num(argv[3]);

    printf("Command: i2c_read_byte(%i, 0x%02x, 0x%02x)\n", dev, addr, flags);
    res = i2c_read_byte(dev, addr, &data, flags);

    if (res == I2C_ACK) {
        _print_i2c_read(dev, NULL, &data, 1);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_read_bytes(int argc, char **argv)
{
    int res;
    uint16_t addr;
    uint8_t flags = 0;
    int len;
    int dev;

    dev = _check_param(argc, argv, 4, 4, "DEV ADDR LENGTH FLAG");
    if (dev == ARG_ERROR) {
        return 1;
    }

    addr = _get_num(argv[2]);
    len = _get_num(argv[3]);
    flags = _get_num(argv[4]);

    if (len < 1 || len > (int)BUFSIZE) {
        puts("Error: invalid LENGTH parameter given");
        return 1;
    }
    else {
        printf("Command: i2c_read_bytes(%i, 0x%02x, %i, 0x%02x)\n", dev,
         addr, len, flags);
        res = i2c_read_bytes(dev, addr, i2c_buf, len, flags);
    }

    if (res == I2C_ACK) {
        _print_i2c_read(dev, NULL, i2c_buf, len);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_write_byte(int argc, char **argv)
{
    int res;
    uint16_t addr;
    uint8_t flags = 0;
    uint8_t data;
    int dev;

    dev = _check_param(argc, argv, 4, 4, "DEV ADDR BYTE FLAG");
    if (dev == ARG_ERROR) {
        return 1;
    }

    addr = _get_num(argv[2]);
    data = _get_num(argv[3]);
    flags = _get_num(argv[4]);

    printf("Command: i2c_write_byte(%i, 0x%02x, 0x%02x, [0x%02x", dev, addr,
        flags, data);
    puts("])");
    res = i2c_write_byte(dev, addr, data, flags);

    if (res == I2C_ACK) {
        printf("Success: i2c_%i wrote 1 byte to the bus\n", dev);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_write_bytes(int argc, char **argv)
{
    int res;
    uint16_t addr;
    uint8_t flags = 0;
    int len = argc - 4;
    int dev;

    dev = _check_param(argc, argv, 4, 3 + BUFSIZE,
                       "DEV ADDR FLAG BYTE0 [BYTE1 [BYTE_n [...]]]");
    if (dev == ARG_ERROR) {
        return 1;
    }

    addr = _get_num(argv[2]);
    flags = _get_num(argv[3]);
    for (int i = 0; i < len; i++) {
        i2c_buf[i] = _get_num(argv[i + 4]);
    }

    printf("Command: i2c_write_bytes(%i, 0x%02x, 0x%02x, [", dev, addr,
        flags);
    for (int i = 0; i < len; i++) {
        if (i != 0) {
            printf(", ");
        }
        printf("0x%02x", i2c_buf[i]);
    }
    puts("])");
    res = i2c_write_bytes(dev, addr, i2c_buf, len, flags);

    if (res == I2C_ACK) {
        printf("Success: i2c_%i wrote %i bytes\n", dev, len);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_write_reg(int argc, char **argv)
{
    int res;
    uint16_t addr;
    uint16_t reg;
    uint8_t flags = 0;
    uint8_t data;
    int dev;

    dev = _check_param(argc, argv, 5, 5, "DEV ADDR REG BYTE FLAG");
    if (dev == ARG_ERROR) {
        return 1;
    }

    addr = _get_num(argv[2]);
    reg = _get_num(argv[3]);
    data = _get_num(argv[4]);
    flags = _get_num(argv[5]);

    printf("Command: i2c_write_reg(%i, 0x%02x, 0x%02x, 0x%02x, [0x%02x", dev,
        addr, reg, flags, data);
    puts("])");
    res = i2c_write_reg(dev, addr, reg, data, flags);

    if (res == I2C_ACK) {
        printf("Success: i2c_%i wrote 1 byte\n", dev);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_write_regs(int argc, char **argv)
{
    int res;
    uint16_t addr;
    uint16_t reg;
    uint8_t flags = 0;
    int len = argc - 5;
    int dev;

    dev = _check_param(argc, argv, 5, 4 + BUFSIZE,
                       "DEV ADDR REG FLAG BYTE0 [BYTE1 ...]");
    if (dev == ARG_ERROR) {
        return 1;
    }

    addr = _get_num(argv[2]);
    reg = _get_num(argv[3]);
    flags = _get_num(argv[4]);
    for (int i = 0; i < len; i++) {
        i2c_buf[i] = _get_num(argv[i + 5]);
    }

    printf("Command: i2c_write_regs(%i, 0x%02x, 0x%02x, 0x%02x, [", dev,
        addr, reg, flags);
    for (int i = 0; i < len; i++) {
        if (i != 0) {
            printf(", ");
        }
        printf("0x%02x", i2c_buf[i]);
    }
    puts("])");
    res = i2c_write_regs(dev, addr, reg, i2c_buf, len, flags);

    if (res == I2C_ACK) {
        printf("Success: i2c_%i wrote %i bytes to reg 0x%02x\n",
            dev, len, reg);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_get_devs(int argc, char **argv)
{
    (void)argv;
    (void)argc;
    printf("Command: return I2C_NUMOF\n");
    printf("Success: Amount of i2c devices: [%d]\n", I2C_NUMOF);
    return 0;
}

int cmd_i2c_get_id(int argc, char **argv)
{
    (void)argv;
    (void)argc;
    puts("Success: [periph_i2c]");
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
    
    // Our distance measurement should now exist in the 3rd and 4th indices of i2c_buf
    return ((i2c_buf[3] << 8) | i2c_buf[2]);
}

// Function for use in the method where we place Lidar to the side of the track
uint16_t find_opposite_length(const int *x, const double *alpha, uint16_t d)
{
    // Required arguments are the distance to the beam-rail intersection, the angle in Radians of the Lidar with respect
    // to the tracks (alpha), and the distance measured by the Lidar (d). Alpha must be between 0 and pi/2.
    // The length of the opposite side of the triangle is returned, 0 if error.
    uint16_t L;

    // Check that the distance measured is not too short. SHOULD ALSO CHECK LONGEST DISTANCE
    double theoretical_shortest_d = *x / cos(*alpha);
    if(d < theoretical_shortest_d)
    {
        return 0;
    }

    // Check that alpha is within the bounds set.
    if (*alpha < 0 || *alpha > M_PI / 2)
    {
        return 0;
    }

    // Calculate the length of the opposite side of the triangle.
    L = d * sin(*alpha);
    return L;
}

int init_lidar(int argc, char **argv) {
    // This method is not used atm.
    (void)argv;
    (void)argc;
    //const int addr = 0x10; // I2C ADDRESS of our lidar sensor
    //const int dev = 0; // I2C device

    // Set update rate to 100 Hz
    //uint8_t data[6] = { 0x5A, 0x06, 0x03,  };

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

int print_opposite(int argc, char **argv)
{
    (void)argv;
    (void)argc;
    const double alpha = M_PI / 4;
    const int x = 100;
    uint16_t d = lidar_distance();
    uint16_t L = find_opposite_length(&x, &alpha, d);
    if (L > 0) {
        printf("Opposite: %i mm\n", L);
    }

    return 0;
}

// Function to calculate velocity (km/h), given two distances(mm) and a time interval between them (ms)
double calculate_velocity(uint16_t d1, uint16_t d2, double t)
{
    if (t == 0)
    {
        // Interval is 0, so velocity must be 0
        return 0;
    }
    uint16_t difference = d2 - d1;
    double velocity = (difference / 1000000) / (t / 3600000); // convert from mm/ms to km/h
    return velocity;
}


int cont_dist(int argc, char **argv) {
    (void)argv;
    (void)argc;

    while (true) {
         uint16_t dist = lidar_distance();
         if (dist > 0) {
             printf("Distance: %i mm\n", dist);
         }
         xtimer_usleep(1000);
    }
    
    return 0;
}

void initialize(void) {
    distance_from_rail = 1000; // 100 cm / 1000 mm
    track_gauge = 1435;
    alpha = M_PI/2;
    printf("Cos(alpha): %f\n", cos(alpha));

    if (alpha == M_PI/2) {
        printf("Halleluja\n");
        theoretical_largest_d = 12000;
        theoretical_shortest_d = 1;
    } else {
        printf("asdasd\n");
        theoretical_largest_d = (distance_from_rail + track_gauge) / cos(alpha);
        theoretical_shortest_d = distance_from_rail / cos(alpha);
    }
}

int cmd_initialise(int argc, char **argv) {
    (void)argv;
    (void)argc;
    initialize();
    return 0;
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

// State 1 of state diagram: Loop until there is an object within distance bounds
int come_here_my_train(void) {
    printf("State 1\n");

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
    return cont_velocity(dist2, 1);
}

int print_flt_avg(uint16_t buff_index) {
    // compute sum of differences
    int16_t sum = 0;
    for (uint16_t i = 0; i < buff_index; i++) {
        sum += rolling_avg_buff[i];
    }
    printf("Sum: %d\n", sum);

    // use sum to compute velocity
    float velocity2 = sum * sin(alpha);  // mm/s
    velocity2 = velocity2 / 277.778;  // Convert from mm/s to km/h
    printf("Velocity in km/h: ");
    print_float(velocity2, 2);
    printf("\n");
    return 0;
}

// State 4: Continuous velocity measurements
int cont_velocity(uint16_t prev_dist, uint16_t buff_index) {
    xtimer_usleep(MEASUREMENT_SLEEP);

    //printf("State 4, i: %i\n", buff_index);
    uint16_t dist = lidar_distance();

    if (dist == 0 || dist > theoretical_largest_d) {
        print_flt_avg(buff_index);
        printf("Moving from state 4 to state 1. Dist: %d mm\n", dist);
        return come_here_my_train();
    } else if (dist > 0 && dist < theoretical_shortest_d) {
        print_flt_avg(buff_index);
        printf("Moving from state 4 to state 2. Dist: %d mm\n", dist);
        return check_valid_region();
    }

    int16_t difference = dist - prev_dist;
    rolling_avg_buff[buff_index] = difference;

    //printf("Should equal zero for velocity reading: %d\n", (buff_index + 1) % UPDATE_FREQ);

    if ((buff_index + 1) % UPDATE_FREQ == 0)
    {
        print_flt_avg(MEASUREMENT_FREQ);
    }
    
    if (buff_index == (MEASUREMENT_FREQ - 1)) {
        return cont_velocity(dist, 0);
    } else {
        return cont_velocity(dist, buff_index + 1);
    }
}


static const shell_command_t shell_commands[] = {
    { "i2c_acquire", "Get access to the I2C bus", cmd_i2c_acquire },
    { "print_distance", "Take a distance measurement from lidar", print_distance },
    { "print_opposite", "Find opposite distance length", print_opposite },
    { "start", "Start state machine", cmd_start },
    { "i2c_release", "Release to the I2C bus", cmd_i2c_release },
#ifdef MODULE_PERIPH_I2C_RECONFIGURE
    { "i2c_gpio", "Re-configures I2C pins to GPIO mode and back.", cmd_i2c_gpio },
#endif
    { "i2c_read_reg", "Read byte from register", cmd_i2c_read_reg },
    { "i2c_read_regs", "Read bytes from registers", cmd_i2c_read_regs },
    { "i2c_read_byte", "Read byte from the I2C device", cmd_i2c_read_byte },
    { "i2c_read_bytes", "Read bytes from the I2C device", cmd_i2c_read_bytes },
    { "i2c_write_byte", "Write byte to the I2C device", cmd_i2c_write_byte },
    { "i2c_write_bytes", "Write bytes to the I2C device", cmd_i2c_write_bytes },
    { "i2c_write_reg", "Write byte to register", cmd_i2c_write_reg },
    { "i2c_write_regs", "Write bytes to registers", cmd_i2c_write_regs },
    { "i2c_get_devs", "Gets amount of supported i2c devices", cmd_i2c_get_devs },
    { "i2c_get_id", "Get the id of the fw", cmd_i2c_get_id },
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
    
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
