/*
 * Demo of I2C ULP routines
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "stack.S"


/*
Working BMP280 i2c example...

Addr   Data   R/W     ACK/NAK
0x76   0xD0   Write   ACK       ChipID?
0x76   0x58   Read    NAK       ChipID = 0x58 (BMP280)
0x76   0x88   Write   ACK       Read Calibration Data (Temp calibration)
0x76   0x3A   Read    ACK       T1
0x76   0x6E   Read    ACK       T1
0x76   0xA0   Read    ACK       T2
0x76   0x67   Read    ACK       T2
0x76   0x32   Read    ACK       T3
0x76   0x00   Read    NAK       T3
0x76   0x8E   Write   ACK       Read Calibration Data (Pressure)
0x76   0x35   Read    ACK       P1
0x76   0x96   Read    ACK
0x76   0xD7   Read    ACK       P2
0x76   0xD5   Read    ACK
0x76   0xD0   Read    ACK       P3
0x76   0x0B   Read    ACK
0x76   0xF3   Read    ACK       P4
0x76   0x1E   Read    ACK
0x76   0x88   Read    ACK       P5
0x76   0xFF   Read    ACK
0x76   0xF9   Read    ACK       P6
0x76   0xFF   Read    ACK
0x76   0x8C   Read    ACK       P7
0x76   0x3C   Read    ACK
0x76   0xF8   Read    ACK       P8
0x76   0xC6   Read    ACK
0x76   0x70   Read    ACK       P9
0x76   0x17   Read    NAK
0x76   0xF5   Write   ACK       Write config   SPI, filter, t_sb
0x76   0x78   Write   ACK       0111 1000 =
0x76   0xF4   Write   ACK       Write config
0x76   0xB7   Write   ACK
0x76   0xF7   Write   ACK
0x76   0x52   Read    ACK
0x76   0xFF   Read    ACK
0x76   0xF0   Read    ACK
0x76   0x7F   Read    ACK
0x76   0x3E   Read    ACK
0x76   0x50   Read    NAK
*/

#define BMP280_REGISTER_CONTROL               (0xF4)
#define BMP280_REGISTER_CONFIG                (0xF5)
#define BMP280_REGISTER_CHIPID                (0xD0)
#define BMP280_REGISTER_VERSION               (0xD1)
#define BMP280_REGISTER_SOFTRESET             (0xE0)
#define BMP280_REGISTER_PRESS                 (0xF7)	// 0xF7-0xF9
#define BMP280_REGISTER_TEMP                  (0xFA)	// 0xFA-0xFC

#define BMP280_REGISTER_DIG_T                 (0x88)	// 0x88-0x8D ( 6)
#define BMP280_REGISTER_DIG_P                 (0x8E)	// 0x8E-0x9F (18)
#define BMP280_REGISTER_END                    0x9F


#define BME280_SAMPLING_DELAY  113 //maximum measurement time in ms for maximum oversampling for all measures = 1.25 + 2.3*16 + 2.3*16 + 0.575 + 2.3*16 + 0.575 ms



/*

Use BMP280 in normal resolution mode with forced sampling

Control register 0xF4

Bit 1,0   mode  [1:0] = 01 (Forced mode)
Bit 4,3,2 osrs_p[2:0] = 011 (Pressure oversampling x4 )
Bit 7,6,5 osrs_t[2:0] = 010  (Temperature Oversampling x2)
Write
01001101 = 0x4D

Control register 0xF5
t_sb[2:0] 111 (4s in normal mode)
filter[2:0] 010 (IIR = 4)
spi3w_en [0] 00 (no SPI)

Write
11101000 = 0xE8


*/

#define BMP280_ADDR 0x76
#define BMP280_CTRL_MEAS 0xF5
#define BMP280_CTRL_MEAS_VAL 0xE8
#define BMP280_CONFIG 0xF4
#define BMP280_CONFIG_VAL 0x4D



/* Define variables, which go into .bss section (zero-initialized data) */
	.bss

	.global temp
temp:	.long 0
	.global	pressure, pressure2
pressure: .long 0
pressure2: .long 0


/*
static struct {
	uint16_t  dig_T1;
	int16_t   dig_T2;
	int16_t   dig_T3;
	uint16_t  dig_P1;
	int16_t   dig_P2;
	int16_t   dig_P3;
	int16_t   dig_P4;
	int16_t   dig_P5;
	int16_t   dig_P6;
	int16_t   dig_P7;
	int16_t   dig_P8;
	int16_t   dig_P9;
	uint8_t   dig_H1;
	int16_t   dig_H2;
	uint8_t   dig_H3;
	int16_t   dig_H4;
	int16_t   dig_H5;
	int8_t    dig_H6;
} bme280_data;
*/

	.global t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9;

t1: .long 0
t2: .long 0
t3: .long 0
p1: .long 0
p2: .long 0
p3: .long 0
p4: .long 0
p5: .long 0
p6: .long 0
p7: .long 0
p8: .long 0
p9: .long 0

	/* Code goes into .text section */
	.text

.global readBMP
readBMP:
    //if t1 == 0 goto initBMP
	move r1, t1
	ld r0,r1,0
	jumpr initBMP,1,lt
didInit:
	move r1, BMP280_ADDR
	push r1
	move r1, BMP280_CONFIG
	push r1
	move r1, BMP280_CONFIG_VAL
	push r1
	psr
	jump write8
	add r3,r3,3 // remove 3 arguments from stack
	move r0,r2 // test for error in r2
	jumpr fail,1,ge

	// Wait 150ms for sensor computation
	move r2,50
	psr
	jump waitMs

	// Read 16 bit result
	/*
	move r1, BMP280_ADDR
	push r1
	move r1, BMP280_REG_RESULT
	push r1
	psr
	jump read16
	add r3,r3,2 // remove call parameters from stack
	move r1,r0 // save result
	move r0,r2 // test for error
	jumpr fail,1,ge
	move r2,temp // store result
	st r1,r2,0

	// Read raw pressure
	move r1,BMP180_ADDR
	push r1
	move r1,BMP180_REG_CONTROL
	push r1
	move r1,BMP180_COMMAND_PRESSURE1
	push r1
	psr
	jump write8
	add r3,r3,3 // remove 3 arguments from stack
	move r0,r2 // test for error in r2
	jumpr fail,1,ge

	// Wait 8 ms for sensor computation
	move r2,8
	psr
	jump waitMs

	move r1,BMP180_ADDR
	push r1
	move r1,BMP180_REG_RESULT
	push r1
	psr
	jump read16
	add r3,r3,2 // remove call parameters from stack
	move r1,r0 // save result
	move r0,r2 // test for error
	jumpr fail,1,ge
	move r2,pressure // store result
	st r1,r2,0

	move r1,BMP180_ADDR
	push r1
	move r1,(BMP180_REG_RESULT+2)
	push r1
	psr
	jump read8
	add r3,r3,2 // remove call parameters from stack
	move r1,r0 // save result
	move r0,r2 // test for error
	jumpr fail,1,ge
	move r2,pressure2 // store result
	st r1,r2,0
	*/

	ret

fail:
	move r1,temp
	move r0,0 // 0 signals error
	st r0,r1,0
	ret

// Setup measurement settings
initBMP:
	MOVE r1, BMP280_ADDR
	PUSH r1
	MOVE r1, BMP280_CTRL_MEAS
	PUSH r1
	MOVE r1, BMP280_CTRL_MEAS_VAL
	PUSH r1
	psr
	JUMP write8
	ADD r3,r3,3 // remove 3 arguments from stack
	MOVE r0,r2 // test for error in r2
	JUMPR fail,1,ge

    MOVE r1, t1
    PUSH r1
    MOVE r1, BMP280_ADDR
    PUSH r1
    MOVE r1, BMP280_REGISTER_DIG_T
    push r1
read_cal:
	psr
	jump read16
	or r2,r2,0 // test error
	jump readok,eq
	jump fail
readok:

	ld r1,r3,12 // r1 <- [r3 + 12]
	st r0,r1,0  // [r1] <- r0
	add r1,r1,1 // r1 += 1
	st r1,r3,12 // [r3+12] = r1 -        next cal parameter address

	ld r0,r3,4  // r0 = [r3 + 4]
	add r0,r0,2 // r0 += 2
	st r0,r3,4 // [r3+4] = r0

	jumpr read_cal, BMP280_REGISTER_END + 2,lt
	add r3,r3,3
	jump didInit

// Wait for r2 milliseconds
waitMs:
	wait 8000
	sub r2,r2,1
	jump doneWaitMs,eq
	jump waitMs
doneWaitMs:
	ret
