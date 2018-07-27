/*
 * Demo of I2C ULP routines
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "stack.S"

#define BME280_SAMPLING_DELAY  113 //maximum measurement time in ms for maximum oversampling for all measures = 1.25 + 2.3*16 + 2.3*16 + 0.575 + 2.3*16 + 0.575 ms

#define BMP280_REGISTER_CHIPID                (0xD0)
#define BMP280_REGISTER_VERSION               (0xD1)
#define BMP280_REGISTER_SOFTRESET             (0xE0)

#define BMP280_REGISTER_PRESS                 (0xF7)	// 0xF7-0xF9
#define BMP280_REGISTER_TEMP                  (0xFA)	// 0xFA-0xFC
#define BMP280_REGISTER_END                    0xFC

#define BMP280_REGISTER_DIG_T                 (0x88)	// 0x88-0x8D ( 6)
#define BMP280_REGISTER_DIG_P                 (0x8E)	// 0x8E-0x9F (18)
#define BMP280_REGISTER_DIG_END                0x9F

#define BMP280_ADDR 0x76
#define BMP280_CTRL_MEAS 0xF4
#define BMP280_CONFIG 0xF5



/* Define variables, which go into .bss section (zero-initialized data) */
	.bss

    .global reg_config, reg_ctrl

reg_config: .long 0
reg_ctrl: .long 0

	.global	pres_msb, pres_lsb, pres_xlsb, temp_msb, temp_lsb, temp_xlsb
pres_msb: .long 0
pres_lsb: .long 0
pres_xlsb: .long 0
temp_msb:	.long 0
temp_lsb:	.long 0
temp_xlsb:	.long 0


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

popfail:
	pop r1 // pop caller return address
	move r2,1
	ret

.global readBMP
readBMP:
    //if t1 == 0 goto initBMP
	move r1, t1
	ld r0,r1,0
	jumpr initBMP,1,lt
didInit:
	move r1, BMP280_ADDR
	push r1
	MOVE r1, BMP280_CTRL_MEAS
	PUSH r1
	MOVE r0, reg_ctrl
	ld r1, r0,0
	PUSH r1
	psr
	jump write8
	add r3,r3,3 // remove 3 arguments from stack
	move r0,r2 // test for error in r2
	jumpr fail,1,ge

	// Wait 50ms for sensor computation
	move r2,50
	psr
	jump waitMs

	//psr
	//jump i2c_start_cond

    move r1, BMP280_ADDR
    push r1

    MOVE r1, BMP280_REGISTER_PRESS
    push r1
    push r1

    psr
    jump read_intro

    add r3,r3,3

	move r2,0 // last byte
	psr
	jump i2c_read_byte
	move r2, pres_msb
    st r0,r2,0


	move r2,0 // last byte
	psr
	jump i2c_read_byte
	move r2, pres_lsb
	st r0,r2,0


	move r2,0 // last byte
	psr
	jump i2c_read_byte
	move r2, pres_xlsb
	st r0,r2,0


	move r2,0 // last byte
	psr
	jump i2c_read_byte
	move r2, temp_msb
	st r0,r2,0


	move r2,0 // last byte
	psr
	jump i2c_read_byte
	move r2, temp_lsb
	st r0,r2,0


	move r2,1 // last byte
	psr
	jump i2c_read_byte
	move r2, temp_xlsb
	st r0,r2,0

    // Done

	psr
	jump i2c_stop_cond
	jump testTemp

fail:
	move r1,temp_msb
	move r0,0 // 0 signals error
	st r0,r1,0
	ret

// Setup measurement settings
initBMP:
    MOVE r1, BMP280_ADDR
    PUSH r1
    MOVE r1, BMP280_REGISTER_SOFTRESET
    PUSH r1
    move r1, 0xB6 // reset sensor
    psr
    JUMP write8
    ADD r3,r3,3

	move r2,200
	psr
	jump waitMs

	MOVE r1, BMP280_ADDR
	PUSH r1
	move r1, BMP280_CONFIG
	push r1
	move r1, reg_config
	ld r0, r1,0
	push r0

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

	ld r1,r3,12 // r1 <- [r3 + 12]  r1 = address of third stack param
	st r0,r1,0  // [r1] <- r0       r0 -> [r1] (result from read16)
	add r1,r1,1 // r1 += 1          //  increment address of third stack param by 1
	st r1,r3,12 // [r3+12] = r1 -        next cal parameter address

	ld r0,r3,4  // r0 = [r3 + 4]
	add r0,r0,2 // r0 += 2
	st r0,r3,4 // [r3+4] = r0

	jumpr read_cal, BMP280_REGISTER_DIG_END + 2,lt
	add r3,r3,3
	jump didInit

.global waitMs
// Wait for r2 milliseconds
waitMs:
	wait 8000
	sub r2,r2,1
	jump doneWaitMs,eq
	jump waitMs
doneWaitMs:
	ret
