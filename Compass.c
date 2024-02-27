/*------------------
-- Company: TEIS AB
-- Engineer: Andrey Yunin
--
-- Create Date: 2021-09-28
-- History:
-- Design Name: Kompass
-- Target Devices: ALTERA MAX 10
-- Tool versions:
-- Nios IIe Software Build Tools for Eclipse (Quartus Prime 18.1)
-- Description:
-- Test-program for Kompass och I2C .
--
------------------*/

#include "sys/alt_irq.h"
#include <stdio.h>
#include  <system.h>
#include <DE10_Lite_Arduino_Driver.h>
#include <altera_avalon_i2c.h>
#include <math.h>


#define LSM303_ADDR 0x1E // LSM303 address

// all register names are according to data sheet LSM 303

#define CRA_REG_M 0x00        //0x0C set this value for CRA_REG_M register
#define CRB_REG_M 0x01        //0x20 set this value for CRB_REG_M register
#define MR_REG_M  0x02        //0x00 set this value for MR_REG_M register
#define OUT_X_H_M 0x03        //to set this register to start read data compass
#define pi        3.14159265
#define div_pi    (180/pi)


ALT_AVALON_I2C_STATUS_CODE status;  // Variable to hold status codes
ALT_AVALON_I2C_DEV_t *i2c_dev;      // Pointer to instance structure

// open_device function provides I2C connection and
// sets connection with slave device (compass)

void open_device(alt_u8 name_device)
{
   //get a pointer to the avalon i2c instance
   i2c_dev = alt_avalon_i2c_open("/dev/arduino_i2c"); //see description in Embedded Peripherals IP User Guide

   //set the address of the device using
   alt_avalon_i2c_master_target_set(i2c_dev, name_device);

   // set the I2C speed (normal_mode, fast mode)
   i2c_set_normal_mode(); // see description in C book

}

//write data to an register at address "reg"

void write_reg(alt_u8 reg, alt_u8 value)
{
   alt_u8 txbuffer[2];

   // byte for register address
   txbuffer[0] = reg;

   // byte for register data
   txbuffer[1] = value;

   alt_avalon_i2c_master_tx( //see description in Embedded Peripherals IP User Guide
         i2c_dev,
         txbuffer,
         2,
         ALT_AVALON_I2C_NO_INTERRUPTS);

}

//read back the data into rxbuffer

void data_receive(int16_t loop)
{


   alt_u8 txbuffer[1];

   // receive buffer declaration - 6 bytes: x_high, x_low....
   alt_u8 rxbuffer[6];



   int16_t x = 0, y = 0, z = 0, x_h = 0,x_l = 0, y_h = 0, y_l = 0, z_h = 0, z_l = 0, azimuth = 0;

   //pointers declaration

   int16_t *p_x, *p_y, *p_z, *p_x_h, *p_x_l, *p_y_h, *p_y_l, *p_z_h, *p_z_l;

   // store addresses of variables in pointer variables

   p_x = &x;
   p_y = &y;
   p_z = &z;

   p_x_h = &x_h;
   p_x_l = &x_l;
   p_y_h = &y_h;
   p_y_l = &y_l;
   p_z_h = &z_h;
   p_z_l = &z_l;

   txbuffer[0] = OUT_X_H_M; // start read from register with this name (see data sheet for LSM303)

   //read back the data into rxbuffer
   //This command sends the 1 byte register data address required by the register
   //Then does a restart and receives the data.
   status = alt_avalon_i2c_master_tx_rx(
         i2c_dev,
         txbuffer,
         1,
         rxbuffer,
         6,
         ALT_AVALON_I2C_NO_INTERRUPTS);


   if (status == 0)
   {

      *p_x_h = rxbuffer[0];
      *p_x_l = rxbuffer[1];
      *p_z_h = rxbuffer[2];
      *p_z_l = rxbuffer[3];
      *p_y_h = rxbuffer[4];
      *p_y_l = rxbuffer[5];

      printf("test N %d\n x_h: %d  x_l %d: y_h: %d y_l: %d z_h: %d z_l: %d\n",
            loop, x_h, x_l, y_h, y_l, z_h, z_l);

      // combine high and low bytes to get x, y, z values

      *p_x = (rxbuffer[0] << 8)|rxbuffer[1];
      *p_z = (rxbuffer[2] << 8)|rxbuffer[3];
      *p_y = (rxbuffer[4] << 8)|rxbuffer[5];

      printf("x: %d  y: %d  z: %d\n",
            x, y, z);



      ////////////////////// azimuth calculation considering division by zero :

      if (x == 0 && y <0){
         azimuth = 90;}
      else
         if (x == 0 && y > 0){
            azimuth = 270;}
         else
            if (x < 0 && y < 0){
               azimuth =  180 - ((atan(y/x))*(div_pi));}
            else
               if (x < 0 && y == 0){
                  azimuth =  180 - ((atan(y/x))*(div_pi));}
               else
                  if (x < 0 && y > 0){
                     azimuth =  180 - ((atan(y/x))*(div_pi));}
                  else
                     if (x > 0 && y <0){
                        azimuth = - (atan(y/x)*(div_pi));}
                     else
                        if (x > 0 && y >0){
                           azimuth =  360 - (atan(y/x)*(div_pi));}






      printf("azimuth: %d degrees\n" , azimuth);
   }

   else
      printf("*****failure****\n");
}


//////////////////////////////////********************////////////////////

int kompass_main()
{

   int16_t  loop = 0;

   printf("*** Starting compass test\n");


   open_device(LSM303_ADDR);

   // 0x0C = 0b00001100
   // DO = 011 (7.5 Hz ODR)
   write_reg(CRA_REG_M, 0x0C);

   // 0x20 = 0b00100000
   // GN = 001 (+/- 1.3 gauss full scale)
   write_reg(CRB_REG_M, 0x20);

   // 0x00 = 0b00000000
   // MD = 00 (continuous-conversion mode)
   write_reg(MR_REG_M,  0x00);

   printf("*** Reading compass data 200 times and then stops.\n");

   while(loop <= 200)

   {

      // call function to get compass data and calculation results:
      data_receive (loop);

      paus(100000);

      loop++;
   }

   printf("Finished testing Compass and I2C!\n\n");

   paus(200000);

   return 0;
}


