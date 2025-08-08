/*************************************************************************
 *  Basic stereo loop code for C6713 DSK and AIC23 codec
 *  D. Richard Brown on 22-Aug-2011
 *  Based on code from "Real-Time Digital Signal Processing Based on TMS320C6000"
 *  by N. Kehtarnavaz and N. Kim.
 *************************************************************************/

#define CHIP_6713 1

#include <stdio.h>
#include <c6x.h>
#include <csl.h>
#include <csl_mcbsp.h>
#include <csl_irq.h>

#include <math.h>
#include <stdlib.h>

#include <dsk6713.h>
#include <dsk6713_aic23.h>
#include <dsk6713_led.h>
#include <dsk6713_dip.h>

#define Ncoef 251
#define stages 1
#define Nnum 3
#define Nden 3
#include "tmwtypes.h"
//#include "bandas.h"

#include "bandasButter.h"


Uint32 data;
short x_inr1, x_inl1;
short x_inr2, x_inl2;
short x_inr3, x_inl3;
short x_inr4, x_inl4;
short x_inr5, x_inl5;
int a,i;
//real64_T *Band_freqs[5] = {Band_1,Band_2,Band_3,Band_4,Band_5};
short x_r[Ncoef], x_l[Ncoef];
real64_T y_r0 = 0.0;
real64_T y_r1 = 0.0;
real64_T y_r2 = 0.0;
real64_T y_r3 = 0.0;
real64_T y_r4 = 0.0;
real64_T y_rt = 0.0;

real64_T y_l0 = 0.0;
real64_T y_l1 = 0.0;
real64_T y_l2 = 0.0;
real64_T y_l3 = 0.0;
real64_T y_l4 = 0.0;
real64_T y_lt = 0.0;


real64_T u_nr1 = 0.0;
real64_T u_nr2 = 0.0;
real64_T u_nr3 = 0.0;
real64_T u_nr4 = 0.0;
real64_T u_nr5 = 0.0;

real64_T u_nl1 = 0.0;
real64_T u_nl2 = 0.0;
real64_T u_nl3 = 0.0;
real64_T u_nl4 = 0.0;
real64_T u_nl5 = 0.0;

real64_T dly_r1[2] = {0,0};
real64_T dly_r2[2] = {0,0};
real64_T dly_r3[2] = {0,0};
real64_T dly_r4[2] = {0,0};
real64_T dly_r5[2] = {0,0};

real64_T dly_l1[2] = {0,0};
real64_T dly_l2[2] = {0,0};
real64_T dly_l3[2] = {0,0};
real64_T dly_l4[2] = {0,0};
real64_T dly_l5[2] = {0,0};


int inc_dec = 1;
int IDsel = 0;
int selG = 0;
int lastIn0 = 1;
int lastIn1 = 1;
int g0 = 1;
int g1 = 1;
int g2 = 1;
int g3 = 1;
int g4 = 1;
int toLeds;
int val_in = 0;

DSK6713_AIC23_CodecHandle hCodec;                           // Codec handle
DSK6713_AIC23_Config config = DSK6713_AIC23_DEFAULTCONFIG;  // Codec configuration with default settings
interrupt void serialPortRcvISR(void);

void main()
{
    DSK6713_init();
    DSK6713_LED_init();
    DSK6713_DIP_init();
    hCodec = DSK6713_AIC23_openCodec(0, &config);   // open codec and get handle

    // Configure buffered serial ports for 32 bit operation
    // This allows transfer of both right and left channels in one read/write
    MCBSP_FSETS(SPCR1, RINTM, FRM);
    MCBSP_FSETS(SPCR1, XINTM, FRM);
    MCBSP_FSETS(RCR1, RWDLEN1, 32BIT);
    MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);

    // set codec sampling frequency
    DSK6713_AIC23_setFreq(hCodec, DSK6713_AIC23_FREQ_44KHZ);

    // interrupt setup
    IRQ_globalDisable();            // Globally disables interrupts
    IRQ_nmiEnable();                // Enables the NMI interrupt
    IRQ_map(IRQ_EVT_RINT1,15);      // Maps an event to a physical interrupt
    IRQ_hook(15, serialPortRcvISR);
    IRQ_enable(IRQ_EVT_RINT1);      // Enables the event
    IRQ_globalEnable();             // Globally enables interrupts
    while(1){
    }
}
// Note: Configurar optimización de código y debug.
/*
interrupt void serialPortRcvISR()
{
    //union {Uint32 combo; short channel[2];} temp;
    data = MCBSP_read(DSK6713_AIC23_DATAHANDLE);
    // Note that right channel is in temp.channel[0]
    // Note that left channel is in temp.channel[1]
    //MCBSP_write(DSK6713_AIC23_DATAHANDLE,(sine_table[loop]*gain));
    g0 = (DSK6713_DIP_get(0) ==  1) ? 3 : 1 ;
    g1 = (DSK6713_DIP_get(1) ==  1) ? 3 : 1 ;
    g2 = (DSK6713_DIP_get(2) ==  1) ? 3 : 1 ;
    g3 = (DSK6713_DIP_get(3) ==  1) ? 3 : 1 ;
    g4 = 3;

    val_in = 0;
    val_in = DSK6713_DIP_get(0);
    val_in =val_in | DSK6713_DIP_get(1) << 1;
    val_in =val_in | DSK6713_DIP_get(2) << 2;
    val_in =val_in | DSK6713_DIP_get(3) << 3;
    DSK6713_rset(DSK6713_USER_REG, (val_in|DSK6713_DIP_get(3) << 3));
    if(val_in == 0){
        MCBSP_write(DSK6713_AIC23_DATAHANDLE,3*data);
    }
    else{
        for(i = Ncoef-1 ; i > 0; i--){
            x_r[i] = x_r[i-1];
            x_l[i] = x_l[i-1];
        }
        x_r[0] = ((short)data);
        x_l[0] = ((short)(data >> 16));
        y_r0 = 0.0;
        y_r1 = 0.0;
        y_r2 = 0.0;
        y_r3 = 0.0;
        y_r4 = 0.0;

        y_l0 = 0.0;
        y_l1 = 0.0;
        y_l2 = 0.0;
        y_l3 = 0.0;
        y_l4 = 0.0;


        for(i = 0; i < Ncoef; i++){
            y_r0 = y_r0 + ((real64_T)(Band_freqs[0][i]) * (real64_T)x_r[i]);
            y_r1 = y_r1 + ((real64_T)(Band_freqs[1][i]) * (real64_T)x_r[i]);
            y_r2 = y_r2 + ((real64_T)(Band_freqs[2][i]) * (real64_T)x_r[i]);
            y_r3 = y_r3 + ((real64_T)(Band_freqs[3][i]) * (real64_T)x_r[i]);
            y_r4 = y_r4 + ((real64_T)(Band_freqs[4][i]) * (real64_T)x_r[i]);

            y_l0 = y_r0 + ((real64_T)(Band_freqs[0][i]) * (real64_T)x_l[i]);
            y_l1 = y_l1 + ((real64_T)(Band_freqs[1][i]) * (real64_T)x_l[i]);
            y_l2 = y_l2 + ((real64_T)(Band_freqs[2][i]) * (real64_T)x_l[i]);
            y_l3 = y_l3 + ((real64_T)(Band_freqs[3][i]) * (real64_T)x_l[i]);
            y_l4 = y_l4 + ((real64_T)(Band_freqs[4][i]) * (real64_T)x_l[i]);
        }
        y_rt = (g0*y_r0)+(g1*y_r1)+(g2*y_r2)+(g3*y_r3)+(g4*y_r4);
        y_lt = (g0*y_l0)+(g1*y_l1)+(g2*y_l2)+(g3*y_l3)+(g4*y_l4);
        MCBSP_write(DSK6713_AIC23_DATAHANDLE,(short)y_rt);
        MCBSP_write(DSK6713_AIC23_DATAHANDLE,(short)y_lt);
    }
    return;
}*/








// Note: Configurar optimización de código y debug.
interrupt void serialPortRcvISR()
{
    //union {Uint32 combo; short channel[2];} temp;
    data = MCBSP_read(DSK6713_AIC23_DATAHANDLE);
    x_inr1 = ((short)data);
            x_inl1 = ((short)(data >> 16));

            x_inr2 = ((short)data);
            x_inl2 = ((short)(data >> 16));

            x_inr3 = ((short)data);
            x_inl3 = ((short)(data >> 16));

            x_inr4 = ((short)data);
            x_inl4 = ((short)(data >> 16));

            x_inr5 = ((short)data);
            x_inl5 = ((short)(data >> 16));
    // Note that right channel is in temp.channel[0]
    // Note that left channel is in temp.channel[1]
    //MCBSP_write(DSK6713_AIC23_DATAHANDLE,(sine_table[loop]*gain));

    //if(DSK6713_DIP_get(0)){
    //    inc_dec = (DSK6713_DIP_get(3) ==  1) ? inc_dec + 1 : inc_dec - 1 ;
    //}

    //if (DSK6713_DIP_get(0) == 0 && lastIn0 == 1) {
    //    if(DSK6713_DIP_get(3)==0){
    //        inc_dec = inc_dec++;
    //    }
    //    inc_dec = inc_dec--;
    //}
    //lastIn0 = DSK6713_DIP_get(0);
    inc_dec = (DSK6713_DIP_get(0) == 0 && lastIn0 == 1) ? ((DSK6713_DIP_get(3) == 0) ? inc_dec + 1 : inc_dec - 1) : inc_dec;
    lastIn0 = DSK6713_DIP_get(0);

    //inc_dec = (DSK6713_DIP_get(0)==0) ? ((DSK6713_DIP_get(3)==0) ? inc_dec + 1 : inc_dec - 1) : inc_dec;
    if (DSK6713_DIP_get(1) == 0 && lastIn1 == 1) {
        selG++;  // Incrementar en flanco de bajada
    }
    lastIn1 = DSK6713_DIP_get(1);

    if(inc_dec > 4){
        inc_dec = 4;
    }
    if(inc_dec < -4){
        inc_dec = -4;
    }
    if(selG > 4){
        selG = 0;
    }

    switch(selG){
        case 0:
            g0 = inc_dec;
            break;
        case 1:
            g1 = inc_dec;
            break;
        case 2:
            g2 = inc_dec;
            break;
        case 3:
            g3 = inc_dec;
            break;
        case 4:
            g4 = inc_dec;
            break;
    }
    toLeds = (DSK6713_DIP_get(2) == 0) ? selG : inc_dec;
    DSK6713_rset(DSK6713_USER_REG, toLeds);
    if((g0 == 1) && (g1 == 1) && (g2 == 1) && (g3 == 1) && (g4 == 1)){
        MCBSP_write(DSK6713_AIC23_DATAHANDLE,3*data);
    }
    else{
        //input = input_sample();
        // u(n) = x(n) - b1u(n-1) - b2u(n-2) - ... - bNu(n-N)
        // y(n) = a0u(n) + a1u(n-1) + a2u(n-2) - ... - aNu(n-N)


        for(i = 0; i < stages ; i++){
            // Banda 1
            u_nr1 = (real64_T)x_inr1 - ((real64_T)b1[1] * dly_r1[0]) - ((real64_T)b1[2] * dly_r1[1]);
            u_nl1 = (real64_T)x_inl1 - ((real64_T)b1[1] * dly_l1[0]) - ((real64_T)b1[2] * dly_l1[1]);

            // Calcular las salidas filtradas
            y_r0 = ((real64_T)a1[0] * u_nr1) + ((real64_T)a1[1] * dly_r1[0]) + ((real64_T)a1[2] * dly_r1[1]);
            y_l0 = ((real64_T)a1[0] * u_nl1) + ((real64_T)a1[1] * dly_l1[0]) + ((real64_T)a1[2] * dly_l1[1]);

            // Actualizar retardos
            dly_r1[1] = dly_r1[0];
            dly_r1[0] = u_nr1;

            dly_l1[1] = dly_l1[0];
            dly_l1[0] = u_nl1;

            x_inr1 = (short)y_r0;
            x_inl1 = (short)y_l0;

            // Banda 2
            u_nr2 = (real64_T)x_inr2 - ((real64_T)b2[1] * dly_r2[0]) - ((real64_T)b2[2] * dly_r2[1]);
            u_nl2 = (real64_T)x_inl2 - ((real64_T)b2[1] * dly_l2[0]) - ((real64_T)b2[2] * dly_l2[1]);

            // Calcular las salidas filtradas
            y_r0 = ((real64_T)a2[0] * u_nr2) + ((real64_T)a2[1] * dly_r2[0]) + ((real64_T)a2[2] * dly_r2[1]);
            y_l0 = ((real64_T)a2[0] * u_nl2) + ((real64_T)a2[1] * dly_l2[0]) + ((real64_T)a2[2] * dly_l2[1]);

            // Actualizar retardos
            dly_r2[1] = dly_r2[0];
            dly_r2[0] = u_nr2;

            dly_l2[1] = dly_l2[0];
            dly_l2[0] = u_nl2;

            x_inr2 = (short)y_r1;
            x_inl2 = (short)y_l1;

            // Banda 3
            u_nr3 = (real64_T)x_inr3 - (((real64_T)b3[1]*dly_r3[0])) - (((real64_T)b3[2]*dly_r3[1]));
            u_nl3 = (real64_T)x_inl3 - (((real64_T)b3[1]*dly_l3[0])) - (((real64_T)b3[2]*dly_l3[1]));
            y_r2 = (((real64_T)a3[0]*u_nr3))+(((real64_T)a3[1]*dly_r3[0]))+(((real64_T)a3[2]*dly_r3[1]));
            y_l2 = (((real64_T)a3[0]*u_nl3))+(((real64_T)a3[1]*dly_l3[0]))+(((real64_T)a3[2]*dly_l3[1]));
            dly_r3[1] = dly_r3[0];
            dly_r3[0] = u_nr3;

            dly_l3[1] = dly_l3[0];
            dly_l3[0] = u_nl3;

            x_inr3 = (short)y_r2;
            x_inl3 = (short)y_l2;

            // Banda 4
            u_nr4 = (real64_T)x_inr4 - (((real64_T)b4[1]*dly_r4[0])) - (((real64_T)b4[2]*dly_r4[1]));
            u_nl4 = (real64_T)x_inl4 - (((real64_T)b4[1]*dly_l4[0])) - (((real64_T)b4[2]*dly_l4[1]));
            y_r3 = (((real64_T)a4[0]*u_nr4))+(((real64_T)a4[1]*dly_r4[0]))+(((real64_T)a4[2]*dly_r4[1]));
            y_l3 = (((real64_T)a4[0]*u_nl4))+(((real64_T)a4[1]*dly_l4[0]))+(((real64_T)a4[2]*dly_l4[1]));
            dly_r4[1] = dly_r4[0];
            dly_r4[0] = u_nr4;

            dly_l4[1] = dly_l4[0];
            dly_l4[0] = u_nl4;

            x_inr4 = (short)y_r3;
            x_inl4 = (short)y_l3;

            // Banda 5
            u_nr5 = (real64_T)x_inr5 - (((real64_T)b5[1]*dly_r5[0])) - (((real64_T)b5[2]*dly_r5[1]));
            u_nl5 = (real64_T)x_inl5 - (((real64_T)b5[1]*dly_l5[0])) - (((real64_T)b5[2]*dly_l5[1]));
            y_r4 = (((real64_T)a5[0]*u_nr5))+(((real64_T)a5[1]*dly_r5[0]))+(((real64_T)a5[2]*dly_r5[1]));
            y_l4 = (((real64_T)a5[0]*u_nl5))+(((real64_T)a5[1]*dly_l5[0]))+(((real64_T)a5[2]*dly_l5[1]));
            dly_r5[1] = dly_r5[0];
            dly_r5[0] = u_nr5;

            dly_l5[1] = dly_l5[0];
            dly_l5[0] = u_nl5;

            x_inr5 = (short)y_r4;
            x_inl5 = (short)y_l4;
        }
        y_rt = (g0*y_r0)+(g1*y_r1)+(g2*y_r2)+(g3*y_r3)+(g4*y_r4);
        y_lt = (g0*y_l0)+(g1*y_l1)+(g2*y_l2)+(g3*y_l3)+(g4*y_l4);
        MCBSP_write(DSK6713_AIC23_DATAHANDLE,(short)y_rt);
        MCBSP_write(DSK6713_AIC23_DATAHANDLE,(short)y_lt);
    }
    return;
}
