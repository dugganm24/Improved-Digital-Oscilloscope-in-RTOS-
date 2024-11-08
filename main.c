/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include "buttons.h"
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "sampling.h"
#include "driverlib/timer.h"

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency
#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz

volatile uint32_t gTime = 8345; // time in hundredths of a second

float load=0;
float unload=0;
float cpu_load=0.0;
uint32_t CPULoad(void);


int sample[LCD_HORIZONTAL_MAX];
int i = 0;
int y = 0;
int prev_y = 0;
int rising = 1;
int trigger;
int voltsPerDiv = 4;
int fVoltsPerDiv[] = {0.1,0.2,0.5,1,2};
volatile int buffer[ADC_BUFFER_SIZE];
char str[50];


DataType button;

const char * const gVoltageScaleStr[] = {
    "100 mV", "200 mV", "500 mV", "1V", "2V"
};

/*
 *  ======== main ========
 */


int main(void)
{
    // hardware initialization goes here
    IntMasterDisable();

        // Enable the Floating Point Unit, and permit ISRs to use it
        FPUEnable();
        FPULazyStackingEnable();

        // Initialize the system clock to 120 MHz
        gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

        // configure M0PWM2, at GPIO PF2, BoosterPack 1 header C1 pin 2
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
        GPIOPinConfigure(GPIO_PF2_M0PWM2);
        GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2,
        GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
        // configure the PWM0 peripheral, gen 1, outputs 2 and 3
        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
        // use system clock without division
        PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
        PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
        PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1,
        roundf((float)gSystemClock/PWM_FREQUENCY));
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
        roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
        PWMGenEnable(PWM0_BASE, PWM_GEN_1);

        Crystalfontz128x128_Init(); // Initialize the LCD display driver
        Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

        ADC_Init();
        ButtonInit();

        tContext sContext;
        GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
        GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

        unload = CPULoad();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

void IntMasterEnableTask(UArg arg1, UArg arg2)
{
    IntMasterEnable();
}

void WaveformTask(void){
    while(1){
        Semaphore_pend(Waveform_sem, BIOS_WAIT_FOREVER);
        trigger = rising ? RisingTrigger(): FallingTrigger();
        for(i = 0; i < LCD_HORIZONTAL_MAX - 1; i++) {
            sample[i] = gADCBuffer[ADC_BUFFER_WRAP(trigger - LCD_HORIZONTAL_MAX/2 + i)];
        }
        Semaphore_post(Processing_sem);
    }
}

void ProcessingTask(void){
    while(1){
        Semaphore_pend(Processing_sem, BIOS_WAIT_FOREVER);
        float fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv[voltsPerDiv]);
        int i = 0;
        for(i=0;i<LCD_HORIZONTAL_MAX-1;i++){
            buffer[i] = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)sample[i]-ADC_OFFSET)); //calculate y value
        }
        Semaphore_post(Display_sem);
        Semaphore_post(Waveform_sem);
    }
}

void DisplayTask(void){

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    while(1){
        Semaphore_pend(Display_sem, BIOS_WAIT_FOREVER);

        load = CPULoad();
        cpu_load = (1.0 - load/unload)*100; //calculate CPU utilization

        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black
        GrContextForegroundSet(&sContext, ClrBlue);

        int j=0;
        for(j = -3; j < 4; j++){
            GrLineDrawH(&sContext, 0, LCD_HORIZONTAL_MAX - 1, LCD_VERTICAL_MAX/2 + j * PIXELS_PER_DIV);
            GrLineDrawV(&sContext, LCD_VERTICAL_MAX /2 + j * PIXELS_PER_DIV, 0, LCD_HORIZONTAL_MAX-1);
        }

        GrContextForegroundSet(&sContext, ClrYellow);
        for(i = 0; i < LCD_HORIZONTAL_MAX - 1; i++) {
            if(buffer[i] >= LCD_HORIZONTAL_MAX){
                y = LCD_HORIZONTAL_MAX - 1;
            }
            else if(buffer[i]<0){
                y = 0;
            }
            else{
                y = buffer[i];
            }
            GrLineDraw(&sContext, i, prev_y, i+1, y);
            prev_y = y;
        }

        GrContextForegroundSet(&sContext, ClrWhite);
                 if(rising){
                     GrLineDraw(&sContext, 105, 10, 115, 10);
                     GrLineDraw(&sContext, 115, 10, 115, 0);
                     GrLineDraw(&sContext, 115, 0, 125, 0);
                     GrLineDraw(&sContext, 112, 6, 115, 2);
                     GrLineDraw(&sContext, 115, 2, 118, 6);
                 }

                 if(!rising){
                     GrLineDraw(&sContext, 105, 10, 115, 10);
                     GrLineDraw(&sContext, 115, 10, 115, 0);
                     GrLineDraw(&sContext, 115, 0, 125, 0);
                     GrLineDraw(&sContext, 112, 3, 115, 7);
                     GrLineDraw(&sContext, 115, 7, 118, 3);
                  }

                 if(!checkTrigger){
                     GrLineDraw(&sContext, 105, 10, 115, 10);
                     GrLineDraw(&sContext, 115, 10, 115, 0);
                     GrLineDraw(&sContext, 115, 0, 125, 0);
                 }


                 GrStringDraw(&sContext, "20 us", -1, 4, 0, false);
                 GrStringDraw(&sContext, gVoltageScaleStr[voltsPerDiv], -1, 50, 0, false);
                 int decimal = (int)cpu_load%10;
                 int integer = (int)cpu_load;
                 snprintf(&str, 40, "CPU Load = %d.%d%%", integer, decimal);
                 GrStringDraw(&sContext, str, -1, 1, 120, false);
                 GrFlush(&sContext);

    }
}

void ButtonTask(void){
    while(1){

    Semaphore_pend(Button_sem, BIOS_WAIT_FOREVER);

    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag

        // read hardware button state
        uint32_t gpio_buttons =
                (~GPIOPinRead(GPIO_PORTJ_BASE, 0xff) & (GPIO_PIN_1 | GPIO_PIN_0)) |
                (~GPIOPinRead(GPIO_PORTH_BASE, 0xff) & (GPIO_PIN_1)) << 1 |
                (~GPIOPinRead(GPIO_PORTK_BASE, 0xff) & (GPIO_PIN_6)) >> 3 |
                (~GPIOPinRead(GPIO_PORTD_BASE, 0xff) & (GPIO_PIN_4)); // EK-TM4C1294XL buttons in positions 0 and 1

        uint32_t old_buttons = gButtons;    // save previous button state
        ButtonDebounce(gpio_buttons);       // Run the button debouncer. The result is in gButtons.
        ButtonReadJoystick();               // Convert joystick state to button presses. The result is in gButtons.
        uint32_t presses = ~old_buttons & gButtons;   // detect button presses (transitions from not pressed to pressed)
        presses |= ButtonAutoRepeat();      // autorepeat presses if a button is held long enough

        static bool tic = false;
        static bool running = true;

        if (presses & 1) { // EK-TM4C1294XL button 1 pressed
            fifo_put('1');
            Mailbox_post(mailbox_button_input, &button, BIOS_WAIT_FOREVER);
        }

        if (presses & 2){
            fifo_put('2');
            Mailbox_post(mailbox_button_input, &button, BIOS_WAIT_FOREVER);
        }

        if (presses & 8){
            fifo_put('3');
            Mailbox_post(mailbox_button_input, &button, BIOS_WAIT_FOREVER);
        }

        if (running) {
            if (tic) gTime++; // increment time every other ISR call
            tic = !tic;
        }
    }
}

void UserInputTask(void){
    while(1){
        Mailbox_pend(mailbox_button_input, &button, BIOS_WAIT_FOREVER);
        while(fifo_get(&button)){
                switch(button){
                case '1':
                    voltsPerDiv = (voltsPerDiv + 1) > 4 ? 4 : (voltsPerDiv + 1);
                    break;
                case '2':
                    voltsPerDiv = (voltsPerDiv -1) <= 0 ? 0 : (voltsPerDiv-1);
                    break;
                case '3':
                    rising = !rising;
                    break;
                }

            }
}
}


void Clock0Task(void){
    Semaphore_post(Button_sem);
}

uint32_t CPULoad(void){
      float i = 0;
      TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
      TimerEnable(TIMER3_BASE,TIMER_A); //start one-shot timer

      while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT)) {
           i++;
       }
        return i;
}



