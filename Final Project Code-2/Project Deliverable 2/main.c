/* 2DX3 Project Deliverable 2
		Bus Speed: 22 MHz 
		
		PN0 / D2 = Measurement Status LED  - flashes each ToF reading
    PN1 / D1 = UART TX Status LED      - single flash at start of TX
    PF4 / D3 = Additional Status LED   - solid ON during CCW unwind
    PF0 / D4 = Scan In Progress LED    - solid ON during CW scan
 
    PJ0 = one 360 degree scan per press 
    PJ1 = send all collected data to PC over UART
		
		all leds flash at reset (+ D3 flash to indicate TOF is working + D2 flash for system ready)
		and all leds flash 3 times for any errors
 */

#include <stdint.h>
#include <stdio.h>
#include "PLL.h"               
#include "SysTick.h"           
#include "uart.h"              
#include "onboardLEDs.h"       
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"       

//I2C REGISTER DEFINES
#define I2C_MCS_ACK      0x00000008
#define I2C_MCS_DATACK   0x00000008
#define I2C_MCS_ADRACK   0x00000004
#define I2C_MCS_STOP     0x00000004
#define I2C_MCS_START    0x00000002
#define I2C_MCS_ERROR    0x00000002
#define I2C_MCS_RUN      0x00000001
#define I2C_MCS_BUSY     0x00000001
#define I2C_MCR_MFE      0x00000010
#define MAXRETRIES       5

//SCAN PARAMETERS
#define MAX_SCANS       25    // maximum scan positions supported
#define NUM_STEPS       32    // ToF readings per 360deg rotation (11.25 deg apart)
#define STEPS_PER_ROT  4096   // 8-step mode: 4096 steps = 360 degrees(half-step: 0.088 deg per step)
#define STEPS_PER_MEAS  128   // motor steps between readings: 4096/32 = 128

// DEMO MODE SWITCH
// DEMO_MODE 1 = for demo 10cm between scans 
// DEMO_MODE 0 = for hallway physically distance between PJ0 presses: REAL_DISPLACEMENT_MM set accordingly
#define DEMO_MODE 1    // 1=demo spec, 0=real hallway
#define REAL_DISPLACEMENT_MM 250  // only used when DEMO_MODE=0, in mm

// AD3 probe on PM0 measures period = 20ms = 50Hz = confirms 22 MHz
#define AD3_DEMO_MODE 0   // 0= scan mode, 1=AD3 bus speed demo

/* 8-STEP HALF-STEP SEQUENCE
  CW:  stepIndex = (stepIndex + 1) % 8  (0->1->2->...->7->0)
  CCW: stepIndex = (stepIndex + 7) % 8  (0->7->6->...->1->0)
       (+7 mod 8 is same as -1 with wrap-around)
 */
uint8_t stepSequence[8] = {
    0x09,  // Orange + Blue(PH3+PH0)
    0x01,  // Blue  only(PH0)    
    0x03,  // Blue  + Pink(PH0+PH1) 
    0x02,  // Pink  only(PH1)    
    0x06,  // Pink  + Yellow(PH1+PH2) 
    0x04,  // Yellow only(PH2)    
    0x0C,  // Yellow+ Orange(PH2+PH3) 
    0x08   // Orange only(PH3)     
};
static int stepIndex = 0;// current position in 8-step sequence (0-7)

//GLOBAL VARIABLES
uint16_t dev   = 0x29;  // VL53L1X I2C address 
int      status = 0;    // I2C/ToF function return status (0=OK, nonzero=error)
uint8_t  tofOK  = 0;   // 1=sensor initialized OK, 0=sensor error

uint16_t scanData[MAX_SCANS][NUM_STEPS]; // all distance readings [scan][step] in mm
int scanNum = 0;                         // number of completed scans

//I2C INITIALIZATION, PB2 = SCL, PB3 = SDA
void I2C_Init(void){ 
    SYSCTL_RCGCI2C_R  |= SYSCTL_RCGCI2C_R0;     // activate I2C0 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;    // activate Port B clock
    while((SYSCTL_PRGPIO_R & 0x0002) == 0){};    // wait for Port B ready

    GPIO_PORTB_AFSEL_R |= 0x0C;   // enable alt function on PB2(SCL), PB3(SDA)
    GPIO_PORTB_ODR_R   |= 0x08;   // PB3 (SDA) open drain - required for I2C
    GPIO_PORTB_DEN_R   |= 0x0C;   // enable digital I/O on PB2, PB3
    GPIO_PORTB_PCTL_R   = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200; // set PB2, PB3 to I2C function
    I2C0_MCR_R  = I2C_MCR_MFE;   // enable I2C master function

    // TPR (timer per register) for 100kHz I2C at 22 MHz bus:
    // TPR = (22,000,000 / (20 * 100,000)) - 1 = 10
    I2C0_MTPR_R = 10;
}

//PORT G INITIALIZATION
void PortG_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;              // activate clock Port G
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0){};   // wait until ready
    GPIO_PORTG_DIR_R  &= 0x00;     // PG0 as input XSHUT pulled up internally
    GPIO_PORTG_AFSEL_R &= ~0x01;   // disable alternate function on PG0
    GPIO_PORTG_DEN_R   |=  0x01;   // enable digital I/O on PG0
    GPIO_PORTG_AMSEL_R &= ~0x01;   // disable analog on PG0
}

//XSHUT TOGGLE
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R  |=  0x01;    // PG0 as output so we can drive it low
    GPIO_PORTG_DATA_R &= 0b11111110; // PG0 = 0 = XSHUT LOW = sensor in reset                
    SysTick_Wait10ms(10);           // hold reset 100ms
    GPIO_PORTG_DIR_R  &= ~0x01;    // PG0 back to input 
}

//PORT J INITIALIZATION
void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;              // enable clock Port J
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0){};   // wait until ready
    GPIO_PORTJ_DIR_R  &= ~0x03;    // PJ0, PJ1 as inputs
    GPIO_PORTJ_DEN_R  |=  0x03;    // enable digital function
    GPIO_PORTJ_PUR_R  |=  0x03;    // pull-up: pins read 1 when not pressed
    GPIO_PORTJ_AFSEL_R &= ~0x03;   // disable alternate functions
    GPIO_PORTJ_AMSEL_R &= ~0x03;   // disable analog mode
}

// PORT M - PM0 as OUTPUT for bus speed demo
void PortM_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};
    GPIO_PORTM_DIR_R  |=  0x01;    // PM0 as OUTPUT for AD3 measurement
    GPIO_PORTM_DIR_R  &= ~0x02;    // PM1 still input
    GPIO_PORTM_DEN_R  |=  0x03;
    GPIO_PORTM_AFSEL_R &= ~0x03;
    GPIO_PORTM_AMSEL_R &= ~0x03;
}

//PORT H INITIALIZATION
void PortH_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;              // enable clock Port H
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){};   // wait until ready
    GPIO_PORTH_DIR_R  |=  0x0F;    // PH0-PH3 (0000 1111) all outputs
    GPIO_PORTH_DEN_R  |=  0x0F;    // enable digital function
    GPIO_PORTH_AFSEL_R &= ~0x0F;   // disable alternate functions
    GPIO_PORTH_AMSEL_R &= ~0x0F;   // disable analog mode
    GPIO_PORTH_DATA_R &= ~0x0F;    // all coils OFF at startup
}

//LED CONTROL FUNCTIONS
// PF4 / D3: solid ON during CCW unwind 
void setStatusLED(uint8_t on){
    if(on) GPIO_PORTF_DATA_R |=  0x10;  // PF4 ON
    else   GPIO_PORTF_DATA_R &= ~0x10;  // PF4 OFF
}

// PF0 / D4: solid ON during entire CW scan phase
void setScanLED(uint8_t on){
    if(on) GPIO_PORTF_DATA_R |=  0x01;  // PF0 ON
    else   GPIO_PORTF_DATA_R &= ~0x01;  // PF0 OFF
}

// BUTTON EDGE DETECTION
uint8_t prevPJ0 = 0;
uint8_t buttonPJ0Pressed(void){
    uint8_t current = !(GPIO_PORTJ_DATA_R & 0x01); // active low
				// ! - flips the logic (GPIO statement reads bit 0(PJ0) which gives 1 when not pressed and 0 when pressed)
    uint8_t pressed = current & !prevPJ0; // return 1 on rising edge (as soon as button pressed)
    prevPJ0 = current; // save new state
    if(pressed) SysTick_Wait10ms(5); // 50ms debounce
    return pressed;
}

uint8_t prevPJ1 = 0;
uint8_t buttonPJ1Pressed(void){
    uint8_t current = !(GPIO_PORTJ_DATA_R & 0x02); 
    uint8_t pressed = current & !prevPJ1;
    prevPJ1 = current;
    if(pressed) SysTick_Wait10ms(5);
    return pressed;
}

//STEPPER MOTOR STEP FUNCTIONS
void stepCW(void){
    GPIO_PORTH_DATA_R = (GPIO_PORTH_DATA_R & ~0x0F) | stepSequence[stepIndex];
    stepIndex = (stepIndex + 1) % 8;  // advance forward through array
    SysTick_Wait10ms(2); // 20ms per step
}

void stepCCW(void){
    GPIO_PORTH_DATA_R = (GPIO_PORTH_DATA_R & ~0x0F) | stepSequence[stepIndex];
    stepIndex = (stepIndex + 7) % 8;  // advance backward: +7 mod 8 = -1
    SysTick_Wait(22000);  // 1 ms at 22 MHz for faster unwind
}

// DATA TRANSMISSION
void sendAllData(void){
    int sc, st; 
		
    // PN1 single flash = transmission block starting 	
		FlashLED1(1);

    // header: tells MATLAB how many scans were collected
    sprintf(printf_buffer, "SCANS:%d\r\n", scanNum);
    UART_printf(printf_buffer);

    // send all data lines 
    for(sc = 0; sc < scanNum; sc++){
        for(st = 0; st < NUM_STEPS; st++){

            // x position calculation:
            #if DEMO_MODE
                int x_mm = sc * 100; // pretend 10cm apart
            #else
                int x_mm = sc * REAL_DISPLACEMENT_MM; // real distance
            #endif

            sprintf(printf_buffer, "%d, %d, %d, %d\r\n",
                    sc,                // scan number (0, 1, 2...)
                    st,                // step number (0 to 31)
                    scanData[sc][st],  // distance in mm (0 if ToF error)
                    x_mm);             // x displacement in mm

            UART_printf(printf_buffer);
        }
    }

    UART_printf("END\r\n"); // MATLAB reads until this marker

}

//AD3 Demo mode
void runAD3Demo(void){
    while(1){
        GPIO_PORTM_DATA_R |=  0x01;   // PM0 HIGH
        SysTick_Wait10ms(1);          // 10ms ON  (220000 cycles at 22 MHz)
        GPIO_PORTM_DATA_R &= ~0x01;   // PM0 LOW
        SysTick_Wait10ms(1);          // 10ms OFF (220000 cycles at 22 MHz)
        // result: 20ms period = 50Hz square wave on AD3
    }
	}

// MAIN FUNCTION
int main(void){

    uint8_t sensorState = 0;  // ToF boot state: 0=not booted, 1=booted
    uint16_t wordData;        // temp storage for sensor ID read
    uint8_t dataReady;        // ToF data ready flag: 1=measurement available
    uint16_t Distance;        // distance reading from ToF in mm
    int i;                    // loop counter

    PLL_Init();
    SysTick_Init();

    onboardLEDs_Init();
	
    PortJ_Init();  
    PortM_Init();  
    PortH_Init(); 
    PortG_Init(); 
    UART_Init();   
    I2C_Init();    

    GPIO_PORTH_DATA_R = 0x00; // motor coils all off
    setScanLED(0);
    setStatusLED(0);

    // startup message
    UART_printf("Program Begins\r\n");
    sprintf(printf_buffer, "2DX3 D2 Spatial Mapper | Student: Anushka Chauhan \r\n");
    UART_printf(printf_buffer);

    // TOF SENSOR INITIALIZATION
    VL53L1X_XSHUT();
    SysTick_Wait10ms(10); 

    // step 1: read sensor ID - confirms I2C bus is working
    // VL53L1X should return 0xEACC
    status = VL53L1X_GetSensorId(dev, &wordData);
    sprintf(printf_buffer, "(Model_ID, Module_Type)=0x%x\r\n", wordData);
    UART_printf(printf_buffer);

    // step 2: wait for device to finish booting
    // poll BootState until it returns 1
    // added timeout so system doesn't hang forever if sensor not found
    UART_printf("Waiting for ToF boot...\r\n");
    int bootTimeout = 0;
    while(sensorState == 0 && bootTimeout < 200){ // max 200 x 100ms = 20 seconds
        status = VL53L1X_BootState(dev, &sensorState);
        SysTick_Wait10ms(10); // check every 100ms
        bootTimeout++;
    }

    if(sensorState == 0){
        // sensor did not boot 
        UART_printf("ToF ERROR: boot timeout");
        UART_printf("System continues: distances stored as 0\r\n");
        FlashAllLEDs(); // error indicator with 3 flashes
        FlashAllLEDs();
        FlashAllLEDs();
        tofOK = 0;      // flag: no ToF, store 0 for all distances
        // system continues to work without ToF
    } else {
        // sensor booted successfully 
        FlashAllLEDs(); //FlashAllLEDs after boot confirmed
        UART_printf("ToF Chip Booted!\r\nPlease Wait...\r\n");

        // step 3: clear interrupt - required before SensorInit 
				VL53L1_WaitMs(dev, 50);
        // step 4: initialize sensor with default configuration
        // writes all default register values 
        status = VL53L1X_SensorInit(dev);
        Status_Check("SensorInit", status); 

        if(status != 0){
            UART_printf("SensorInit failed - distances will be 0\r\n");
            FlashAllLEDs();// error indicator with 3 flashes
						FlashAllLEDs();
						FlashAllLEDs();
            tofOK = 0;
        } else {
            // step 5: start continuous ranging
            status = VL53L1X_StartRanging(dev);
            Status_Check("StartRanging", status);

            if(status != 0){
                UART_printf("StartRanging failed - distances will be 0\r\n");
                FlashAllLEDs(); // error indicator with 3 flashes
								FlashAllLEDs();
								FlashAllLEDs();
                tofOK = 0;
            } else {
                tofOK = 1; // sensor fully ready
                UART_printf("ToF sensor ready!\r\n");

							// PF4 / D3 flashes: sensor OK indicator
                FlashLED3(10);
                
            }
        }
    }

    // PN0 / D2 flashes = system fully ready, press PJ0 now
    FlashLED2(10);

    // print final status
    if(tofOK){
        UART_printf("Ready: PJ0=scan, PJ1=send | ToF OK\r\n");
    } else {
        UART_printf("Ready: PJ0=scan, PJ1=send | ToF ERROR (dist=0)\r\n");
    }

    // ensure all LEDs off before entering main loop
    setScanLED(0);
    setStatusLED(0);

    int totalSteps; // steps taken in current rotation
    int readCount;  // readings taken in current scan
	
		#if AD3_DEMO_MODE
    UART_printf("AD3 DEMO MODE: PM0 toggling at 50Hz\r\n");
    UART_printf("Connect AD3 probe to PM0, measure period=20ms\r\n");
    runAD3Demo(); // loops forever, never reaches while(1) below
		 #endif
    
    while(1){

        if(buttonPJ0Pressed()){

            if(scanNum >= MAX_SCANS){
                // array full - flash all LEDs as warning
                UART_printf("Array full! Press PJ1 to send.\r\n");
                FlashAllLEDs();
                FlashAllLEDs();

            } else {

                sprintf(printf_buffer, "\r\n--- Scan %d starting ---\r\n",
                        scanNum + 1);
                UART_printf(printf_buffer);

                //clockwise scan
                setScanLED(1);   // D4 solid ON = scan phase active
                totalSteps = 0;  // reset step counter for this rotation
                readCount  = 0;  // reset reading counter

                UART_printf("Scanning CW...\r\n");

                // rotate 4096 steps = full 360 degrees CW
                for(i = 0; i < STEPS_PER_ROT; i++){

                    stepCW();      // one 8-step CW step, 20ms delay inside
                    totalSteps++;

                    // take ToF reading every 128 steps = 11.25 degrees
                    if(totalSteps % STEPS_PER_MEAS == 0 && readCount < NUM_STEPS){

											// stop motor briefly before measuring for accuracy
											GPIO_PORTH_DATA_R &= ~0x0F;   // turn OFF coils (stop motion)
											SysTick_Wait10ms(5);          // 50ms settle time

											if(tofOK){
													dataReady = 0;

													// wait until sensor is ready
													int retries = 0;
													while(dataReady == 0 && retries < MAXRETRIES){
															status = VL53L1X_CheckForDataReady(dev, &dataReady);
															VL53L1_WaitMs(dev, 5);
															retries++;
													}

													// get distance
													status = VL53L1X_GetDistance(dev, &Distance);
													status = VL53L1X_ClearInterrupt(dev);

													scanData[scanNum][readCount] = (dataReady) ? Distance : 0;
											} else {
													scanData[scanNum][readCount] = 0;
											}

											// re-energize coils at current stepIndex before continuing CW rotation
											GPIO_PORTH_DATA_R = (GPIO_PORTH_DATA_R & ~0x0F) | stepSequence[stepIndex];
											SysTick_Wait10ms(2);          // 20ms settle before resuming steps

											// measurement LED flash
											FlashLED2(1);

											sprintf(printf_buffer, "  [%d/32] %d mm\r\n",
															readCount + 1, scanData[scanNum][readCount]);
											UART_printf(printf_buffer);

											readCount++;
									}
																			}
                // scan complete
                setScanLED(0);               // D4 OFF = scan done
                GPIO_PORTH_DATA_R &= ~0x0F; // de-energize coils before reversing
                SysTick_Wait10ms(30);        // 300ms settle time

                //PHASE 2: CCW UNWIND BACK TO HOME
                UART_printf("Unwinding CCW...\r\n");
                setStatusLED(1); // D3 solid ON = unwinding

                // return exactly 4096 steps CCW = back to home position
                for(i = 0; i < STEPS_PER_ROT; i++){
                    stepCCW();
                }

                GPIO_PORTH_DATA_R &= ~0x0F; // de-energize coils at home
                setStatusLED(0);            // D3 OFF = motor at home position

                totalSteps = 0; // reset for next scan
                scanNum++;      // count this completed scan

                sprintf(printf_buffer,
                        "Scan %d done. PJ0=next scan | PJ1=send data\r\n",
                        scanNum);
                UART_printf(printf_buffer);
            }
        }

        
        if(buttonPJ1Pressed()){

            if(scanNum == 0){
                // no scans done: all leds flashes 3x = no data warning
                UART_printf("No scans yet! Press PJ0 first.\r\n");
                FlashAllLEDs();
								FlashAllLEDs();
								FlashAllLEDs();

            } else {
                sprintf(printf_buffer, "Sending %d scans to PC...\r\n", scanNum);
                UART_printf(printf_buffer);

                sendAllData(); 

                UART_printf("Done! Run MATLAB script to plot.\r\n");
                scanNum = 0;   // reset for new session
            }
        }
				
    }
	}