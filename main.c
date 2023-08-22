/*
 * main.c
 *
 * Created: 2023/08/09 for
 *  Author: skyler
 */ 

//#define F_CPU 4000000UL
#define F_CPU 16000000UL
//#define F_CPU 16000000UL 
#define MAIN_CLOCK_FREQ CLKCTRL_FRQSEL_16M_gc


#include <xc.h>
#include <stdint.h>
#include <util/delay.h>
#include <stdbool.h>
#include <math.h> 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/cpufunc.h>
#include "petitfs/pff.h"
#include "petitfs/diskio.h"

#define N_BAT 5
#define V_HIGH 2.7
#define V_LOW 2.0
//#define V_CHECK_PERIOD (10) //period to check voltages in ms, also period to switch fuses or not (must be less than 32.7ms)
#define PROCESS_VOLTAGES_PERIOD 500 //period to process all voltages and switch MOSFETS on ovrchrg or ovrdischrg conditions
#define V_CUTOFF 6 //voltage below which not to switch the MOSFETS
#define R_SHUNT 0.005 // resistance of shunt resistors in ohms
#define I_CHARGE_FUSE 65 // max charge current before charge port auto shut off
#define I_CHARGE_FUSE2 25 //lower fuse
#define I_LOAD_FUSE 65 // max discharge / load current before load port auto shut off
#define I_LOAD_FUSE2 25 //lower fuse
#define I_CHARGE_BACKCURRENT_FUSE (-25) // back flow current cutoff fuse level on the charge port
#define I_CHARGE_BACKCURRENT_FUSE2 (-1.5)
#define FUSE2_period 5 //fuse 2 period in ms
#define SAMPLE_PERIOD (10) // Write voltages every X seconds


// Fuse compare values
#define I_CHARGE_FUSE_COMP_VAL	(I_CHARGE_FUSE*R_SHUNT*ADC_MAX_VALUE * ADC_ACCUMULATIONS_fuses_integer / VOLTAGE_REF)
#define I_CHARGE_FUSE2_COMP_VAL	(I_CHARGE_FUSE2*R_SHUNT*ADC_MAX_VALUE * ADC_ACCUMULATIONS_fuses_integer / VOLTAGE_REF)
#define I_CHARGE_BACKCURRENT_FUSE_COMP_VAL	(I_CHARGE_BACKCURRENT_FUSE*R_SHUNT*ADC_MAX_VALUE * ADC_ACCUMULATIONS_fuses_integer / VOLTAGE_REF)
#define I_CHARGE_BACKCURRENT_FUSE2_COMP_VAL	(I_CHARGE_BACKCURRENT_FUSE2*R_SHUNT*ADC_MAX_VALUE * ADC_ACCUMULATIONS_fuses_integer / VOLTAGE_REF)
#define I_LOAD_FUSE_COMP_VAL	(I_LOAD_FUSE*R_SHUNT*ADC_MAX_VALUE * ADC_ACCUMULATIONS_fuses_integer / VOLTAGE_REF)
#define I_LOAD_FUSE2_COMP_VAL	(I_LOAD_FUSE2*R_SHUNT*ADC_MAX_VALUE * ADC_ACCUMULATIONS_fuses_integer / VOLTAGE_REF)


// Timer variables: timer A = process voltages every PROCESS_VOLTAGES_PERIOD ms
#define TIMER_A_INTERRUPT_MS PROCESS_VOLTAGES_PERIOD
#define TIMER_A_PRESCALER 1024
#define TIMER_A_COMPARE_VALUE ((F_CPU / TIMER_A_PRESCALER / 1000) * TIMER_A_INTERRUPT_MS)


// Timer variables: timer B0 = takes a ADC reading every V_CHECK_PERIOD ms
#define TIMER_B0_INTERRUPT_MS FUSE2_period
#define TIMER_B0_PRESCALER 2
#define TIMER_B0_COMPARE_VALUE ((F_CPU / TIMER_B0_PRESCALER / 1000) * TIMER_B0_INTERRUPT_MS)


/*
// Timer variables: timer B1 = checks if charge fuse is blown and switches C-MOSFET every 10 ms
#define TIMER_B1_INTERRUPT_MS 10
#define TIMER_B1_PRESCALER 2
#define TIMER_B1_COMPARE_VALUE ((F_CPU / TIMER_B1_PRESCALER / 1000) * TIMER_B1_INTERRUPT_MS)

// Timer variables: timer B2 = checks if load fuse is blown and switches D-MOSFET every 10 ms
#define TIMER_B2_INTERRUPT_MS 10
#define TIMER_B2_PRESCALER 2
#define TIMER_B2_COMPARE_VALUE ((F_CPU / TIMER_B2_PRESCALER / 1000) * TIMER_B2_INTERRUPT_MS)
*/


// ADC Related Variables --------------------------------------------------------------
//#define ADC_OFFSET_LSb 100 //LSb offset error of ADC
#define ADC_MUXPOS_AIN23_gc (0x17<<0)
#define ADC_MUXPOS_AIN24_gc (0x18<<0)
#define ADC_MUXPOS_AIN25_gc (0x19<<0)
#define ADC_MUXPOS_AIN26_gc (0x1A<<0)

#define B_GND_CNTRL PORTD_PIN1CTRL
#define B1_DIV_CNTRL PORTD_PIN2CTRL
#define B2_DIV_CNTRL PORTD_PIN3CTRL
#define B3_DIV_CNTRL PORTD_PIN4CTRL
#define B4_DIV_CNTRL PORTD_PIN5CTRL
#define B5_DIV_CNTRL PORTD_PIN6CTRL
#define V1_NEG_CNTRL PORTA_PIN3CTRL
#define V1_POS_CNTRL PORTA_PIN4CTRL
#define V2_NEG_CNTRL PORTA_PIN5CTRL
#define V2_POS_CNTRL PORTA_PIN6CTRL

#define B0R_MUX ADC_MUXPOS_AIN1_gc
#define B1_DIV_MUX ADC_MUXPOS_AIN2_gc
#define B2_DIV_MUX ADC_MUXPOS_AIN3_gc
#define B3_DIV_MUX ADC_MUXPOS_AIN4_gc
#define B4_DIV_MUX ADC_MUXPOS_AIN5_gc
#define B5_DIV_MUX ADC_MUXPOS_AIN6_gc
#define V1_NEG_MUX ADC_MUXPOS_AIN23_gc
#define V1_POS_MUX ADC_MUXPOS_AIN24_gc
#define V2_NEG_MUX ADC_MUXPOS_AIN25_gc
#define V2_POS_MUX ADC_MUXPOS_AIN26_gc

#define ADC_MAX_VALUE   ((1 << 11) - 1) /* In differential, the max value is (2^12)/2-1 */
#define NUM_SAMPLES 1 // number of samples to avg ADC reading (samples are accumulated samples already) 
#define ADC_ACCUMULATIONS_cells ADC_SAMPNUM_ACC8_gc // number of accumulations of the ADC0
#define ADC_ACCUMULATIONS_cells_integer 8 // number of accumulations of the ADC0
#define ADC_ACCUMULATIONS_fuses ADC_SAMPNUM_NONE_gc // number of accumulations of the ADC0
#define ADC_ACCUMULATIONS_fuses_integer 1  // number of accumulations of the ADC0
#define ADC_sample_length_cells  ADC_SAMPLEN5_bm //length to add to every ADC sample
#define ADC_sample_length_fuses ADC_SAMPLEN0_bm // length to add oto every ADC sample when checking uses
#define vDiv (15.000/(15.000+221.000))
#define VOLTAGE_REF 1.024
//static volatile float VOLTAGE_REF;
static volatile float adc_reading;
static volatile int16_t adc_reading_shunt;
static volatile float V1_voltage;
static volatile float V2_voltage;
static volatile float V1_voltage_ini;
static volatile float V2_voltage_ini;
static volatile float I_charge;
static volatile float I_load;
static volatile float bat0_voltage;
static volatile float bat1_voltage;
static volatile float bat2_voltage;
static volatile float bat3_voltage;
static volatile float bat4_voltage;
static volatile float bat5_voltage;
static volatile float bat_tot_voltage;
static volatile float ADC_accumuations_rightnow;
//static volatile float vDiv;
static volatile float voltages[6];
static volatile float bat_voltages[6];
static volatile bool charge_fuse2_prev_blown;
static volatile bool load_fuse2_prev_blown;
static volatile bool backcurrent_fuse2_prev_blown;


// ADC offset voltages
static volatile float V1_voltage_offset;
static volatile float V2_voltage_offset;
static volatile float bat0_voltage_offset;




// end ADC related variables -----------------------------------------------------------


// Define LED ports
#define LEDPORT PORTF
#define eject_LEDPIN PIN5_bm
#define data_LEDPIN PIN4_bm
#define eject_LED_OFF() (LEDPORT.OUTCLR = eject_LEDPIN)
#define eject_LED_ON() (LEDPORT.OUTSET = eject_LEDPIN)
#define data_LED_OFF() (LEDPORT.OUTCLR = data_LEDPIN)
#define data_LED_ON() (LEDPORT.OUTSET = data_LEDPIN)

// Define Master slave variables
#define MSTR_SLV_SEL_PORT PORTA
#define MSTR_SLV_SEL_PIN PIN7_bm
#define MSTR_SLV_SEL_CTRL_PIN PIN7CTRL
#define SLAVE_C_MOSFET_PORT PORTC
#define SLAVE_C_MOSFET_PIN PIN3_bm
#define SLAVE_C_MOSFET_CTRL_PIN PIN3CTRL
#define SLAVE_D_MOSFET_PORT PORTC
#define SLAVE_D_MOSFET_PIN PIN2_bm
#define SLAVE_D_MOSFET_CTRL_PIN PIN2CTRL



#define USE_INTERNAL_OSC true

typedef struct {
	uint8_t  second;
	uint8_t  minute;
	uint8_t  hour;
	uint8_t  date;
	uint8_t  month;
	uint16_t year;
	uint32_t totSeconds;
} time;


/* File system object */
FATFS file_system;

volatile time        t;
volatile static bool tick = false;

void    init_sd_card(void);
void    init_io(void);
void    init_rtc(void);
void	init_rtc_xtal(void);
void    send_buffer(void);
uint8_t not_a_leap_year(void);
void    process_start_time(void);
void    tock(void);
void    update_time(void);
void    process_voltages(void);
void	read_voltages(bool);
void	charger_load_init(void); 
void	overcharged(void) ;
void	overdischarged(void) ;
void	not_overcharged(void);
void	not_overdischarged(void) ;
//uint8_t LED_CHECK(void);
void	adc_init(void);
double ADC_read_avg(void);
float ADC_accumu_read(void);
bool SD_detect(void);
bool button_pushed(void);
void set_MSTR_SLV(void);
void write_header_SD(void);
void SD_eject(void);
void check_result(FRESULT result);
void timer_A_init(void);
void timer_B0_init(void);
void timer_B1_init(void);
void timer_B2_init(void);
int16_t ADC_fuse_read(void);
//void send_endline(void);


static volatile bool overcharged_state;
static volatile bool overdischarged_state;
static volatile uint8_t LED_status;
static volatile bool SD_ejected;
static volatile bool SD_start_enable;
static volatile bool charge_fuse_blown;
static volatile bool load_fuse_blown;
static volatile bool is_slave;
static volatile bool slave_overdischarged; // flag for is slave overdischarged
static volatile bool slave_overcharged; // flag for is slave overcharged 
//static volatile bool debug1;
FRESULT fresultDebug1;
FRESULT fresultDebug2;
FRESULT fresultDebug3;
FRESULT fresultDebug4;
DSTATUS statusDebug1;
DSTATUS statusDebug2;
DSTATUS statusDebug3;
DSTATUS statusDebug4;
bool boolDebug2;



/* Buffer */
#define BUFFER_SIZE 200
#define HEADER2_SIZE 200
#define BUFFER1_TEMPLATE "\r\n%.0f,%.0f.%.0f.%.0f,%.0f:%.0f:%.0f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.0f,%.0f,%.3f,%.3f,"
char buffer[BUFFER_SIZE + 1];
char endline1[] = "\rSD HAS BEEN EJECTED,";
char header1[] = "========== 5S D-MBMS dv7 / Firmware version: 230821 /  AVR64DD32 ==========";
char header2[HEADER2_SIZE+1];
char header3[] = "\r\nTotal sec,Date(Y.M.D),Time(H:M:S),Cell 1,Cell 2,Cell 3,Cell 4,Cell 5,Total,OvrChg,OvrDischg,I_charge,I_discharge";
char filename[] = "log.csv";



int main(void)
{
	statusDebug1 = 1;
	SD_eject();
	SD_ejected = true;
	SD_start_enable = true;
	charge_fuse_blown = false;
	load_fuse_blown = false;
	load_fuse2_prev_blown = false;
	charge_fuse2_prev_blown = false;
	backcurrent_fuse2_prev_blown = false;
	is_slave = true; 
	slave_overdischarged = false;
	slave_overcharged = false;
	CPU_CCP = CCP_IOREG_gc; // write to a protected register
	CLKCTRL.OSCHFCTRLA = MAIN_CLOCK_FREQ;

	sprintf(header2,"\rV_HIGH,%.2f,V,\rV_LOW,%.2f,V,\rV_CUTOFF,%.2f,V,\rPROCESS_V_PERIOD,%.2f,ms,\rSAMPLE_PERIOD,\t%.2f,\ts,",
		(float)V_HIGH,
		(float)V_LOW,
		(float)V_CUTOFF,
		(float)PROCESS_VOLTAGES_PERIOD,
		(float)SAMPLE_PERIOD);
	
	init_io();
	init_rtc_xtal();
	adc_init(); 
	charger_load_init();
	ADC_accumuations_rightnow = (float)ADC_ACCUMULATIONS_cells_integer;
	
	// Correct for ADC offset voltages
	TCA0.SINGLE.INTCTRL &= ~TCA_SINGLE_CMP0_bm; // disable TCA0 interrupt
	TCB0.INTCTRL &= ~TCB_CAPT_bm; // disable TCB0 interrupt
	read_voltages(true);
	TCB0.INTCTRL = TCB_CAPT_bm; // re enable TCB0 interrupt
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm; // re enable TCA0 interrupt
	V1_voltage_offset = V1_voltage;
	V2_voltage_offset = V2_voltage;
	bat0_voltage_offset = bat0_voltage;
	
	
	// Initialize peripherals and external HW
	timer_A_init();
	timer_B0_init();
	process_start_time(); // initial time stamp
	sei(); // Enable global interrupts
	

    while(1)
    {
		// Handle Fuses -------------------------------------------------------------------------------------------
		TCA0.SINGLE.INTCTRL &= ~TCA_SINGLE_CMP0_bm; // disable TCA0 interrupt
		TCB0.INTCTRL &= ~TCB_CAPT_bm; // disable TCB0 interrupt
		
		ADC0.SAMPCTRL = ADC_sample_length_fuses; // try to reduce noise by increasing sample length
		ADC0.CTRLB = ADC_ACCUMULATIONS_fuses; // Set ADC to accumulation mode with 16 accumulations
				
		
		//V1 Voltage differential (charge shunt)
		ADC0.MUXNEG = V1_NEG_MUX;
		ADC0.MUXPOS = V1_POS_MUX;
		adc_reading_shunt = ADC_fuse_read();
		
		if(adc_reading_shunt < I_CHARGE_BACKCURRENT_FUSE_COMP_VAL && charge_fuse_blown==false) {
			overcharged();
			charge_fuse_blown = true;
		}
		
		else if(adc_reading_shunt > I_CHARGE_FUSE_COMP_VAL && charge_fuse_blown==false) {
			overcharged();
			charge_fuse_blown = true;
		}
		
		//V2 Voltage differential (load shunt)
		ADC0.MUXNEG = V2_NEG_MUX;
		ADC0.MUXPOS = V2_POS_MUX;
		adc_reading_shunt = ADC_fuse_read();         // Read ADC sample Calculate voltage on ADC pin, VDD = 3.3V, 12-bit resolution
		if(adc_reading_shunt > I_LOAD_FUSE_COMP_VAL && load_fuse_blown==false) {
			load_fuse_blown = true;
			overdischarged();
		}

		
		ADC0.SAMPCTRL = ADC_sample_length_cells; // try to reduce noise by increasing sample length
		ADC0.CTRLB = ADC_ACCUMULATIONS_cells; // Set ADC to accumulation mode
		
		
		TCB0.INTCTRL = TCB_CAPT_bm; // re enable TCB0 interrupt
		TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm; // re enable TCA0 interrupt
		//end Handle Fuses ----------------------------------------------------------------------------------------


			
		// If the board is a master, read the slave's state and respond
		if(!is_slave) { // if the board is a master
			if(SLAVE_C_MOSFET_PORT.IN & SLAVE_C_MOSFET_PIN) // if slave board C-MOSFET is 'on', the slave board is not overcharged
				slave_overcharged = false;
			else { // if slave board C-MOSFET is 'off', the slave board is overcharged
				overcharged();
				slave_overcharged = true;
			}
				
			if(SLAVE_D_MOSFET_PORT.IN & SLAVE_D_MOSFET_PIN) // if slave board D-MOSFET is 'on', the slave board is not overdischarged
			slave_overdischarged = false;
			else { // if slave board D-MOSFET is 'off', the slave board is overdischarged
				overdischarged();
				slave_overdischarged = true;
			}
				
		}
		
		
		
    }
}


ISR(RTC_CNT_vect)
{
	/* Raise flag for tock routine */
	tock();

	/* Clear interrupt flag */
	RTC.INTFLAGS = RTC_OVF_bm;
}


ISR(TCA0_CMP0_vect) { 
	// Timer A interrupt: to process voltages every PROCESS_VOTLAGES_PERIOD
	
	//Voltages {bat0_voltage},{bat1_voltage},{bat2_voltage},{bat3_voltage},{bat4_voltage},{bat5_voltage};V1 = {V1_voltage}, V2 = {V2_voltage}, I_charge={I_charge}, I_load={I_load}
	//Voltages V1pos={V1_pos_voltage},V1neg={V1_neg_voltage},    V2pos={V2_pos_voltage},V2neg={V2_neg_voltage},     V1 = {V1_voltage}, V2 = {V2_voltage},     I_charge={I_charge}, I_load={I_load}
	//{bat0_voltage},{bat1_voltage},{bat2_voltage},{bat3_voltage},{bat4_voltage},{bat5_voltage};         Voltages V1pos={V1_pos_voltage},V1neg={V1_neg_voltage},    V2pos={V2_pos_voltage},V2neg={V2_neg_voltage},     V1 = {V1_voltage}, V2 = {V2_voltage},     I_charge={I_charge}, I_load={I_load}
	//read_shunt_voltages();
	//read_battery_voltages();
	
	
	//SD card =====================================================
	boolDebug2 = button_pushed();
	if(button_pushed() && !SD_ejected) { // if button pushed and SD is not ejected, eject SD and disable resarting data recording (disable until sd is phsycially removed)
		//send_endline();
		SD_eject();
		SD_start_enable = false;
	}
	if(SD_detect() && SD_ejected && SD_start_enable) { // if SD is detected and start is enabled, it has just been inserted
		init_sd_card();
		//SD_start_enable = false;
	}
	if(!SD_detect() && SD_ejected && !SD_start_enable) { // if SD is ejected and not in the socket, it has been safely removed from socket ( re-enable the start_enable flag)
		SD_start_enable = true;
	}
	if(!SD_detect() && !SD_ejected) { // if SD card is not ejected and is not detected, it has bee incorrectly removed
		SD_eject();
		SD_start_enable = false;
	}
	//SD card END ================================================
	
	// detect if the board is a master or if is a slave
	set_MSTR_SLV(); 
	
	//Reset fuses
	charge_fuse_blown = false;
	load_fuse_blown = false;
	
	
	TCB0.INTCTRL &= ~TCB_CAPT_bm; // disable TCB0 interrupt
	read_voltages(true);
	TCB0.INTCTRL = TCB_CAPT_bm; // re enable TCB0 interrupt
	
	process_voltages();
	


	// Clear the interrupt flag
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;
}


ISR(TCB0_INT_vect) {
	// Timer B0 interupt: handle fuse2
	
	TCA0.SINGLE.INTCTRL &= ~TCA_SINGLE_CMP0_bm; // disable TCA0 interrupt
	
	ADC0.SAMPCTRL = ADC_sample_length_fuses; // try to reduce noise by increasing sample length
	ADC0.CTRLB = ADC_ACCUMULATIONS_fuses; // Set ADC to accumulation mode with 16 accumulations
	
	
	//V1 Voltage differential (charge shunt)
	ADC0.MUXNEG = V1_NEG_MUX;
	ADC0.MUXPOS = V1_POS_MUX;
	adc_reading_shunt = ADC_fuse_read();
	 // I_charge_fuse2
	if(adc_reading_shunt > I_CHARGE_FUSE2_COMP_VAL && charge_fuse2_prev_blown==true &&  charge_fuse_blown==false) {
		overcharged();
		charge_fuse_blown = true;
		charge_fuse2_prev_blown = false;
	} 
	else if(adc_reading_shunt > I_CHARGE_FUSE2_COMP_VAL && charge_fuse2_prev_blown==false &&  charge_fuse_blown==false) {
		charge_fuse2_prev_blown = true;
	}
	else if(adc_reading_shunt < I_CHARGE_FUSE2_COMP_VAL &&  charge_fuse_blown==false) {
		charge_fuse2_prev_blown = false;
	}
	//I_charge_backcurrent_fuse2
	if(adc_reading_shunt < I_CHARGE_BACKCURRENT_FUSE2_COMP_VAL && backcurrent_fuse2_prev_blown==true &&  charge_fuse_blown==false) {
		overcharged();
		charge_fuse_blown = true;
		backcurrent_fuse2_prev_blown = false;
	}
	else if(adc_reading_shunt < I_CHARGE_BACKCURRENT_FUSE2_COMP_VAL && backcurrent_fuse2_prev_blown==false &&  charge_fuse_blown==false) {
		backcurrent_fuse2_prev_blown = true;
	}
	else if(adc_reading_shunt < I_CHARGE_BACKCURRENT_FUSE2_COMP_VAL &&  backcurrent_fuse2_prev_blown==false) {
		backcurrent_fuse2_prev_blown = false;
	}
	
	//V2 Voltage differential (load shunt)
	ADC0.MUXNEG = V2_NEG_MUX;
	ADC0.MUXPOS = V2_POS_MUX;
	adc_reading_shunt = ADC_fuse_read();         // Read ADC sample Calculate voltage on ADC pin, VDD = 3.3V, 12-bit resolution
	if(adc_reading_shunt > I_LOAD_FUSE2_COMP_VAL && load_fuse2_prev_blown==true && load_fuse_blown==false) {
		overdischarged();
		load_fuse2_prev_blown = false;
		load_fuse_blown = true;
	}
	if(adc_reading_shunt > I_LOAD_FUSE2_COMP_VAL && load_fuse2_prev_blown==false &&  load_fuse_blown==false) {
		load_fuse2_prev_blown = true;
	}
	else if(adc_reading_shunt < I_LOAD_FUSE2_COMP_VAL &&  load_fuse_blown==false) {
		load_fuse2_prev_blown = false;
	}
	

	
	ADC0.SAMPCTRL = ADC_sample_length_cells; // try to reduce noise by increasing sample length
	ADC0.CTRLB = ADC_ACCUMULATIONS_cells; // Set ADC to accumulation mode
	
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm; // re enable TCA0 interrupt
	
	
	
	// Clear the interrupt flag
	TCB0.INTFLAGS = TCB_CAPT_bm;
}


/*
ISR(TCB1_INT_vect) {
	// Timer B1 interrupt routine:  throw C-MOSFET if charge fuse is blown and wait FUSE_TIME ms
	if(I_charge > I_CHARGE_FUSE && charge_fuse_blown==false) {
		charge_fuse_blown = true;
		overcharged();
	}
		
	// Clear the interrupt flag
	TCB1.INTFLAGS = TCB_CAPT_bm;
}

ISR(TCB2_INT_vect) {
	// Timer B2 interrupt routine:  throw D-MOSFET if load fuse is blown and wait FUSE_TIME ms
	if(I_load > I_LOAD_FUSE && load_fuse_blown==false) {
		load_fuse_blown = true;
		overdischarged();
	}
		
	// Clear the interrupt flag
	TCB2.INTFLAGS = TCB_CAPT_bm;
}
*/


void timer_A_init(void) {
	// Timer A init: to process voltages every PROCESS_VOTLAGES_PERIOD
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1024_gc | TCA_SINGLE_ENABLE_bm ; // Set prescaler to 8 and Enable timer
	// Configure compare value
	TCA0.SINGLE.PER = TIMER_A_COMPARE_VALUE - 1;
	// Enable Compare Match interrupt
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm;
}


void timer_B0_init(void) {
	// Timer B init: to read voltages every "V_CHECK_PERIOD" ms
	TCB0.CCMP = TIMER_B0_COMPARE_VALUE - 1;
	TCB0.CTRLA = TCB_ENABLE_bm	| TCB_CLKSEL_DIV2_gc;
	TCB0.INTCTRL = TCB_CAPT_bm; 
}

/*
void timer_B1_init(void) {
	// Timer B1 init: throw C-MOSFET if charge fuse is blown and wait FUSE_TIME ms
	TCB1.CCMP = TIMER_B1_COMPARE_VALUE - 1;
	TCB1.CTRLA = TCB_ENABLE_bm	| TCB_CLKSEL_DIV2_gc;
	TCB1.INTCTRL = TCB_CAPT_bm;
}

void timer_B2_init(void) {
	// Timer B2 init: throw D-MOSFET if charge fuse is blown and wait FUSE_TIME ms
	TCB2.CCMP = TIMER_B2_COMPARE_VALUE - 1;
	TCB2.CTRLA = TCB_ENABLE_bm	| TCB_CLKSEL_DIV2_gc;
	TCB2.INTCTRL = TCB_CAPT_bm;
}
*/




void tock(void) {
	update_time();
	
	if (SD_detect() && !SD_ejected) { //if SD is not ejected and is detected, write another data line
		data_LED_ON();
		send_buffer(); // Send data buffer
		data_LED_OFF();
	}
	
	
	
}

void update_time()
{
	/* Increment seconds by count value */
	t.second += SAMPLE_PERIOD;
	
	//Increment total amount of seconds
	t.totSeconds += SAMPLE_PERIOD;

	/* Need minute increment? */
	if (t.second >= 60) {
		t.second -= 60;
		/* Need hour increment? */
		if (++t.minute == 60) {
			t.minute = 0;
			/* Need day increment? */
			if (++t.hour == 24) {
				t.hour = 0;
				/* Need month increment? */
				if (++t.date == 32) {
					t.month++;
					t.date = 1;
				} else if (t.date == 31) {
					if ((t.month == 4) || (t.month == 6) || (t.month == 9) || (t.month == 11)) {
						t.month++;
						t.date = 1;
					}
				} else if (t.date == 30) {
					if (t.month == 2) {
						t.month++;
						t.date = 1;
					}
				} else if (t.date == 29) {
					if ((t.month == 2) && (not_a_leap_year())) {
						t.month++;
						t.date = 1;
					}
				}
				/* Need year increment? */
				if (t.month == 13) {
					t.year++;
					t.month = 1;
				}
			}
		}
	}
}

uint8_t not_a_leap_year()
{
	/* Returns:
	 *	TRUE if year is NOT a leap year
	 *  FALSE if year IS a leap year
	 */

	/* Years divisible by 100 not leap years... */
	if (!(t.year % 100)) {
		/* ...unless also divisible by 400 */
		return (t.year % 400);
	} else {
		/* Otherwise, every 4th year is a leap year */
		return (t.year % 4);
	}
}

void send_buffer()
{
	UINT    bytes_written;
	FRESULT result;
	//result = pf_lseek(file_system.fptr);
	//check_result(result);
	
	
	sprintf(buffer,
		BUFFER1_TEMPLATE,
		(float)t.totSeconds,
		(float)t.year,
		(float)t.month,
		(float)t.date,
		(float)t.hour,
		(float)t.minute,
		(float)t.second,
		bat1_voltage,
		bat2_voltage,
		bat3_voltage,
		bat4_voltage,
		bat5_voltage,
		bat_tot_voltage,
		(float)overcharged_state,
		(float)overdischarged_state,
		I_charge,
		I_load);
		
	result = pf_write(buffer, BUFFER_SIZE, &bytes_written);
	
	// Check if the write was successful
	if (result != FR_OK || bytes_written != BUFFER_SIZE) {
		SD_eject();
		return;
	}
}

void init_sd_card(void)
{
	DSTATUS status;
	FRESULT result;
	UINT    bytes_written;
	uint8_t it = 0;
		

	/* Initialize physical drive */
	statusDebug1 = 1;
	statusDebug1 = disk_initialize();
	//debug1 = status; 
	if (status != 0) {
		SD_eject();
	} else {
		
		SPI0.CTRLA = (SPI_MASTER_bm | SPI_PRESC_DIV16_gc | SPI_ENABLE_bm); //increase SPI rate to div16 
		
		// Mount Volume
		fresultDebug1 = pf_mount(&file_system);
		while(fresultDebug1 != FR_OK && it < 10) {
			fresultDebug1 = pf_mount(&file_system);
			_delay_ms(10);
			it++;
		}
		it = 0;
		
		// Open File
		fresultDebug2 = pf_open(filename);
		while(fresultDebug2 != FR_OK && it < 10) {
			fresultDebug2 = pf_open(filename);
			_delay_ms(10);
			it++;
		}
		
		if(fresultDebug1 == FR_OK  &&  fresultDebug2 == FR_OK) {
			SD_ejected=false;
			eject_LED_OFF();
			write_header_SD();
			SD_start_enable = false;
		}
	}
	
	;

}

void adc_init() {
	
	//Initialze ADC inputs for meas voltages of shunt resistors (V1+- and V2+-)
	V1_NEG_CNTRL &= ~PORT_ISC_gm; //  Disable digital input buffer
	V1_NEG_CNTRL |= PORT_ISC_INPUT_DISABLE_gc; //   Disable digital input buffer
	V1_NEG_CNTRL &= ~PORT_PULLUPEN_bm; //  Disable pull-up resistor
	
	V1_POS_CNTRL &= ~PORT_ISC_gm; //  Disable digital input buffer
	V1_POS_CNTRL |= PORT_ISC_INPUT_DISABLE_gc; //   Disable digital input buffer
	V1_POS_CNTRL &= ~PORT_PULLUPEN_bm; //  Disable pull-up resistor

	V2_NEG_CNTRL &= ~PORT_ISC_gm; //  Disable digital input buffer
	V2_NEG_CNTRL |= PORT_ISC_INPUT_DISABLE_gc; //   Disable digital input buffer
	V2_NEG_CNTRL &= ~PORT_PULLUPEN_bm; //  Disable pull-up resistor
	
	V2_POS_CNTRL &= ~PORT_ISC_gm; //  Disable digital input buffer
	V2_POS_CNTRL |= PORT_ISC_INPUT_DISABLE_gc; //   Disable digital input buffer
	V2_POS_CNTRL &= ~PORT_PULLUPEN_bm; //  Disable pull-up resistor
	
	// Initialize ADC input pins for meas voltages of battery
	B_GND_CNTRL &= ~PORT_ISC_gm; //  Disable digital input buffer
	B_GND_CNTRL |= PORT_ISC_INPUT_DISABLE_gc; //   Disable digital input buffer
	B_GND_CNTRL &= ~PORT_PULLUPEN_bm; //  Disable pull-up resistor

	B1_DIV_CNTRL &= ~PORT_ISC_gm; //  Disable digital input buffer
	B1_DIV_CNTRL |= PORT_ISC_INPUT_DISABLE_gc; //   Disable digital input buffer
	B1_DIV_CNTRL &= ~PORT_PULLUPEN_bm; //  Disable pull-up resistor

	B2_DIV_CNTRL &= ~PORT_ISC_gm; //  Disable digital input buffer
	B2_DIV_CNTRL |= PORT_ISC_INPUT_DISABLE_gc; //   Disable digital input buffer
	B2_DIV_CNTRL &= ~PORT_PULLUPEN_bm; //  Disable pull-up resistor

	B3_DIV_CNTRL &= ~PORT_ISC_gm; //  Disable digital input buffer
	B3_DIV_CNTRL |= PORT_ISC_INPUT_DISABLE_gc; //   Disable digital input buffer
	B3_DIV_CNTRL &= ~PORT_PULLUPEN_bm; //  Disable pull-up resistor

	B4_DIV_CNTRL &= ~PORT_ISC_gm; //  Disable digital input buffer
	B4_DIV_CNTRL |= PORT_ISC_INPUT_DISABLE_gc; //   Disable digital input buffer
	B4_DIV_CNTRL &= ~PORT_PULLUPEN_bm; //  Disable pull-up resistor

	B5_DIV_CNTRL &= ~PORT_ISC_gm; //  Disable digital input buffer
	B5_DIV_CNTRL |= PORT_ISC_INPUT_DISABLE_gc; //   Disable digital input buffer
	B5_DIV_CNTRL &= ~PORT_PULLUPEN_bm; //  Disable pull-up resistor
	
	// Initialize ADC0
	VREF.ADC0REF = VREF_REFSEL_1V024_gc; // set voltage reference as 1.024V internal ref
	//ADC0.CTRLC = ADC_PRESC_DIV16_gc; // try to reduce noise by lowering the clock speed
	ADC0.SAMPCTRL = ADC_sample_length_cells; // try to reduce noise by increasing sampel length
	ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_12BIT_gc | ADC_CONVMODE_bm; // ADC_CONVMODE_bm selects differential mode
	ADC0.CTRLB = ADC_ACCUMULATIONS_cells; // Set ADC to accumulation mode with 16 accumulations

	//ADC0.MUXPOS = B0R_MUX; //intiizize for B_GND to be the input of the ADC
	
}

void init_io(void)
{
	LEDPORT.DIRSET = eject_LEDPIN;
	LEDPORT.DIRSET = data_LEDPIN;
	data_LED_OFF();
	
	PORTF_DIRSET |= PIN2_bm; //C-MOSFET output = PF2
	PORTF_DIRSET |= PIN5_bm; //SD-Ejected output LED = PF5
	//PORTB_DIRSET |= PIN2_bm; //BM output
	PORTF_DIRSET |= PIN3_bm; //D-MOSFET output = PF3
	
	// Set inputs for SD detection and PUsh button
	PORTA_OUTCLR = PIN2_bm;  //Pin A2 is SD card detect input
	PORTA.PIN2CTRL = PORT_PULLUPEN_bm; // Pin A2 pullup enable
	//PORTD.DIR &= ~ PIN7_bm;  //Pin D7 is button push input
	PORTD_DIRCLR = PIN7_bm;
	PORTD.PIN7CTRL = PORT_PULLUPEN_bm; //Pin D7 pullup enable
	
	
	// Set pins for master in as inputs
	SLAVE_D_MOSFET_PORT.DIRCLR = SLAVE_D_MOSFET_PIN; // D-MOSFET_MSTR-IN
	SLAVE_D_MOSFET_PORT.SLAVE_D_MOSFET_CTRL_PIN = PORT_INLVL_bm; // selelct TTL levels as input levels
	
	SLAVE_C_MOSFET_PORT.DIRCLR = SLAVE_C_MOSFET_PIN; // C-MOSFET_MSTR-IN
	SLAVE_C_MOSFET_PORT.SLAVE_C_MOSFET_CTRL_PIN = PORT_INLVL_bm; // selelct TTL levels as input levels
	
	// Set pins for master/slave select header
	MSTR_SLV_SEL_PORT.DIRCLR = MSTR_SLV_SEL_PIN; // set MSTR-SLV-SEL as input
	MSTR_SLV_SEL_PORT.MSTR_SLV_SEL_CTRL_PIN = PORT_PULLUPEN_bm;
	
}

void process_voltages(void) {
	int it;
	bool overcharged_flag = false;
	bool overdischarged_flag = false;
		
			
	// If the battery voltage is too low to switch the mosfets, dont switch them.
	if(bat_tot_voltage < V_CUTOFF) {
		overcharged_flag = true;
		overdischarged_flag = true;
		overcharged();
		overdischarged();
		return;
	}
	
	// otherwise, process voltages and throw mosfets
	for(it = 0; it < N_BAT; it++) {
		if(bat_voltages[it] > V_HIGH) {
			overcharged_flag = true;
		}
	}
	for(it = 0; it < N_BAT; it++) {
		if(bat_voltages[it] < V_LOW) {
			overdischarged_flag = true;
		}
	}
	if(overcharged_flag || slave_overcharged) {
		overcharged();
	}
	else if(!charge_fuse_blown)  {
		not_overcharged();
	}
	
	if(overdischarged_flag || slave_overdischarged) {
		overdischarged();
	}
	else if(!load_fuse_blown) {
		not_overdischarged();
	}
	
	

}

void init_rtc(void) //skyler's init_rtc
{
	
	while(CLKCTRL.MCLKSTATUS & CLKCTRL_OSC32KS_bm)
	{
		; // Wait until XOSC32KS becomes 0
	}
	
	// Initialize RTC: 
	while (RTC.STATUS > 0)
	{
		; /* Wait for all register to be synchronized */
	}
	
	//CLKCTRL.OSC32KCTRLA = CLKCTRL_RUNSTDBY_bm;// for DEBUG
	
	
	// Set period 
	RTC.PER = SAMPLE_PERIOD*32768/8192 - 1; //(RTC clock freq)/(RTC prescaler) is the value that the period in sec needs to be multiplied by 
	RTC.CLKSEL = RTC_CLKSEL_OSC32K_gc; //Set RTC clock source to be 32.768kHz internal oscillator
	
	// Run in debug: enabled
	RTC.DBGCTRL |= RTC_DBGRUN_bm;

	RTC.CTRLA = RTC_PRESCALER_DIV8192_gc /* 16384, 32768 */
		| RTC_RTCEN_bm /* Enable: enabled */
		| RTC_RUNSTDBY_bm; /* Run In Standby: enabled */

	/* Enable Overflow Interrupt */
	RTC.INTCTRL |= RTC_OVF_bm;

}

void init_rtc_xtal(void)  {
	/* Code from https://ww1.microchip.com/downloads/en/AppNotes/TB3213-Getting-Started-with-RTC-90003213A.pdf */
	
	
	/* Initialize 32.768kHz Oscillator: */
	 /* Disable oscillator: */
	uint8_t temp;
	temp = CLKCTRL.XOSC32KCTRLA;
	temp &= ~CLKCTRL_ENABLE_bm;
	/* Enable writing to protected register */
	CPU_CCP = CCP_IOREG_gc;
	CLKCTRL.XOSC32KCTRLA = temp;

	while(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm)
	{
	; /* Wait until XOSC32KS becomes 0 */
	}

	/* SEL = 0 (Use External Crystal): */
	temp = CLKCTRL.XOSC32KCTRLA;
	temp &= ~CLKCTRL_SEL_bm;
	/* Enable writing to protected register */
	CPU_CCP = CCP_IOREG_gc;
	CLKCTRL.XOSC32KCTRLA = temp;

	/* Enable oscillator: */
	temp = CLKCTRL.XOSC32KCTRLA;
	temp |= CLKCTRL_ENABLE_bm;
	/* Enable writing to protected register */
	CPU_CCP = CCP_IOREG_gc;
	CLKCTRL.XOSC32KCTRLA = temp;
	 
	/* Initialize RTC: */
	while (RTC.STATUS > 0)
	{
	; /* Wait for all register to be synchronized */
	}

    // Set RTC clock source to be 32.768kHz external oscillator
    RTC.CLKSEL = RTC_CLKSEL_XTAL32K_gc; 

    // Set period 
    RTC.PER = SAMPLE_PERIOD*32768/8192 - 1;

    // Run in debug: enabled
    RTC.DBGCTRL |= RTC_DBGRUN_bm;

    RTC.CTRLA = RTC_PRESCALER_DIV8192_gc /* Prescaler 8192 */
                | RTC_RTCEN_bm /* Enable: enabled */
                | RTC_RUNSTDBY_bm; /* Run In Standby: enabled */

    /* Enable Overflow Interrupt */
    RTC.INTCTRL |= RTC_OVF_bm;
}

void process_start_time(void)
{
	t.totSeconds = 0;
	
	/* Get compile date and time to use as initial time */

	char *ptr = __DATE__;

	/* Month */
	if (*ptr == 'J') {
		ptr++;
		if (*ptr == 'a') {
			t.month = 1;
			ptr += 3;
		} else if (*ptr == 'u') {
			ptr++;
			if (*ptr == 'n') {
				t.month = 6;
			} else if (*ptr == 'l') {
				t.month = 7;
			}
			ptr += 2;
		}
	} else if (*ptr == 'F') {
		t.month = 2;
		ptr += 4;
	} else if (*ptr == 'M') {
		ptr += 2;
		if (*ptr == 'r') {
			t.month = 3;
		} else if (*ptr == 'y') {
			t.month = 5;
		}
		ptr += 2;
	} else if (*ptr == 'A') {
		ptr++;
		if (*ptr == 'p') {
			t.month = 5;
		} else if (*ptr == 'u') {
			t.month = 8;
		}
		ptr += 3;
	} else if (*ptr == 'S') {
		t.month = 9;
		ptr += 4;
	} else if (*ptr == 'O') {
		t.month = 10;
		ptr += 4;
	} else if (*ptr == 'N') {
		t.month = 11;
		ptr += 4;
	} else if (*ptr == 'D') {
		t.month = 12;
		ptr += 4;
	}

	/* Day */
	char date[3] = {*ptr, *(ptr + 1), '\0'};
	t.date       = atoi(date);
	ptr += 3;
	/* Year */
	char year[5] = {*ptr, *(ptr + 1), *(ptr + 2), *(ptr + 3), '\0'};
	t.year       = atoi(year);

	ptr = __TIME__;
	/* Hour */
	date[0] = *ptr;
	date[1] = *(ptr + 1);
	ptr += 3;
	t.hour = atoi(date);
	/* Minute */
	date[0] = *ptr;
	date[1] = *(ptr + 1);
	ptr += 3;
	t.minute = atoi(date);
	/* Second */
	date[0] = *ptr;
	date[1] = *(ptr + 1);
	ptr += 3;
	t.second = atoi(date);
}


float ADC_accumu_read(void) {
	// Reads ADC in accumulated mode and returns the mean of all the ADC samples
	
	// Start ADC conversion
	ADC0.COMMAND = ADC_STCONV_bm;
	
	// Wait until ADC conversion done
	while ( !(ADC0.INTFLAGS & ADC_RESRDY_bm) )  {         ;    }
		
	// Clear the interrupt flag by writing 1:
	ADC0.INTFLAGS = ADC_RESRDY_bm;
		
	int16_t ADC_result = ADC0.RES;
	float ADC_result_float = (float)(ADC_result / ADC_accumuations_rightnow);
	//ADC_result_float = ADC_result_float / ADC_accumuations_rightnow;
	//float ADC_result_float = (float)ADC_result / 4;
	
	
	return ADC_result_float;
}

int16_t ADC_fuse_read(void) {
	// Reads ADC in accumulated mode and returns the sum of all the samples
	
	// Start ADC conversion
	ADC0.COMMAND = ADC_STCONV_bm;
	
	// Wait until ADC conversion done
	while ( !(ADC0.INTFLAGS & ADC_RESRDY_bm) )  {         ;    }
	
	// Clear the interrupt flag by writing 1:
	ADC0.INTFLAGS = ADC_RESRDY_bm;
	
	//int16_t ADC_result = ADC0.RES;
	//float ADC_result_float = (float)(ADC_result / ADC_accumuations_rightnow);
	//ADC_result_float = ADC_result_float / ADC_accumuations_rightnow;
	//float ADC_result_float = (float)ADC_result / 4;
	
	
	return ADC0.RES;
}

double ADC_read_avg(void) {
    double sum = 0;
	
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        sum += (double)ADC_accumu_read();  // function to read an ADC value
    }
    return (double)(sum / NUM_SAMPLES);
}



void read_voltages(bool read_cell_voltages) {
	
	//V1 Voltage differential
	ADC0.MUXNEG = V1_NEG_MUX;
	ADC0.MUXPOS = V1_POS_MUX;
	adc_reading = ADC_accumu_read();         // Read ADC sample Calculate voltage on ADC pin, VDD = 3.3V, 12-bit resolution
	V1_voltage = (float) (adc_reading * VOLTAGE_REF) / ADC_MAX_VALUE ;
	
	
	//V2 Voltage differential
	ADC0.MUXNEG = V2_NEG_MUX;
	ADC0.MUXPOS = V2_POS_MUX;
	adc_reading = ADC_accumu_read();         // Read ADC sample Calculate voltage on ADC pin, VDD = 3.3V, 12-bit resolution
	V2_voltage = (float) (adc_reading * VOLTAGE_REF) / ADC_MAX_VALUE ;
	
	//Correct for offset voltages
	V1_voltage = V1_voltage - V1_voltage_offset;
	V2_voltage = V2_voltage - V2_voltage_offset;
	
	I_charge = V1_voltage/R_SHUNT;
	I_load = V2_voltage/R_SHUNT;
	
	
	
	
	if(read_cell_voltages) {
		//Battery 0 voltage
		ADC0.MUXNEG = ADC_MUXPOS_GND_gc;
		ADC0.MUXPOS = B0R_MUX;
		adc_reading = ADC_accumu_read();;
		bat0_voltage = (float)(adc_reading * VOLTAGE_REF) / ADC_MAX_VALUE;
	
		//Correct for offset voltages
		bat0_voltage = bat0_voltage - bat0_voltage_offset;
	
		//Battery 1 voltage
		ADC0.MUXNEG = ADC_MUXPOS_GND_gc;
		ADC0.MUXPOS = B1_DIV_MUX;
		adc_reading = ADC_accumu_read();
		bat1_voltage = (float)(adc_reading * VOLTAGE_REF) / ADC_MAX_VALUE / vDiv - bat0_voltage;
	
		//Battery 2 voltage
		ADC0.MUXNEG = B1_DIV_MUX;
		ADC0.MUXPOS = B2_DIV_MUX;
		adc_reading = ADC_accumu_read();
		bat2_voltage = (float)(adc_reading * VOLTAGE_REF) / ADC_MAX_VALUE / vDiv;

		//Battery 3 voltage
		ADC0.MUXNEG = B2_DIV_MUX;
		ADC0.MUXPOS = B3_DIV_MUX;
		adc_reading = ADC_accumu_read();
		bat3_voltage = (float)(adc_reading * VOLTAGE_REF) / ADC_MAX_VALUE / vDiv;

		//Battery 4 voltage
		ADC0.MUXNEG = B3_DIV_MUX;
		ADC0.MUXPOS = B4_DIV_MUX;
		adc_reading = ADC_accumu_read();
		bat4_voltage = (float)(adc_reading * VOLTAGE_REF) / ADC_MAX_VALUE / vDiv;

		//Battery 5 voltage
		ADC0.MUXNEG = B4_DIV_MUX;
		ADC0.MUXPOS = B5_DIV_MUX;
		adc_reading = ADC_accumu_read();
		bat5_voltage = (float)(adc_reading * VOLTAGE_REF) / ADC_MAX_VALUE / vDiv;

		bat_tot_voltage = bat1_voltage + bat2_voltage + bat3_voltage + bat4_voltage + bat5_voltage;

		bat_voltages[0] = bat1_voltage;
		bat_voltages[1] = bat2_voltage;
		bat_voltages[2] = bat3_voltage;
		bat_voltages[3] = bat4_voltage;
		bat_voltages[4] = bat5_voltage;
	}
	
	
	
	
}

void charger_load_init() {
	PORTF_OUT &= ~PIN2_bm; //break connection with charger on PF2
	PORTF_OUT &= ~PIN3_bm; //break connection with load on PF3
	//PORTB_OUT &= ~PIN2_bm; //turn off the "charged and ready" light
	overcharged_state = true;
	overdischarged_state = true;
	return;
}
void overcharged() {
	PORTF_OUT &= ~PIN2_bm; //break connection with charger on PF2
	//PORTB_OUT &= ~PIN2_bm; //turn off the BM light
	overcharged_state = true;
	return;
}
void overdischarged() {
	PORTF_OUT &= ~PIN3_bm; //break connection with load on PF3
	//PORTB_OUT &= ~PIN2_bm; //turn off the BM light
	overdischarged_state = true;
	return;
}
void not_overcharged() {
	PORTF_OUT |= PIN2_bm; //connect with charger on PF2
	overcharged_state = false;
	return;
}
void not_overdischarged() {
	PORTF_OUT |= PIN3_bm; //connect with load on PF3
	overdischarged_state = false;
	return;
}

bool SD_detect() { 
	// Returns true iff SD is in slot, false if not in slot
	if(PORTA.IN & PIN2_bm)
		return false;
	else
		return true;
	
}
bool button_pushed() {
	// Returns true iff button is currently depressed, false if it is not depressed
	if(PORTD.IN & PIN7_bm)
		return false;
	else
		return true;
}

void SD_eject() {
	// Ejects the SD card
	// Terminates the file so the SD card can be ejected with out data corruption
	// and sets SD state as ejected, and turns on SD_ejected LED
	UINT    bytes_written;
	FRESULT result;
	
	result = pf_write(0, 0, &bytes_written);
	SD_ejected = true;
	eject_LED_ON();
}

void set_MSTR_SLV(void) {
	if(MSTR_SLV_SEL_PORT.IN & MSTR_SLV_SEL_PIN) // if MST-SLV-SEL pin is high, the board is set as slave
		is_slave = true;
	else // if MST-SLV-SEL pin is low, the board is set as a master
		is_slave = false;
	
	slave_overdischarged = false;
	slave_overcharged = false;
}


// If there is an error with the SD card, eject it
void check_result(FRESULT result) {
	if (result != FR_OK)
		SD_eject();
}

void write_header_SD() {
	// Writes headers to SD card at the begining of the file
	UINT    bytes_written;
	FRESULT result;
	
	// Send header to sd card
	//result = pf_lseek(file_system.fptr);
	//check_result(result);
	result = pf_write(header1, strlen(header1), &bytes_written);
	//check_result(result);
	result = pf_write(header2, strlen(header2), &bytes_written);
	result = pf_write(header3, strlen(header3), &bytes_written);
	//check_result(result);
	
}

/*
void send_endline(void) {
	UINT    bytes_written;
	FRESULT result;
	result = pf_lseek(file_system.fptr);
	//check_result(result);
	
	result = pf_write(endline1, strlen(endline1), &bytes_written);
	//check_result(result);
}
*/
