/*
 * =========================================================================
 * PWM-Driver mit Taster
 * 
 * crashc0de 2014
 * BSD LICENSE
 */


/*
 * Schaltbild der für die Spannungsmessung relevanten Teile
 *             VCC
 *              |
 *              D1  <- hier fallen VD ab
 * --------     |
 *      8 |-----| 
 *        |     R1
 *      7 |-----|
 *        |     |
 * Attiny |     R2
 *        |     |
 *        |    GND
 *        |
 */
#define LOWBATT_VOLTAGE     3.5    // kritische Akkuspannung in Volt Bei dieser Spannung wird auf PWM_LOWBATT geschaltet
#define R1                 17.8    // R1 in kOhm (siehe Schaltbild)
#define R2                 4.67    // R2 in kOhm (siehe Schaltbild)
#define VD                   0    // Spannungsabfall über die Diode.
#define PWM_LOWBATT         40    // Auf diesen Wert wird runter geschaltet wenn LOWBATT_VALUE erreicht wurde

#define BATTMON			          // Battery Monitor aktivieren? Wenn nicht definert, wird die RAMP-Funktion aktiviert
#define BATTMON_REF_VOLTAGE  3.5  // Referenzspannung für Akkumonitor. Bei dieser Spannung blitzt er ein Mal

#define BATTMON_STEP         6 // schrittweite. je kleiner um so öfter blitzt es bei gleicher spannung
#define BATTMON_VORBLITZZEIT_AN    500
#define BATTMON_VORBLITZZEIT_AUS  1000
#define BATTMON_BLITZ_AN    40
#define BATTMON_BLITZ_AUS  200

#define RAMP_ANSCHLAG_AN   100
#define RAMP_ANSCHLAG_AUS   10

#define MODES			0,32,125,255,100	// Modes, erster Wert muss 0 sein, letzter Mode ist immer Battmon mit helligkeit WERT

// umrechnen der Spannungen in ADC-Wandler werte (8 bit)
#define LOWBATT_VALUE     ((int) (((LOWBATT_VOLTAGE     - VD) * R2 * 255) / ((R1 + R2) * 1.1)))
#define BATTMON_REF_VALUE ((int) (((BATTMON_REF_VOLTAGE - VD) * R2 * 255) / ((R1 + R2) * 1.1)))

#define PWM_HOT             32
#define TURBO_TIMEOUT_SEC   70                // nach dieser Zeit im high-mode wird auf PWM_HOT zurückgeschaltet

#define TURBO_TIMEOUT	TURBO_TIMEOUT_SEC * 62.5 // Sekunden in ticks umrechnen (1 tick = .016 s)

#define SHORT_TIMEOUT	   63 // nach dieser Anzahl ticks wird auf RUN geschaltet

#define MAX_RAMP	       255 // 
#define MIN_RAMP	         7 // 
			
/*
 * =========================================================================
 */

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>	
#include <avr/eeprom.h>
#include <avr/sleep.h>
//#include <avr/power.h>

#define STAR2_PIN   PB0
#define STAR3_PIN   PB4
#define SWITCH_PIN  PB3		// Wo ist der Taster aneschlossen. Hier: Star 4
#define PWM_PIN     PB1
#define VOLTAGE_PIN PB2
#define ADC_CHANNEL 0x01	// MUX 01 entspricht PB2
#define ADC_DIDR 	ADC1D	// Digital input disable bit für PB2
#define ADC_PRSCL   0x06	// clk/64

#define PWM_LVL OCR0B       // OCR0B ist das output compare register für PB1
#define PWM_SET_LVL(y) PWM_LVL=y

#define DEBOUNCE_PATTERN 0b00001111 // Vergleichspattern für debounce_buffer
							        // die Bits in debounce_buffer werden alle 16ms gesetzt wenn der Taster
                                    // gedrückt ist. 0b00001111 entspricht also 4*16ms = 64ms
                                    // wenn dein Taster länger prellt, hier mehr Bits setzen.

// Switch handling
#define LONG_PRESS_DUR   32 // Wie Viele WDT ticks ein long press sein sollen
                            // 32 sind 512ms, also ca. halbe sekunde	
#define UP 1
#define DOWN -1

#define OFF 1
#define SWITCH 2
#define RAMP 4
#define HOT 8
#define RUN 16
#define WAKEUP 32

#define WDTIME 0b01000000  //16ms
#define adcinit() do{ ADMUX =0b01100000|ADC_CHANNEL; ADCSRA=0b11000100; }while(0) //ref1.1V, left-adjust, ADC1/PB2; enable, start, clk/16
#define adcread() do{ ADCSRA|=64; while (ADCSRA&64); }while(0)
#define adcresult ADCH
#define ADCoff ADCSRA&=~(1<<7) //ADC off (enable=0);
#define ADCon  ADCSRA|=(1<<7)  //ADC on
#define ACoff  ACSR|=(1<<7)    //Analog Comparator off (save power)
#define ACon   ACSR&=~(1<<7)   //Analog Comparator on
#define SLEEP asm volatile ("SLEEP")
#define wdtinit() do{ WDTCR=WDTIME; sei(); MCUCR=(MCUCR &~0b00111000)|0b00100000; }while(0) //WDT-int and Idle-Sleep
#define portinit() do{DDRB = (1 << PWM_PIN|1<<STAR3_PIN);PORTB = (1 << SWITCH_PIN) | (1 << STAR2_PIN)| (1 << STAR3_PIN) ; } while(0)
#define pwminit() do{ TCCR0A=0b10100001; TCCR0B=0b00000001; }while(0)  //chan A and B, phasePWM, clk/1  ->2.35kHz@1.2MHz


/*
 * Das eigentliche Programm
 * =========================================================================
 */

PROGMEM  uint8_t modes[] = { MODES };
volatile uint8_t mode_idx = 0;
static uint8_t press_duration = 0;
#ifndef BATTMON
volatile int8_t ramp_dir = UP;
static uint8_t ramp_delay = 0;
static uint8_t ramp_dir_switched = 0;
#endif
static uint8_t old_lvl= 0;
volatile uint8_t state = OFF;
static uint16_t run_ticks = 0;


inline void prepare_sleep() {
	GIMSK |= (1 << PCIE);       // enable pin change interrupt
	PCMSK = 1<<PCINT3;			// PCINT3/PB3 for triggers pin change interrupt
	set_sleep_mode( SLEEP_MODE_PWR_DOWN);		// prepare sleep
	sleep_enable();
}

// empty interrupt for wakeup on keypress
EMPTY_INTERRUPT(PCINT0_vect);

inline void WDT_on() {
    // watchdog timer löst alle 16ms einen Interrupt aus
    cli();                          // Disable interrupts
    WDTCR |= (1<<WDCE) | (1<<WDE);  // Start timed sequence
    WDTCR = (1<<WDTIE);             // Enable interrupt every 16ms
    sei();                          // Enable interrupts
}

inline void WDT_off()
{
    cli();                          // Disable interrupts
    MCUSR &= ~(1<<WDRF);            // Clear Watchdog reset flag
    WDTCR |= (1<<WDCE) | (1<<WDE);  // Start timed sequence
    WDTCR = 0x00;                   // Disable WDT
    sei();                          // Enable interrupts
}

void sleep_until_switch_press()
{
    // WDT aus, sonst weckt der uns aus dem sleep-mode
    WDT_off();
    press_duration = 0;
    // CPU anhalten
	SLEEP;
    // WDT wieder starten
    WDT_on();
}


// Debounced switch press value
int is_pressed()
{
	// Zwischenspeicher für switch readouts
	static uint8_t buffer = 0x00;
	// alle bits um 1 nach links schieben und aktuellen Wert anhängen, 0 ist low für gedrückt, 1 für offen (pull-up)
	buffer = (buffer << 1) | ((PINB & (1 << SWITCH_PIN)) == 0);
	// vergleich mit DEBOUNCE_PATTERN. Wenn true, dann war taster lange genug gedrückt.
	return (buffer & DEBOUNCE_PATTERN);
}

inline void tap() {
	run_ticks = 0;

	if( state & ( RAMP | RUN | HOT ) ){ state = OFF; return;}
	if( state & ( OFF | WAKEUP) ) { state = SWITCH; mode_idx = 1; return;}
	if( state & SWITCH ) {
		mode_idx ++;
		if(mode_idx >= sizeof(modes)) mode_idx = 1;
	}
}

inline void long_press() {
	#ifndef BATTMON
	if( state & (RUN | SWITCH ) ) state = RAMP;
	if( state & RAMP ) do_ramp();
	#endif
	if( state & ( OFF | WAKEUP | RUN | SWITCH ) ) {
		#ifndef BATTMON
		ramp_dir = UP;
		#endif
		PWM_SET_LVL( 255 );
		state = RUN;
		while(is_pressed());
	}
}

inline void timeout_short() {
	if( state & SWITCH ) state = RUN;
	// unterspannung signalisieren
	adcread();
	if( PWM_LVL > PWM_LOWBATT && (adcresult <= LOWBATT_VALUE) ) PWM_SET_LVL( PWM_LOWBATT );
}

inline void timeout_long() {
	if( state & RUN && PWM_LVL == 255 ) state = HOT ;
}

#ifdef BATTMON
void flick(uint16_t off,uint16_t on) {
	old_lvl = PWM_LVL;
	PWM_LVL = 0;
	delay_ms( off );
	PWM_LVL = old_lvl;
	delay_ms( on );
	PWM_LVL = 0;
	delay_ms( off );
	PWM_LVL = old_lvl;
}
#else
void flick() {
	old_lvl = PWM_LVL;
	PWM_LVL = 0;
	_delay_ms( RAMP_ANSCHLAG_AUS );
	PWM_LVL = old_lvl;
	_delay_ms( RAMP_ANSCHLAG_AN );
}

#endif


void delay_ms(uint16_t msec) { 
    while (msec > 0) { 
        msec--; 
        _delay_ms(1); 
    } 
}  
inline void do_ramp() {
	#ifdef BATTMON
	return;
	#else
	if( ramp_dir < 0 && PWM_LVL == MIN_RAMP
	 || ramp_dir > 0 && PWM_LVL == MAX_RAMP ) {flick(); return; }
	PWM_LVL += ramp_dir;
	#endif
}

inline void battmon() {
	state = RUN;
	#ifdef BATTMON
	   uint8_t akkuspannung_copy;
	   adcread();
       akkuspannung_copy = adcresult;
//	   akkuspannung_copy=129; // debug
	   flick(BATTMON_VORBLITZZEIT_AUS,BATTMON_VORBLITZZEIT_AN);
	   while (akkuspannung_copy > BATTMON_REF_VALUE && (state & RUN)) {
         flick( BATTMON_BLITZ_AUS, BATTMON_BLITZ_AN );
	     akkuspannung_copy = akkuspannung_copy - BATTMON_STEP;
	   }
	   state = OFF;
	#endif
}

// The watchdog timer is called every 16ms
ISR(WDT_vect) {
	if (is_pressed()) {
		#ifndef BATTMON
		ramp_dir_switched = 0;
		#endif
		if (press_duration < 255) {
			press_duration++;
		}
		
		if (press_duration >= LONG_PRESS_DUR ) {
			long_press();
		}
	} else {
		// Not pressed
		if (press_duration > 0 && press_duration < LONG_PRESS_DUR) {
			tap();
		} else {
			run_ticks++;
			if( state & OFF ) run_ticks=0;
			#ifndef BATTMON
			if( state & RAMP && !ramp_dir_switched ){ramp_dir_switched = 1; ramp_dir = -ramp_dir; }
			#endif
			if( run_ticks > SHORT_TIMEOUT) timeout_short();
			if( run_ticks > TURBO_TIMEOUT) timeout_long();
		}
		press_duration = 0;
	}
}

int main(void)
{	
	uint8_t modevalue;
	wdtinit();
	prepare_sleep();
	portinit();
	pwminit();

	adcinit();
    ACoff; // turn analog Comparator off to save power
	
	while(1) {
		// Endlosschleife. im WDT interrupt wird der Taster abgefragt
		// modes werden hier geändert in abhängigkeit der state-Variable
		if( state & SWITCH ) {
			PWM_SET_LVL( pgm_read_byte(&modes[mode_idx]) );
			if( mode_idx == sizeof(modes)-1) battmon();
			run_ticks = 0;
		}
		if( state & HOT ) 	PWM_SET_LVL( PWM_HOT );

		if (state & OFF ) {
			PWM_SET_LVL( 0 );
			delay_ms(10); // ohne Delay wird PWM-Wert tatsächlich nicht gesetzt vor sleep
			sleep_until_switch_press();
			#ifndef BATTMON
				ramp_dir = UP;
			#endif
			state = WAKEUP;
		}
	}

    return 0; // Standard Return Code
}
