/*
 * =========================================================================
 * PWM-Driver mit Taster
 * für den ATtiny 85
 * 
 * crashc0de 2014
 * BSD LICENSE
 */

#define VERSION 0.4

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>	
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include "config.h"


/********************************************
 * ab hier gibts nichts mehr zum einstellen *
 ********************************************
 */

// umrechnen der Spannungen in ADC-Wandler werte (integer)
#define LOWBATT_VALUE     ((uint16_t) (((LOWBATT_VOLTAGE     - VD) * R2 * 255) / ((R1 + R2) * 1.1)))
#define BATTMON_REF_VALUE ((uint16_t) (((BATTMON_REF_VOLTAGE - VD) * R2 * 255) / ((R1 + R2) * 1.1)))

// Sekunden in ticks umrechnen (1 tick = .016 s)
#define TURBO_TIMEOUT	TURBO_TIMEOUT_SEC * 62 

		

#define VOLTAGE_PIN PB2
#define ADC_DIDR 	ADC1D	// Digital input disable bit für PB2
#define ADC_PRSCL   0x06	// clk/64

#define PWM_LVL OCR0B       // OCR0B ist das output compare register für PB1
// #define PWM_SET_LVL(y) PWM_LVL=y

#define DEBOUNCE_PATTERN 0b00001111 // Vergleichspattern für debounce_buffer
							        // die Bits in debounce_buffer werden alle 16ms gesetzt wenn der Taster
                                    // gedrückt ist. 0b00001111 entspricht also 4*16ms = 64ms
                                    // wenn dein Taster länger prellt, hier mehr Bits setzen.

// Switch handling
#define UP 1
#define DOWN -1

#define OFF 1
#define SWITCH 2
#define RAMP 4
#define HOT 8
#define RUN 16
#define WAKEUP 32

#define ADCoff ADCSRA&=~(1<<7) //ADC off (enable=0);
#define ADCon  ADCSRA|=(1<<7)  //ADC on
#define ACoff  ACSR|=(1<<7)    //Analog Comparator off (save power)
#define ACon   ACSR&=~(1<<7)   //Analog Comparator on
#define SLEEP asm volatile ("SLEEP")

/*
 * Das eigentliche Programm
 * =========================================================================
 */

PROGMEM  uint8_t modes[] = { MODES };
volatile uint8_t mode_idx = 0;
volatile uint8_t press_duration = 0;
volatile int8_t ramp_dir = UP;
static uint8_t ramp_dir_switched = 0;

static uint8_t old_lvl= 0;
volatile uint8_t state = OFF;
volatile uint16_t run_ticks = 0;
volatile uint16_t debug_ticks = 0;
#define DEBUG_TIMEOUT 60
volatile uint8_t wdt_fired = 0;
static uint16_t temperatur = 0;
#ifdef TEMP_LOG 
const uint16_t * TEMP_MIN_ADDR = (uint16_t *) (TEMP_LOG*2);
const uint16_t * TEMP_MAX_ADDR = (uint16_t *) (TEMP_LOG*2+2);

static uint16_t temp_log_addr = 0;
volatile uint16_t tempmess_ticks = 0;
volatile uint16_t templog_ticks = 0;
static uint16_t temp_max = 0;
static uint16_t temp_min = 0;
#endif

// PWM konfigurieren
// chan A and B, phasePWM, Clockbits je nach PWM_FREQ
void pwminit(){
    TCCR0A = 1<< COM0A1 | 0<< COM0A0 | 1<< COM0B1 | 0<< COM0B0 | 0<< WGM01| 1<< WGM00;
    TCCR0B = 0<< FOC0A | 0<< FOC0B | 0<< WGM02 ;
	if( PWM_FREQ == 16 ) TCCR0B |= 0<< CS02 | 0<< CS01 | 1<< CS00;
	if( PWM_FREQ ==  2 ) TCCR0B |= 0<< CS02 | 1<< CS01 | 0<< CS00;
	if( PWM_FREQ == 32 ) {
		TCCR0B |= 0<< CS02 | 0<< CS01 | 1<< CS00;
		TCCR0A &= ~(_BV(WGM01) | _BV(WGM00) );
    	TCCR0A |= 1<< WGM01| 1<< WGM00;
    }
}

// outputs un pull-ups setzen
inline void portinit() {
    DDRB =  1 << PWM_PIN    | 1 << SERIAL_PIN;
    PORTB = 1 << SWITCH_PIN | 1 << STAR2_PIN;
}

inline void prepare_wdt_and_sleep() {
    // WDT Interrupt alle 16ms
    WDTCR = 0<< WDIF | 1<< WDIE | 0<< WDP3 | 0<< WDCE | 0<< WDE | 0<< WDP2 | 0<< WDP1 |0<< WDP0;
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
    WDTCR = (1<<WDIE);             // Enable interrupt every 16ms
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

uint8_t _volt_mess() {
  ADMUX = 1<<REFS1 | 0<<REFS0 | 1<<ADLAR | 0<<REFS2 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 1<<MUX0;
                // REF = 1.1V, 8Bit, ADC1/PB2
  ADCSRA = 1<<ADEN | 1<<ADSC | 0<<ADATE | 0<<ADIF | 0<<ADIE | 1<<ADPS2 | 0<<ADPS1 | 0<<ADPS0;
                // 8MHz / 16, Start Conversion

    while (ADCSRA&64);
    return ADCH;
}
uint8_t volt_mess() {
    #define VOLT_MESS_AVG 200
    uint16_t buf=0, i;
    for(i=0;i<VOLT_MESS_AVG;i++) {
        buf += _volt_mess();
    }
    buf = buf / VOLT_MESS_AVG;
    return buf;
}

uint16_t _temp_mess()
{
  ADMUX = 1<<REFS1 | 0<<REFS0 | 0<<ADLAR | 0<<REFS2 | 1<<MUX3 | 1<<MUX2 | 1<<MUX1 | 1<<MUX0;
                // REF = 1.1V, 10Bit, Temp
  ADCSRA = 1<<ADEN | 1<<ADSC | 0<<ADATE | 0<<ADIF | 0<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 0<<ADPS0;
                // 8MHz / 64 = 125kHz, Start Conversion
  while( ADCSRA & 1<<ADSC );            // until conversion done
  return  ADC; 
}
uint16_t temp_mess() {
    #define TEMP_MESS_AVG 200
    float buf=0;
    uint16_t i;
    for(i=0;i<TEMP_MESS_AVG;i++) {
        buf += _temp_mess();
    }
    buf = buf / TEMP_MESS_AVG;
    return buf;
}

#include <avr/io.h>

void print_logged_temps() {
	uint16_t buf;
    #ifdef TEMP_LOG
    temp_min = eeprom_read_word((uint16_t *) TEMP_MIN_ADDR);
    temp_max = eeprom_read_word((uint16_t *) TEMP_MAX_ADDR);
    #ifdef SERIAL_DEBUG
    Serial.println("-= Min/Max Temperaturwerte =-");
    Serial.print("min: ");
    Serial.print(temp_min,DEC);
    Serial.print(" | max: ");
    Serial.println(temp_max,DEC);
    Serial.println("");
    Serial.println("-= Temperatur-Log =-");
    for(temp_log_addr=0;temp_log_addr<TEMP_LOG*2;temp_log_addr+=2) {
    	buf = eeprom_read_word((uint16_t *) temp_log_addr);
        Serial.print(buf,DEC);
        Serial.print(", ");
    }
    Serial.println("EOF");
    #endif
    eeprom_write_word((uint16_t *) TEMP_MIN_ADDR,65534);
    eeprom_write_word((uint16_t *) TEMP_MAX_ADDR,0);
    temp_min = 65534;
    temp_max = 0;
    #endif
}

void log_temp() {
	#ifdef TEMP_LOG
    Serial.println('log_temp');
    uint16_t temp;
    temp = temp_mess();
    eeprom_write_word((uint16_t *) temp_log_addr,temp);
    eeprom_write_word((uint16_t *) temp_log_addr+2,0);
    temp_log_addr+=2;
    if(temp_log_addr == TEMP_LOG) temp_log_addr = 0;
    if( temp < temp_min ) eeprom_write_word((uint16_t *) TEMP_MIN_ADDR,temp);
    if( temp > temp_max ) eeprom_write_word((uint16_t *) TEMP_MAX_ADDR,temp);
    #endif
}

void serialinit() {
    #ifdef SERIAL_DEBUG
    Serial.begin(9600); 
    #endif
}

void print_version() {
    uint8_t i;
    #ifdef SERIAL_DEBUG
    for(i=0;i<20;i++) Serial.print("-");
    Serial.println("");
    Serial.print("- Bull V");
    Serial.println(VERSION);
    for(i=0;i<20;i++) Serial.print("-");
    Serial.println("");
    #endif
}

#if defined(SERIAL_DEBUG) && defined( SERIAL_VERBOSE )
void akku_debug() {
    float voltage;
    uint8_t buf;
    buf=volt_mess();
    Serial.print("Akku: ");
    voltage = (1.1/255) * (buf*0.86)*(R1+R2)/R2;
    Serial.print(voltage);
    Serial.print(" (");
    Serial.print(buf,DEC);
    Serial.println(" )");
}

void temp_debug() {
    Serial.print("Temp: ");
    temperatur=temp_mess();
    Serial.println(temperatur,DEC);
}
#else
void akku_debug() {}
void temp_debug() {}
#endif

// beim schalten auf eine niedrigere PWM
// expo ausfaden
void PWM_SET_LVL(uint8_t setpoint) {
	float power;

//	if( PWM_LVL > setpoint ) {
		power = PWM_LVL;
		while ( abs(power - setpoint) > MIN_RAMP ) {
				power += ( (float)(setpoint - power) * 0.03);
		        PWM_LVL = (uint8_t) power;
		        _delay_ms(3);
		}
//	}
	PWM_LVL = setpoint;
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
	if( state & ( OFF | WAKEUP) ) {
		state = SWITCH;
		mode_idx = 1;
		ramp_dir = DOWN;
		return;
	}
	if( state & SWITCH ) {
		mode_idx ++;
		if(mode_idx >= sizeof(modes)){
		    ramp_dir = DOWN;
			mode_idx = 1;
		}
	}
}

inline void long_press() {
	if( state & (RUN | SWITCH ) ) state = RAMP;
	if( state & RAMP ) do_ramp();

	if( state & ( OFF | WAKEUP ) ) {
		ramp_dir = DOWN;
		PWM_LVL = 255;
		state = RUN;
		while(is_pressed());
	}
}

inline void timeout_short() {
    if( state & SWITCH ) state = RUN;
    uint8_t buf;
    // unterspannung und übertemperatur signalisieren
    // (nur wenn wenn state == RUN)
    if( state & RUN ) {
	    buf = volt_mess();
	    if( (PWM_LVL > PWM_LOWBATT) && (buf <= LOWBATT_VALUE) ){
	        #ifdef SERIAL_DEBUG
	        Serial.print("low BATT detected: ");
	        Serial.println(buf,DEC);
	        #endif
	    	PWM_SET_LVL( PWM_LOWBATT );
	    }
	    // Temperatur messen
	    #ifdef TEMP_LOG
        if( tempmess_ticks >= TEMP_INTERVAL ) {
            tempmess_ticks = 0;
	        temperatur=temp_mess();        
            if( temperatur >= TEMP_CRIT ){
                #ifdef SERIAL_DEBUG
                Serial.print("HOT");
                #endif
                state = HOT;
            }
        }
        #endif
    }
}

inline void timeout_long() {
	if( state & RUN && PWM_LVL == 255 ){
        #ifdef SERIAL_DEBUG
        Serial.println("HOT timeout.");
        #endif
		state = HOT ;
	}
}

void flick(uint16_t on,uint16_t off) {
    uint8_t buf;
    buf= PWM_LVL;
	delay_ms( on );
    PWM_LVL = 0;
    delay_ms( off );
    PWM_LVL = buf;
}


void delay_ms(uint16_t msec) { 
    while (msec > 0) { 
        msec--; 
        _delay_ms(1); 
    } 
}  
inline void do_ramp() {
    float power;
    uint8_t setpoint;
    uint16_t i;

    cli();

    if( ramp_dir == UP) setpoint = MAX_RAMP;
    if( ramp_dir == DOWN) setpoint = MIN_RAMP;
    power = PWM_LVL;
    while ( abs(power - setpoint) > MIN_RAMP && is_pressed()) {
            if( ramp_dir == UP)
                power += ( (float)(setpoint - (255-power)) * 0.01);
            else
                power += ( (float)(setpoint - power) * 0.02);

            PWM_LVL = (uint8_t) power;
            _delay_ms(5);
    }
    while(is_pressed()) {
        i=i+1;
        if( i == RAMP_ANSCHLAG_AN) PWM_LVL = (uint8_t) power;
        if( i == RAMP_ANSCHLAG_AUS) PWM_LVL = ramp_dir == UP ? RAMP_ANSCHLAG_HIGH_AUS : 0;
        if( i == RAMP_ANSCHLAG_AUS + RAMP_ANSCHLAG_AN ) i=0;
        delay_ms(1);
    }
    ramp_dir = -ramp_dir;
    state = RUN;
    PWM_LVL = (uint8_t) power;
    sei();
}

inline void battmon() {
	state = RUN;
	   uint8_t akkuspannung;
       akkuspannung = volt_mess();
       #ifdef SERIAL_DEBUG
       Serial.println("battmon");
       #endif
       old_lvl = PWM_LVL;
       PWM_LVL = 0;
       delay_ms( BATTMON_PAUSE );
       PWM_LVL = old_lvl;
	   flick(BATTMON_VORBLITZZEIT_AN, BATTMON_VORBLITZZEIT_AUS);

	   while (akkuspannung > BATTMON_REF_VALUE && (state & RUN)) {
         flick( BATTMON_BLITZ_AN, BATTMON_BLITZ_AUS );
	     akkuspannung = akkuspannung - BATTMON_STEP;
	   }
       PWM_LVL = 0;
       delay_ms( BATTMON_PAUSE );
       PWM_LVL = old_lvl;
       delay_ms(BATTMON_NACHBLITZZEIT_AN);
       PWM_LVL = 0;
       #ifdef SERIAL_DEBUG
       Serial.println("battmon end");
       #endif
	   state = OFF;
	   delay_ms(50);
}

// The watchdog timer is called every 16ms
ISR(WDT_vect) {
	if (is_pressed()) {
		ramp_dir_switched = 0;
		if (press_duration < 255) {
			press_duration++;
		}
	} else {
		// Not pressed
		if (press_duration > 0 && press_duration < LONG_PRESS_DUR) {
			tap();
		} else {
			run_ticks++;
            debug_ticks++;
            #ifdef TEMP_LOG
            templog_ticks++;
            #endif
			if( state & OFF ) run_ticks=0;
			if( state & RAMP && !ramp_dir_switched ){ramp_dir_switched = 1; ramp_dir = -ramp_dir; }
		}
		press_duration = 0;
	}
}


int main(void)
{	
	uint8_t modevalue;
	prepare_wdt_and_sleep();
	portinit();
	pwminit();
	serialinit();
    cli();
    print_version();
    print_logged_temps();
    #ifdef TEMP_LOG
    tempmess_ticks = TEMP_INTERVAL;
    #endif

    ACoff; // turn analog Comparator off to save power
    sei();
	
	while(1) {
		// Endlosschleife. im WDT interrupt wird der Taster abgefragt
		// modes werden hier geändert in abhängigkeit der state-Variable
		if( state & SWITCH ) {
			if( mode_idx == sizeof(modes)-1){
				PWM_LVL = pgm_read_byte(&modes[mode_idx]);
				battmon();
			} else {
				PWM_SET_LVL( pgm_read_byte(&modes[mode_idx]) );
			}
		}
		if( state & HOT ) {
            old_lvl = PWM_LVL;
            PWM_SET_LVL(10);
			PWM_SET_LVL( old_lvl / 2 );
            state = RUN;
		}

		if (state & OFF ) {
            #ifdef TEMP_LOG
            tempmess_ticks = TEMP_INTERVAL;
            #endif
			PWM_SET_LVL( 0 );
	#if defined(SERIAL_DEBUG) && defined( SERIAL_VERBOSE )
			Serial.println("sleep");
	#endif
			delay_ms(10); // ohne Delay wird PWM-Wert tatsächlich nicht gesetzt vor sleep
			sleep_until_switch_press();
			state = WAKEUP;
		}
        if( run_ticks > SHORT_TIMEOUT) timeout_short();
        #if TURBO_TIMEOUT > 0
        if( run_ticks > TURBO_TIMEOUT ) timeout_long();
        #endif
        if (press_duration >= LONG_PRESS_DUR ) {
            long_press();
        }

        #ifdef TEMP_LOG
        if( templog_ticks > TEMP_LOG_INTERVAL){templog_ticks=0; log_temp();}
        #endif
        if( debug_ticks > DEBUG_TIMEOUT ){
            akku_debug();
            debug_ticks=0;
            temp_debug();
        }
	}

    return 0; // Standard Return Code
}
