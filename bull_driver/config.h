// CONFIG für Bull-Driver
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

#define LOWBATT_VOLTAGE     3.2    // kritische Akkuspannung in Volt Bei dieser Spannung wird auf PWM_LOWBATT geschaltet
#define R1                 17.8    // R1 in kOhm (siehe Schaltbild)
#define R2                 4.67    // R2 in kOhm (siehe Schaltbild)
#define VD                   0    // Spannungsabfall über die Diode.
#define PWM_LOWBATT         40    // Auf diesen Wert wird runter geschaltet wenn LOWBATT_VALUE erreicht wurde

#define BATTMON_REF_VOLTAGE  3.0  // Referenzspannung für Akkumonitor. Bei dieser Spannung blitzt er ein Mal

#define BATTMON_STEP         5 // schrittweite. je kleiner um so öfter blitzt es bei gleicher spannung
// Blitzzeiten. alles in ms
#define BATTMON_VORBLITZZEIT_AN     500
#define BATTMON_VORBLITZZEIT_AUS   1000
#define BATTMON_BLITZ_AN             50
#define BATTMON_BLITZ_AUS           250
#define BATTMON_NACHBLITZZEIT_AN    400
#define BATTMON_NACHBLITZZEIT_AUS  1500

/* ######## Modes */
#define MODES			0,32,125,255,100	// Modes, letzter Mode ist immer Battmon mit helligkeit WERT
#define SHORT_TIMEOUT	    63 // nach dieser Anzahl ticks wird auf RUN geschaltet, 1 TICK = 62ms
#define LONG_PRESS_DUR   32 // wie viele Ticks dauert ein long press
                            // 32 sind 512ms, also ca. halbe sekunde	

/* ### RAMP Settings */
#define MAX_RAMP	       255 // Max. Helligkeit bei Ramp UP
#define MIN_RAMP	         7 // Min. Helligkeit bei Ramp DOWN
#define RAMP_ANSCHLAG_AN   100 // Blitzverhalten bei RAMP-Anschlag
#define RAMP_ANSCHLAG_AUS   10

/* ######## Temperaturmessung */
#define PWM_HOT             32    // diese Helligkeit setzen wenn im HOT-Mode
#define TEMP_CRIT           300   // kritischer Temperaturwert (Wert in ADC), hat nichts mit Grad Celsius zu tun
#define TEMP_LOG            253   // Anzahl zu speichernder Werte. min und Max werden immer gespeichert
#define TEMP_LOG_INTERVAL   2*62  // alle 20 sekunden Temperatur loggen

/* ######## TURBO Timeout */
#define TURBO_TIMEOUT_SEC   5   // Timeout in Sekunden für HIGH Mode
                                 // wenn nicht verwendet auf 0 setzen.

 /* #### PWM-Frequenz in kHz */
 // immer nur eine der drei Zeilen einkommentieren!
#define PWM_FREQ           32    
//#define PWM_FREQ           16
//#define PWM_FREQ           2

/* ######## Pin funktionen */
#define STAR2_PIN   PB0
#define SERIAL_PIN  PB3
#define SWITCH_PIN  PB4		// Wo ist der Taster aneschlossen
#define PWM_PIN     PB1

//#define SERIAL_DEBUG           // debugging über serielle Ausgabe an SERIAL_PIN
//#define SERIAL_VERBOSE         // Jede kleinigkeit Seriell ausgeben. NUR ZUM TESTEN!
