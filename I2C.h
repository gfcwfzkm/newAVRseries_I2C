 /**
 * @mainpage TWI-Bibliothek für den (neuen) Microchip AVR MCU
 *
 * @brief Two-Wire Interface Master (I2C) Bibliothek für die neue AVR-Microcontroller-Serie mit Hardware-TWI
 *  und Timeouts um loop-locks zu verhindern.
 *
 * @version 1.0
 * -Release der Bibliothek für newSeries-AVR
 *
 * @author gfcwfzkm
 * @date 20.02.2020
 */

/**
 * @file twi_lib.h
 * @brief Headerdatei der TWI-Bibliothek
 *
 * Die TWI-Bibliothek bietet einfache und grundlegende Funktionen zur Ansteuerung
 * von I2C-Kompatible Peripherie für den Atmel AVR Microcontroller, die ein Hardware-TWI haben.
 */


#ifndef TWI_LIB_H_
#define TWI_LIB_H_

#ifndef F_CPU
/**
 * @brief CPU-Taktfrequenz des Microcontrollers (falls nicht schon definiert)
 */
#error "F_CPU wurde noch nicht definiert!"
#endif

#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>

#ifndef TWI_TIMEOUT
/**
 * @brief Timeout-Zeit in Mikrosekunden
 *
 * Die Zeit in Mikrosekunden bis die \a TWI_start() & \a TWI_write() Funktionen
 * den Loop beenden und eine Fehlermeldung zurückgeben
 */
#define TWI_TIMEOUT 5000 
#endif

/**
 * @brief TWI im Lesemodus starten, wird bei \a TWI_start() verwendet
 */
#define TWI_READ	1
/**
 * @brief TWI im Schreibmodus starten, wird bei \a TWI_start() verwendet
 */
#define TWI_WRITE	0

typedef enum {
	NO_ERROR					= 0x00,
	SLAVE_RESPONDED_NACK		= 0x01,
	SLAVE_RESPONDED_ACK			= 0x02,
	ARBITRATION_LOST			= 0x04,
	BUS_ERROR					= 0x08,
	MASTER_NOT_CONTROLLING_BUS	= 0x10,
	TIMEOUT_ERROR				= 0x20
}TWI_ERROR;

enum TWI_ACKFLAG {
	TWI_ACK	=0,
	TWI_NACK=1
};

enum i2c_useMUX {
	NORMAL_I2C_PINPOS	= 0,	/**< I2C Pin MUX Position unverändert lassen */
	MUXED_I2C_PINPOS	= 1		/**< I2C Pin MUX Position verändern */
};

enum i2c_usePullup {
	NO_PULLUP			= 0,	/**< I2C Pin MUX Position unverändert lassen */
	USE_INTERNAL_PULLUP	= 1		/**< I2C Pin MUX Position verändern */
};

/**
 * @brief TWI wiederholter Start durchführen
 */
#define TWI_rstart(adr) (TWI_start(adr))

/**
 * @brief TWI Hardware initialisieren & TWI-Takt einstellen
 *
 * Initialisiert die TWI-Hardware des Microcontrollers. Erfordert die Microcontroller-Taktfrequenz
 * und die gewünschte BUS-Taktgeschwindigkeit.\n
 * Beispiel: \n \code{.c}
 * TWI_init(100000UL, MUXED_I2C_PINPOS, USE_INTERNAL_PULLUP);
 * \endcode Im Beispiel wird die TWI Schnittstelle mit 100kHz Geschwindigkeit, den alternativen I2C-Pins & 
 * den internen Pull-Ups initialisiert.
 * @param twi_clk	Taktfrequenz für den TWI-BUS
 * @param _mux		Normale oder alternative Pinposition
 * @param _pullEN	Interne Pullups verwenden oder nicht
 */
void TWI_init(uint32_t twi_clk, enum i2c_useMUX _mux, enum i2c_usePullup _pullEN);

/**
 * @brief Liest ein Byte vom TWI-BUS
 *
 * Liest ein Byte von der TWI-Peripherie und teilt der Peripherie mit, ob dieser
 * weitere Daten senden soll oder mit der Übertragung aufhören soll.
 * \n Beispiel: \n \code{.c}
 * int myData;
 * TWI_read(&myData, TWI_NACK);
 * \endcode Dies würde 1 Byte vom TWI-BUS lesen und die Übertragung weiterer Daten 
 * stoppen
 * @param data Pointer zum zu lesendem Byte
 * @param _ack I2C-Mastersignal, \a TWI_ACK können weitere Daten folgen, 
 * bei \a TWI_NACK wird danach die Übertragung gestoppt
 * @return \a TWI_ERROR Rück/Fehlermeldung
 */
TWI_ERROR TWI_read(uint8_t *data, enum TWI_ACKFLAG _ack);

/**
 * @brief Ein Byte in den TWI-BUS senden
 *
 * Schreibt ein Byte in den BUS. 
 * \n Beispiel: \n \code{.c}
 * if(TWI_write(MyByte))
 * {
 * 	//FEHLER
 * }
 * \endcode Im Beispiel wird ein Byte an die TWI_Peripherie gesendet.
 * @param data Zu sendende Byte
 * @return \a TWI_ERROR Rück/Fehlermeldung
 */
TWI_ERROR TWI_write(uint8_t data);

/**
 * @brief TWI BUS starten und Peripherie ansprechen
 *
 * Spricht die Peripherie mit der Adresse und dem Modus an.
 * \n Beispiel: \n \code{.c}
 * if(TWI_start(0x55 | TWI_WRITE))
 * {
 *		//FEHLER
 * }
 * \endcode Im Beispiel wird die TWI_Busübertragung gestartet und der Sensor in der Adresse 0x55 wird im Schreibmodus aufgerufen.
 * @param address Adresse des ICs und Modus (\a TWI_READ / \a TWI_WRITE)
 * @return Fehlermeldung bei Timeout
 */
TWI_ERROR TWI_start(uint8_t addr);


/**
 * @brief TWI-BUS schliessen und freigeben
 */
void TWI_stop();


TWI_ERROR TWI_wait_ACK();

#endif /* TWI-LIB_H_ */