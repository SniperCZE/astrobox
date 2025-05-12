/**
 *
 * Ovladani napajeciho modulu Astroboxu.
 * 
 * Procesor napajecího modulu Astroboxu má za úkol 
 * spustit napájení do dalších částí a zároveň přes
 * modul DHT11 hlídat vnitřní prostředí - Rh a T.
 * Pokud hodnoty teploty nebo Rh dosáhnou limitních
 * hodnot, spustí přes PWM řízení větráku.
 * Procesor se chová jako I2C slave a je možné z něj
 * vyčíst informace o aktuální T a Rh.
 *
 * Konfigurace pinů:
 *         ______
 * PB5 - 1 |    | 8 - VCC
 * PB3 - 2 |    | 7 - PB2
 * PB4 - 3 |    | 6 - PB1
 * GND - 4 |    | 5 - PB0
 *         ------
 * 
 * VCC, GND = Napájení
 * PB0 - I2C SDA
 * PB1 - Aktivace napájení v celém obvodu
 * PB2 - I2C SCL
 * PB3 - Signalizace přiliš vysoké Rh
 * PB4 - PWM ovládání větráčku
 * PB5 - Reset / Data z DHT11
 *
 * @author: Ondrej Flidr <ondrej.flidr@seznam.cz>
 * @version: 0.1
 * @date: 26/05/2024
 *
 */

#define F_CPU 1000000 // frekvence CPU na 1MHz (default, nepouzivam krystal)
#define I2C_ADDRESS 0x51 // I2C adresa na ktere posloucham
#define WANT_RELHUM 0x1 // Master pozaduje udaje o vlhkosti
#define WANT_TEMPER 0x2 // Master pozaduje udaje o teplote
#define DHT_TYPE DHT11 // Typ DHT senzoru
#define DHT_PIN 5 // Pin DHT senzoru

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <TinyWireS.h>
#include <TinyDHT.h>

int8_t requestType=0;
int8_t humidity;
int16_t temperature;

void i2c_request(void)
{
  if (requestType == WANT_RELHUM) {
    TinyWireS.send(0x58);
  } else if (requestType == WANT_TEMPER) {
    TinyWireS.send(0x47);
  }
}

void i2c_receive(uint8_t data)
{
  requestType = data;
}

int main(void)
{

  DDRB = 0b00001010;
  
  // Po zapnuti eletkriny sepnu relatka
  PORTB = 0b00000010;
  _delay_ms(200);

  TinyWireS.begin(I2C_ADDRESS);
  TinyWireS.onReceive(i2c_receive);
  TinyWireS.onRequest(i2c_request);

  DHT dht(DHT_PIN, DHT_TYPE);
  dht.begin();

  _delay_ms(500);


	while(1) {

    int8_t h = dht.readHumidity();
    int16_t t = dht.readTemperature();

    _delay_ms(500);

	}

}