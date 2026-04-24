/**
 *
 * Ovladani napajeciho modulu Astroboxu.
 *
 * Funkce:
 * - Cteni potenciometru na PA0 (ADC)
 * - PWM vystup na PB3 (Timer0)
 * - Ovladani vystupu pomoci PB0 (PWM enable) a PB1 (Output enable)
 * - Zobrazeni % (0-99) na 2x 7-seg displeji (PORTA - desitky, PORTD - jednotky)
 *
 * @author: Ondrej Flidr <ondrej.flidr@seznam.cz>
 * @version: 0.1
 * @date: 21/11/2025
 *
 */

#ifndef F_CPU
#define F_CPU 1000000UL // 1 MHz
#endif

#include <avr/io.h>
#include <util/delay.h>

// Definice pinu
#define PIN_PWM_ENABLE  PB0
#define PIN_OUTPUT_EN   PB1
#define PIN_PWM_OUT     PB3

// Makra pro cteni vstupu
#define IS_PWM_ENABLE_LOW()   bit_is_clear(PINB, PIN_PWM_ENABLE)
#define IS_PWM_ENABLE_HIGH()  bit_is_set(PINB, PIN_PWM_ENABLE)
#define IS_OUTPUT_EN_LOW()    bit_is_clear(PINB, PIN_OUTPUT_EN)
#define IS_OUTPUT_EN_HIGH()   bit_is_set(PINB, PIN_OUTPUT_EN)

/**
 * Mapovani segmentu na piny
 * Standardni 7-seg:      A, B, C, D, E, F, G
 * Index bitu (standard): 0, 1, 2, 3, 4, 5, 6
 *
 * Desitky (PORTA):
 * A->PA5, B->PA4, C->PA1, D->PA2, E->PA3, F->PA7, G->PA6
 *
 * Jednotky (PORTD):
 * A->PD7, B->PD6, C->PD4, D->PD2, E->PD1, F->PD5, G->PD3
 */

// Standardni definice cislic 0-9 (abcdefg)
// 0=0x3F, 1=0x06, 2=0x5B, 3=0x4F, 4=0x66, 5=0x6D, 6=0x7D, 7=0x07, 8=0x7F, 9=0x6F
const uint8_t digits_standard[] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

// Prevodni tabulky (vypoctene rucne pro optimalizaci)
// PORTA (Tens) - PA0 je input (ADC), takze maskujeme jen bity 1-7
const uint8_t segment_map_tens[] = {
    // 0: ABCDEF -> 5,4,1,2,3,7 -> 0b10111110 = 0xBE
    0xBE,
    // 1: BC -> 4,1 -> 0b00010010 = 0x12
    0x12,
    // 2: ABDEG -> 5,4,2,3,6 -> 0b01111100 = 0x7C
    0x7C,
    // 3: ABCDG -> 5,4,1,2,6 -> 0b01110110 = 0x76
    0x76,
    // 4: BCFG -> 4,1,7,6 -> 0b11010010 = 0xD2
    0xD2,
    // 5: ACDFG -> 5,1,2,7,6 -> 0b11100110 = 0xE6
    0xE6,
    // 6: ACDEFG -> 5,1,2,3,7,6 -> 0b11101110 = 0xEE
    0xEE,
    // 7: ABC -> 5,4,1 -> 0b00110010 = 0x32
    0x32,
    // 8: ABCDEFG -> vse krome 0 -> 0b11111110 = 0xFE
    0xFE,
    // 9: ABCDFG -> 5,4,1,2,7,6 -> 0b11110110 = 0xF6
    0xF6
};

// PORTD (Units) - PD0 nepripojen
const uint8_t segment_map_units[] = {
    // 0: ABCDEF -> 7,6,5,4,2,1 -> 0b11110110 = 0xF6
    0xF6,
    // 1: BC -> 6,4 -> 0b01010000 = 0x50
    0x50,
    // 2: ABDEG -> 7,6,2,1,3 -> 0b11001110 = 0xCE
    0xCE,
    // 3: ABCDG -> 7,6,4,2,3 -> 0b11011100 = 0xDC
    0xDC,
    // 4: BCFG -> 6,4,5,3 -> 0b01111000 = 0x78
    0x78,
    // 5: ACDFG -> 7,4,2,5,3 -> 0b10111100 = 0xBC
    0xBC,
    // 6: ACDEFG -> 7,4,2,1,5,3 -> 0b10111110 = 0xBE
    0xBE,
    // 7: ABC -> 7,6,4 -> 0b11010000 = 0xD0
    0xD0,
    // 8: ABCDEFG -> vse krome 0 -> 0b11111110 = 0xFE
    0xFE,
    // 9: ABCDFG -> 7,6,4,2,5,3 -> 0b11111100 = 0xFC
    0xFC
};

void adc_init() {
    // ADMUX: REFS0=0, REFS1=0 (AREF, Internal Vref turned off)
    //        MUX=00000 (ADC0/PA0)
    ADMUX = 0x00;

    // ADCSRA: ADEN=1 (Enable), ADPS=011 (Prescaler 8 -> 1MHz/8 = 125kHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read() {
    // Start conversion
    ADCSRA |= (1 << ADSC);
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void timer0_init() {
    // Fast PWM mode (WGM01 | WGM00)
    // Non-inverting mode (COM01=1, COM00=0) - Clear OC0 on match, set at BOTTOM
    // Clock: No prescaling (CS00=1) -> 1MHz PWM freq ~3.9kHz
    // Inicialne odpojime OC0 od pinu (bude se ovladat v loopu)
    TCCR0 = (1 << WGM00) | (1 << WGM01) | (1 << CS00);
    DDRB |= (1 << PIN_PWM_OUT); // PB3 jako vystup
}

void update_display(uint8_t percent) {
    if (percent > 99) percent = 99;

    uint8_t tens = percent / 10;
    uint8_t units = percent % 10;

    // PORTA: Zachovat PA0 (ADC input), prepsat PA1-PA7
    // PORTA input buffer (PIN A) by nemel byt ovlivnen, zapisujeme do LAT (PORT)
    // Ale PA0 je input bez pullupu (reseno v DDR), takze zapis 0 na PA0 je OK.
    // Pro jistotu cteme aktualni stav PORTA, zachovame bit 0 a zbytek pre napiseme.
    uint8_t current_porta = PORTA & 0x01;
    PORTA = current_porta | (segment_map_tens[tens] & 0xFE);

    // PORTD: PD0 nepripojen, muzeme prepsat, ale slusnost je zachovat
    uint8_t current_portd = PORTD & 0x01;
    PORTD = current_portd | (segment_map_units[units] & 0xFE);
}

int main(void) {
    // 1. Nastaveni portu
    // PORTA: PA0 Input (ADC), PA1-PA7 Output (Display Tens)
    DDRA = 0xFE; // 1111 1110
    PORTA = 0x00; // Vse LOW

    // PORTD: PD0 Input/Unused, PD1-PD7 Output (Display Units)
    DDRD = 0xFE; // 1111 1110
    PORTD = 0x00; // Vse LOW

    // PORTB: PB0, PB1 Input, PB3 Output (PWM)
    DDRB = (1 << PB3); // Ostatni jsou input (0)
    // Volitelne: Zapnout pull-up na vstupech, pokud nejsou externi?
    // Zadala se "Aktivace PWM", "Aktivace outputu", predpokladam aktivni logiku zvenku.
    // Necham bez pull-upu (High-Z), pokud neni specifikovano jinak.

    // 2. Inicializace periferii
    adc_init();
    timer0_init();

    uint16_t adc_val = 0;
    uint8_t percent = 0;
    uint8_t pwm_val = 0;

    while (1) {
        // Cteni ADC
        adc_val = adc_read(); // 0 - 1023

        // Prepocet
        // Procenta: (adc * 100) / 1024 ... zjednodusene adc/10.23
        // Pro lepsi presnost: (adc * 100) >> 10 je cca spravne, ale max 102300/1024 = 99.9
        percent = (adc_val * 100UL) / 1024UL;
        if (percent > 99) percent = 99;

        // PWM duty: mapovani 0-1023 na 0-255 -> deleno 4
        pwm_val = adc_val >> 2;

        // Aktualizace displeje
        update_display(percent);

        //// Logika rizeni vystupu PB3
        //if (IS_OUTPUT_EN_LOW()) {
        //    // PB1 == LOW -> Vystup vzdy LOW (Safety OFF)
        //    // Odpojit Timer od pinu
        //    TCCR0 &= ~(1 << COM01);
        //    // Nastavit pin LOW
        //    PORTB &= ~(1 << PIN_PWM_OUT);
        //}
        //else {
        //    // PB1 == HIGH
        //    if (IS_PWM_ENABLE_LOW()) {
        //        // PB0 == LOW -> Vystup trvale HIGH (Full ON)
        //        // Odpojit Timer od pinu
        //        TCCR0 &= ~(1 << COM01);
        //        // Nastavit pin HIGH
        //        PORTB |= (1 << PIN_PWM_OUT);
        //    }
        //    else {
        //        // PB1 == HIGH && PB0 == HIGH -> PWM Signal
        //        // Nastavit stridu
        //        OCR0 = pwm_val;
        //        // Pripojit Timer k pinu (Clear OC0 on Compare Match, Set at BOTTOM)
        //        TCCR0 |= (1 << COM01);
        //    }
        //}

        _delay_ms(10); // Maly delay pro stabilitu
    }
}
