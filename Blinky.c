#include "LPC11xx.h"
#include <stdint.h>

/* LPC1114 DevKit V5.6 - Keil MDK 4.74
 * UART 8O1 + IRQ, DS18B20 on PIO0_6, DS1307, Timer32_0,
 * SSP1 IRQ, SPI Flash logger, PIO1_9 LED and PIO3_5 key.
 * DS18B20 log starts at 0x011000 and uses the remaining 4KB sectors
 * of the 512KB Flash. Old NTC records at 0x010000 are preserved.
 * DS18B20 DQ is PIO0_6 with a 4.7k pull-up to 3.3V.
 * Timer16_0 provides microsecond timing for the 1-Wire bus.
 * Startup diagnostics include GPIO, presence pulse, ROM and CRC.
 */

#define RW       0xD0
#define RR       0xD1
#define AA       0x04
#define SI       0x08
#define STO      0x10
#define STA      0x20
#define IEN      0x40
#define LIMIT    300000UL
#define SPI_LIM  2000000UL
#define BASE     0x011000UL /* Preserve old NTC log at 0x010000 */
#define LOG_END  0x080000UL /* JEDEC 0B 40 13: 4Mbit / 512KB */
#define SECTOR_SIZE 4096UL
#define RECORDS_PER_SECTOR 409U
#define COUNT_MAX (((LOG_END - BASE) / SECTOR_SIZE) * RECORDS_PER_SECTOR)

/* Verified against LPC1114 DevKit V5.6 schematic: D2/K4. */
#define BOARD_LED_ENABLED 1
#define BOARD_KEY_ENABLED 1
#define BOARD_LED_PORT 1
#define BOARD_LED_PIN 9
#define BOARD_KEY_PORT 3
#define BOARD_KEY_PIN 5
#define BOARD_LED_ACTIVE_LOW 1
#define BOARD_KEY_ACTIVE_LOW 1
#define CS0()    (LPC_GPIO2->DATA &= ~1UL)
#define CS1()    (LPC_GPIO2->DATA |= 1UL)

typedef struct {
    uint8_t y, m, d, h, n, s;
} RTC;

volatile uint8_t tick;
volatile uint8_t ds18b20_ready;
volatile int16_t ds18b20_temp_x10;
volatile uint8_t spi_done, spi_rx, spi_err;
volatile char rxbuf[20];
volatile uint8_t rxlen, cmd_ready;

uint16_t records;
uint8_t log_error;
uint16_t dump_index, dump_end;
uint8_t dump_active;
volatile uint8_t record_enable = 1;
volatile uint8_t seconds_count = 0;
volatile uint8_t key_check = 0; /* 10 ms key samples pending */


/* ---------- Optional status LED and button ---------- */
/* GPIO ports 0..3 are supported by LPC1114.
 * The default build leaves these pins untouched until verified. */
#if BOARD_LED_ENABLED || BOARD_KEY_ENABLED
static LPC_GPIO_TypeDef *board_gpio(uint8_t port)
{
    switch (port) {
    case 0: return LPC_GPIO0;
    case 1: return LPC_GPIO1;
    case 2: return LPC_GPIO2;
    default: return LPC_GPIO3;
    }
}
#endif

void Board_Init(void)
{
#if BOARD_LED_ENABLED
    LPC_SYSCON->SYSAHBCLKCTRL |= (1UL << 6);
    LPC_IOCON->PIO1_9 = (LPC_IOCON->PIO1_9 & ~7UL);
    board_gpio(BOARD_LED_PORT)->DIR |= (1UL << BOARD_LED_PIN);
    if (BOARD_LED_ACTIVE_LOW)
        board_gpio(BOARD_LED_PORT)->DATA |= (1UL << BOARD_LED_PIN);
    else
        board_gpio(BOARD_LED_PORT)->DATA &= ~(1UL << BOARD_LED_PIN);
#endif
#if BOARD_KEY_ENABLED
    LPC_SYSCON->SYSAHBCLKCTRL |= (1UL << 6);
    /* K4 has an external 10k pull-up on PIO3_5. */
    LPC_IOCON->PIO3_5 = (LPC_IOCON->PIO3_5 & ~7UL);
    board_gpio(BOARD_KEY_PORT)->DIR &= ~(1UL << BOARD_KEY_PIN);
#endif
}

void Board_Update(void)
{
#if BOARD_LED_ENABLED
    uint8_t on = (record_enable && !log_error &&
                  records < COUNT_MAX && (seconds_count & 1U));
    uint8_t high = BOARD_LED_ACTIVE_LOW ? !on : on;
    if (high) board_gpio(BOARD_LED_PORT)->DATA |= (1UL << BOARD_LED_PIN);
    else board_gpio(BOARD_LED_PORT)->DATA &= ~(1UL << BOARD_LED_PIN);
#endif
#if BOARD_KEY_ENABLED
    static uint8_t previous = 0, consecutive = 0, latched = 0;
    uint8_t raw = (board_gpio(BOARD_KEY_PORT)->DATA &
                    (1UL << BOARD_KEY_PIN)) ? 1 : 0;
    uint8_t pressed = BOARD_KEY_ACTIVE_LOW ? !raw : raw;
    if (pressed == previous) { if (consecutive < 3) ++consecutive; }
    else { previous = pressed; consecutive = 0; }
    if (consecutive >= 3) {
        if (pressed && !latched) {
            record_enable = !record_enable;
            latched = 1;
        } else if (!pressed) latched = 0;
    }
#endif
}

/* Key debounce timing only; does not perform temperature sampling. */
void SysTick_Handler(void)
{
    if (key_check < 10) key_check++;
}

void KeyTimer_Init(void)
{
    /* 10ms tick, based on SystemCoreClock. */
    if (SysTick_Config(SystemCoreClock / 100UL)) {
        while (1) { /* invalid SysTick reload */ }
    }
}

/* ---------- UART ---------- */

void putc1(uint8_t c)
{
    while (!(LPC_UART->LSR & 0x20));
    LPC_UART->THR = c;
}

void puts1(const char *s)
{
    while (*s) putc1((uint8_t)*s++);
}

void num(uint32_t x)
{
    char b[11];
    uint8_t i = 0;
    do {
        b[i++] = (char)('0' + x % 10);
        x /= 10;
    } while (x);
    while (i) putc1(b[--i]);
}

void two(uint8_t x)
{
    putc1('0' + x / 10);
    putc1('0' + x % 10);
}

void hex8(uint8_t x)
{
    const char h[] = "0123456789ABCDEF";
    putc1(h[x >> 4]);
    putc1(h[x & 15]);
}

void tempout(int32_t t)
{
    uint32_t x;
    if (t < 0) {
        putc1('-');
        x = (uint32_t)(-t);
    } else x = (uint32_t)t;
    num(x / 10);
    putc1('.');
    putc1('0' + x % 10);
}

void UART_Init(void)
{
    LPC_SYSCON->SYSAHBCLKCTRL |= (1UL << 16) | (1UL << 12);
    LPC_IOCON->PIO1_6 = (LPC_IOCON->PIO1_6 & ~7UL) | 1UL;
    LPC_IOCON->PIO1_7 = (LPC_IOCON->PIO1_7 & ~7UL) | 1UL;
    LPC_SYSCON->UARTCLKDIV = 4;
    LPC_UART->LCR = 0x8B;          /* 8 data, odd parity, DLAB */
    LPC_UART->DLL = 4;
    LPC_UART->DLM = 0;
    LPC_UART->FDR = 0x85;
    LPC_UART->LCR = 0x0B;
    LPC_UART->FCR = 7;
    LPC_UART->TER = 0x80;
    LPC_UART->IER = 1;
    NVIC_EnableIRQ(UART_IRQn);
}

void UART_IRQHandler(void)
{
    uint8_t c;
    while (LPC_UART->LSR & 1) {
        c = (uint8_t)LPC_UART->RBR;
        if (cmd_ready) continue;

        if (c == '\r' || c == '\n') {
            if (rxlen) {
                rxbuf[rxlen] = 0;
                cmd_ready = 1;
            }
            continue;
        }

        if (!rxlen) {
            if (c == 'R' || c == 'H' || c == 'C' || c == '?' ||
                c == 'S' || c == 'P' || c == 'Q' || c == 'D') {
                rxbuf[0] = c;
                rxbuf[1] = 0;
                rxlen = cmd_ready = 1;
                continue;
            }
            if (c != 'T' && c != 'E') continue;
        }

        if (rxlen >= 19) {
            rxlen = 0;
            continue;
        }

        rxbuf[rxlen++] = (char)c;
        if ((rxbuf[0] == 'T' && rxlen == 13) ||
            (rxlen == 5 && rxbuf[0] == 'E' &&
             rxbuf[1] == 'R' && rxbuf[2] == 'A' &&
             rxbuf[3] == 'S' && rxbuf[4] == 'E')) {
            rxbuf[rxlen] = 0;
            cmd_ready = 1;
        }
    }
}

/* ---------- I2C / DS1307 ---------- */

void I2C_Init(void)
{
    LPC_SYSCON->SYSAHBCLKCTRL |= (1UL << 16) | (1UL << 5);
    LPC_SYSCON->PRESETCTRL |= 1UL << 1;
    LPC_IOCON->PIO0_4 = (LPC_IOCON->PIO0_4 & ~0x3FUL) | 1UL;
    LPC_IOCON->PIO0_5 = (LPC_IOCON->PIO0_5 & ~0x3FUL) | 1UL;
    LPC_I2C->CONCLR = AA | SI | STA | IEN;
    LPC_I2C->SCLH = 240;
    LPC_I2C->SCLL = 240;
    LPC_I2C->CONSET = IEN;
}

uint8_t iw(void)
{
    uint32_t n = LIMIT;
    while (!(LPC_I2C->CONSET & SI))
        if (--n == 0) return 0xFF;
    return (uint8_t)LPC_I2C->STAT;
}

uint8_t istopwait(void)
{
    uint32_t n = LIMIT;
    while (LPC_I2C->CONSET & STO)
        if (--n == 0) return 0;
    return 1;
}

void istop(void)
{
    LPC_I2C->CONSET = STO;
    LPC_I2C->CONCLR = STA | SI;
}

uint8_t istart(void)
{
    LPC_I2C->CONCLR = AA | SI | STA;
    LPC_I2C->CONSET = STA;
    return iw();
}

uint8_t irestart(void)
{
    LPC_I2C->CONSET = STA;
    LPC_I2C->CONCLR = SI;
    return iw();
}

uint8_t isend(uint8_t x)
{
    LPC_I2C->DAT = x;
    LPC_I2C->CONCLR = STA | SI;
    return iw();
}

uint8_t iread(uint8_t *x, uint8_t ack)
{
    uint8_t st;
    if (ack) LPC_I2C->CONSET = AA;
    else LPC_I2C->CONCLR = AA;
    LPC_I2C->CONCLR = SI;
    st = iw();
    if (st == (ack ? 0x50 : 0x58))
        *x = (uint8_t)LPC_I2C->DAT;
    return st;
}

uint8_t bcd(uint8_t x)
{
    return (uint8_t)(((x / 10) << 4) | (x % 10));
}

uint8_t dec(uint8_t x)
{
    return (uint8_t)((x >> 4) * 10 + (x & 15));
}

/* Preserve the START/RESTART sequence tested on the board. */
uint8_t RTC_Read(RTC *t)
{
    uint8_t b[7], i, h;
    if (!istopwait()) return 0;
    if (istart() != 0x08) goto fail;
    if (isend(RW) != 0x18) goto fail;
    if (isend(0) != 0x28) goto fail;
    if (irestart() != 0x10) goto fail;
    if (isend(RR) != 0x40) goto fail;

    for (i = 0; i < 7; i++)
        if (iread(&b[i], i < 6) != (i < 6 ? 0x50 : 0x58))
            goto fail;

    istop();
    if (b[0] & 0x80) return 0;

    t->s = dec(b[0] & 0x7F);
    t->n = dec(b[1] & 0x7F);
    if (b[2] & 0x40) {
        h = dec(b[2] & 0x1F);
        if (h == 12) h = 0;
        if (b[2] & 0x20) h += 12;
    } else h = dec(b[2] & 0x3F);
    t->h = h;
    t->d = dec(b[4] & 0x3F);
    t->m = dec(b[5] & 0x1F);
    t->y = dec(b[6]);

    return t->s < 60 && t->n < 60 && t->h < 24 &&
           t->m >= 1 && t->m <= 12 &&
           t->d >= 1 && t->d <= 31;

fail:
    istop();
    return 0;
}

uint8_t RTC_Set(const RTC *t)
{
    uint8_t b[7], i;
    b[0] = bcd(t->s) & 0x7F;
    b[1] = bcd(t->n);
    b[2] = bcd(t->h);
    b[3] = 1;
    b[4] = bcd(t->d);
    b[5] = bcd(t->m);
    b[6] = bcd(t->y);

    if (!istopwait()) return 0;
    if (istart() != 0x08) goto fail;
    if (isend(RW) != 0x18) goto fail;
    if (isend(0) != 0x28) goto fail;
    for (i = 0; i < 7; i++)
        if (isend(b[i]) != 0x28) goto fail;

    istop();
    return istopwait();
fail:
    istop();
    return 0;
}

/* ---------- DS18B20 1-Wire on PIO0_6 ---------- */
#define OW_PIN       (1UL << 6)
#define DS18B20_ROM_SKIP  0xCC
#define DS18B20_CONVERT   0x44
#define DS18B20_READ      0xBE
#define DS18B20_WRITE     0x4E

/*
 * DS18B20 DQ connects to PIO0_6 with a 4.7k pull-up to 3.3V.
 * PIO0_6 is used as GPIO (not SCK0).
 * Timer16_0 runs from the 48MHz peripheral clock and is used as a
 * free-running microsecond reference (project clock is 48MHz). The
 * existing TIMER32_0 remains the one-second sampling scheduler.
 */
static void OW_DriveLow(void)
{
    /*
     * IMPORTANT for LPC1114 GPIO:
     * While PIO0_6 is configured as input, writes to GPIOnDATA do not
     * change the pad level. When switching input -> output, the pin can
     * initially drive the sampled input level (HIGH due to R22 pull-up).
     * Therefore enable output first, then write LOW to the DATA latch.
     * The low write follows immediately and is required for the bus to
     * produce a real low pulse. OW_Release() returns the pin to input.
     */
    LPC_GPIO0->DIR |= OW_PIN;
    LPC_GPIO0->DATA &= ~OW_PIN;
}

static void OW_Release(void)
{
    /* Input mode releases the open-drain 1-Wire bus; external 4.7k pulls high. */
    LPC_GPIO0->DIR &= ~OW_PIN;
}

static uint8_t OW_ReadPin(void)
{
    return (LPC_GPIO0->DATA & OW_PIN) ? 1U : 0U;
}

/* Last reset-cycle measurements, printed only when startup detection fails. */
static void OW_DelayUs(uint16_t us);

static uint8_t ow_diag_idle = 0xFFU;
static uint8_t ow_diag_drive_low = 0xFFU;
static uint8_t ow_diag_release = 0xFFU;
static uint8_t ow_presence_samples = 0U;
static uint16_t ow_presence_first_us = 0xFFFFU;
static uint16_t ow_presence_last_us = 0xFFFFU;
static uint8_t ow_write1_at15 = 0xFFU;
static uint8_t ow_write0_at30 = 0xFFU;
static uint8_t ow_rom_bit0_levels = 0U;
static uint16_t ow_rom_bit0_ticks[6];
static uint16_t ow_ticks_per_us;

static void OW_LineDiagnostic(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    /* Bus should be high when released by the master. */
    OW_Release();
    OW_DelayUs(10);
    ow_diag_idle = OW_ReadPin();

    /* Verify that GPIO0.6 can actively pull the bus low. */
    OW_DriveLow();
    OW_DelayUs(10);
    ow_diag_drive_low = OW_ReadPin();

    /* Verify that input mode releases the line to the pull-up. */
    OW_Release();
    OW_DelayUs(10);
    ow_diag_release = OW_ReadPin();

    if (!primask) __enable_irq();

    puts1("\r\n1-Wire GPIO diagnostic\r\n");
    puts1("DQ_IDLE=");
    putc1(ow_diag_idle ? '1' : '0');
    puts1("\r\nDQ_DRIVE_LOW=");
    putc1(ow_diag_drive_low ? '1' : '0');
    puts1("\r\nDQ_RELEASE=");
    putc1(ow_diag_release ? '1' : '0');
    puts1("\r\n");
}

static void OW_DelayUs(uint16_t us)
{
    uint16_t start = (uint16_t)LPC_TMR16B0->TC;
    uint16_t ticks = (uint16_t)(ow_ticks_per_us * us);
    while ((uint16_t)((uint16_t)LPC_TMR16B0->TC - start) < ticks) {
        /* Hardware timer provides the delay. */
    }
}

/* Wait for a deadline measured from the start of the 1-Wire slot.
 * Separate delays accumulate C call overhead and can sample after the
 * DS18B20's 15us read-data window. All deadlines here are below one
 * 16-bit timer wrap at 48MHz. */
static void OW_WaitUntil(uint16_t start, uint16_t us)
{
    uint16_t ticks = (uint16_t)(ow_ticks_per_us * us);
    while ((uint16_t)((uint16_t)LPC_TMR16B0->TC - start) < ticks) {
        /* Hardware timer provides the deadline. */
    }
}

void DS18B20_Init(void)
{
    /* Cortex-M0 has no hardware divide. Compute this before any 1-Wire
     * slot; software division inside a slot misses the 15us read window. */
    ow_ticks_per_us = (uint16_t)(SystemCoreClock / 1000000UL);
    LPC_SYSCON->SYSAHBCLKCTRL |= (1UL << 6) | (1UL << 7) | (1UL << 16);

    /* IOCON function 0 = GPIO, default pull-up mode; external 4.7k is fitted. */
    LPC_IOCON->PIO0_6 = 0x00;
    LPC_GPIO0->DATA |= OW_PIN;
    LPC_GPIO0->DIR &= ~OW_PIN;

    /* CT16B0 free-running counter, one TC count per 48MHz clock. */
    LPC_TMR16B0->TCR = 0;
    LPC_TMR16B0->TCR = 2;
    LPC_TMR16B0->PR = 0;
    LPC_TMR16B0->PC = 0;
    LPC_TMR16B0->TC = 0;
    LPC_TMR16B0->MCR = 0;
    LPC_TMR16B0->TCR = 1;
}

static uint8_t OW_Reset(void)
{
    uint8_t line_released, present;
    uint16_t start, elapsed, first = 0xFFFFU, last = 0U;
    uint16_t ticks_per_us = ow_ticks_per_us;
    uint32_t primask = __get_PRIMASK();

    ow_presence_samples = 0U;
    ow_presence_first_us = 0xFFFFU;
    ow_presence_last_us = 0xFFFFU;

    /*
     * DS18B20 reset: host low >=480us.  Use 500us for margin.
     * Time each low sample against the actual hardware timer, not the
     * number of C loop iterations. A presence pulse must follow an
     * initial high level and have a plausible duration.
     */
    __disable_irq();
    OW_DriveLow();
    OW_DelayUs(500);
    start = (uint16_t)LPC_TMR16B0->TC;
    OW_Release();
    OW_WaitUntil(start, 10);
    line_released = OW_ReadPin();

    do {
        elapsed = (uint16_t)((uint16_t)LPC_TMR16B0->TC - start);
        if (OW_ReadPin() == 0U) {
            if (first == 0xFFFFU) first = elapsed;
            last = elapsed;
            if (ow_presence_samples < 255U)
                ow_presence_samples++;
        }
    } while (elapsed < (uint16_t)(300U * ticks_per_us));

    if (first != 0xFFFFU) {
        ow_presence_first_us = (uint16_t)(first / ticks_per_us);
        ow_presence_last_us = (uint16_t)(last / ticks_per_us);
    }
    present = line_released && first != 0xFFFFU &&
              first >= (uint16_t)(12U * ticks_per_us) &&
              first <= (uint16_t)(75U * ticks_per_us) &&
              (uint16_t)(last - first) >= (uint16_t)(45U * ticks_per_us) &&
              (uint16_t)(last - first) <= (uint16_t)(250U * ticks_per_us);

    if (!primask) __enable_irq();

    /* Complete the 480us reset-high/recovery interval after release. */
    OW_WaitUntil(start, 480);
    return present;
}

static void OW_WriteBit(uint8_t bit)
{
    uint16_t start;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    OW_DriveLow();
    start = (uint16_t)LPC_TMR16B0->TC;
    if (bit) {
        OW_WaitUntil(start, 6);
        OW_Release();
        OW_WaitUntil(start, 15);
        if (ow_write1_at15 == 0xFFU)
            ow_write1_at15 = OW_ReadPin();
    } else {
        OW_WaitUntil(start, 30);
        if (ow_write0_at30 == 0xFFU)
            ow_write0_at30 = OW_ReadPin();
        OW_WaitUntil(start, 60);
        OW_Release();
    }
    OW_WaitUntil(start, 70);
    if (!primask) __enable_irq();
}

static uint8_t OW_ReadBit(void)
{
    uint8_t bit;
    uint16_t start;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    OW_DriveLow();
    start = (uint16_t)LPC_TMR16B0->TC;
    OW_WaitUntil(start, 3);
    OW_Release();
    OW_WaitUntil(start, 12);
    bit = OW_ReadPin();
    OW_WaitUntil(start, 70);
    if (!primask) __enable_irq();
    return bit;
}

/* Diagnose the first ROM bit (DS18B20 family 0x28, LSB = 0) without
 * transmitting on UART until the complete ROM transaction is finished. */
static uint8_t OW_ReadFirstROMBit(void)
{
    static const uint8_t sample_us[6] = {8, 12, 15, 20, 30, 45};
    uint8_t i, bit;
    uint16_t start;
    uint32_t primask = __get_PRIMASK();

    ow_rom_bit0_levels = 0;
    __disable_irq();
    OW_DriveLow();
    start = (uint16_t)LPC_TMR16B0->TC;
    OW_WaitUntil(start, 3);
    OW_Release();
    for (i = 0; i < 6; i++) {
        OW_WaitUntil(start, sample_us[i]);
        if (OW_ReadPin()) ow_rom_bit0_levels |= (uint8_t)(1U << i);
        ow_rom_bit0_ticks[i] =
            (uint16_t)((uint16_t)LPC_TMR16B0->TC - start);
    }
    bit = (ow_rom_bit0_levels & (1U << 1)) ? 1U : 0U;
    OW_WaitUntil(start, 70);
    if (!primask) __enable_irq();
    return bit;
}

static void OW_WriteByte(uint8_t value)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        OW_WriteBit(value & 1U);
        value >>= 1;
    }
}

static uint8_t OW_ReadByte(void)
{
    uint8_t i, value = 0;
    for (i = 0; i < 8; i++) {
        if (OW_ReadBit()) value |= (uint8_t)(1U << i);
    }
    return value;
}

static uint8_t DS18B20_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t i, j, crc = 0;
    for (i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (j = 0; j < 8; j++) {
            uint8_t mix = (uint8_t)((crc ^ inbyte) & 1U);
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

/* Read the 64-bit ROM code of the single device on this 1-Wire bus.
 * Command 0x33 is valid only when exactly one device is connected.
 */
uint8_t DS18B20_ReadROM(uint8_t rom[8])
{
    uint8_t i, bit;

    if (!OW_Reset())
        return 0;

    OW_WriteByte(0x33);
    rom[0] = OW_ReadFirstROMBit();
    for (bit = 1; bit < 8; bit++)
        if (OW_ReadBit()) rom[0] |= (uint8_t)(1U << bit);
    for (i = 1; i < 8; i++)
        rom[i] = OW_ReadByte();

    return 1;
}

/* Print ROM bytes and verify family code and Dallas CRC. */
uint8_t DS18B20_ROM_Diagnostic(void)
{
    uint8_t rom[8];
    uint8_t crc;
    uint8_t i;

    puts1("\r\n=== DS18B20 ROM DIAGNOSTIC ===\r\n");

    if (!DS18B20_ReadROM(rom)) {
        puts1("ROM RESET/PRESENCE ERROR\r\n");
        return 0;
    }

    puts1("ROM DATA: ");
    for (i = 0; i < 8; i++) {
        hex8(rom[i]);
        putc1(' ');
    }
    puts1("\r\n");

    puts1("WRITE1 DQ@15us=");
    putc1((char)('0' + ow_write1_at15));
    puts1(" WRITE0 DQ@30us=");
    putc1((char)('0' + ow_write0_at30));
    puts1("\r\nROM BIT0 samples (actual us:level): ");
    for (i = 0; i < 6; i++) {
        num(ow_rom_bit0_ticks[i] / ow_ticks_per_us);
        putc1(':');
        putc1((ow_rom_bit0_levels & (1U << i)) ? '1' : '0');
        putc1(' ');
    }
    puts1("\r\n");

    crc = DS18B20_CRC8(rom, 7);
    puts1("ROM CRC CALC=");
    hex8(crc);
    puts1(" ROM CRC READ=");
    hex8(rom[7]);
    puts1("\r\n");

    if (crc != rom[7]) {
        puts1("ROM CRC MISMATCH\r\n");
        return 0;
    }

    if (rom[0] != 0x28) {
        puts1("ROM FAMILY CODE NOT DS18B20: ");
        hex8(rom[0]);
        puts1("\r\n");
        return 0;
    }

    puts1("DS18B20 ROM IDENTIFIED OK\r\n");
    return 1;
}

/* Configure 12-bit resolution (750ms maximum conversion time). */
uint8_t DS18B20_Configure(void)
{
    if (!OW_Reset()) return 0;
    OW_WriteByte(DS18B20_ROM_SKIP);
    OW_WriteByte(DS18B20_WRITE);
    OW_WriteByte(0x4B); /* TH = +75C */
    OW_WriteByte(0x46); /* TL = +70C */
    OW_WriteByte(0x7F); /* 12-bit resolution */
    return 1;
}

/* Start a conversion; the main loop reads its result at the next 1s tick. */
uint8_t DS18B20_StartConversion(void)
{
    if (!OW_Reset()) return 0;
    OW_WriteByte(DS18B20_ROM_SKIP);
    OW_WriteByte(DS18B20_CONVERT);
    return 1;
}

/* Read scratchpad, verify Dallas CRC, return temperature in 0.1C units. */
uint8_t DS18B20_ReadTemperature(int16_t *temp_x10)
{
    uint8_t sp[9], i;
    uint8_t crc;
    int16_t raw;
    int32_t scaled;

    if (!OW_Reset()) {
        puts1("SCRATCHPAD RESET/PRESENCE ERROR\r\n");
        return 0;
    }

    OW_WriteByte(DS18B20_ROM_SKIP);
    OW_WriteByte(DS18B20_READ);
    for (i = 0; i < 9; i++)
        sp[i] = OW_ReadByte();

    crc = DS18B20_CRC8(sp, 8);
    if (crc != sp[8]) {
        puts1("SCRATCHPAD: ");
        for (i = 0; i < 9; i++) {
            hex8(sp[i]);
            putc1(' ');
        }
        puts1("\r\nCRC_CALC=");
        hex8(crc);
        puts1(" CRC_READ=");
        hex8(sp[8]);
        puts1("\r\nSCRATCHPAD CRC ERROR\r\n");
        return 0;
    }

    /* DS18B20 raw temperature uses 1/16 degree Celsius per LSB. */
    raw = (int16_t)(((uint16_t)sp[1] << 8) | sp[0]);
    if (raw < -880 || raw > 2000) {
        puts1("SCRATCHPAD TEMPERATURE OUT OF RANGE\r\n");
        return 0;
    }

    if (raw >= 0) scaled = ((int32_t)raw * 10 + 8) / 16;
    else scaled = -(((-((int32_t)raw)) * 10 + 8) / 16);

    *temp_x10 = (int16_t)scaled;
    return 1;
}

/* ---------- 1-second timer ---------- */

void TIMER_Init(void)
{
    LPC_SYSCON->SYSAHBCLKCTRL |= 1UL << 9;
    LPC_TMR32B0->TCR = 0;
    LPC_TMR32B0->TCR = 2;
    LPC_TMR32B0->PR = 47999;
    LPC_TMR32B0->MR0 = 999;
    LPC_TMR32B0->MCR = 3;
    LPC_TMR32B0->IR = 1;
    NVIC_EnableIRQ(TIMER_32_0_IRQn);
    LPC_TMR32B0->TCR = 1;
}

void TIMER32_0_IRQHandler(void)
{
    if (LPC_TMR32B0->IR & 1) {
        LPC_TMR32B0->IR = 1;
        tick = 1;
        seconds_count++;
    }
}

/* ---------- SSP1, Mode 0, interrupt ---------- */

void SPI_Init(void)
{
    volatile uint32_t dummy;
    LPC_SYSCON->SYSAHBCLKCTRL |=
        (1UL << 16) | (1UL << 6) | (1UL << 18);
    LPC_SYSCON->PRESETCTRL |= 1UL << 2;
    LPC_SYSCON->SSP1CLKDIV = 1;

    LPC_IOCON->PIO2_0 &= ~7UL;
    LPC_GPIO2->DATA |= 1;
    LPC_GPIO2->DIR |= 1;
    LPC_IOCON->PIO2_1 = (LPC_IOCON->PIO2_1 & ~7UL) | 2;
    LPC_IOCON->PIO2_2 = (LPC_IOCON->PIO2_2 & ~7UL) | 2;
    LPC_IOCON->PIO2_3 = (LPC_IOCON->PIO2_3 & ~7UL) | 2;

    LPC_SSP1->CR1 = 0;
    LPC_SSP1->CR0 = 7;
    LPC_SSP1->CPSR = 24;
    LPC_SSP1->IMSC = 0;
    LPC_SSP1->ICR = 3;
    while (LPC_SSP1->SR & 4) {
        dummy = LPC_SSP1->DR;
        (void)dummy;
    }
    NVIC_ClearPendingIRQ(SSP1_IRQn);
    NVIC_EnableIRQ(SSP1_IRQn);
    LPC_SSP1->CR1 = 2;
}

void SSP1_IRQHandler(void)
{
    uint32_t st = LPC_SSP1->MIS;
    uint8_t got = 0;
    if (st & 1) spi_err = 1;
    if (st & 3) {
        while (LPC_SSP1->SR & 4) {
            spi_rx = (uint8_t)LPC_SSP1->DR;
            got = 1;
        }
        LPC_SSP1->ICR = 3;
        LPC_SSP1->IMSC = 0;
        if (got) spi_done = 1;
    }
}

/* One byte, reception completed by SSP1 IRQ. */
uint8_t spi(uint8_t tx, uint8_t *rx)
{
    uint32_t n = SPI_LIM;
    volatile uint32_t dummy;

    while (LPC_SSP1->SR & 16)
        if (--n == 0) return 0;

    while (LPC_SSP1->SR & 4) {
        dummy = LPC_SSP1->DR;
        (void)dummy;
    }

    spi_done = spi_err = 0;
    LPC_SSP1->ICR = 3;
    NVIC_ClearPendingIRQ(SSP1_IRQn);
    LPC_SSP1->IMSC = 3;

    n = SPI_LIM;
    while (!(LPC_SSP1->SR & 2))
        if (--n == 0) goto fail;

    LPC_SSP1->DR = tx;
    n = SPI_LIM;
    while (!spi_done && !spi_err)
        if (--n == 0) goto fail;

    if (spi_err) goto fail;
    *rx = spi_rx;
    return 1;
fail:
    LPC_SSP1->IMSC = 0;
    return 0;
}

void spi_end(void)
{
    uint32_t n = SPI_LIM;
    while (LPC_SSP1->SR & 16)
        if (--n == 0) break;
    CS1();
}

uint8_t spi_addr(uint32_t a)
{
    uint8_t d;
    return spi((uint8_t)(a >> 16), &d) &&
           spi((uint8_t)(a >> 8), &d) &&
           spi((uint8_t)a, &d);
}

/* ---------- Flash ---------- */

uint8_t flash_id(uint8_t id[3])
{
    uint8_t d, ok;
    CS0();
    ok = spi(0x9F, &d);
    if (ok) ok = spi(0xFF, &id[0]);
    if (ok) ok = spi(0xFF, &id[1]);
    if (ok) ok = spi(0xFF, &id[2]);
    spi_end();
    return ok;
}

uint8_t flash_status(uint8_t *s)
{
    uint8_t d, ok;
    CS0();
    ok = spi(5, &d);
    if (ok) ok = spi(0xFF, s);
    spi_end();
    return ok;
}

uint8_t flash_ready(void)
{
    uint32_t n;
    uint8_t s;
    for (n = 0; n < 100000UL; n++)
        if (!flash_status(&s)) return 0;
        else if (!(s & 1)) return 1;
    return 0;
}

uint8_t flash_wren(void)
{
    uint8_t d, s, ok;
    if (!flash_ready()) return 0;
    CS0();
    ok = spi(6, &d);
    spi_end();
    return ok && flash_status(&s) && (s & 2);
}

uint8_t flash_erase(uint32_t a)
{
    uint8_t d, ok;
    if (a < BASE || a >= LOG_END || (a & (SECTOR_SIZE - 1UL)))
        return 0;
    if (!flash_wren()) return 0;
    CS0();
    ok = spi(0x20, &d);
    if (ok) ok = spi_addr(a);
    spi_end();
    return ok && flash_ready();
}

uint8_t flash_read(uint32_t a, uint8_t *b, uint16_t n)
{
    uint8_t d, ok;
    uint16_t i;
    CS0();
    ok = spi(3, &d);
    if (ok) ok = spi_addr(a);
    for (i = 0; ok && i < n; i++)
        ok = spi(0xFF, &b[i]);
    spi_end();
    return ok;
}

/* Splits writes automatically at 256-byte boundaries. */
uint8_t flash_write(uint32_t a, const uint8_t *b, uint16_t n)
{
    uint8_t d, ok;
    uint16_t k, i;
    while (n) {
        k = (uint16_t)(256UL - (a & 255UL));
        if (k > n) k = n;
        if (!flash_wren()) return 0;

        CS0();
        ok = spi(2, &d);
        if (ok) ok = spi_addr(a);
        for (i = 0; ok && i < k; i++)
            ok = spi(b[i], &d);
        spi_end();
        if (!ok || !flash_ready()) return 0;

        a += k;
        b += k;
        n -= k;
    }
    return 1;
}

/* ---------- 10-byte DS18B20 Flash log (legacy NTC sector preserved) ---------- */

uint32_t rec_addr(uint16_t i)
{
    /* The first 409 addresses are unchanged from the original firmware.
     * Six unused bytes at each sector end keep records inside one sector. */
    return BASE + ((uint32_t)i / RECORDS_PER_SECTOR) * SECTOR_SIZE +
           ((uint32_t)i % RECORDS_PER_SECTOR) * 10UL;
}

uint8_t flash_blank(uint32_t a, uint16_t n)
{
    uint8_t b[16], i;
    uint16_t k;
    while (n) {
        k = n > sizeof b ? sizeof b : n;
        if (!flash_read(a, b, k)) return 0;
        for (i = 0; i < k; i++)
            if (b[i] != 0xFF) return 0;
        a += k;
        n -= k;
    }
    return 1;
}

uint8_t checksum(const uint8_t *b)
{
    uint8_t x = 0, i;
    for (i = 0; i < 9; i++) x ^= b[i];
    return x;
}

uint8_t rec_read(uint16_t i, uint8_t *b)
{
    return i < COUNT_MAX && flash_read(rec_addr(i), b, 10);
}

uint8_t rec_valid(const uint8_t *b)
{
    int16_t temp_x10 = (int16_t)((uint16_t)b[7] |
                                  ((uint16_t)b[8] << 8));
    return b[0] == 0xD8 && b[9] == checksum(b) &&
           temp_x10 >= -550 && temp_x10 <= 1250 &&
           b[2] >= 1 && b[2] <= 12 &&
           b[3] >= 1 && b[3] <= 31 && b[4] <= 23 &&
           b[5] <= 59 && b[6] <= 59;
}

/* On boot, restore the count without deleting old records. */
uint8_t log_init(void)
{
    uint8_t b[10], j, empty;
    uint16_t i;
    uint32_t a, sector_end;

    records = 0;
    log_error = 0;

    for (i = 0; i < COUNT_MAX; i++) {
        if (!rec_read(i, b)) goto fail;

        empty = 1;
        for (j = 0; j < 10; j++)
            if (b[j] != 0xFF) empty = 0;

        if (empty) {
            /* A partially filled sector must have an erased tail. Later
             * sectors are checked only when recording reaches them. */
            a = rec_addr(i);
            sector_end = BASE +
                (((uint32_t)i / RECORDS_PER_SECTOR) + 1UL) * SECTOR_SIZE;
            if (!flash_blank(a, (uint16_t)(sector_end - a))) goto fail;
            return 1;
        }

        if (!rec_valid(b)) goto fail;
        records++;
    }
    return 1;
fail:
    log_error = 1;
    return 0;
}

uint8_t log_save(const RTC *t, int16_t temp_x10)
{
    uint8_t b[10], check[10], i;
    uint32_t a;
    if (log_error || records >= COUNT_MAX) return 0;

    a = rec_addr(records);
    /* Never overwrite non-erased data. Check each new sector in full so
     * unknown existing contents are not silently erased. */
    if ((records % RECORDS_PER_SECTOR) == 0U) {
        if (!flash_blank(a, (uint16_t)SECTOR_SIZE)) goto fail;
    } else if (!flash_blank(a, 10)) goto fail;

    b[0] = 0xD8; /* DS18B20 record format, distinct from legacy NTC 0xA5 */
    b[1] = t->y;
    b[2] = t->m;
    b[3] = t->d;
    b[4] = t->h;
    b[5] = t->n;
    b[6] = t->s;
    b[7] = (uint8_t)temp_x10;
    b[8] = (uint8_t)((uint16_t)temp_x10 >> 8);
    b[9] = checksum(b);

    if (!flash_write(a, b, 10) ||
        !rec_read(records, check)) goto fail;

    for (i = 0; i < 10; i++)
        if (b[i] != check[i]) goto fail;

    records++;
    return 1;
fail:
    log_error = 1;
    return 0;
}

uint8_t log_clear(void)
{
    uint32_t a;
    for (a = BASE; a < LOG_END; a += SECTOR_SIZE) {
        if (!flash_erase(a) ||
            !flash_blank(a, (uint16_t)SECTOR_SIZE)) {
            log_error = 1;
            return 0;
        }
    }

    records = 0;
    log_error = 0;
    return 1;
}

/* ---------- Display / serial commands ---------- */

void print_time(const RTC *t)
{
    puts1("20"); two(t->y); putc1('-');
    two(t->m); putc1('-'); two(t->d);
    putc1(' ');
    two(t->h); putc1(':');
    two(t->n); putc1(':'); two(t->s);
}

void history(uint16_t n)
{
    uint16_t i;
    uint8_t b[10];
    RTC t;
    int16_t temp_x10;

    if (!records) {
        puts1("No records\r\n");
        return;
    }

    if (n > records) n = records;

    for (i = records - n; i < records; i++) {
        if (!rec_read(i, b) || !rec_valid(b)) {
            puts1("Record read error\r\n");
            return;
        }

        t.y = b[1]; t.m = b[2]; t.d = b[3];
        t.h = b[4]; t.n = b[5]; t.s = b[6];
        temp_x10 = (int16_t)((uint16_t)b[7] |
                             ((uint16_t)b[8] << 8));

        putc1('#');
        num((uint32_t)i + 1);
        putc1(' ');
        print_time(&t);
        puts1(" Temp=");
        tempout(temp_x10);
        puts1(" C\r\n");
    }
}

/* Emit one CSV row per main-loop pass so a full export does not stop the
 * one-second sampling task. The end index is captured when D is received. */
void dump_step(void)
{
    uint8_t b[10];
    RTC t;
    int16_t temp_x10;

    if (!dump_active) return;
    if (dump_index >= dump_end) {
        puts1("END\r\n");
        dump_active = 0;
        return;
    }
    if (!rec_read(dump_index, b) || !rec_valid(b)) {
        puts1("EXPORT ERROR\r\n");
        dump_active = 0;
        return;
    }

    t.y = b[1]; t.m = b[2]; t.d = b[3];
    t.h = b[4]; t.n = b[5]; t.s = b[6];
    temp_x10 = (int16_t)((uint16_t)b[7] | ((uint16_t)b[8] << 8));
    num((uint32_t)dump_index + 1);
    putc1(',');
    print_time(&t);
    putc1(',');
    tempout(temp_x10);
    puts1("\r\n");
    dump_index++;
}

uint8_t valid_time(const RTC *t)
{
    static const uint8_t days[12] =
        {31,28,31,30,31,30,31,31,30,31,30,31};
    uint8_t maxday;

    if (t->m < 1 || t->m > 12 ||
        t->h > 23 || t->n > 59 || t->s > 59)
        return 0;

    maxday = days[t->m - 1];
    if (t->m == 2 && (t->y % 4 == 0))
        maxday = 29;

    return t->d >= 1 && t->d <= maxday;
}

uint8_t parse_time(const char *s, RTC *t)
{
    uint8_t b[6], i;
    if (s[0] != 'T' || s[13] != 0) return 0;
    for (i = 1; i <= 12; i++)
        if (s[i] < '0' || s[i] > '9') return 0;

    for (i = 0; i < 6; i++)
        b[i] = (uint8_t)((s[1 + 2 * i] - '0') * 10 +
                          s[2 + 2 * i] - '0');

    t->y = b[0]; t->m = b[1]; t->d = b[2];
    t->h = b[3]; t->n = b[4]; t->s = b[5];
    return valid_time(t);
}

void help(void)
{
    puts1("\r\nCommands:\r\n");
    puts1("TYYMMDDHHMMSS Set RTC\r\n");
    puts1("Q Read RTC\r\nD Export all records as CSV\r\n");
    puts1("R Last record\r\nH Last 10\r\n");
    puts1("C Count\r\nS Start recording\r\nP Pause recording\r\n");
    puts1("ERASE Clear all log sectors\r\n? Help\r\n");
}

void command(const char *s)
{
    RTC t;
    if (s[0] == 'T') {
        if (!parse_time(s, &t))
            puts1("Invalid time\r\n");
        else
            puts1(RTC_Set(&t) ? "RTC Set OK\r\n" :
                               "RTC Set Failed\r\n");
    } else if (s[0] == 'R' && !s[1]) {
        history(1);
    } else if (s[0] == 'Q' && !s[1]) {
        if (RTC_Read(&t)) {
            print_time(&t);
            puts1("\r\n");
        } else puts1("RTC READ ERROR\r\n");
    } else if (s[0] == 'D' && !s[1]) {
        dump_index = 0;
        dump_end = records;
        dump_active = 1;
        puts1("index,time,temp_c\r\n");
    } else if (s[0] == 'H' && !s[1]) {
        history(10);
    } else if (s[0] == 'S' && !s[1]) {
        record_enable = 1;
        puts1("Recording ON\r\n");
    } else if (s[0] == 'P' && !s[1]) {
        record_enable = 0;
        puts1("Recording PAUSED\r\n");
    } else if (s[0] == 'C' && !s[1]) {
        puts1("Records: "); num(records); puts1("\r\n");
    } else if (s[0] == 'E' && s[1] == 'R' &&
               s[2] == 'A' && s[3] == 'S' &&
               s[4] == 'E' && !s[5]) {
        /* A full-region erase takes several seconds. Do not use a stale
         * pre-erase conversion as the first new record afterwards. */
        dump_active = 0;
        puts1("Erasing all log sectors...\r\n");
        puts1(log_clear() ? "LOG ERASE OK\r\n" :
                            "LOG ERASE FAILED\r\n");
        ds18b20_ready = 0;
        tick = 0;
    } else if (s[0] == '?' && !s[1]) {
        help();
    } else puts1("Unknown command\r\n");
}

/* ---------- Main ---------- */

int main(void)
{
    uint8_t id[3], i;
    int16_t temp_x10;
    RTC t;
    char cmd[20];

    SystemCoreClockUpdate();
    UART_Init();
    I2C_Init();
    SPI_Init();
    DS18B20_Init();
    Board_Init();
    KeyTimer_Init();

    puts1("\r\nLPC1114 Temperature Logger\r\n");

    if (!flash_id(id)) {
        puts1("FLASH ID ERROR\r\n");
        while (1);
    }

    puts1("JEDEC ID: ");
    hex8(id[0]); putc1(' ');
    hex8(id[1]); putc1(' ');
    hex8(id[2]); puts1("\r\n");

    if (id[0] != 0x0B || id[1] != 0x40 || id[2] != 0x13) {
        puts1("FLASH ID MISMATCH\r\n");
        while (1);
    }

    if (!log_init())
        puts1("LOG CHECK ERROR: use ERASE\r\n");

    puts1("Stored DS18B20 records: ");
    num(records);
    puts1("\r\n");

    puts1("SystemCoreClock=");
    num(SystemCoreClock);
    puts1(" Hz\r\n");
    OW_LineDiagnostic();

    /* First identify the single device with the 0x33 Read ROM command. */
    if (DS18B20_ROM_Diagnostic() && DS18B20_Configure()) {
        puts1("DS18B20 detected, 12-bit mode\r\n");
        ds18b20_ready = DS18B20_StartConversion();
        if (ds18b20_ready) puts1("Initial conversion started\r\n");
        else puts1("DS18B20 conversion start failed\r\n");
    } else {
        ds18b20_ready = 0;
        puts1("DS18B20 IDENTIFICATION/CONFIGURATION FAILED on PIO0_6\r\n");
        puts1("Presence low samples=");
        num(ow_presence_samples);
        puts1("\r\nPresence first low us=");
        if (ow_presence_first_us == 0xFFFFU)
            puts1("NONE");
        else
            num(ow_presence_first_us);
        puts1("\r\nPresence last low us=");
        if (ow_presence_last_us == 0xFFFFU)
            puts1("NONE");
        else
            num(ow_presence_last_us);
        puts1("\r\n");
    }

    help();
    TIMER_Init();

    while (1) {
        /* UART command prepared in interrupt. */
        if (cmd_ready) {
            NVIC_DisableIRQ(UART_IRQn);
            for (i = 0; i < 19; i++) {
                cmd[i] = (char)rxbuf[i];
                if (!cmd[i]) break;
            }
            cmd[19] = 0;
            rxlen = cmd_ready = 0;
            NVIC_EnableIRQ(UART_IRQn);
            command(cmd);
        }

        /* Every 10ms scan button, debounce for about 30ms. */
        if (key_check) {
            __disable_irq();
            key_check--;
            __enable_irq();
            Board_Update();
        }

        /* One DS18B20 conversion is read every second; next conversion is
         * started immediately afterwards. 12-bit conversion needs <=750ms,
         * so the one-second scheduler provides sufficient conversion time. */
        if (tick) {
            tick = 0;

            if (ds18b20_ready) {
                if (!DS18B20_ReadTemperature(&temp_x10)) {
                    puts1("DS18B20 READ/CRC ERROR\r\n");
                } else if (!record_enable) {
                    /* Sensor remains active while recording is paused. */
                } else if (!RTC_Read(&t)) {
                    puts1("RTC READ ERROR\r\n");
                } else if (log_error) {
                    puts1("LOG FAULT\r\n");
                } else if (records >= COUNT_MAX) {
                    puts1("LOG FULL\r\n");
                } else if (log_save(&t, temp_x10)) {
                    if (!dump_active) {
                        puts1("Saved #");
                        num(records);
                        putc1(' ');
                        print_time(&t);
                        puts1(" Temp=");
                        tempout(temp_x10);
                        puts1(" C\r\n");
                    }
                } else {
                    puts1("FLASH WRITE ERROR\r\n");
                }
            } else {
                puts1("DS18B20 NOT READY\r\n");
            }

            /* Keep conversions running even when the user pauses logging. */
            ds18b20_ready = DS18B20_StartConversion();
            if (!ds18b20_ready)
                puts1("DS18B20 START ERROR\r\n");
        }

        if (!tick) dump_step();
    }
}
