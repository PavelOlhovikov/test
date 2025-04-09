#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>

#define DEBUG                   1

#define LED_PORT                GPIOA // светодиод
#define LED_PIN                 GPIO7

#define BUZZER_PORT             GPIOB // PB3
#define BUZZER_PIN              GPIO3

#define BATT_VOLTAGE_PORT       GPIOA //PA3
#define BATT_VOLTAGE_PIN        GPIO3

#define KEY_PORT                GPIOC // PC7, PC8, PC9
#define KEY1_PIN                GPIO7
#define KEY2_PIN                GPIO8
#define KEY3_PIN                GPIO9

#define USART_CONSOLE           USART2

#define VOLTAGE_DIVIDER         0 // Учитывать коэффициент делителя напряжения on/off
#define VOLTAGE_DIVIDER_RATIO   (0.1118f) // Коэффициент делителя напряжения

#define PRECHARGE_CIRCUIT_PORT  GPIOB //PB14
#define PRECHARGE_CIRCUIT_PIN   GPIO14

const float kVRef = 3.3f;
const uint32_t TestCycles = 1000;

volatile uint32_t systick_counter;
volatile uint8_t stat_cycle = 1;
volatile uint32_t time = 0;
 
static void ClockSetup(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    /* Enable GPIOD clock for LED & USARTs. */
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Enable clocks for USART2 and dac */
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_DAC);

    /* And ADC*/
    rcc_periph_clock_enable(RCC_ADC1);
}

/* Called when systick fires */
extern "C" void sys_tick_handler(void)
{
	systick_counter++;
}

/* sleep for delay milliseconds */
static void MSleep(uint32_t delay)
{
	uint32_t wake = systick_counter + delay;
	while (wake > systick_counter);
}

/* Set up a timer to create 1mS ticks. */
static void SystickSetup(void)
{
	/* clock rate / 1000 to get 1mS interrupt rate */
	systick_set_reload(168000);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	/* this done last */
	systick_interrupt_enable();
}

static void GpioSetup(void)
{
    /* Setup GPIOA pin GPIO7 on GPIO port A for LED. */
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN); // led pin

    /* Setup GPIOB pin GPIO3 on GPIO port B. */
    gpio_mode_setup(BUZZER_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BUZZER_PIN); // led pin

}

static void UsartSetup(void)
{
    /* Setup GPIO pins for USART2 transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    /* Setup USART2 TX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2);

    /* Setup USART2 parameters. */
    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART2);
}

// Отправка строки через UART
void UartPrint(const char* str) {
    while (*str) {
        usart_send_blocking(USART2, *str);
        str++;
    }
}

static void AdcSetup(void)
{
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);

    adc_power_off(ADC1);
    adc_disable_scan_mode(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);

    adc_power_on(ADC1);

}

static void DacSetup(void)
{
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO5);
    dac_disable(DAC1, DAC_CHANNEL2);
    dac_disable_waveform_generation(DAC1, DAC_CHANNEL2);
    dac_enable(DAC1, DAC_CHANNEL2);
    dac_set_trigger_source(DAC1, DAC_CR_TSEL2_SW);
}

static uint16_t ReadAdcNaiive(uint8_t channel)
{
    uint8_t channel_array[16];
    channel_array[0] = channel;
    adc_set_regular_sequence(ADC1, 1, channel_array);
    adc_start_conversion_regular(ADC1);
    while (!adc_eoc(ADC1));
    uint16_t reg16 = adc_read_regular(ADC1);
    return reg16;
}

// Чтение напряжения
float ReadBatteryVoltage() {
    uint16_t input_adc0 = ReadAdcNaiive(0);
    float voltage = static_cast<float>(input_adc0) * kVRef / 4095.0f;
#if(VOLTAGE_DIVIDER)
    return voltage * VOLTAGE_DIVIDER_RATIO; // Учитываем коэффициент делителя
#else
    return voltage; // Без коэффициента делителя
#endif
}

// Чтение температуры терморезистора
float ReadResistorTemp() {
    float voltage = ReadAdcNaiive(0);
    // Примерная формула 
    float resistance = 10000.0f * voltage / (kVRef - voltage);
    // Нужно подобрать коэффициенты
    float temp_kelvin = 1.0f / (1.0f/298.15f + 1.0f/3950.0f * log(resistance/10000.0f));
    return temp_kelvin - 273.15f; // Конвертация в °C
}

float test_voltage()
{
    float voltage = 0;
    for(size_t i = 0; i < 10; ++i)
    {
        voltage += ReadBatteryVoltage();
    }
    return voltage /= 10;
}

void CucleTestVoltage()
{
    //  1. Измерить выходное напряжение пином PA3, линия batt_voltage, если оно приблизительно равно напряжению акб прервать тест, включить пищалку на линии PB3
    volatile float voltage = 0;
#if(VOLTAGE_DIVIDER)
    if (voltage > (2.66f - 0.1) || voltage < (2.66 + 0.1f)) // 24 * 0.111118(коэффициент делителя напряжения) = 2,666832 // Измерить выходное напряжение пином PA3, линия batt_voltage, если оно приблизительно равно напряжению акб прервать тест
    {
        stat_cucle = 2; 
        pio_set(BUZZER_PORT, BUZZER_PIN); // включить пищалку на линии PB3 /* BUZZER on */
        time = systick_counter;
#if(DEBUG)
        gpio_toggle(LED_PORT, LED_PIN); /* LED on/off */
#endif
    }
    else 
    {
        voltage = test_voltage();
    }
#else
    if (voltage > kVRef - 0.1 || voltage < kVRef + 0.1) // Измерить выходное напряжение пином PA3, линия batt_voltage, если оно приблизительно равно напряжению акб прервать тест
    {
        stat_cycle = 2;
        gpio_set(BUZZER_PORT, BUZZER_PIN); // включить пищалку на линии PB3 /* BUZZER on */
        time = systick_counter;
#if(DEBUG)
        gpio_toggle(LED_PORT, LED_PIN); /* LED on/off */
#endif
    }
    else
    {
        voltage = test_voltage();
    }
#endif

}

void OnBattVoltage()
{
    volatile float voltage = 0;
    // 2. Включить цепь предзаряда(PB14) через 2.5 секунды открыть основные ключи (PC7, PC8, PC9) проверить что  ключи открылись измерев линию batt_voltage
    gpio_set(BATT_VOLTAGE_PORT, BATT_VOLTAGE_PIN); // Включить цепь предзаряда(PB14)
    if(systick_counter >= time + 2500){

        gpio_set(KEY_PORT, KEY1_PIN); // Открыть ключ (PC7)
        gpio_set(KEY_PORT, KEY2_PIN); // Открыть ключ (PC8)
        gpio_set(KEY_PORT, KEY3_PIN); // Открыть ключ (PC9)

#if(VOLTAGE_DIVIDER) // проверить что  ключи открылись измерев линию batt_voltage
if (voltage > (2.66f - 0.1) || voltage < (2.66 + 0.1f)) // 24В(напряжение на входе) * 0.111118(коэффициент делителя напряжения) = 2,666832 
        {
            stat_cucle = 3; 
            time = systick_counter;
#if(DEBUG)
            gpio_toggle(LED_PORT, LED_PIN); /* LED on/off */
#endif
        }
#else
        if (voltage > kVRef - 0.1f && voltage < kVRef + 0.1f)
        {
            stat_cycle = 3;
            time = systick_counter;
#if(DEBUG)
            gpio_toggle(LED_PORT, LED_PIN); /* LED on/off */
#endif
        }
#endif
    }
}

void OffBattVoltage() // 3. подержать ключи открытыми 3 секунды и закрыть ключи и линию предзаряда
{
    if(systick_counter >= time + 3000){
        gpio_clear(KEY_PORT, KEY1_PIN); // Закрыть ключ (PC7)
        gpio_clear(KEY_PORT, KEY2_PIN); // Закрыть ключ (PC8)
        gpio_clear(KEY_PORT, KEY3_PIN); // Закрыть ключ (PC9)
        gpio_clear(BATT_VOLTAGE_PORT, BATT_VOLTAGE_PIN); // Выключить цепь предзаряда(PB14)
        
        stat_cycle = 4;
#if(DEBUG)
        gpio_toggle(LED_PORT, LED_PIN); /* LED on/off */
#endif
    }
}

int main(void)
{
    ClockSetup();
    GpioSetup();
    UsartSetup();
    AdcSetup();
    DacSetup();
    SystickSetup();

    size_t cycle = 0;
    char uart_buf[64];

    while (1) // Прогнать этот цикл 1000 раз, на кждом круге выводить сообщение в uart с температурой резистора предзаряда (PA7)
    {
        switch(stat_cycle)
        {
            case 0:
            {
                UartPrint("Test complite...\r\n");
                MSleep(1000);
                break;
            }
            case 1:
            {
                CucleTestVoltage();
                break;
            }
            case 2:
            {
                OnBattVoltage();
                break;
            }
            case 3:
            {
                OffBattVoltage();
                break;
            }
            case 4:
            {
                float temp = ReadResistorTemp();
                snprintf(uart_buf, sizeof(uart_buf), "Cucle: %lu, Temp: %.2f°C\r\n",cycle + 1, temp);
                UartPrint(uart_buf);
        
                ++cycle;
        
                if(cycle < TestCycles)
                {
                    stat_cycle = 1;
                }
                else 
                {
                    stat_cycle = 0;
                }
                break;
            }
        }

    }

    return 0;
}