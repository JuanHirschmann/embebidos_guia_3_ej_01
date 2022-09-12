//=====[Libraries]=============================================================

#include "mbed.h"
#include "arm_book_lib.h"

//=====[Defines]===============================================================

#define NUMBER_OF_KEYS 4

//=====[Declaration and initialization of public global objects]===============

AnalogIn potentiometer(A0);
DigitalIn aButton(D4);
DigitalIn bButton(D5);
DigitalIn cButton(D6);
DigitalIn dButton(D7);

DigitalOut alarmLed(LED1);

UnbufferedSerial uartUsb(USBTX, USBRX, 115200);

//=====[Declaration and initialization of public global variables]=============

bool alarmState    = OFF;
//=====[Declarations (prototypes) of public functions]=========================

void inputsInit();
void outputsInit();

void uartTask();
void availableCommands();
bool areEqual();

//=====[Main function, the program entry point after power on or reset]========
/*
1) Pines disponibles como entrada analógica:
Pines de A0 a A5 y los ADC1x  a ADC3x.  
*/
int main()
{
    inputsInit();
    outputsInit();
    availableCommands();
    while (true) {
        uartTask();
    }
}

void inputsInit()
{
    aButton.mode(PullDown);
    bButton.mode(PullDown);
    cButton.mode(PullDown);
    dButton.mode(PullDown);
}

void outputsInit()
{
    alarmLed = OFF;
}

void uartTask()
{
    char receivedChar = '\0';
    float voltage;
    if( uartUsb.readable() ) {
        uartUsb.read( &receivedChar, 1 );
        switch (receivedChar) { 
        /*Lectura del potenciometro y envio por puerto serie*/
        case '1':
            printf("Valor del A0: ");
            printf("%.4f \r\n",potentiometer.read());
            break;
        case '2':
            voltage=5*potentiometer.read();
            printf("Tension del potenciometro: ");
            printf("%.4f \r\n",voltage);
            break;
        default:
            availableCommands();
            break;
        }
    }
}

void availableCommands()
{
    uartUsb.write( "Available commands:\r\n", 21 );
    uartUsb.write( "Press '1' para medir pin A0\r\n", 29 );//29+1?
    uartUsb.write( "Press '2' para obtener la tensión en A0\r\n", 41 );//41+1?
}

/**
*DigitalIn: 
*@brief Clase para el manejo de entradas digitales (Hereda de gpio())
*    Métodos
*        ->DigitalIn(PinName pin)
*        @brief Constructor, internamente llama a gpio_init(gpio_t *obj, PinName pin)
*               ->gpio_init(gpio_t *obj, PinName pin) 
*               @brief Inicializa el structo obj con 
*                      obj->pin=pin
*                      obj->mask=gpio_set(pin) Devuelve la máscara para ese pin
*                      obj->gpio=gpio clock de gpio
*                      obj->ll_pin=ll_pin_defines[STM_PIN(obj->pin)] Low-level pin (ll)
*                      obj->reg_in=&gpio->IDR input data register (16 bits - Lee los 16 pins del puerto)               
*                      obj->reg_set=&gpio->BSRR bit set reset register (32 bits - Los primeros 16 tienen 1 donde los pines se setean a 1
*                       y los otros 16 tienen 1 donde los pines se setean a 0) 
*                      obj->reg_clr=&gpio->BSRR o BRR   bit reset register (32 bits BRR es el complemento de BSRR)

*                      
*               @param obj tipo gpio_t (definido en gpio_object.h)
*               @param pin pin a asignar
*        ->DigitalIn(PinName pin, PinMode mode)
*        @brief Constructor, internamente llama a gpio_init_in(gpio_t *obj, PinName pin) y a gpio_mode(gpio_t *obj, PinMode mode)
*        ->~DigitalIn() Destructor, internamente llama a gpio_free(gpio_t *obj) (No la encontré).
*DigitalOut
*@brief Clase para el manejo de Salidas digitales (Hereda de gpio())
*    Métodos
*        ->DigitalOut(PinName pin)
*        @brief Constructor, internamente llama a gpio_init_out(gpio_t *obj, PinName pin)
*               ->gpio_init_out(gpio_t *obj, PinName pin,0) 
*               @brief Inicializa el structo obj con 
*                      obj->pin=pin
*                      obj->mask=gpio_set(pin) Devuelve la máscara para ese pin
*                      obj->gpio=gpio clock de gpio
*                      obj->ll_pin=ll_pin_defines[STM_PIN(obj->pin)] ¿?
*                      obj->reg_in=&gpio->IDR input data register (16 bits - Lee los 16 pins del puerto)               
*                      obj->reg_set=&gpio->BSRR bit set reset register (32 bits - Los primeros 16 tienen 1 donde los pines se setean a 1
*                       y los otros 16 tienen 1 donde los pines se setean a 0) 
*                      obj->reg_clr=&gpio->BSRR o BRR   bit reset register (32 bits BRR es el complemento de BSRR)

*                      
*               @param obj tipo gpio_t (definido en gpio_object.h)
*               @param pin pin a asignar
*        ->DigitalOut(PinName pin, PinMode mode, int val)
*           Inicializa la salida con un valor
*        @brief Constructor, internamente llama a gpio_init_in(gpio_t *obj, PinName pin) y a gpio_mode(gpio_t *obj, PinMode mode)
*        ->~DigitalOut() Destructor, internamente llama a gpio_free(gpio_t *obj).
*
*AnalogIn
*AnalogOut
*/