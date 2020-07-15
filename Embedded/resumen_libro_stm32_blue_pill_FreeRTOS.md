# Cap1

- Primero tenemos que setear el proyecto

## Usando el `st-link`

- Cuando hacemos `st-info --probe` nos da la informacion de nuestro micro y si
  esta todo ok

- Cuando hacemos `st-flash erase` borramos todo el firmware que esta en el chip

- Si queremos leer que es lo que esta en la memoria del chip podemos hacer:

  `st-flash read ./saved.img 0x8000000 0x1000`

   Ese comando lo que hace es leer desde la posicion de memoria `0x8000000` y
   guardar `0x1000` (4K) de datos y los pone en el file `saved.img`

- Para chequear el contenido podemos utilizar el comando de UNIX `hexedit saved.img`

- Podemos escribir la memoria del chip usando el subcomando `write`

  `st-flash write ./saved.img 0x8000000`


## El API de los GPIO en stm y libopencm3

Los headers que tenemos que importar son:

```C
#include<libopencm3/stm32/rcc.h>
#include<libopencm3/stm32/gpio.h>
```

El `rcc.h` es necesario para las definiciones de los clocks del chip y el `gpio.h`
es necesario para utilizar las funciones de gpio:

```C
void gpio_set(uint32_t gpioport, uint16_t gpios);

void gpio_clear(uint32_t gpioport, uint16_t gpios);

uint16_t gpio_get(uint32_t gpioport, uint16_t gpios);

void gpio_toggle(uint32_t gpioport, uint16_t gpios);

void gpio_port_write(uint32_t gpioport, uint16_t data);

void gpio_port_config_lock(uint32_t gpioport, uint16_t gpios);
```

y para todas las funciones anteriores el argumento `gpioport` puede ser una de
las siguientes macros:

|Port Macro | Descripcion
|---        |---
| GPIOA     | GPIO port A
| GPIOB     | GPIO port B
| GPIOC     | GPIO port C


Podemos setear o limpiar uno o mas GPIOS a la vez

|Pin Macro  | Definicion   | Descripcion
|---        |---           |---
| `GPIO0`   |  `(1 << 0)`  | Bit0
| `GPIO1`   |  `(1 << 1)`  | Bit1
| `GPIO2`   |  `(1 << 2)`  | Bit2
| `GPIO3`   |  `(1 << 3)`  | Bit3
| `GPIO4`   |  `(1 << 4)`  | Bit4
| `GPIO5`   |  `(1 << 5)`  | Bit5
| `GPIO6`   |  `(1 << 6)`  | Bit6
| `GPIO7`   |  `(1 << 7)`  | Bit7
| `GPIO8`   |  `(1 << 8)`  | Bit8
| `GPIO9`   |  `(1 << 9)`  | Bit9
| `GPIO10`  |  `(1 << 10)` | Bit10
| `GPIO11`  |  `(1 << 11)` | Bit11
| `GPIO12`  |  `(1 << 12)` | Bit12
| `GPIO13`  |  `(1 << 13)` | Bit13
| `GPIO14`  |  `(1 << 14)` | Bit14
| `GPIO15`  |  `(1 << 15)` | Bit15
| `GPIO_ALL`|  `0xffff`    | Bit16

Por ejemplo: `gpio_clear(PORTB, GPIO_ALL);` "limpia" todos los pins del `PORTB`

Una de las caracteristicas de este chip es que podemos hacer que una definicion
de un `GPIO/IO` quede fija para todo el programa, osea que nadie puede modificar
por accidente ese `GPIO/IO`. En `libopencm3` lo hacemos con la siguiente funcion

```C
void gpio_port_config_lock(uint32_t gpioport, uint16_t gpios);
```

Despues de usar esta funcion sobre un `GPIO/IO` este se congela hasta el nuevo
reset del chip(o sea para siempre). Esto es util para sistemas criticos


## Configurando los GPIOS
