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

Dentro del codigo que hace el `miniblink` tenemos la funcion que hace de veces
de setup que se llama: `gpio_setup()` dentro de ella encontramos la funcion
de `libopencm3`:

```C
rcc_periph_clock_enable(RCC_GPIOC);
```

Con la misma lo que hacemos es encender el clock para el `GPIOC`. Si este clock
no es habilitado entonces, por eso el autor dice que este es uno de "lo patitos
que tiene que estar en fila" para que el sistema funcione. La razon por la que
los clocks estan todos desabilitados por default es por consumo

Luego la proxima llamada es la siguiente:

```C
gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
```

Como vemos se requieren 4 argumentos:

 - El primero especifica el `GPIO` que sera afectado
 - Los otros argumentos definen en que modo lo vamos a operar

|Nombre de Modo            | Valor   | Descripcion
|---                       |---      |---
| `GPIO_MODE_INPUT`        | `0x00`  | Modo de Input
| `GPIO_MODE_OUTPUT_2_MHZ` | `0x02`  | Modo Output @ 2MHZ
| `GPIO_MODE_OUTPUT_10_MHZ`| `0x01`  | Modo Output @ 10 MHZ
| `GPIO_MODE_OUTPUT_50_MHZ`| `0x03`  | Modo Output @ 50 MHZ

Como esperamos el macro `GPIO_MODE_INPUT` define al pin como entrada, luego los
otros definen al pin como salida y que rapido podra este responder a algun
cambio como podemos sospechar si elegimos el modo de menor velocidad sera el que
mas eficiente en cuanto a consumo. El tercer argumento especifica como el `PORT`
debe ser configurado

|Nombre de Macro de especializacion| Valor   | Descripcion
|---                               |---      |---
| `GPIO_CNF_INPUT_ANALOG`          | `0x00`  | Modo entrada analogica
| `GPIO_CNF_INPUT_FLOAT`           | `0x01`  | Modo entrada digital, flotante(default)
| `GPIO_CNF_INPUT_PULL_UPDOWN`     | `0x02`  | Modo entrada digital "pull up" y "pull down"
| `GPIO_CNF_OUTPUT_PUSHPULL`       | `0x00`  | Modo salida digital "push/pull"
| `GPIO_CNF_OUTPUT_OPENDRAIN`      | `0x01`  | Modo salida digital "open drain"
| `GPIO_CNF_OUTPUT_ALTFN_PUSHPULL` | `0x02`  | Modo salida funcion alternante "push/pull"
| `GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN`| `0x03`  | Modo salida funcion alternante "open drain"


## Ports de entradas

