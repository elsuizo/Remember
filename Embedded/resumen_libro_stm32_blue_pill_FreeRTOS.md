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

El macro que se llama con `INPUT` solo aplica cuando el segundo argumento inplica
un puerto de entrada. Podemos ver en la tabla anterior que las entradas pueden
ser especializadas de tres maneras posibles:

 - Analogica

 - Digital, con entrada flotante

 - Digital con entrada pull down y pull up

Cuando elegimos la opcion `GPIO_CNF_INPUT_FLOAT` para los dos modos digitales
de entrada el schmitt trigger se activa para asi proveer una salida mas limpia

Cuando configuramos un periferico como salida tenemos que asegurarnos que usamos
una de las macros que son "alternate", de otra manera solo seniales de GPIO pueden
ser configuradas


## Ports de salida

Cuando un puerto GPIO es configurado como una salida, podemos tener cuatro
alternativas para ello

 - GPIO push/pull

 - GPIO open-drain

 - funcion "alternate" push/pull

 - funcion "alternate" open-drain

Cuando hacemos una operaciones con GPIOs siempre tenemos que elegir las funciones
"non-alternate". Para perifericos como la `USART` tenemos que elegir las
funciones "alternates"
O sea para el caso particular de la `USART` cuando configuramos el pin `TX` el
macro que debemos utilizar seria `GPIO_CNF_OUTPUT_ALTFN_PUSHPULL` para este
periferico


## Entradas como GPIOS

Cuando configuramos pins GPIOs como entradas, usaremos el siguiente protocolo
para configurarlo. Esto se aplica solo a entradas GPIOs (no a una entrada de
periferico como el `USART`)

 1. Habilita el "clock" del puerto GPIO. Por ejemplo si el pin GPIO es sobre
   El puerto C, entonces habilitar el "clock" con una llamada a
   `rcc_periph_clock_enable(RCC_GPIOC)`. Debemos habilitar cada "port" usado
   individualmente usando los macros `RCC_GPIOx`

 2. Setear el modo del pin de entrada con `gpio_set_mode()` especificando el
   "port" en el primer argumento y el macro `GPIO_MODE_INPUT` en el segundo
   argumento

 3. Luego de haber llamado a la funcion `gpio_set_mode()` elegir el macro
   apropiado de especializacion `GPIO_CNF_INPUT_ANALOG`, `GPIO_CNF_INPUT_FLOAT`
   o `GPIO_INPUT_PULL_UPDOWN`

 4. Finalmente especificar en el ultimo argumento de `gpio_set_mode()` todos los
   pines que queremos aplicar. Estos se pueden "ordar" como `GPIO12|GPIO15` and
   so on...



## Salidas digitales, "PUSH/PULL"

Normalmente las salidas digitales son configuradas como "push/pull". Estos son
los pasos que deberiamos seguir cuando lo hacemos:


 1. Habilitar los "clocks" del port. Por ejemplo, si el pin del GPIO es sobre
   el port B, entonces habilitar el "clock" con una llamada a la funcion
   `rcc_periph_clock_enable(RCC_GPIOB)`. Deberiamos habilitar cada port usado
   individualmente usando los macros `RCC_GPIOx`

 2. Setear el modo de output del pin con `gpio_set_mode()`, especificando el port
   en el primer argumento y uno de los macros `GPIO_MODE_OUTPUT_*_MHZ` en el
   segundo argumento. Para seniales que no tienen un rate critico deberiamos
   elegir los valores mas bajos de MHZ para salvar ballenas

 3. Especificar `GPIO_CNF_OUTPUT_PUSHPULL` en el tercer argumento en la llamada
   a `gpio_set_mode()`. No deberiamos usar ninguno de los macros `ALTFN` para
   GPIO(estos son para utilizarlos con los perifericos)

 4. Finalmente especificar en los argumentos finales en la funcion `gpio_set_mode()`
   todos los pines que queremos afectar. Estos se pueden "ordear" por ejemplo:
   `GPIO12|GPIO15|GPIO10` and so on...


## Salidas digitales como "open drain"

cuando trabajamos con un "bus" donde mas de un transistor puede ser usado para
"pull-down" un volage una salida "open-drain" debe ser requerida. Por ejemplo
cuando estamos trabajando con I2C o con el protocolo CAN. Las siguientes
pasos para configurar el pin digital de salida "open-drain":

 1. Habilitar el port GPIO. Por ejemplo si el pin GPIO es en el portB entonces
 habilitar el "clock" con una llamada a la funcion `rcc_periph_clock_enable(RCC_GPIOB)`
 Deberiamos tambien habilitar cada port usando los macros `RCC_GPIOx`

 2. Setear el modo de salida del pin con `gpio_set_mode()` especificando el port
 en el primer argumento y uno de los macros `GPIO_MODE_OUTPUT_*_MHZ` en el
 segundo argumento. Para seniales que no tienen rates que son criticos de alcanzar
 deberiamos elegir los valores mas bajos de MHZ, para salvar ballenas

 3. Especificar `GPIO_CNF_OUTPUT_OPENDRAIN` en el tercer argumento de la funcion
 `gpio_set_mode()`. No deberiamos utilizar ninguna de las macros `ALTFN` para
 `GPIOS` (esas son para los perifericos!!!)

 4. Finalmente especificar en el ultimo argumento de `gpio_set_mode()` los pines
 que queremos que sean afectados(uno o mas). Estos se pueden "ordear" por ejemplo
 `GPIO12|GPIO15|GPIO10` and so on...


## Caracteristicas de los GPIOs

las capacidades del los GPIOs se pueden ver en la tabla de la pag: 54. Una
convencion que se usa es llamar al `GPIO` port B pin `GPIO5` como `PB5`


# Cap5: FreeRTOS

## Las caracteristicas principales de FreeRTOS

Las principales caracteristicas que puede ofrecernos FreeRTOS son:

 - Multitasking y un scheduller

 - Queues de mensajes

 - Semaforos y Mutex

 - Timers

 - Event Groups


### Tareas

En un programa de un micro simple como un Arduino todo corre en una sola tarea
con un solo stack para las variables y direcciones de memorias de retorno

Este estilo de programacion requiere que corramos un loop que este mirando en
cada iteracion a todos los perifericos. Con los RTOS en gral y FreeRTOS en
particular las funciones logicas son puestas dentro de tareas separadas que
corren independientemente. Una tarea puede ser responsable de leer la temperatura
otra puede ser responsable de imprimir los datos por el USART and so on...

FreeRTOS es muy flexible. Provee dos types de schedulling para las tareas:

 - Preemptive multi-tasking

 - Cooperative multi-tasking (coroutines)

Con un esquema preemptive una tarea puede correr hasta que se se queda sin tiempo
estipulado (sin time-slice) o se ha bloqueado o delega el control explicitamente
El scheduller maneja cual es la tarea que debe ejecutarse a continuacion, tomando
como criterio principal las prioridades de las mismas. Este es el esquema que se
usa en este libro. Otra forma de schedulling es coroutines, la diferencia es que
la tarea que esta actualmente corriendo seguira corriendo hasta que ella seda
el control. No hay time-slice o timeout, este esquema se usa a menudo en sistemas
criticos que necesitan un control estricto de como se usa el CPU


### Queues de mensajes

La manera que tenemos de comunicarnos con otras tareas o con otros entornos es
con queus que FreeRTOS nos ofrece, tienen un monton de ventajas sobre lo que
se hace en los sistemas como arduino en los que se usa casi siempre una variable
global que se va pasando entre funciones

### Semaforos y Mutex

Dentro de la implementacion de las queues, esta una operacion implicita que son
los mutex. Dentro de FreeRTOS una queue es diseniada para que podamos agregar
mensajes de manera atomica para poder tener esto es que se implementa un device
que hace las veces de mutex. Similarmente que con los mutex, podemos querer tener
un "dispositivo" que nos permita manejar quienes acceden a la informacion de la
queue y en que momento, para ello se utilizan los semaforos


## Timers

Como sabemos cuando trabajamos con FreeRTOS tenemos a disposicion el `SysTick`
del sistema(o sea que FreeRTOS lleva la cuenta de los ticks de reloj que se van
consumiendo) con ello por ejemplo tenemos "gratis" la implementacion de un
delay desde la misma libreria

## Event Groups

Un problema que ocurre a menudo es que una tarea puede tener que monitorear muchas
queues a la vez, por ejemplo una tarea puede necesitar bloquear hasta que un msg
halla arribado desde alguna de las dos queues que esta "escuchando". FreeRTOS nos
provee la creacion de "conjuntos de queues", esto hace que podamos agruparlas y
asi poder "escuchar" un conjunto de queues en lugar de cada una por separado.
Que hay de poder utilizar eventos que definimos nosotros?. Se puede, mapeando
cada nuevo evento con un numero binario para identificarlo


## ejemplo blinky2

si vamos a `rtos/blinky2` tenemos un programa de ejemplo que lo que hace como su
nombre lo indica es hacer el famoso blink pero con FreeRTOS, utilizando el delay
que provee la libreria `vTaskDelay()`. En este ejemplo tambien se define una
funcion auxiliar(en realidad solo define un prototipo) que se llama:
`vApplicationStackOverflowHook()` ya que FreeRTOS no provee por default esto lo
tenemos que definir nosotros (elsuizo: quuue)


## `FreeRTOSConfig.h`

Las configuraciones mas importantes que estan en este archivo son las siguientes
que son las que configuran al SO para que funcione correctamente en funcion del
la velocidad de reloj del sistema que estamos utilizando

   - `configCPU_CLOCK_HZ ((unsigned long) 72000000)`
   - `configSYSTICK_CLOCK_HZ ((configCPU_CLOCK_HZ / 8))`
   - `configTICK_RATE_HZ ((TickType_t) 250)`

Estas tres declaraciones lo que hacen es que tenemos un sistema con un clock de
72Mhz, un timer que se incrementara cada 8 ciclos de CPU y que queremos que las
interrupciones sucedan 250 veces por segundo(cada 4ms). Osea que si no tenemos
estos valores de manera correcta por ejemplo los delays no van a funcionar
correctamente. Luego si no queremos tener un hook que se active cuando haya un
*stack-overflow* podemos no setear la macro que lo configura que en este caso
es `configCHECK_FOR_STACK_OVERFLOW`. Luego todas las funciones que no utilizamos
es mejor que no esten presentes asi tenemos una aplicacion que es mas chica, por
ejemplo en esta app no utlizamos `vTaskDelete()` entonces la sacamos desde la
config


## Conveciones de nombres en FreeRTOS

En FreeRTOS se tiene muchas convensiones que algunas son polemicas, por ejemplo
en los nombres de las variables se intenta describir el type de esta, por eso
se tienen los siguientes prefijos que hacen de descriptores de las variables


|Prefijo | Descripcion
|---     |---
| v      | `void` (para los nombres de funciones)
| c      | `char` type
| s      | `short` type
| l      | `long` type
| x      | `BaseType_t` y otros...
| u      | `unsigned` type
| p      | puntero

por ejemplo si la variable es del tipo `unsigned char` entonces ellos usan el
prefijo `uc`

## Macros mas importantes en FreeRTOS

|Prefix    | Example               | Source
|---       |---                    |---
| `port`   |  `portMAX_DELAY`      | `portable.h`
| `task`   | `taskENTER_CRITICAL`  | `task.h`
| `pd`     | `pdTRUE`              | `projdefs.h`
| `config` | `configUSE_PREEMPTION`| `FreeRTOSConfig.h`
| `err`    | `errQUEUE_FULL`       | `projdefs.h`


# Cap6: USART

UART y USART son casi lo mismo lo unico que cambia en el caso de este chip es
la configuracion (que en el caso de USART estamos en presencia de una comunicacion
Syncronica).

## Proyecto UART

### uart1

En la carpeta `rtos/uart`

En este ejemplo tenemos una unica tarea que lo que hace es mandar un caracter
por vez y con un cierto delay al puerto serie, ademas de togglear el led para
tener una manera de vizualizar a que velocidad esta pasando la transmision.
La configuracion de la UART la hace en una funcion aparte que es llamada en el
`main()` antes de llamar al `scheduller` y antes de crear la tarea.

### uart2

En la carpeta `rtos/uart2` tenemos el source que esta definido en el archivo
`uart.c`, tenemos algunas mejoras con respecto al anterior, por ejemplo ahora
definimos ana queue para el Tx que contiene un maximo de 256 mensajes cada uno
de los cuales tiene una longitud de un byte. Tenemos una tarea que se llama
`uart_task()` que tiene dentro de ella una llamada a la funcion de la API
`xQueueReceive()` que toma la queue de la que queremos recibir un msg, el mensaje
que esperamos `&ch` y el timeout que debe esperar la funcion en ticks, si se
cumple este timeout la funcion retorna el `pdFAIL`, mientras que cuando la funcion
recibe el msg retorna un `pdPASS`. Luego vemos que tenemos un `while` loop que
saldra de ese estado cuando la funcion de la API de `libopencm3` que testea varios
registros de status, en este caso se testea el bit del *status-register* `TXE`
(transmision vacia). Como en el otro ejemplo tenemos un led que togglea para dar
seniales de vida

### Resumen del periferico en el chip


|USART    | Macro   | 5V   | TX   | RX     | CTS     | RTS
|---      |---      |---   |---   |---     |---      |---
| 1       | USART1  | si   | PA9  | PA10   | PA11    | PA12
| 2       | USART2  | no   | PA2  | PA3    | PA0     | PA1
| 3       | USART3  | si   | PB10 | PB11   | PB14    | PB12



# Cap8: SPI

En el libro usa el chip `W25Q32/64` que es una memoria adicional que podemos
utilizar cuando el chip se queda corto. Utiliza para comunicarse con el el bus
SPI(Serial Peripheral Interface Bus) que es un bus serial sincronico que sirve
para comunicaciones de corta distancia usando solo tres cables y una senial de
seleccion. Osea que en esta aplicacion vamos a utilizar al stm32 como "master"
y al chip de memoria como "slave".

Como dijimos en este protocolo existen cuatro seniales principales:

   - `SCK`: Sistem Clock: provee los pulsos de clock para tener una referencia
      de tiempos
   - `MOSI`: Master Out Slave In: data
   - `MISO`: Master In Slave Out: data
   - `CS`: Chip Select: Es usada para activar el device que corresponda(porque
      podemos tener varios "slaves"), a veces a esta senial se la conoce como
      `SS`(Slave Select)

El SPI master siempre genera el pulso de clock (`SCK`) para tener una referencia
de tiempos de cuando los datos son sampleados y corridos en el registro de entrada
Una vez que el toda la palabra de datos es recibido, el "slave" y el "master"
pueden simultaneamente leer lo que recibieron. Este protocolo tiene algunas cosas
que no son muy elegantes por ejemplo cuando el "slave" no sabe que datos mandar
hasta que recibe un comando desde el master. Consecuentemente cuando el master
envia el comando al "slave" la primer palabra que recibe este es descartada
porque no tiene ningun significado. Una vez que el "slave" ha recibido el comando
y entonces sabe que responder. Pero el "slave" necesita enviar al master que le
permita enviar el mensaje. Por estas caracteristicas la programacion con el bus
SPI muchas veces requiere que descartemos alguno de los datos que son recibidos
y que enviemos palabras que no tienen sentidos. El tamanio de palabra para el
controlador del STM32 SPI puede ser de 8 o 18 bits en largo

## Chip Select

Nos podriamos preguntar porque tenemos que tener un `CS` si solo tenemos un "slave"
El problema siempre es el ruido que puede haber en el canal de comunicacion, para
hacer mas robusto contra ello es que el "slave" necesita saber cuando una transmision
comienza y cuando termina. Si se ha recibido ruido sobre esta linea, por ejemplo
el "slave" podria terminar un bit fuera de lugar con el "master". Con ello podriamos
tener un comando equivocado que haga borrar a toda la memoria del chip que seria
algo muy malo. Por esta razon el `CS` va con nivel bajo antes de que el primer
bit de dato sea enviado al "master". Esto le dice al "slave" que el primer bit
es el que sigue. Cuando la ultima palabra de datos ha sido enviada el `CS` retorna
el estado alto para senialar el final de transmision


# Cap11: I2C

Es un protocolo de comunicacion muy conveniente principalmente porque requiere
solo dos cables para ello. Las dos seniales que se usan para la comunicacion son
las siguientes:

   - System clock (usualmente llamada `SCL`)
   - System data (usualmente llamada `SDA`)

## Master y Slave

Con todos los dispositivos sobre el bus, tiene que haber cierto tipo de protocolo
para mantener las cosas organizadas, de otra manera en el bus pueden ocurrir muchas
conversaciones de los dispositivos a la vez. Por esta razon es que se elije a un
dispositivo que sea el "master" y que los demas sean los "slaves". Este "master"
siempre comienza la conversacion y lleva el conteo de la senial de clock, puede
ocurrir que un "slave" excepcionalmente haga uso de la linea `SCL`.
Los "slaves" solo responden cuando se les habla a ellos. Cada "slave" tiene una
unica direccion o id de 7 bits entonces sabe cuando un mensaje en el bus es para
el. Esta es una de las diferencias principales con SPI. Cada dispositivo I2C es
llamado por su id, mientras que en SPI los dispositivos son seleccionados por
la senial *chip-select*

## Comienzo y Final

El I2C esta inactivo cuando ambas seniales `SDA` y `SCL` son puestas en alto. En
este caso ningun dispositivo ("master" o "slave") esta poniendo las lineas en
el estado bajo. El comienzo de una transaccion en I2C esta indicada por los siguientes
eventos:

   - La linea `SCL` se mantiene en alto
   - la linea `SDA` se la pone en estado bajo

El segundo paso usualmente sucede dentro de un ciclo de clock, aunque no precisa
ser exactamente cuando sucede. Cuando el bus esta inactivo es suficiente con ver
la linea de `SDA` que vaya al estado bajo mientras que el `SCL` se mantiene en
alto


## Bits de datos

Los datos son transmitidos mediante la variacion de alto a bajo en la senial
`SDA` El sampleo de esta linea se da cuando la linea de clock pasa de alto a
bajo


## Direcciones de memoria en I2C

El byte de direccion que es utilizada para identificar al dispositivo "slave" que se
espera que responda y adicionalmente a esta direccion un bit de lectura/escritura
que indica la intension de leer o de escribir desde o hacia el dispositivo "slave"

```text
A6|A5|A4|A3|A2|A1|A0|R/W
```

En el caso de el bit de lectura/escritura:

   - `1` indica que una operacion de lectura desde el "slave" va a ser realizada
   - `0` indica que una operacion de escritura desde el "slave" va a ser realizada

Este byte que es la direccion o id del dispositivo mas el bit de lectura/escritura
siempre viene seguido de un bit que simboliza que la operacion comienza o se va
a repetir sobre el bus de I2C. Este bit requiere que el controlador chequee que
el bus no este siendo utilizado por otro "master" y si lo esta se tiene que
hacer un algoritmo de arbitraje entre las partes(esto en el caso de que tengamos
mas de un master). Pero una vez que el acceso es ganado por un dispositivo solo
sera liberado cuando el controlador coloque en el bus un bit de stop


# Transacciones con I2C

Tenemos la transaccion de escritura, que son los siguientes eventos que suceden:

 - El controlador del I2C toma el control del mismo y emite un bit de comienzo
 - El "master" (controlador) escribe una direccion de 7 bits seguida por un `0`
   que indica que se va a tratar de una operacion de escritura
 - El dispositivo "slave" cuando recibe el pedido pone en bajo el bit `ACK` del
   bus de data. Si no hay respuesta del "slave" la linea de data se pondra otra
   vez en alto haciendo que se traduzca en el mensaje `NAK` (negative acknowlege)
 - Como esta es una transaccion de escritura, el byte de datos es escrito en el
   dispositivo
 - El dispositivo "slave" reconoce que ha recibido la data poniendo bajo la linea
   de data durante el tiempo de bit de `ACK`
 - Si el "master" no envia mas data envia un bit de stop y deja el bus libre


Las transacciones de lectura son las siguientes:

 - El controlador del I2C toma control del bus emitiendo un bit de comienzo
 - El "master" (controlador) escribe los 7 bits de direccion seguidos por un `1`
   indicando que esta va a ser una transaccion de lectura
 - El dispositivo "slave" contesta ese mensaje poniendo la linea de datos baja
   durante el bit de `ACK`. Si el "slave" no responde, la linea de data seguira
   en estado alto y esto se toma como un `NAK` que tiene que ser recibido por el
   "master"
 - Ya que esta es una transaccion de lectura, el master continua escribiendo
   bits de clock para permitir que el dispositivo "slave" sincronice su data de
   respuesta hacia el "master"
 - Con cada pulso de clock el dispositivo "slave" escribe los 8 bytes de datos
   hacia el controlador "master"
 - Durante el tiempo de `ACK` el controlador "master" normalmente envia un `NAK`
   cuando no hay mas bytes para leer
 - El controlador envia un bit de stop el cual siempre termina con la transaccion
   con el "slave" (sin importar el ultimo `NAK` o `ACK` enviado)


Lista de interrupciones y los nombres de las ISR en el STM32F103 Tabla 11-2
