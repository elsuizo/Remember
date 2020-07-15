# Cosas para recordar de las diferentes crates

Para que no tener que leer la documentacion 10 veces

## Crate nb

link: https://docs.rs/nb/1.0.0/nb/

Este crate es una abstraccion para utilizar cuando queremos hacer I/O. Si bien
no nombre proviene del las siglas *non-blocking* aun asi con esta libreria
podemos escribir codigo que sea bloquente y no bloqueante y se puede adaptar a
trabajar con frameworks asyncs. La idea principal es que define el siguiente
`enum`:

```rust
enum nb::Error<E> {
      Other(E),
      WouldBlock,
}
```

Con esa variante le hacemos saber al que llama a la operacion que todavia no se
ha completado la operacion (de I/O) y que necesita bloquear al recurso para
completar la operacion. O sea que esta variante `WouldBlock` es un error
especial en el sentido que no es fatal; la operacion se puede completar aun
reintentando de nuevo mas tarde.

Podemos mapear `WouldBlock` a diferentes modelos que son bloqueantes y no
bloqueantes:

   - En un modo bloqueante: `WouldBlock` quiere decir intenta de nuevo ahora
   - En un modo `futures`(async): `WouldBlock` quiere decir `Async::NotReady`
   - En un modo `await`: `WouldBlock` quiere decir `yield` (suspendeme el
     generador)

### Como usar el crate

Los errores de las aplicaciones especificos se pueden poner dentro de la variante `Other(E)`
Entonces en nuestra API en lugar de retornar `Result<T, MyError>` retornamos
`nb::Result<T, MyError>`

o sea:

```rust
enum MyError {
   Error1,
   Error2,
   // ...
   Errorn
}

// esta funcion es bloqueante
fn antes() -> Result<(), MyError> {
   // ...
}

// esta otra funcion que es potencialmente no bloqueante retorna un `nb::Result`
fn despues() -> nb::Result<(), MyError> {
   // ...
}
```

Podemos usar el `struct` `infallible` para senializar que alguna API no tiene
errores fatales pero puede bloquear

```rust
use core::convert::Infallible;

// this returns `Ok(())` or `Err(nb::Error::WouldBlock)`
fn maybe_blocking_api() -> nb::Result<(), Infallible> {
   //...
}
```

Una vez que nuestra API usa el `nb::Result` podemos usar el macro `block!` para
adaptar este a una operacion bloqueante, o hacer schedulling por nosotros mismos


## Ejemplos:

### Una API para I/O

Imaginemos que el codigo que sigue representa un HAL para algun micro (en este
y en el ejemplo que sigue suponemos por simplicidad que los perifericos son
tratados como singletons globales y que no hay preemption)

```rust
// this is the HAL crate
use nb;

/// an LED
pub struct Led;

impl Led {
   pub fn off(&self) {
      // ...
   }

   pub fn on(&self) {
      // ...
   }
}

pub struct Serial;

pun enum Error {
   Overrun,
   // ...
}

impl Serial {
   /// Reads a single byte from the serial interface
   pub fn read(&self) -> nb::Result<u8, Error> {
      // ...
   }

   /// Writes a single byte to the serial interface
   pub fn write(&self, byte: u8) -> nb::Result<(), Error> {
      // ...
   }

}

pub struct Timer;

impl Timer {
   /// Waits until the timer times out
   pub fn wait(&self) -> nb::Result<(), Infallible> {
      // ...
      // Notemos que el `Infallible` indica que esta operacion puede bloquear
      // pero que no tendra otra fuente de error
   }
}
```

Luego utilizando la API que constuimos seria: Si queremos por ejemplo encender
el LED por un segundo y entonces enviar datos por el puerto serie

```rust
use core::convert::Infallible;
use nb::block;

use hal::{Led, Serial, Timer};

// turn the LED on for one second
led.on();
block!(Timer.wait())?;
led.off();

// serial interface loopback
loop {
   let byte = block!(Serial.read())?;
   block!(Serial.write(byte))?;
}
```
