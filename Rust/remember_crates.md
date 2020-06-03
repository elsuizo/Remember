# Cosas para recordar de las diferentes crates

Para que no tener que leer la documentacion 10 veces

## Crate nb

link: https://docs.rs/nb/0.1.2/nb/

Este crate es una abstraccion para utilizar cuando queremos hacer I/O. Si bien
no nombre proviene del las siglas *non-blocking*
aun asi con esta libreria podemos escribir codigo que sea bloquente y no bloqueante
y se puede adaptar a trabajar con frameworks asyncs. La idea principal es que
define el siguiente `enum`:

```rust
enum nb::Error<E> {
      Other(E),
      WouldBlock,
}
```
Con esa variante le hacemos saber al que llama a la operacion que todavia no
se ha completado la operacion (de I/O) y que necesita bloquear al recurso para
completar la operacion. O sea que esta variante `WouldBlock` es un error especial
en el sentido que no es fatal; la operacion se puede completar aun reintentando
de nuevo mas tarde.

Podemos mapear `WouldBlock` a diferentes modelos que son bloqueantes y no
bloqueantes:

   - En un modo bloqueante: `WouldBlock` quiere decir intenta de nuevo ahora
   - En un modo `futures`(async): `WouldBlock` quiere decir `Async::NotReady`
   - En un modo `await`: `WouldBlock` quiere decir `yield` (suspendeme el generador)

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
Podemos usar el type que nunca retorna `!` para senialar que esta API no tiene errores
fatales pero puede bloquear.

```rust
#![feature(never_type)]

// esto puede retornar `Ok(())` o un `Err(nb::Error::WouldBlock)`
fn quizas_bloquee_api() -> nb::Result<(), !> {
   // ...
}
```

Una vez que nuestra API usa `nb::Result` podemos aprobechar los macros que nos ofrece
esta libreria: `block!`, `try_nb!` y `await!` para adaptar a las operaciones que tengan
un comportamiento bloqueante o no bloqueante


