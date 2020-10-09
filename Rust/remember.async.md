# Cosas para recordar de Async

- Para expresar valores que todavia no estan listos usamos valores del type
  `futures`: https://crates.io/crates/futures
  Un `future` representa una computacion asincrona, es un valor que puede no haber
  terminado su computo todavia
- El metodo mas importante que soporta este type es `poll`, que intenta resolver
  al type en un resultado final. Este metodo no es bloqueante si el valor no esta
  listo todavia. En lugar lo que se hace es un schedulling para que sea llamada
  una vez que se hallan obtenidos resultados exitosos
