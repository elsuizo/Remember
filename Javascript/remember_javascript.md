# Cosas para recordar de este fantastico lenguaje

 - Las variables se declaran con `let`
 - Los types con primitivos son:
   - `undefined`
   - `null`
   - `boolean`
   - `number`
   - `string`
   - `symbol`
 - Podemos convertir entre types con el constructor del type, por ejemplo tenemos un numero
 ```javascript
    let x = 3;
    let x_string = String(x);
 ```
 - Tenemos dos operadores de comparacion `==` y `===`, la diferencia es que `===` requiere que los types sean equivalentes
 - Los types primitivos son inmutables, los `objects` son mutables y se guardan por referencia
 - Podemos crear un nuevo `object`: `const obj = new Object()`, otra manera es: `const obj = {}` la segunda es la que mas se utiliza
 - Podemos asignarle variables: `obj.firstName = "Martin"` `obj.['lastName'] = 'Noblia'`
