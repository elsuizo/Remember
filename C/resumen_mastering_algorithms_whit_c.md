# Resumen del libro Mastering algorithms with C

## Cap1 Introduccion

Hay tres razones por las que usar estructuras de datos y algoritmos:
- Eficiencia
- Abstraccion
- Reusabilidad

Los algoritmos tienen varias maneras de afrontar un problema, por eso podemos
clasificarlos a groso modo en como encaran los problemas:

- Algoritmos con pasos aleatorios

Estos algoritmos utilizan alguna propiedad estadistica para crear los pasos que
va a realizar el algoritmo, por ejemplo el algoritmo llamado "quicksort"(cap12)
Este algoritmo lo podemos resumir con el siguiente ejemplo:

Supongamos que tenemos una pila de cheques que tenemos que ordenar a mano,
comenzamos con la pila desordenada la cual la partimos en dos. En una pila
ponemos los cheque que son menores o iguales a un valor que es la estimacion de
el valor medio(propiedad estadistica), una vez que tenemos dos pilas repetimos
el proceso hasta que tenemos un cheque por pila y a ese punto los cheques
estaran ordenados Para realizar esto tenemos que calcular la media. Sin embargo
ya que calcular la media requiere que escanear todos los valores de los
cheques, pero no hacemos esto elegimos uno al azar y sera el numero en el cual
vamos a realizar la particion

- Algoritmos de dividir y conquistar

Este tipo de algoritmo lo podemos dividir siempre en los siguientes pasos:

 - Dividir
 - Conquistar
 - Combinar

Cuando dividimos es para tener una cantidad de datos mas manejable

- Algoritmos que utilizan programacion dinamica

Tienen algo del anterior pero en este caso los problemas que subdivimos no son
independientes de los anteriores(como en el caso de dividir y conquistar), por
ello se utiliza recursion para resolver los problemas y combinarlos


- Algoritmos Greedy

Estos algoritmos toman desiciones que son las mejores en el contexto en el que
se las plantea, en otras palabras toman desiciones que son localmente optimas
con la esperaza de que estas lleven a una solocion global optima. Un ejemplo es
el algoritmo de Huffman, cuando armamos el arbol de Huffman

## Cap2 Manipulacion de punteros

En C para cualquier type T, podemos tener una variable de un type que se
corresponda con T la cual contenga la direccion de memoria de esa variable del
type T. Una manera de "mirar" a esa variable es como si estuviera "apuntando"
al objeto. Por ello estas variables se las llama punteros


### Punteros como parametros de funciones

Los punteros son una parte importante de las llamadas a funciones en C. Lo mas
importante es que ellos son usados para soportar lo que se llama parametros de
funciones llamados por referencia o por valor, cuando pasamos un parametro por
referencia cuando una funcion cambia el parametro que le pasamos, los cambios
persisten una vez que la funcion retorna, en contraste cuando usamos un
paramtro por valor este solo cambia en el contexto de la funcion

### Punteros a punteros como parametros

Una situacion en la cual los punteros a punteros como parametro son validos es
cuando una funcion debe modificar un puntero pasado dentro de el. Por ejemplo
consideremos el caso de la funcion `list_rem_next()` la cual se define para
remover un elemento de una lista enlazada. Hasta que no haya retornado, `data`
apunta a los datos que seran removidos de la lista:

`int list_rem_next(List* next, ListElmt* element, void** data)`

Dado que la operacion debe modificar el puntero `data` para hacer que apunte a
los datos removidos, debemos pasarle la direccion de el puntero `data` para
simular un llamado por referencia. Asi la operacion toma un puntero a un
puntero como su tercer parametro. Esta es la manera en como los datos son
removidos para la mayoria de las estructuras de datos en este libro

y cuando llamamos a la funcion tenemos que acordarnos de hacer el cast
correctamente:

`list_rem_next(..., (void**)&iptr)`

en este caso tenemos un puntero que apunta a un dato que es un entero:

```C
int a = 10;
iptr = &a;
list_rem_next(.., (void**)&iptr);
```

## Cap 3: Recursion

Recursion es un topico importante porque nos permite algo que puede ser
definido en terminos de pequenios instancias de si mismo. En programacion,
recursion es realizado mediante funciones recusivas, una funcion recursiva es
una funcion que se llama a si misma, en cada llamada sucesiva va redefiniendo
la entrada a la nueva llamada dejando asi cada vez mas cerca de la solucion
buscada.

### Recursion basica

Un ejemplo que podemos tratar es el caso de que queremos calcular el factorial
de un numero, este problema muchas veces no se piensa como un problema
recursivo pero se puede hacer, supongamos que queremos calcular el factorial de
un numero `n` o sea `n!` como sabemos `n!` se define como el producto de todos
los numeros desde `n` hasta `1`, por ejemplo: `4!=(4)(3)(2)(1)`. Una manera de
calcularlo es con un loop que vaya desde `1` hasta `n` esto es la forma
iterativa que se puede plantear como: `n!=(n)(n-1)(n-2)...(1)`. Otra manera de
verlo es como un producto de pequenios factoriales. Para hacer esto definimos a
`n!` como `n` veces el factorial de `n-1` y asi hasta llegar al resultado o sea
hasta que `n=1` esto se puede definir formalmente como:

```text
       |--
       | 1           si n == 0 , n == 1
F(n) = |
       | n F(n - 1)  si n > 1
       |--
```

Existen dos fases basicas en la recursion que son "winding" y "unwinding". En
la fase de "winding" cada llamada recursiva llama a la recursion marcando una
llamada recursiva adicional. La fase de "winding" termina cuando una de las
llamadas alcanza una condicion de finalizacion, esta condicion define el estado
en cual la funcion recursiva debe retornar en lugar de hacer otra llamada mas
recursiva. Por ejemplo en el problema del factorial seria cuando `n` alcanza el
valor `1` o `n=0` para los cuales la funcion solo debe devolver `1`, siempre
tenemos que tener esta condicion sino no terminara nunca esta fase. Una vez que
hemos alcanzado esta fase se entra en la fase de "unwinding" en la cual se se
revisan las instancias previas de la funcion en orden reverso. Esta fase
termina cuando la llamada original retorna y en ese punto el proceso recursivo
es completado El proceso de "winding" puede llevar a utilizar mucha memoria del
stack, ya que en cada llamada debe guardar el estado de la funcion y su
resultado, por ello es que cuando hacemos una recursion muy larga podemos
obtener un "stack-overflow" que es cuando el programa se queda sin memoria en
la stack, pero existe un tipo especial de recursion que nos puede servir para
cuando esto sucede, que se conoce como "tail recursion"

### Tail recursion

Una funcion recursiva se dice que es "tail recursive" si todas sus llamadas
recursivas dentro de ella son "tail recursive". Una llamada es "tail recursive"
cuando su ultimo operacion es ejecutada dentro del cuerpo de la funcion y su
valor de retorno no es parte de una expresion. Estas funciones se caraterizan
por no tener operaciones en la parte de "unwinding". Esto es importante porque
muchos de los compiladores modernos pueden generar codigo que tome ventaja de
ello.  Para enteder como las funciones recursivas "tail" funcionan volvamos al
ejemplo del calculo del factorial recursivo. La manera que definimos la funcion
no era "tail" porque en cada activacion repetiamos la multiplicacion hasta que
`n=1` Esto no es "tail" porque el valor de retorno de cada iteracion depende de
multiplicar `n` veces el valor de retorno por las subsecuentes activaciones.
Por ello cuando se guardan estas activaciones deben permanecer en el stack
hasta que la las llamadas subsiguientes sean guardadas tambien. Ahora
consideremos una definicion de factorial que es "tail"

```text
          |--
          | a           si n == 0 , n == 1
F(n, a) = |
          | F(n - 1, na)  si n > 1
          |--
```
Esta definicion es similar pero usa un segundo parametro `a`(que inicialmente se
setea a `1`) el cual mantiene el valor del factorial computado hasta el momento
en el proceso de recursion. Esto nos previene de tener que multiplicar el valor
de retorno de cada activacion por `n`


## Analisis de Algoritmos

Esto lo vemos mas en detalle en el curso del MIT


## Parte 2 Estructura de Datos

## Cap5: Listas enlazadas

Son una estructura de datos fundamental, consiste de un numero de elementos
agrupados o "linkeados" en un orden especifico. Son utiles cuando queremos
mantener una coleccion de datos, en contraste con los arrays las listas
enlazadas son mas eficientes cuando ponemos datos en un determinado lugar y
cuando borramos elementos, tambien cuando no sabemos de antemano cuantos datos
vamos a utilizar.  Descripcion: Lista enlazada simple o lista simplemente
enlazada, estan compuestas de elementos individuales, cada uno del la cual esta
enlazado por un simple puntero. Cada elemento consiste de dos partes: un
miembro con datos y un puntero que apunta al proximo elemento El puntero `next`
del ultimo elemento apunta a `NULL`, que sirve como valor centinela para saber
donde termina la lista. el primer elemento de la lista se llama `head` y el
ultimo elemento de la lista se llama `tail`. Para acceder a los elementos de la
lista comenzamos desde la `head` y usando el puntero `next` vamos recorriendo
la misma hasta que llegamos al elemento deseado, cuando la lista es simple solo
podemos recorrer la lista en un sentido, en cambio cuando es doblemente
enlazada podemos recorrerla en los dos sentidos
