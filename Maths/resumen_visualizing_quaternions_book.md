# Visualizing Quaternions


## Preliminares

Las principales ventajas de los Quaterniones son:

 - Los Quaterniones normalizados son simplemente vectores de 4 dimensiones `V4`
   con `norm2 = 1` y asi sus puntos estan sobre una *hyperesfera*(conocida por
   los matematicos como *esfera-tres*) que es una esfera que vive en 4D, asi como
   la esfera unitaria comun tiene dos grados de libertad(latitud y longitud) la
   hyperesfera tiene tres grados de libertad

 - Por eso hay una relacion entre los tres grados de libertad de una hyperesfera
   y los tres grados de libertad que se necesitan para representar una rotacion
   en el espacio que por ello puede ser representada por un quaternion normalizado

 - Ya que los quaterniones relacionan frames en tres dimensiones con puntos sobre
   la hyperesfera unitaria, se desprende que los Quaterniones son una herramienta
   clara y eficiente que nos permite medir distancias o similitudes entre dos frames
   en 3D

 - Como son puntos que estan sobre la hyperesfera, los Quaterniones pueden usarse
   para definir metodos optimos para interpolar entre muestras de 3D


Se hace incapie en los *quaternions manifold* los cuales tienen tres representaciones
basicas:


1. Curvas de Quaterniones: Se denomina asi a las interpolaciones resultantes entre
frames de 3D.
Estudiando las familias de todas los posibles frames usando a los Quaterniones
como representacion se llega a una representacion elegante de un sistema diferencial
de Quaterniones que describen la evolucion de los frames en general

2. Superficies de Quaterniones: Uno de los objetos matematicos que se utilizan
mucho en la computacion grafica son los *Gauss Maps* que son esencialmente una
*mesh* sobre una esfera de radio 1. Se pueden pensar a este mapa en funcion de
Quaterniones *Quaternions Gauss Maps* que son temas de geometria diferencial

3. Volumen de Quaterniones: Ya que los Quaterniones que se ven en este libro son
principalmente *unit quaternions* que son esencialmente tridimensionales y como
vimos el espacio de los quaterniones es una hyperesfera, el unico espacio util
que puede ser util construir desde quaterniones consiste de objetos volumetricos
acotados. Estos volumenes de Quaterniones son una nueva(el libro es del 2006 ojo eh)
forma de construir *dominios de orientacion* (*orientation domains*) como puede
ser el caso particular de una articulacion robotica con tres grados de libertad
puede tener cualquiera de sus tres estados representados por un punto de un
quaternion. La coleccion de todos esos puntos es un volumen, normalmente continuo
y el espacio permitido de esos estados esta acotado por ese volumen, de este modo
podemos conocer de manera elegante el espacio de trabajo del robot


## 01: El descubrimiento de los Quaterniones

Alla por el 1843 Sir William Rowan Hamilton intentaba generalizar a los numeros
complejos a 3D, pero lo que hacia era que queria forzar un elemento
"imaginario" mas fue el dia 16 de octubre de 1843 a los 38 años de edad tuvo la
brillante idea de construir un objeto matematico con "tres partes imaginarias"
que se convertiria en el algebra de quaterniones(dice que iba con su esposa
caminando por un puente cuando de repente tuvo esa sensasion unica como una
chispa y entonces tomo un cuchillo y escribio las ecuaciones que luego serian
famosas, la razon principal por lo que hizo esto es porque sintio miedo de que
le pudiera pasar algo y que no se pueda saber de su descubrimiento!!!)

```math
i^{2} = j^{2} = k^{2} = ijk = -1
```
que contiene la solucion al probelema que estaba buscando solucionar. Las
reglas de multiplicacion de quaterniones de Hamilton que contiene tres
subreglas que son identicas a la de la multiplicacion de numeros complejos,
expresa la profunda conexion que existe entre vectores de 4D normalizados y
rotaciones en 3D en el espacio euclideo. La aparicion de tres copias de lo que
es una rotacion en dos dimensiones expresada por un numero complejo no es
accidental. Como vamos a ver cada una de las reglas que se ponen para la
multiplicacion involucran a dos de los tres ejes posibles(que es lo que se
queria expresar) Hamilton siguio con el desarrollo de los Quaterniones y en el
año 1853 publica el clasico libro del tema llamado: *Lectures on Quaternions*
para luego publicar otro libro mas extenso llamado *Elements of Quaternions*
que se publica un año mas tarde de su muerte en 1865. La posta de este topico
fue tomada por Peter Tait, quien espero diplomatica y respetuosamente la muerte
de Hamilton para publicar su version del topico llamado *Elementary Treatise on
Quaternions*

### Entonces vinieron los *Octonions*

Una de las preguntas fascinantes que se hizo la gente luego de conocer los quaterniones
es: "Si los quaterniones generalizan a los numeros complejos, pueden los quaterninones
ser generalizados tambien?"
Y se llego que bajo ciertas cirscuntancias existe exactamente una generalizacion
posible los *octonions*. La historia de como comenzo a pensarse en los *octonions*
es tan interesante como la de *quaternions*, muy poco tiempo despues de que Hamilton
descubriera los *quaternions* le envio una carta a su amigo Jhon T. Graves contandole
su descubrimiento. Este le constesto que le parecia una idea brillante pero agrego:
"Hay algo en el sistema que todavia que no me cierra. No tengo una clara vision
de si podemos crear arbitrariamente mas imaginarios y de dotarlos de propiedades
sobrenaturales", a lo que tambien pregunto: "Si con tu alquimia tu puedes hacer
tres libras de oro, porque te tendrias que detener ahi?". En el 26 de diciembre
de 1843 Graves escribio a Hamilton que habia encontrado una generalizacion para
los *Quaternions* que los llamo *octaves* una algebra de 8D con la cual el pudo
probar que el producto de la suma de 8 numeros cuadrados perfectos es la suma de
otros 8 numeros cuadrados perfectos, pero parece que Hamilton no pudo darle mucha
ayuda a Graves y en 1845 los *Octonions* los descubre Arthur Cayley. Sin hacer caso
de que Graves y Hamilton reclamaban que ellos lo habian descubierto antes, por
ello lamentablemente los *Octonions* se conocen como "los numeros de Cayley" y
la contribucion de Graves se olvida

### El renacimiento de los Quaterniones

Una de las cosas por las que peleo Hamilton fue para que se considerara a la
notacion de quateriones como la standar para las operaciones de 3D como el producto
dot y el producto cross ya que la notacion fue siempre percibida como fea y asi
la notacion alternativa de tensores se convirtio en el estandar que es por
ejemplo la manera de expresar un vector en 3D como `x = (x, y, z)`, luego cuando
se comenzo con la mecanica cuantica tambien se pudo elegir a los quaterniones
para expresar a los "spinors" pero se eligio otro sistema. La comunidad de aeronauticos
y astronauticos fue quien comenzo a explotar a los quaterniones en aplicaciones
practicas ver:
 - [98] "P. Hughes Spacecraft Attitude Dynamics, New York 1986"
 - [169] "J. Wertz. Spacecraft Attitude Determination and Control. 1985"
 - [103] "J. Junkins and J. Turner. Optimal Spacecraft Rotational Maneuvers". Elsevier 1986

La tecnologia no habia penetrado aun en el mundo de las animaciones o videojuegos
hasta que llego el paper de Ken Shoemake: [149]: "Animating Rotation with Quaternion Curves"
Este paper presenta a los quaterniones por primera vez en este ambito y los compara
con los angulos de euler que son mas ineficientes, lo que da a un gran auge de
trabajos al respecto:
 - [139]: "D. Pletinks. Quaternions Calculus as a basic Tool in Computer Graphics. 1989"
 - [4]: "S. L. Altmann. Rotations, Quaternions, and Double Groups. Oxford 1986"
 - [104]: "B. Juttler. Visualization of Moving Objects using Dual Quaternion curves. Computer and Graphics. 1994"
 - [115]: "J. B. Kuipers. Quaternions and Rotation Sequences. Princeton, NJ 1999"

La principal herramienta que introdujo el paper [149] es una formula de interpolacion
para un gran circulo conectando dos puntos sobre una esfera de dimensiones arbitrarias
(note: no se si esta bien la traduccion y es una esfera de dimensiones arbitrarias
o es una hypersfera)
Por analogia al acronimo "LERP" que se usa cuando hacemos interpolacion lineal
ordinaria, Shoemake le puso el nombre de "SLERP" por "Spherical Linear Interpolation"
que se siguio usando hasta ahora

## Cap2: El folklore de las Rotaciones

Vamos a ver tres ejemplos distintos: una cinturon comun y corriente, una bola y un
gyroscopo, vamos a ver que cada uno tiene una peculariedad que hace que sea imposible
de explicar con el lenguaje de todos los dias, una vez que veamos como se ven desde
la perspectiva de los quaterniones vamos a cambiar de parecer

### El truco de la cinturon

Este ejemplo lo podemos experimentar en cualquier momento ya que no necesitamos
de nada extranio solo un cinturon o una cinta, vamos a ver que pasa algo inexplicable
en la logica de todos los dias cuando hacemos una secuencia de rotaciones determinadas

1. Dos personas comienzan por sostener firmemente los extremos del cinturon
2. Existe una regla: Cada uno de los que tiene la punta del cinturon puede mover
   la punta a cualquier punto en el espacio que quiera, siempre que se mantenga
   la orientacion en el espacio en la cual comenzamos(tampoco se permite que se
   corte o haga algo raro al cinturon)
3. Los jugadores tuercen el cinturon por una vuelta entera(360 grados) siempre
   manteniendo la orientacion del comienzo, los jugadores deben hacer todo lo posible
   para que el cinturon vuelva al estado original
4. Luego si ello tuercen el cinturon por dos vueltas completas(720 grados), van
   a descubrir que bajo las mismas condiciones el cinturon puede rapidamente retornar
   a su estado original sin torceduras!!!

Porque???

No existe una explicacion que no exponga cierto grado de matematicas y con quaterniones


### La bola que rueda

Lo unico que necesitamos es una pelota(que puede ser de tenis por ejemplo), nuevamente
nos vamos a concentrar en el resultado de una secueucia de rotaciones

1. Poner la pelota sobre algo plano
2. Poner la mano arriba de la pelota con la palma horizontal a la mesa, tocando
   la bola en un solo punto(la parte de arriba de la pelota). Moviendo la mano
   paralela a la mesa resultara en una rotacion alrededor de un eje en el plano
   de la mesa
3. Sin torcer la munieca de ninguna manera sobre el eje vertical y mantieniendo
   la palma paralela a la mesa, hacer un movimiento circular
4. Mirar un punto cualquiera sobre el equador de la pelota
5. Si frotamos en pequenios circulos, el punto en el equador rotara muy poco en
   sentido antihorario. Si frotamos en pequenios circulos en sentido antihorario
   el punto en el equador va a rotar en sentido horario
6. Para obtener resultados mas dramaticos, podemos rotar la bola 90 grados moviendo
   la palma a la derecha 90 grados moviendo la palma hacia tu cuerpo y 90 grados
   una vez mas moviendo la palma hacia la izquierda. El resultado es que una rotacion
   en sentido antihorario sobre el eje vertical, mientras que no hemos hecho ninguna
   rotacion sobre este eje!!!

Porque sucede esto???

Vamos a ver que con quaterniones podemos expresar este problema de manera simple


### El incidente del apollo 10 llamado gimbal-lock

Los *Quaternions* se usan ahora extensamente en el area aero-espacial para contol
de actitud de cohetes, satelites etc. Podemos consultar mas info de estos temas
de aplicacion por ejemplo en el libro: [115] "Quaternions and Rotations Sequences"
de J. B. Kuipers.

El aparato que se usa para medir cual es la correcta orientacion de una nave, era
en su momento del apollo 10 mecanico(IMU mecanica). Una muestra de los logs de
conversacion de esa mision cuando estaba volviendo el modulo llamado "snoopy" al
modulo comando, en los manuales que tenian los pilotos tenian la especial recomendacion
de evitar que haya un "gimbal lock". Pero que es este "gimbal lock" y porque tenian
que tener especial cuidado con el?
El contexto basico es que la IMU del apollo era un sistema mecanico con tres grados
de libertad aparentes. La caracteristica critica de estos aparatos que esta diseniado
para mantener la orientacion de la nave con respecto al mundo exterior, este sistema
es acoplado al frame de la nave usando tres anillos rotantes que son perpendiculares
uno del otro. El frame de afuera puede rotar libremente sin problemas alrededor del
eje 1 o del eje 3. Sin embargo si rotamos 90 grados alrededor del eje 2 los ejes 1
y 3 se alinean, entonces hemos perdido efectivamente un grado de libertad: la habilidad
de rotar libremente alrededor del eje1.


### Cuando el F-16 hacia un flip

Basicamente tenian un sistema de control que se llama "fly-by-wire" que lo que hacia
cuando el piloto llegaba al ecuador del planeta es que cambie la orientacion del
avion haciendolo navegar apuntando hacia abajo(todo esto fue en una simulacion)


## Notacion Basica

lo puse en una hoja

## Que son los *Quaternions*

Los *Quaternions* son un vector de 4D: `q = (q_0, q_1, q_2, q_3) = (q_0, q)`


## El camino a la visualizacion de los *Quaternions*

### Los cimientos para visualizar a los *Quaternions*

Usando las propiedades de los numeros complejos asociadas con rotaciones en 2D
como punto de partida vamos a estudiar muchos conceptos que podremos extender a
las propiedades de las rotaciones en 3D. Los cuatro pilares de nuestro edificio
son:

1. Geometria: Como vimos los Quaterniones son vectores de 4D unitarios y asi su
   corresponden geometricamente a puntos que tienen un radio constante desde el
   origen, la cual es la definicion de una esfera

2. Algebra: El algebra de los Quaterniones tiene su interpretacion geometrica que
   incluye a los numeros complejos como una subalgebra y los resultados de operaciones
   algebraicas pueden visualizarse usando metodos geometricos

3. Calculo, logaritmos y exponenciales: La relacion que existe entre los logaritmos
   de los Quaterniones sus exponenciales y el calculo con los Quaterniones provee
   otra de las caracteristicas visualizables, la cual vamos a estudiar empezando
   por la forma polar de los numeros complejos

4. Interpolacion: La interpolacion desde un Quaternion hacia otro tiene muchas
   analogias con las interpolaciones estandar entre polinomios en el espacio
   euclideo. Vamos a ver que las curvas geodesicas o de "gran circulo" sobre
   esferas proveen el punto de partida para una rica familia de metodos de
   interpolacion y su descripcion grafica


## Fundamentos de Rotaciones

### Rotaciones en 2D

Como sabemos de Algebra lineal las rotaciones de de vectores en 2D pueden ser
implementadas aplicando a ellos la transformacion de matrices ortogonales simetricas
que como sabemos tienen determinante 1
