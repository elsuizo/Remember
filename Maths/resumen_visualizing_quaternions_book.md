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
complejos a 3D, pero lo que hacia era que queria forzar un elemento "imaginario"
mas fue un dia en 1843 a los 38 años de edad tuvo la brillante idea de construir
un objeto matematico con "tres partes imaginarias" que se convertiria en el algebra
de quaterniones(dice que iba con su esposa caminando por un puente cuando de repente
tuvo esa sensasion unica como una chispa y entonces tomo un cuchillo y escribio
las ecuaciones que luego serian famosas)

```math
i^{2} = j^{2} = k^{2} = ijk = -1
```

