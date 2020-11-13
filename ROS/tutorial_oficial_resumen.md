# Cosas para recordar del tutorial oficial

## Creando un workspace de ROS

Primero creamos la carpeta en donde queremos tener el _workspace_:

`mkdir -p ~/name_folder/src`

Una vez que tenemos las carpetas listas hacemos, dentro de la misma:

`catkin_make`

Con eso hemos creado el _workspace_ de catkin, para activarlo en la sesion de
`bash` o `zsh` hacemos `source devel/setup.bash` o `source devel/setup.zsh`
(segun sea el tipo de consola que utilizemos)

## Trabajando con Packages

- Existen dos packages mannager(deben haber mas pero los mas usados son estos)
   - `catking`
   - `rosbuild`

 - Los packages son la unidada de organizacion de software de ROS. Cada package
   contiene librerias, ejecutables, scripts y mas cosas
 - **Manifests**(`package.xml`): Es una descripcion del package. Sirve para
   definir dependencias entre `packages` y para capturar **meta-informacion**
   de estos como puede ser version, quien lo mantiene, licencias, etc...

## Herramientas para trabajar con *FilesSystems*

Como el codigo esta desperdigado por muchos lugares del sistema, seria tedioso
tener que ir a cada uno a mano con `ls` o `cd`.
Por eso ROS nos da herramientas para que sea mas facil navegar por los `packages`

### Usando `rospack`

`rospack` nos permite obtener informacion de los `packages`. Por ejemplo si
queremos encontrar un `package` podemos utilizar `find`

`rospack find [package name]`

### Usando `roscd`

Como la utilidad de `bash` `roscd` nos cambia de directorio, por ejemplo podemos
movernos a donde ROS guarda los archivos log

`roscd log`

### Usando `rosls`

Es parte de los comandos para navegar `packages` y nos permite hacer un `ls`
directamente sobre el nombre de un `package`

`rosls [location_name/subdir]`, por ejemplo: `rosls roscpp_tutorials`

## Creando un `package` de ROS

## Que tiene que tener un `package` de catking???

 - Debe contener un archivo `xml` que cumpla con los requisitos que estan aca:
   https://wiki.ros.org/catkin/package.xml
   - Este `package.xml` provee la meta-informacion del `package`
 - Debe contener un archivo de cmake(el build system que usa ROS por default)
   que cumpla con los requisitos que estan aca:

   https://wiki.ros.org/catkin/CMakeLists.txt
   - Si es un metapaquete de catkin debe tener su propio archivo de cmake
   tambien:
   https://wiki.ros.org/catkin/package.xml#Metapackages
 - Cada `package` debe tener su propia carpeta
   - Esto quiere decir que no podemos anidar `packages` para compartirlos en
   una misma carpeta

El `package` mas simple que podamos hacer debe tener una minima estructura como sigue:

```bash
   my_package/
      CMakeLists.txt
      package.xml
```

### `packages` en un *workspace* de catkin

La manera recomendada de trabajar con `packages` de catkin es en su espacio de
trabajo que configuramos anteriormente, pero
podemos si queremos construir un `package` de manera aislada. Un workspace que
recien empieza se puede ver asi:

```bash
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```

### Creando un `package` de catkin

Primero vamos a nuestro workspace que creamos anteriormente(en este caso parece
que no tenemos un `roscd`, por lo que podriamos
hacernos un `alias` en nuestro `.zshrc` para que nos lleve a el con un solo
comando) (ojo que tenemos que ir `nombre_carpeta_catkin/src`)

Ahora vamos a crear el `package` con el script llamado `catkin_create_pkg` que
lo vamos a llamar `beginner_tutorials` el cual va a depender de
`std_msgs`, `roscpp` y `rospy`

```bash
zsh> catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```
O sea que primero va el nombre que le vamos a dar a el `package` y luego van las
dependencias

### Construyendo el `package`

Luego necesitamos construir el `package` en el workspace de catkin

`cd catkin_workspace_folder`
`catkin_make`

### Dependencias en los `package`s

Cuando creamos el `package` anteriormente hicimos explicitos los `package`s de
los que dependia, podemos saber esa informacion con el comando
`rospack depends1 beginner_tutorials`

y cuando queremos toda la lista de dependencia tanto la que tiene nuestro
`package` como la que dependen nuestras dependencias hacemos:

`rospack depends nombre_package`

## Entendiendo los nodos de ROS

### Un poco de terminologia

 - Node: un nodo es un ejecutable que usa ROS para comunicarse con otros nodos:
   https://wiki.ros.org/Nodes

 - Messages: Son un type de ROS usado cuando nos subscribimos o publicamos en
   un `topic`: https://wiki.ros.org/Messages

 - `topic`: es el canal de comunicacion al cual nos podemos subscribir o enviar
   mensajes (`message`)
   https://wiki.ros.org/Topics

 - `Master`: nombre de los servicios de ROS(lo usamos para que los nodos se
   puedan encontrar unos a otros): https://wiki.ros.org/Master

 - `rosout`: el equivalente de ROS a `stdout` o `stderr`:
   https://wiki.ros.org/rosout

 - `roscore`: `Master` + `rosout` + los parametros del server:
   https://wiki.ros.org/roscore


### Nodos

Los nodos no son mas que ejecutables dentro de un `package` de ROS. ROS usa los
nodos para comunicarse con otros nodos. Los nodos pueden publicar o subscribirse
a un `Topics`. Los nodos tambien pueden proveer o usar un `Service`

### Librerias `client`

Las librerias `client` de ROS nos permiten escribir nodos en lenguajes de
programacion diferentes para comunicarnos:

 - `rospy` = cliente en Python
 - `roscpp` = cliente en Cpp
 - `rosrust` = cliente en Rust(https://github.com/adnanademovic/rosrust)

### `roscore`

Es el primer comando que tenemos que llamar cuando usamos ROS

### Usando `rosnode`

Cuando llamamos a `rosnode` (siempre que `roscore` este corriendo) vamos a ver
informacion acerca de los nodos que estan corriendo en este instante, por
ejemplo si queremos ver los nodos que estan activos: `rosnode list` y como solo
tenemos corriendo `roscore` vemos `rosout` como salida. Osea que el unico nodo
que esta corriendo es ese (que lo que hace es recolectar logs y outputs)

Podemos preguntar mas info de un nodo con `rosnode info /node`

### Usando `rosrun`

Nos permite usar el nombre del `package` directamente para correr un nodo que
esta dentro de ese `package` (sin tener que saber el path)

`rosrun [package_name] [node_name]`

Ahora podemos correr el nodo `turtlesim_node` del `package` `turtlesim`

`rosrun turtlesim turtlesim_node`

y ahora si hacemos de nuevo `rosnode list` vemos al nodo que acabamos de lanzar
que es de la tortuga(que no se te escape) le podemos cambiar el nombre del nodo
cuando lo lanzamos si queremos

## Entendiendo los `topics` de ROS

los nodos `tutlesim_node` y `turtle_teleopt_key` se comunican mediante un
`topic` de ROS, `turtle_teleopt_key` publica las teclas que estamos presionando
en un `topic`, mientras que `turtlesim_node` se subscribe a el mismo `topic` para
recibir los valores de las teclas.

para correr `turtle_teleopt_key` hacemos:

`rosrun turtlesim turtle_teleop_key`


Podemos usar la herramienta `rqt_graph` para
ver como estan interactuando actualmente los nodos y `topic`s

Hay mas subcomandos para `rostopic` los podemos ver haciendo `rostopic -h`

### Usando `rostopic echo`

Con este subcomando podemos ver los datos que son publicados en un `topic`
`rostopic echo [topic]`
Por ejemplo podemos ver los comandos de velocidad que publica el nodo
`turtle_teleopt_key` con `rostopic echo /turtle1/cmd_vel`

## Mensages en ROS

la comunicacion entre los `topic`s sucede enviando mensajes de ROS entre nodos.
Desde el que publica(`turtle_teleopt_key`) al que se subscribe(`turtlesim_node`),
tanto el que se subscribe como el que recibe el mensaje deben usar el mismo
"type" de mensaje(queremos decir "type" en el sentido de tipo de datos).
El type de un mensaje puede consultarse usando el comando `rostopic type`

Y podemos ver los detalles de un mensaje usando `rosmsg`:

`rosmsg show turtlesim/Velocity`

### Usando `rostopic pub`

Con este comando podemos publicar datos en un `topic` que este corriendo
actualmente: `rostopic pub [topic] [msg_type] [args]`, y con esto podriamo
mover a la tortuga por ejemplo. Tambien podemos publicar un mensage de forma
constante haciendo: `rostopic pub -r`. Que hace que la tortuga se mueva
constantemente. Por ejemplo:

`rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'`

 - El `-1` le dice a ROS que solo publique un msg
 - La opcion de `--` le dice al parser que ninguno de los argumentos que sigue
   son una opcion. Esto se requiere en casos donde los argumentos tienen un solo
   `-` guion, como por ejemplo con los numeros negativos

Y cuando quermos publicar un msg constantemente(al rate que esta manejando el
node) hacemos:

`rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'`

Usando `rostopic hz [topic]` podemos ver a la tasa a la que se esta publicando
los datos

## Entendiendo los servicios y paramtros de ROS

Los _service_s son otra manera que los nodos pueden comunicarse unos con otros,
estos permiten a los _nodes_ enviar una _request_ y recibir un _response_

### Usando `rosservice`

`rosservice` puede facilmente adjuntar a un _framework_ de ROS con servicios.
`rosservice` tiene muchos comandos que pueden ser utilizados sobre _service_s como
mostramos a continuacion:

```text
rosservice list      (imprime la informacion sobre los servicios activos)
rosservice call      (llama al servicio con los args que le proveemos)
rosservice type      (imprime el type del servicio)
rosservice find      (encuentra el servicio por el type)
rosservice uri       (imprime el servicio ROSPC uri)
```

#### `rosservice list`

La lista de comando que el node `turtlesim` provee nueve servicios:

```text
/clear
/kill
/reset
/rosout/get_loggers
/rosout/set_logger_level
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/get_loggers
/turtlesim/set_logger_level
```
Vemos que tambien se incluye a los nodes que pertenecen al node `rosout`

Podemos ver cual es el type de un node como vimos con el comando: `rosservice type [service]`

que en el caso del service `clear` nos devuelve: `std_srvs/Empty`

Lo que significa que este servicio esta vacio osea que cuando una llamada es hecha
no se necesitan argumentos(por ejemplo: no se envia datos cuando hacemos un _request_
y no revibimos datos cuando tenemos una respuesta), vendria a ser que `type` the
da cuales son los types que tenes que pasarle al _service_ para que ande

#### usando `rosservice call`

`rosservice call [service] [args]`

Por ejemplo para el service anterior podemos hacer:

`rosservice call /clear` (que lo unico que hace en este caso es limpiar la pantalla de la
tortuga)

Podemos ver un caso mas interesante en el que el `service` toma parametros, por
ejemplo el `service` `spawm`:

`rosservice type /spawn | rossrv show`

que nos muestra:

```text
float32 x
float32 y
float32 theta
string name
---
string name
```
Que como podemos deducir lo que hace este `service` es crear una nueva tortuga(que no se te escape)
en una dada posicion y orientacion(el nombre es opcional), entonces llamamos a `call`:

`rosservice call /spawn 2 2 0.0 ""`

## Usando `rosparam`

`rosparam` nos permite guardar y manipular datos sobre el server de parametros de
ROS, este puede guardar enteros, floats, Strings, boolean y diccionarios. `rosparam`
utiliza YAML como sintaxis, los comandos mas utilizados son:

```text
rosparam set    (setea los parametros)
rosparam get    (getter de parametros)
rosparam load   (carga los parametros desde un archivo)
rosparam dump   (dumpea parametros a un archivo)
rosparam delete (borra parametros)
rosparam list   (lista los nombres de los parametros)
```
Podemos ver cuales son los parametros que estan actualmente en el server con
`rosparam list`:

```text
/rosdistro
/roslaunch/uris/host_nxt__43407
/rosversion
/run_id
/turtlesim/background_b
/turtlesim/background_g
/turtlesim/background_r
```

podemos cambiar alguno de estos parametros usando `rosparam set`:

`rosparam set [param_name]`

`rosparam get [param_name]`

por ejemplo podemos cambiar el color del canal rojo del fondo donde camina la tortuga(que nunca
se te escape)

`rosparam set /turtlesim/background_r 150`

luego para que el cambio que hicimos tenga efecto tenemos que llamar al `service`
`/clear`

`rosservice call /clear`

Tambien podemos usar `/` para ver todos los parametros que estan en el server

`rosparam get /`

Podriamos querer guardar estos parametros en un archivo para despues volver a
cargarlos, podemos hacerlo con:

`rosparam dum [filename] [namespace]`

`rosparam load [filename] [namespace]`

Por ejemplo:

`rosparam dump params.yaml`

Y podriamos cargar estos parametros en un nuevo `namespace` (`copy_turtle`)

`roparam load params.yaml copy_turtle`

`rosparam get /copy_turtle/turtlesim/background_b`


## Usando `rqt_console` y `roslaunch`

### Usando `rqt_concole` y `rqt_logger_level`

con `rqt_console` podemos adjuntar un framework de login para mirar las salidas
de diferentes nodos. `rqt_logger_level` nos permite setear cuando y como se ven
esas salidas (`DEBUG`, `WARN`, `INFO` y `ERROR`)

Por ejemplo vemos como son las salidas con el node `turtlesim` que venimos utilizando

para poder utilizar esta herramienta debemos llamar al package y a su node con
`rosrun`:

`rosrun rqt_console rqt_console`

`rosrun rqt_logger_level rqt_logger_level`

o sea que es una mas amigable de mirar los logs de los nodes

los logs tienen niveles de prioridades como sigue:

 1. `Fatal`
 2. `Error`
 3. `Warn`
 4. `Info`
 5. `Debug`

Osea que si ha sucedido un msg `Error` y despues un `Fatal` primero vemos el `Fatal`

## Usando `roslaunch`

Podemos comenzar un node que se definio en un file del tipo `.launch`

`roslaunch [package] [filename.launch]`:

Con el package ejemplo que hicimos al principio que se llamaba `beginner_tutorials`
podemos ir a esa carpeta como vimos con `roscd` una vez que estamos en ella vamos
a crea un directorio para los archivos `.launch`

```bash
   mkdir launch`
   cd launch
```
Este paso no es necesario(ya que `roslaunch` se fija automaticamente en los `.launch`)
pero es una buena practica mantener el package ordenado


### El archivo de `.launch`

Vamos a crear el siguiente archivo llamado `turtlemimic.launch`

```xml
<launch>

  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>
```

### El `.launch` explicado

Este archivo del estilo `.xml` tiene "tags" el primero que vemos es `<launch>`
que es como identificamos un archivo `launch`. Luego comenzamos dos grupos con
espacio de nombres(`namespaces`) `turtlesim1` y `turtlesim2`, esto lo que hace
es que podamos comenzar dos simulaciones sin conflictos de nombres


Aquí comenzamos el nodo mimico con los temas de entrada y salida renombrados a
`turtlesim1` y `turtlesim2`. Este cambio de nombre hara que `turtlesim2` imite a
`turtlesim1`.

Osea es como hacer con simulink con una flecha la salida de un bloque con la
entrada de otro(muuuy buenooo)

### Corriendo el archivo con `roslaunch`

`roslaunch beginner_tutorials turtlemimic.launch`

## Usando `rosed`

`rosed` es parte de la suite `rosbash`. Este permite editar directamente un `file`
que esta en un `package` usando el nombre de `package`

`rosed [package_name] [filename]`

ejemplo: `rosed roscpp Logger.msg`

Y lo que hace este comando es abrir nuestro editor de texto(neovim) con el file
que le pasamos

## Creando un `msg` en ROS y un `srv`

Vamos a ver como crear y construir archivos `msg` y un `srv`

### Introduccion a los `msg` y los `srv`

 - archivos `msg` son simples archivos de texto que describen los fields de un
   mensaje en ROS. Son usads para generar codigo cuando compilamos un package
 - un archivo `srv` describe un servicio. Esta compuesto de dos partes: un `request`
   y un `response`

los archivos `msg` son guardados en los carpeta `msg` de un package y los `srv` en
en la carpeta `srv`, los `msg` son simples archivos de texto con un type y un nombre
por linea. Los types que se pueden utilizar son:

 - `int8`, `int16`, `int32`, `int64`, `uint`
 - `float32`, `float64`
 - `string`
 - `time`, `duration`
 - otros archivos de `msg`
 - array de largo variable: `array[]` y de largo fijo: `array[C]`

Existe tambien un type especial en ROS que es `Header` este contiene datos del
tiempo, informacion de los frames, que es comunmente utilizado en ROS

Podemos ver casi siempre en un archivo `msg` que tiene este type `Header` en la
cabecera del archivo, por ejemplo veamos un `msg` que usa un `Header`, un `string`
y otros dos `msg`s mas:

```text
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

archivos `srv` son como los archivos `msg` exepto que contienen dos partes: un `request`
y un `response` las dos partes son separadas por un `---` por ejemplo:

```text
int64 A
int64 B
---
int64 Sum
```

Osea que lo que vemos en este ejemplo que `A` y `B` son la parte de `request` y
que `Sum` es la parte de `response`

## Usando `msg`s

### Creando un `msg`

Vamos a definir un nuevo `msg` en el package que creamos

```text
roscd beginner_tutorials
mkdir msg
echo "int64 num" > msg/Num.msg
```

Hay un paso mas para que podamos utilizar el nuevo `msg` y es que debemos asegurarnos
de que en el tiempo de compilacion este archivo se convierta en en lenguaje de programacion
que estamos utilizando, para ello debemos verificar en el archivo `package.xml`
y debemos asegurarnos de que las siguientes dos lineas esten descomentadas:

```xml
<build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

Esto como dijimos pone de maniefiesto que en tiempo de compilacion `build_depend`
necesitamos la generacion de los mensajes

Ademas debemos agregar esta generacion de msg en el archivo `Cmake` del proyecto
para ello abrimos el archivo `CmakeLists.txt` y agregamos la linea:
`message_generation` como dependencia:

```cmake
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```
Tambien debemos asegurarnos de que exportamos el mensaje como `runtime`, para
ello debemos modificar las lineas en donde se ve:

```cmake
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```
Luego debemos agregar al `msg` en el bloque de codigo:

```cmake
add_message_files(
  FILES
  Num.msg
)
```
Luego debemos asegurarnos de que la funcion `generate_messages()` sea llamada,
para ello descomentamos las siguientes lineas:

```cmake
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

## Usando `rosmsg`

Eso es todo lo que necesitamos para generar un archivo de `msg`. Podemos asegurarnos
de que ROS "ve" a nuestro simple `msg` utilizando el comando `show`:

`rosmsg show [message type]`

Ejemplo:

`rosmsg show beginner_tutorials/Num`

Nos tiene que mostrar los types que definimos en el archivo

## Usando `srv`

### Creando un `srv`

Vamos a crea un `srv` en el package que estamos utilizando de pruebas:

```text
roscd beginner_tutorials
mkdir srv
```

En lugar de hacer el archivo a mano vamos a copiar uno de otro package, para esto
`roscp` es un comando util para copiar archivos desde un package a otro:

```text
roscp [package_name] [file_to_copy] [path]
```
Ahora podemos entoces copiar un servicio que esta en el package `rospy_tutorials`

`roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv`

Como con los archivos `msg` debemos activar su generacion en el `cmake`

Y como tambien para `msg` necesitamos agregar el archivo con la extension en el
bloque de codigo:

```cmake
add_service_files(
  FILES
  AddTwoInts.srv
)
```

### Usando `rossrv`

Como con `msg` podemos verificar que ROS "ve" a el `srv` que creamos con el comando
`rossrv show`:

`rossrv show <service type>`

Ejemplo:

`rossrv show beginner_tutorials/AddTwoInts`

## Escribiendo un simple _publisher_ y _subscriber_ con Python

Primero creamos una carpeta donde el `cmake` va a buscar nuestro _script_ que se
tiene que llamar `scripts`

Entonces en nuestro proyecto que venimos haciendo:

```text
roscd beginner_tutorials
mkdir scripts
cd scripts
```

en esa carpeta ponemos el siguiente script de Python(con el nombre `talker.py`):

```python
#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
Al que tenemos que darle permisos de ejecucion: `sudo chmod +x talker.py`

Luego debemos modificar el `cmake` para que sepa donde tenemos el script:

```text
catkin_install_python(PROGRAMS scripts/talker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
## El codigo explicado

Como sabemos todos los scripts de Python deben comenzar con esta linea para que
el interprete sepa que se trata de un archivo de Python
```python
#!/usr/bin/env python
```

Necesitamos importar `rospy` que es la libreria principal que hace que podamos
escribir el codigo en Python, luego el otro `import` es para que podamos reutilizar
el type `std_msg/String` (que es un simple String) para publicar el mensaje

```python
import rospy
from std_msgs.msg import String
```

Luego con la siguiente linea:
```python
pub = rospy.Publisher('chatter', String, queue_size=10)
rospy.init_node('talker', anonymous=True)
```
Estamos declarando nuestro nodo va a publicar en el `topic` `chatter` usando el
type para el mensaje como un `String` (que viene del import que hicimos) y luego
`queue_size` es el limite que queremos que tenga la cola de mensajes si algun
subscriptor no esta recibiendo lo suficientemente rapdido. La proxima linea es
importante ya que le dice a `rospy` el nombre de nuestro nodo (hasta que `rospy`
no tiene esta informacion, este no puede comenzar la comunicacion con el `master`
de ROS) en este caso el nodo se va a llamar `talker`, el parametro `anonymous=True`
nos asegura que nuestro nodo tiene un unico nombre al que se le van a aniadir
al final del `NAME` (en este caso `talker`) unos caracteres random.
Luego con la linea:

```python
rate = rospy.Rate(10) # 10Hz
```
creamos un objeto `Rate` que con uno de sus metodos llamados `sleep()` nos va
a permitir implementar un loop de `10Hz`(o sea que vamos a correr este loop 10
veces por seg)

Luego en las lineas:

```python
while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
```
chequeamos que el flag de `.is_shutdown()` que nos dice si el programa debe
finalizar, luego creamos un `str` que va a ser lo que publiquemos con las funciones
`publish()` y `loginfo()` una es la que publica en el canal de comunicacion y la
otra publica en los lugares donde se hacen `logs` habitualmente(en el `screen`, en
el archivo de log del `node` y en `rosout`) osea que es una herramienta para debbugear

## Escribiendo del nodo _subscriber_

### El codigo

```python
#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

Para agregar este nodo debemos hacer como en el caso del `talker.py` agregarlo
en el `cmake`:

```cmake
catkin_install_python(PROGRAMS scripts/talker.py scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### El codigo explicado

El codigo para este `listener.py` es similar que el de `talker.py` a exepcion que
hemos introducido un nuevo mecanismo de "callback" para subscribirnos a `msg`s

```python
rospy.init_node('listener', anonymous=True)

rospy.Subscriber("chatter", String, callback)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
```

Osea estamos dandole un nombre al nodo y luego nos subscrivimos al `topic` `chatter`
que tiene como type `std_msgs.String` y cuando un nuevo `msg` es recibido llamamos
a la funcion `callback` con el `msg` como primer argumento. La ultima linea llama
al metodo `spin()` que simplemente mantiene nuestro nodo "vivo" hasta que se le de
un "shutdown"

Una vez que hemos hecho esto solo nos resta hacer `catkin_make` en el _workspace_
de `catking` para que se genere el codigo que necesitamos

## Creando un _publisher_ en Cpp

Primero creamos una carpeta que va a ser donde el `cmake` busque los archivos de
codigo

```text
roscd beginner_tutorials
mkdir -p src
```

Creamos un archivo con el nombre `talker.cpp`:

```cpp
/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    chatter_pub.publish(msg);
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%
```

### El codigo explicado

Primero importamos los headers:

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
```
En `ros.h` tenemos la mayoria de los headers que son necesarios para que funcione
ROS. Luego incluimos el header para el type de `msg` `String` que esta en el package
como vimos `std_msgs`

```cpp
ros::init(argc, argv, "talker");
```
Luego inicializamos ROS y la damos un nombre a nuestro node.

```cpp
ros::NodeHandle n;
```
Creamos un _handle_ para este proceso. EL primer nodo hara la inicializacion de
los nodos y el ultimo hara la destruccion limpiando todos los recursos que usamos(que miedito...)

```cpp
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
```
Le decimos al nodo `master` que vamos a publicar un `msg` de type `std_msgs/String`
sobre el `topic` "chatter". Esto hace que el nodo `master` pueda vincular a cualquier
nodo con este `topic` en el que estamos publicando y el segundo argumento es el
tamanio de la queue que queremos

`NodeHandle::advertise()` retorna un objeto `ros::Publisher` el cual sirve para dos
cosas:

 1. Contiene un metodo `publish()` que nos deja publicar mensajes sobre un `topic`
 2. Cuando queda fuera del scope automaticamente se dejara de publicar o _advertise_

```cpp
ros::Rate loop_rate(10);
```
Creamos un objeto `Ros::Rate` que como en el caso de Python nos permite setear
la frecuencia de nuestro loop principal

```cpp
int count = 0;
while (ros::ok())
{
```

Por default `roscpp` mira si la senial `SIGINT` (que es la que se lanza cuando
presionamos `Ctrl-c`) se ha emitido o no, lo que causara que `ros::ok()` retorne
un `false` si esto ha sucedido

`ros::ok()` retornara un `false` si:

 - una senial `SIGINT` ha sido emitida(`Ctrl-c`)
 - `ros::shutdown()` ha sido llamado por otra parte de la aplicacion
 - todos los `ros::NodeHandles` han sido destruidos

Una vez que `ros::ok()` sea `false` todos los otras llamadas que suceden en ROS
van a fallar tambien

```cpp
   std_msgs::String msg;

   std::stringstream ss;
   ss << "hello world " << count;
   msg.data = ss.str()
```

Aqui hacemos un _broacast_ del mensaje sobre ROS usando una clase que esta adaptada
para esto, generalmente generada desde un archivo `msg` datatypes mas complicados
son posibles, pero por ahora solo usamos el `String` de la libreria estandar
que tiene un miembro que se llama `data`

```cpp
chatter_pub.publish(msg);
```
Luego enviamos el mesaje a traves del objeto que creamos `ros::Publisher` con
esto ponemos el mensaje en el canal para que cualquiera que pueda recibirlo que
este conectado

```cpp
ROS_INFO("%s", msg.data.c_str());
```
Con esta macro reemplazamos a `cout` y `printf` para imprimir los mensajes en
la consola

```cpp
ros::spinOnce();
```

esta llamada a `spinceOnce()` no es necesaria para este simple programa porque
no estamos recibiendo ningun `callback`. Sin embargo si a este nodo le agregamos
que se subscriba a algun `topic` y no tenemos la llamada a esta funcion no vamos
a obtener nunca las llamadas a los `callback` entonces agregarla no esta de mas
(y quiero creer que no agrega overhead...)

```cpp
loop_rate.sleep();
```
Luego hacemos el `sleep` que necesitamos para cumplir con los requisitos de periodicidad
para publicar que pusimos (10 Hz)
Luego lo que nos queda es escribir un nodo para que reciba el mensaje


## Escribiendo el nodo _subscriber_

### El codigo

```cpp

/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%

```

### Explicacion del codigo

```cpp
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
```

Creamos una funcion que va a ser las veces de `callback` que sera la funcion
que llamamos cuando recibimos un mensaje(o sea la accion que tomamos cuando
recibimos un mensaje)

Esta funcion toma como parametro un puntero constante a un `std_msgs::String`
que es una no se que de la libreria `boost` o sea que tenemos que salir corriendo
porque no debe ser algo bueno

```cpp
ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
```
Creamos un `Subscriber` y nos subscribimos al `topic` `chatter` con un limite de
mensajes de 1000 y con la funcion que va a ser llamada cuando tengamos un mensaje
llamada `chatterCallback`

Luego si hacemos esta vez un `spin` ya que en este node si tenemos una funcion de
`callback`

## Escribiendo un servicio en Cpp

Primero vamos a crear un package nuevo que como dependencias tenga a las librerias
`roscpp` y `rospy`

`catkin_create_pkg add_two_ints roscpp rospy`

luego creamos una carpeta que se tiene que llamar `srv` en donde vamos a poner
el archivo que define a nuestro servicio, ponemos en este caso el archivo que
define el servicio lo llamamos `AddTwoInts.srv`(el nombre es importante porque define
como vamos a llamar despues a los objetos en el codigo):

```text
int64 num1
int64 num2
---
int64 sum
```

Donde podemos ver que los primeros dos parametros son los que espera el service
como entrada y el que esta despues de las lineas es el type y el nombre de salida

Luego creamos el server en un archivo llamado `add_two_ints_server.cpp`

```cpp
#include "ros/ros.h"
#include "add_two_ints/AddTwoInts.h"

/* esta va a ser la funcion de callback que llame el server cuando le hagan una
 * consulta */
bool add(add_two_ints::AddTwoInts::Request& request,
      add_two_ints::AddTwoInts::Response& response) {
   /* hacemos el super calculo que vamos a guardar en la variable response */
   response.sum = request.num1 + request.num2;

   ROS_INFO("request: x=%ld, y=%ld", (long int)request.num1, (long int)request.num2);
   ROS_INFO("sending back reponse: [%ld]",(long int) response.sum);
   return true;
}

int main(int argc, char** argv) {
   ros::init(argc, argv, "add_two_ints_server");
   ros::NodeHandle node_handle;

   ros::ServiceServer service = node_handle.advertiseService("add_two_ints", add);
   ROS_INFO("Ready to add two ints.");

   /* NOTE(elsuizo:2020-08-19): necesitamos esta funcion para que sea llamada el callback!!! */
   ros::spin();

   return 0;
}
```
y el client que va a llamar al service que creamo en el server:

```cpp
#include "ros/ros.h"
/* NOTE(elsuizo:2020-08-19): aca va siempre el nombre del package que creamos
 * o donde esta los archivos .srv y el nombre de los archivos .srv pero con
 * .h como extension o sea que son el resultado de la generacion de codigo
 * que pasa los .src ---> .h */
#include "add_two_ints/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char** argv) {
   ros::init(argc, argv, "add_two_ints_client");

   if (argc != 3) {
      ROS_INFO("usage: add_two_ints_client X Y");
      return 1;
   }

   /* creamos el handle */
   ros::NodeHandle node_handle;
   /* NOTE(elsuizo:2020-08-19): nos estamos subscribiendo al servicio que hicimos
    * en el server por eso tiene que tener el mismo nombre */
   ros::ServiceClient client = node_handle.ServiceClient<add_two_ints::AddTwoInts>("add_two_ints");

   add_two_ints::AddTwoInts service;
   service.request.num1 = atoll(argv[1]);
   service.request.num2 = atoll(argv[2]);

   if (client.call(service)) {
      ROS_INFO("sum: %ld", (long int)service.response.sum);
   } else {
      ROS_ERROR("failed to call service add_two_ints");
      return 1;
   }

   return 0;
}
```
Luego como tenemos la generacion de service y msgs y la generacion de binarios
(cosa que no se recomienda en proyectos grandes, lo que se debe hacer es packages
para los msgs y service aparte de los binarios que definen los *nodes*)

tenemos que modificar el `CMakeLists.txt` para que compile los archivos `cpp`
y genere los binarios que queremos de la siguiente manera:

```cmake
#-------------------------------------------------------------------------
#                        server
#-------------------------------------------------------------------------
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_server src/add_two_ints_server.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_server ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#-------------------------------------------------------------------------
#                        client
#-------------------------------------------------------------------------
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_client src/add_two_ints_client.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_client ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

como vemos lo que hacemos es que coincidan los nombres de los archivos que creamos
y tambien le damos los nombres de los ejecutables que queremos

Y con eso si hacemos un `catkin_make` en el *root* del workspace tendremos disponibles
el server y el client

para utilizarlos primero tenemos que tener el serer corriendo:

`rosrun add_two_ints add_two_ints_server`

y luego para hacer un request al service que esta corriendo hacemos:

`rosrun add_two_ints add_two_ints_client 17 20`


## Grabando datos y luego utilizando esos datos

Podemos grabar datos de una simulacion de ROS en archivos de extension `.bag` para
luego podamos procesarla o utilizarla de nuevo

### Grabando datos

Vamos a tomar el ejemplo de la tortuga(que no se te escape) para grabar los datos
que publican todos los topics

como vimos para correr la simulacion de la tortuga hacemos:

`rosrun turtlesim turtlesim_node`

y para poder moverla con el teclado utilizamos:

`rosrun turtlesim turtle_teleop_key`

Esto comienza dos *nodes* que son el visualizador de la totuga y el que nos permite
manejarla con el teclado

#### Grabando todos los `topic`s que estan publicando datos

Primero como sabemos podemos ver cuales son los `topic`s que estan publicando
datos con el comando:

`rostopic list -v`

que nos da la siguiente lista:

```text
Published topics:
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /rosout [rosgraph_msgs/Log] 4 publishers
 * /turtle1/pose [turtlesim/Pose] 1 publisher
 * /turtle1/color_sensor [turtlesim/Color] 1 publisher
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 publisher

Subscribed topics:
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 subscriber
```

La lista que vemos *Published topics:* tiene solo los *types* de mensajes que
pueden ser potencialmente grabados en un archivo log. El *topic* `/turtle1/cmd_vel [geometry_msgs/Twist] 1 publisher`
es el mensaje que publica `turtle_teleop_key` que es tomado como entrada por
el proceso de la tortuga. Luego los mensajes:

`/turtle1/color_sensor [turtlesim/Color] 1 publisher`

`/turtle1/pose [turtlesim/Pose] 1 publisher`

son publicados por la tortuga.

Podemos grabar los datos publicado. Primero preparamos una carpeta donde vamos
a poner los archivos que guardamos

```text
mkdir bagfiles
cd bagfiles
rosbag record -a
```

Ya que pusimos la opcion `-a` estara grabando todos los *topic*s que estan disponibles
Luego de que hemos grabado obtendremos un archivo `.bag` que sera la data que hemos
grabado

#### Examinando los archivos `bag`

Una vez que hemos grabado los archivos usando `rosbag record` podemos "jugar" con
ellos usando el comando `rosbag info` o `rosbag play`


`rosbag info <your bagfile>`

en nuestro ejemplo de la tortuga vemos lo siguiente:

```text
path:        2020-08-19-16-51-16.bag
version:     2.0
duration:    57.8s
start:       Aug 19 2020 16:51:16.68 (1597866676.68)
end:         Aug 19 2020 16:52:14.44 (1597866734.44)
size:        525.8 KB
messages:    7339
compression: none [1/1 chunks]
types:       geometry_msgs/Twist [9f195f881246fdfa2798d1d3eebca84a]
             rosgraph_msgs/Log   [acffd30cd6b6de30f120938c17c593fb]
             turtlesim/Color     [353891e354491c51aabe32df673fb446]
             turtlesim/Pose      [863b248d5016ca62ea2e895ae5265cf9]
topics:      /rosout                   19 msgs    : rosgraph_msgs/Log   (2 connections)
             /rosout_agg               16 msgs    : rosgraph_msgs/Log
             /turtle1/cmd_vel         107 msgs    : geometry_msgs/Twist
             /turtle1/color_sensor   3598 msgs    : turtlesim/Color
             /turtle1/pose           3599 msgs    : turtlesim/Pose
```

Luego el siguiente paso es reproducir lo que hemos grabado en el sistema que estamos
corriendo(el de la tortuga). Primero tenemos que cerrar el *node* que esta corriendo
`turtle_teleop_key`. Luego desde la carpeta que tenemos el `.bag` hacemos

`rosbag play <bag_file>` y si esta corriendo el *node* de la tortuga vamos a ver
que repite todo lo que hemos grabado!!!


#### Grabando un cojunto mas chico de datos

Cuando tenemos un sistema que es mas complicado no podemos grabar todos lo topics
porque seria mucho esfuerzo, por ello podemos grabar ciertos `topic`s que creamos
que son los mas importantes. Por ejemplo en este caso podemos grabar solo los comandos
de velocidad que vimos de la siguiente manera:

`rosbag record -O subset /turtle1/cmd_vel /turtle1/pose`

y con esto hemo reducido la cantidad de datos al ser almacenados


#### Las limitaciones de `rosbag` `record/play`

Cuando hacemos un `play` de la data que tenemos grabada puede ser que no se reproduzca
de la exacta misma manera en la que la hemos grabados, la razon para esto es que
`turtlesim` y cualquier sistema que sea muy sensible a pequenios cambios en el
sistema de toma de tiempos, por eso debemos ser precabidos cuando lo utilizamos
en estos tipos de sistemas


## Leyendo mensajes desde un archivo `bag`

Primero lo que necesitamos es un archivo `bag`. Podemos producir uno o bajarlo
de algun lado, por ejemplo utilizando `wget`

```bash
wget https://open-source-webviz-ui.s3.amazonaws.com/demo.bag
```
