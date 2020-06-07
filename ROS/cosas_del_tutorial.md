# Cosas para recordar del tutorial oficial

 - Existen dos packages mannager(deben haber mas pero los mas usados son estos)
   - `catking`
   - `rosbuild`

## Trabajando con Packages

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

 - `topic`: Nodos a los cuales nos podemos subscribir para recibir mensajes:
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
en un `topic`, mientras que `turtlesim` se subscribe a el mismo `topic` para
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

