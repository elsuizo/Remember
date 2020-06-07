# Cosas para recordar de ROS(Robot Operating System)

## ROS workspace environment

El workspace defaul se carga con `source /opt/ros/melodic/setup.bash`
en el proceso de instalacion creamos nuestro propio workspace, para verificar
los workspaces que tenemos miramos la variable `ROS_PACKAGE_PATH` haciendo `echo`
`echo $ROS_PACKAGE_PATH`

## ROS master

 - Maneja la comunicacion entre nodos
 - Cualquier nodo se tiene que registrar con el master para poder funcionar
 - Se lo inicializa con el siguiente comando: `roscore`

## ROS nodes

 - Programas de un solo proposito
 - Compilados, ejecutados y manejados individualmente
 - Organizados en paquetes
 - Para "correr" un node: `rosrun package_name node_name`
 - Para ver los nodes activos: `rosnode list`
 - Ver informacion de algun node: `rosnode info node_name`

## ROS Topics

 - Los nodes se comunican a traves de topics
  - Los nodes pueden publicar o suscribirse a un topic
 - Topic es como un alias para un canal de comunicacion de mensajes
 - Para ver los topics activos: `rostopic list`
 - Para suscribirse e imprimir el contenido de un topic: `rostopic echo topic`
 - Para mostrar informacion de un topic: `rostopic info topic`

## ROS Messages

 - Una estructura de datos que define el type de un topic
 - Puede ser un conjunto de estructuras de datos agrupados en una sola
 - Estan definidos en un archivo de extension `.msg`
 - Para ver el type de un topic: `rostopic type topic`
 - Para publicar un message a un topic: `rostopic pub topic type args`
 - Podemos verificar el type de un topic: `rostopic type topic_name`
 - Podemos ver el mensaje de un topic: `rostopic echo topic_name`
 - Podemos analizar la frecuencia de actualizacion de un topic:
   `rostopic hz topic_name`

## ROS making packages

 - `catking` es el sistema para manejar packages
 - Dicen que es mejor hacer `catkin build` en lugar de `catking_make`
 - El workspace de catking contiene los siguientes espacios: `src`, `build` y
   `devel`
 - `src`: Es el espacio donde se encuentra el codigo fuente. Aca es donde
   podemos clonar, crear y editar el codigo. Osea que debemos trabajar alli
 - `build`: Es el espacio en donde `cmake` crea los packages. No debemos
   modificar nada alli
 - `devel`: Es el espacio donde los packages a ser creados son guardados. No
   debemos modificar nada alli
 - Si por algun motivo queremos limpiar el contenido de las carpetas `build` y
   `devel` lo podemos hacer con el comando: `catkin clean`

## Obteniendo codigo de terceros

Podemos clonar repos de terceros que estan en formato catkin pkg para luego
compilarlo y usarlo, dicen que son buenas practicas hacer un enlace simbolico
desde donde lo hemos descargado hacia nuestro workspace de catkin, nos puede
servir cuando tengamos mas de un workspace

`sudo ln -s path/to/packages path/to/workspace`

## ROS launch

El comando `roslaunch` es una herramienta para correr muchos nodes de una sola vez
 - Son escritos en un archivo xml con extension `.launch`
 - Si no esta corriendo, `roslaunch` lanza a roscore automaticamente

## Estructura de un archivo `.launch`

Como ejemplo tomamos el archivo que lanzamos anteriormente que coreesponde al ejemplo: `roscpp_tutorials`
que lo podemos encontrar con el comando: `roscd roscpp_tutorials`

``` xml
<launch>
  <node name="listener" pkg="roscpp_tutorials" type="listener" output="screen"/>
  <node name="talker" pkg="roscpp_tutorials" type="talker" output="screen"/>
</launch>
```

 - `launch`: Es el elemento *root* de un archivo `.launch`
 - `node`: cada tag `<node>` especifica un *node* que va a ser lanzado
 - `name`: El nombre del *node* (es opcional)
 - `pkg`: El *pkg* de donde proviene el *node*
 - `type`: El *type* del *node* que debe corresponderse con el nombre del ejecutable
 - `output`: Especifica donde guardar el *log*

## `args` *tags* AKA argumentos

Podemos reusar archivos *.launch* ya que podemos hacer que acepten parametros, por ejemplo en el siguiente archivo:

```
<?xml version="1.0"?>
<launch>

  <!-- these are the arguments you can pass this launch file, for example paused:=true -->
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="extra_gazebo_args" default=""/>
  <arg name="gui" default="true"/>
  <arg name="debug" default="false"/>
  <arg name="physics" default="ode"/>
  <arg name="verbose" default="true"/>
  <arg name="output" default="screen"/>
  <arg name="world" default="gazebo_ros_range"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find gazebo_plugins)/test/test_worlds/$(arg world).world"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="extra_gazebo_args" value="$(arg extra_gazebo_args)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="debug" value="$(arg debug)"/>
    <arg name="physics" value="$(arg physics)"/>
    <arg name="verbose" value="$(arg verbose)"/>
    <arg name="output" value="$(arg output)"/>
  </include>

</launch>
```

 - Creamos reusables archivos *.launch* con el *tag* `<arg>`, que funciona como
   un parametro
 - Usamos estos parametro en una llamada a un archivo *.launch* con:
   `$(arg arg_name)`
 - Cuando lanzamos un *.launch* los argumentos pueden pasarse:

   `roslaunch launch_file_name.launch arg_name:=valor`

 - Podemos incluir un *.launch* dentro de un *.launch* con el *tag* `<include>`
   para organizar mejor projectos largos

   `<include file="package_name/>"`


## Simulador Gazebo

 - Simula objetos 3D y su dinamica
 - Simula una amplia variedad de sensores
 - Incluye una base de datos de muchos robots comerciales y de ambientes conocidos
 - Provee una interfaz directa con ROS
 - Se puede extender con *plugins*
 - Para correr *gazebo*: `rosrun gazebo_ros gazebo`
