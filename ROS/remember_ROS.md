# Cosas para recordar de ROS(Robot Operating System)

## ROS workspace environment

el workspace defaul se carga con `source /opt/ros/melodic/setup.bash`
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
 - Podemos analizar la frecuencia de actualizacion de un topic: `rostopic hz topic_name`

## ROS making packages

 - `catking` es el sistema para generar packages
 - Dicen que es mejor hacer `catkin build` en lugar de `catking_make`
 - El workspace de catking contiene los siguientes espacios: `src`, `build` y `devel`
 - `src`: Es el espacio donde se encuentra el codigo fuente. Aca es donde podemos clonar, crear y editar el codigo. Osea que debemos trabajar alli
 - `build`: Es el espacio en donde `cmake` crea los packages. No debemos modificar nada alli
 - `devel`: Es el espacio donde los packages a ser creados son guardados. No debemos modificar nada alli
 - Si por algun motivo queremos limpiar el contenido de las carpetas `build` y `devel` lo podemos hacer con el comando: `catkin clean`

## Obteniendo codigo de terceros

Podemos clonar repos de terceros que estan en formato catkin pkg para luego compilarlo y usarlo, dicen que son buenas practicas hacer
un enlace simbolico desde donde lo hemos descargado hacia nuestro workspace de catkin, nos puede servir cuando tengamos mas de un workspace

`sudo ln -s path/to/packages path/to/workspace`

## ROS launch

El comando `roslaunch` es una herramienta para correr muchos nodes de una sola vez
 - Son escritos en un archivo xml con extension `.launch`
 - Si no esta corriendo, `roslaunch` lanza a roscore automaticamente

