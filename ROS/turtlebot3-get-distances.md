# Calculando las distancias a los objetos en el `world` con el turtlebot3

## Preliminares

Primero debemos correr la simulacion del `turtlebot3` con el mundo en el que
queremos que aparezca, como vimos en el otro tutorial: `instalando_turtlebot3.md`
debemos correr los siguientes comandos:

1. lanzamos el `core` de ROS: `roscore`
2. corremos la simulacion de `gazebo` con: `roslaunch turtlebot3_gazebo turtlebot3_world.launch`

Segundo lo que necesitamos es saber cual es el `topic` que publica la pose del
robot, para eso nos podemos ayudar como vimos con el comando `rostopic list`
lo que nos muestra una lista como esta aproximadamente:

```text
/clock
/cmd_vel
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/imu
/joint_states
/odom
/rosout
/rosout_agg
/scan
/tf
```

Entonces el topic que necesitamos puede ser que sea `/odom`, para comprobar que
es la data que necesitamos podemos hacer un `echo` de ese `topic`:

`rostopic echo /odom`

Donde vemos que son los datos que necesitamos para saber la posicion del robot
en el mundo que lo lanzamos

Luego necesitamos saber cual es el type que tiene ese mensaje para poder interpretarlo
en el codigo de `cpp`, por eso hacemos:

`rostopic info odom`

El cual nos dice que `Type: nav_msgs/Odometry`

Luego en el codigo hacemos todo el preambulo que vimos en el tutorial, nos subscribimos
al `topic` `odom`:

```cpp
ros::Subscriber sub = n.subscribe("odom", 100, odom_callback);
```
la cual va a llamar a la funcion `callback`: `odom_callback`

```cpp
void odom_callback(const nav_msgs::Odometry::ConstPtr& odometry_msg) {
}
```

que como vimos tenemos que hacer que entienda el `msg` que le llega desde el `topic`
por eso el type que tiene es: `nav_msgs::Odometry::ConstPtr&` que como regla general
como vimos en el tutorial se encuentran en el header `nav_msgs/Odometry.h`(que tenemos
que incluirlo!!!)

Luego para saber cuales son los fields de ese `msg` podemos hacer que nos lo muestre
con el comando: `rosmsg show nav_msgs/Odometry` que nos da como salida:

```text
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
```
podemos crear un package desde cero ayudado con el comando: `catkin_create_pkg`

en este caso lo llamamos `location-monitor` y le ponemos las dependencias que son
basicas `std_msg` y `roscpp` en definitiva queda:

`catkin_create_pkg location-monitor std_msgs roscp`

creamos una carpeta `src` y comenzamos a editar el primer node de este package
que va a ser el que tenga un callback para reacciona cuando lee de un topic

`nvim src/odom_listener.cpp`

que nos queda mas o menos asi:

```cpp
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg) {
   double x = msg->pose.pose.position.x;
   double y = msg->pose.pose.position.y;
   ROS_INFO("x: %f, y: %f", x, y);
}

int main(int argc, char **argv)
{
   /* creo que este nombre que le ponemos es el mismo con el que llamamos al package */
   ros::init(argc, argv, "location_monitor");
   /* creamos un handle para el node(con el que nos permite conectarnos a un topic) */
   ros::NodeHandle node_handle;
   /* nos subscribimos al topic que queremos, en este caso odom que va a llamar a la funcion chatterCallback */
   ros::Subscriber sub = node_handle.subscribe("odom", 10, chatterCallback);
   /* necesitamos esta funcion si o si para que el callback sea llamado!!! */
   ros::spin();
   return 0;
}
```

Luego debemos modificar el `CMakeLists.txt` para que tenga en cuenta las dependencias
que le necesitamos y para que esten bien los nombres de los archivos

Primero en donde dice el nombre tenemos que poner el que va en el projecto:

`project(location-monitor)`

Luego en `find_packages` ponemos las dependencias que puede encontrar en el mismo
ROS

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
)
```

Luego en `catkin_packages` ponemos las mismas dependencias que pusimos antes:

```cmake
catkin_package(
 CATKIN_DEPENDS roscpp std_msgs nav_msgs
)
```

en `include_directories` ponemos lo que esta por default

```cmake
include_directories(
  ${catkin_INCLUDE_DIRS}
)
```

Luego vienen los nombres de los archivos y los ejecutables que primero ponemos
en `add_executable`

Osea que el primer argumento es como se va a llamar el ejecutable y el segundo
ponemos donde estan las fuentes para construirlo

```cmake
add_executable(${PROJECT_NAME}_node src/odom_listener.cpp)
```

Luego vienen los nombres de las dependencias:

```cmake
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

Y por ultimo el linking:

```cmake
# lo que hacemos aca es linkear las librerias
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
```

Luego debemos compilar desde el workspace de catkin con `catkin_make` y si todo
va bien podemos correr el node con el comando:

`rosrun location-monitor location-monitor_node`
