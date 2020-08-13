# Los pasos a seguir para instalar el turtlebot

## Clonando los repos

Primero vamos a la carpeta donde tenemos nuestro workspace de catking: `cd catkin-workspace`
(yo me hice un alias en `zshrc` que se llama `CAT` para esto)

desde ahi vamos a la carpeta que tenemos todos los packages que se debe llamar: `src`

`cd src`

Luego clonamos los repos que necesitamos:

`git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git`

`git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git`

`git clone https://github.com/ROBOTIS-GIT/turtlebot3.git`

luego vamos de nuevo a la carpeta del workspace `cd ..` y compilamos los packages:

`catkin_make`

## Corriendo los ejemplos clasicos

1. primero tenemos que activar el nodo `core` con el comando `roscore`

2. luego corremos la simulacion de gazabo con el ambiente que queremos

   `roslaunch turtlebot3_gazebo turtlebot3_world.launch`

   Tambien tenemos el modelo de una casa donde podemos lanzar la simulacion:

   `roslaunch turtlebot3_gazebo turtlebot3_house.launch`

3. luego para manejar el robot con el teclado lanzamos el nodo para teleoperarlo:

   `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`

## Donde seguir

Para mas ejemplos podemos visitar la pagina del proyecto:

`https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#ros-1-simulation`
