# Resumen del libro *Programming Robots with ROS*

ROS es un *framework* para programar robots, osea que es una coleccion de
herramientas, librerias y convenciones que su principal proposito es simplificar
el flujo de trabajo y hacer que no reiventemos la rueda cada vez que queremos
comenzar con un proyecto de robotica

## `roscore`

`roscore` es un servicio que provee informacion a los nodos que quieren transmitir
un mensaje. Todos los nodos cuando al inicio de se ejecucion se conectan con `roscore`
para obtener la informacion necesaria para transmitir y recibir mensajes. La arquitectura
de comunicacion de ROS es un hibrido entre lo que conocemos como conexion clasica
de client/server y un sistema totalmente distribuido. Cuando un nodo de ROS comienza
a ejecutarse el espera que haya un variable de entorno llamada `ROS_MASTER_URI`.
Esta variable contiene un string de la forma: `https://hostname:11311/` el cual
en este caso implica que tenemos una instancia de `roscore` accesible en el puerto
11311(este numero de puerto se eligio porque es un numero primo palindromico que
no es usado por casi nadie). Sabiendo que el nodo de `roscore` esta corriendo, los
nodos lo utilizan como punto de encuentro para encontrarse con los mensajes que son
enviados por los otros nodos

## catkin

`catkin` es un conjunto de archivos `cmake` que hacen las veces de package mannager
Es bueno conocer algo de `cmake` para poder dominar mejor a `catkin`. Sin embargo
para comenzar podemos decir que hay dos archivos que son importantes a la hora
de crear codigo, y esos son `CMakeLists.txt` y `package.xml`

## Topics

Como vimos ROS consiste en un numero de nodos independientes que componen un grafo
Estos nodos por si mismo no son utiles. Las cosas se ponen interesantes solo cuando
los nodos se comunican uno con otro intercambiando informacion y datos. La manera
mas comun de realizar esto es con `topics`. Un `topic` es un nombre para un `stream`
de mensajes con un type definidos. Por ejemplo, los datos desde un laser deben ser
enviados sobre un `topic` llamados `scan`, con un type del mensaje `LaserScan`,
mientras que los datos de una camara que debe ser enviados `image` con un type de
mensaje `Image`.
Topics implementan un mecanismo de comunicacion  `publish/subscribe` como una de
las maneras mas comunes de intercambiar datos en un sistema distribuido

## Cap5: Actions

Mientras que los `services` son piolas para tener interacciones simples como consultar
el status y cosas de configuracion, estas no funcionan bien cuando tenemos iniciar
una tarea que sea larga en el tiempo. Por ejemplo imaginemos que queremos hacer
que un robot vaya a un lugar determinado llamando a un node `goto_position()`. No
podemos saber de antemano cuanto tiempo le requerira hacer esta tarea al robot.
Si hicieramos esto con un `service` estariamos bloqueando al robot a que haga
cualquier otro tipo de tarea mientras esperamos un mensaje de vuelta. Por ello
se crearon los `actions` que son la mejor manera de implementar interfaces del estilo
*go to goal*, mientras los `services` son sincronicos los `actions` son asincronicos
y ademas podemos usar retroalimentacion en ellos para proveer un update de los estados
y de el comportamiento. Las `actions` son implementadas usando `topics`, una `action`
es esencialmente un protocolo de alto nivel que especifica como setear un `topic`
(objetivo, resultado, retroalimentacion)
Usando una `action` de interface para implementar nuestra `goto_position()` lo que
hacemos es enviar un objetivo, entonces empezamos a movernos a el y nos movemos
a las otras tareas mientras el robot esta yendo al objetivo. Mientras pasa esto
vamos recibiendo updates periodicos (distancia recorrida, tiempo estimado a la meta, etc...)
terminando en un mensaje de llegada, y algo importante es que podemos abortar la
mision y cambiar el objetivo on-the-fly

## Definiendo una `action`

Lo primero que debemos hacer es definir los mensajes de `goal`, `result` y
`feedback` en el archivo de definicion de un `action` que por convecion tiene la
extension `.action`. Este tipo de archivo es como los archivos `.srv` lo unico que
se agrega es un campo mas y como pasaba en los `services` cada campo se convertia
en un mensaje propio.

Como un ejemplo simple vamos a definir un `action` que se comporte como un timer
Lo que queremos es contar hacia atras una determinada cantidad de tiempo y emitir
una senial cuando este contador expire, ademas nos va a decir periodicamente
cuanto tiempo resta para finalizar. Como en los archivos de `services` tenemos
las tres lineas (---) que hacen las veces de separador entre las partes del archivo
Mientras que las definiciones de los servicios tienen dos partes (request y response)
los archivos `action` tienen tres partes y por eso ponemos dos veces el separador
(---) para que queden separados los campos (objetivo, resultado, retroalimentacion)

Este archivo lo debemos poner en la carpeta `action` dentro de nuestro package(o
sea dentro de la carpeta en donde estamos codeando el package)

Y para que sean generados las clases y definiciones de nuestro archivo de `action`
debemos agregar en el archivo de configuracion de `cmake` `CMakeLists.txt` la linea
`actionlib_msgs` en donde dice:

```cmake
find_package(catkin REQUIRED COMPONENTS actionlib_msgs)
```

Entonces luego usamos la funcion `add_action_files()` para decirle a `catkin` sobre
cuales son los archivos que queremos compilar:

```cmake
add_action_files(DIRECTORY action
                 FILES timer.action)
```

Luego debemos asegurarnos de que los ponemos como dependencia:

```cmake
generate_messages(DEPENDENCIES
                  actionlib_msgs
                  std_msgs)
```

### Implementando un server basico de un `action`

Ahora que hemos definido el objetivo, el resultado y la retroalimentacion para
la `action` timer podemos implementar el codigo que lo use. Asi como los
`topics` y los `services` las `actions` tienen el mismo mecanismo de llamada a
funciones `callback` con el cual esta funcion es llamada cuando recibimos el
mensaje esperado desde otro nodo. La manera mas facil de construir un server
para estas `actions`es usando la clase `SimpleActionServer` de el package
`actionlib`. Comenzamos definiendo la funcion de callback que sera invocada
cuando un nuevo objetivo es enviado por un cliente de la `action`, en este
callback hacemos todo el trabajo que queremos para el timer, entonces
retornamos un resultado cuando haya terminado.

```python
#! /usr/bin/env python

import rospy
import time
import actionlib
from actions.msg import timerAction, timerGoal, timerResult

def do_timer(goal):
    start_time = time.time()
    time.sleep(goal.time_to_wait.to_sec())
    result = timerResult()
    result.time_elapsed = rospy.Duration.from_sec(timer.time() - start_time)
    result.updates_sent = 0
    server.set_succeeded(result)

rospy.init_node('timer_action_server')

server = actionlib.SimpleActionServer('timer', timerAction, do_timer, False)
server.start()
rospy.spin()
```

Lo que hemos usado aqui es la libreria de tiempos de Python y la clase del package
actionlib `SimpleActionServer`, tambien utilizamos los mensajes que son autogenerados
desde nuestro `timer.action`

`from basic.msg import TimerAction, TimerGoal, TimerResult`

luego creamos el type que vamos a utilizar como resultado y ponemos el tiempo
que ha transcurrido en el y como ultimo paso ponemos que todo ha salido ok.

Luego en el codigo global, hacemos lo que hacemos siempre que es inicializar el
server, con el constructor de la clase:

`server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)`

en el cual el primer parametro es el nombre del server, el segundo parametro es
es type del `action` para el cual el server va a esperar mensajes, el tercer
argumento el nombre de la funcion de callback(que en este caso es nuestra funcion
`do_timer()`) que hemos definido arriba y finalmente el cuarto parametro es `False`
para desabilitar el auto-comienzo del server, por ello debemos poner la linea que
le sigue `server.start()` para que comience el server. Y como ultimo lo que hacemos
es un `spin` para que siga funcionando en un bucle infinito

Podemos probar si todo anda bien corriendo el node:

`rosrun nombre_node simple_action_server.py`

Que no va a realizar ninguna salida (por el codigo que le pusimos) pero prodemos
ver si esta corriendo bien verificando los topics que se crearon:

`rostopic list`

```text
/rosout
/rosout_agg
/timer/cancel
/timer/feedback
/timer/goal
/timer/result
/timer/status
```

Como vemos se crearon correctamente los topics de `timer`

Si queremos podemos publicar y subscribirnos directamente a un server que publica
un `action` ya que podemos usar los mensajes autogenerados de los types de `actions`
Esto es ya que los `actions` son un protocolo de alto nivel construidos en base
a los `messages` de ROS, pero para la mayoria de las aplicaciones la libreria
`actionlib` hace el trabajo


### Usando un `action`

La manera mas facil de usar un `action` es via la clase `SimpleActionClient` de el
package `actionlib`, por ejemplo:

```python
#! /usr/bin/env python

import rospy
import actionlib
from actions.msgs import timerAction, timerGoal, timerResult

rospy.init_node('timer_action_client')
client = actionlib.SimpleActionClient('timer', timerAction)
client.wait_for_server()
goal = timerGoal()
goal.timer_to_wait = rospy.Duration.from_sec(7.0)
client.send_goal(goal)
client.wait_for_result()
print('Time elapsed: %f'%(client.get_result().time_elapsed.to_sec()))
```

### Implementando un server `action` mas sofisticado

Como vimos los `actions` son parecidos a los `services` solo con un poco mas de
configuraciones y puesta a punto. Pero todavia no hemos utilizado la parte mas
importante de ellos que es su funcion asincronica que es lo que los hace distintos

Vamos a comenzar en el server haciendo algunos cambios a lo que hicimos hasta
ahora para demostrar como abortar un objetivo, como manejar un solicitud de
preferencia y como proveer retroalimentacion mientras estamos persiguiendo un
objetivo. En el siguiente ejemplo vemos algunas mejoras de lo que teniamos hasta
ahora.

```python
#! /usr/bin/env python

import rospy
import time
import actionlib
from actions.msg import timerAction, timerGoal, timerResult, timerFeedback

def do_timer(goal):
    start_time = time.time()
    update_count = 0

    if goal.time_to_wait.to_sec() > 60.0:
        result = timerResult()
        result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
        result.updates_sent = update_count
        server.set_aborted(result, "Timer aborted due to too long wait...")
        return
    while (time.time() - start_time) < goal.time_to_wait.to_sec():
        if server.is_preempt_requested():
            result = timerResult()
            result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
            result.updates_sent = update_count
            server.set_preempted(result, "Timer preempted")
            return
        feedback = timerFeedback()
        feedback.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
        feedback.time_remaining = goal.time_to_wait - feedback.time_elapsed
        server.publish_feedback(feedback)
        update_count += 1

        time.sleep(1.0)

    result = timerResult()
    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    result.updates_sent = update_count
    server.set_succeeded(result, "Timer completed successfully")

rospy.init_node('timer_action_server')
server = actionlib.SimpleActionServer('timer', timerAction, do_timer, False)
server.start()
ropy.spin()
```

Veamos los cambios que introducimos. Como ahora utilizamos retroalimentacion
entonces importamos ese mensaje `timerFeedback` a la lista de mensajes que importamos
Ademas agregamos una variable que cuenta la cantidad de veces que publicamos una
retroalimentacion: `update_count`. Luego agregamos un chequeo de error, no queremos
usar este timer con periodos largos de espera, por ello chequeamos que el tiempo
de respueta sea menor que 60 segundos y si sucede eso abortamos el objetivo llamando
al metodo `set_aborted()`, la cual envia un mensaje al cliente que el objetivo fue
cancelado y tambien se agrega un mensaje para que sea una salida de la consola
y que sepamos que fue lo que paso, este string es opcional pero no cuesta nada
ponerlo. Luego si pasamos ese chequeo en lugar de hacer un sleep de todo el tiempo
que tenemos que esperar lo hacemos en partes, esto nos permite hacer cosas mientras
esperamos llegar al objetivo, como chequear por preemption y proveer una retroalimentacion
Luego en el loop, primero chequeamos por preemption preguntando con el metodo
`is_preempt_requested()` si es `True` el cliente ha requerido que paremos de perseguir
el objetivo (esto puede suceder si un segundo cliente nos envia un nuevo objetivo)
Si pasa eso y como un caso parecido cuando hicimos el aborto, rellenamos un resultado
y proveemos un string de status. Luego enviamos una retroalimentacion que corresponde
al type que creamos en el archivo `.action`. Lo rellenamos entonces en terminos de
`time_elapsed` y `time_remaining` y entonces publicamos la retroalimentacion con
el metodo `publish_feedback()` que se lo envia al cliente, tambien incrementamos
`update_count` para reflejar el hecho de que hemos enviado otra actualizacion, luego
una vez que hemos dejado el loop con el `while` quiere decir que hemos pasado el
tiempo requerido, entonces es tiempo de notificar al cliente esto que es muy similar
al ejemplo anterior


### Usando el server mas sofisticado

Ya que hemos modificado el server, entonces podemos crear otro cliente que se aprobeche
de todos los cambios que hemos hecho: vamos a procesar la retroalimentacion, preempt
el objetivo y trigerear un abort

```python
#! /usr/bin/env python

import rospy
import time
import actionlib
from actions.msg import timerAction, timerGoal, timerResult, timerFeedback

def feedback_cb(feedback):
    print('[Feedback] Time elapsed: %f'%(feedback.time_elapsed.to_sec()))
    print('[Feedback] Time remaining: %f'%(feedback.time_remaining.to_sec()))

rospy.init_node('timer_action_client')
client = actionlib.SimpleActionClient('timer', timerAction)
client.wait_for_server()
goal = timerGoal()
goal.time_to_wait = rospy.Duration.from_sec(5.0)
# descomentar la siguiente linea para testear si funciona bien la parte del aborto
# goal.time_to_wait = rospy.Duration.from_sec(61.0)
client.send_goal(goal, feedback_cb=feedback_cb)

# descomentar las siguientes lineas si queremos testear la preemption de un objetivo nuevo
# time.sleep(3.0)
# client.cancel_goal()

client.wait_for_result()
print('[Result] State: %d'%(client.get_state()))
print('[Result] Status: %s'%(client.get_goal_status_text()))
print('[Result] Time elapsed: %f'%(client.get_result().time_elapsed.to_sec()))
print('[Result] Updates sent: %d'%(client.get_result().updates_sent))
```

## Cap7 El robot wander

Vamos a ir paso a paso para crear un package minimalista para un robot que haga
las cosas minimas, tambien vamos a testear los codigos que creemos

### Creando un workspace y el package

Primero podemos crear un nuevo workspace para este robot y todos los packages que
hagamos para el, en el directorio que creamos conveniente podemos hacer:

```text
elsuizo@archlinux$ mkdir -p dir/wanderbot_ws/src
elsuizo@archlinux$ cd wanderbot_ws/src
elsuizo@archlinux$ catkin_init_workspace
```

Y con eso hemos creado un nuevo workspace que lo vamos a utilizar solo para cosas
de este robot, luego podemos comenzar creando el primer package en el con el comando

```text
elsuizo@archilux$ cd wanderbot_ws/src
elsuizo@archilux$ catkin_create_pkg wanderbot rospy geometry_msgs sensor_msgs
```

O sea que hemos creado el package wanderbot con las dependencias que le siguen
al nombre del package. Ahora que hemos creado este package vamos a testear que
compila bien creando un nuevo nodo minimo que lo que va a hacer es stremear una
serie de comandos de movimientos 10 veces por segundo alternandolos cada 3 segundos
entre moviendose y parandose
Cuando estamos moviendonos, el programa enviara un comando de ir hacia adelante
a una velocidad de 0.5 metros por segundo y cuando paramos enviara el comando de
ir a la velocidad 0

```python
#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

# publicamos en el topic 'cmd_vel' con un mensaje de type Twist
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('red_light_green_light')

red_light_twist = Twist()
green_light_twist = Twist()
green_light_twist.linear.x = 0.5

driving_forward = False
light_change_time = rospy.Time.now()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if driving_forward:
        cmd_vel_pub.publish(green_light_twist)
    else:
        cmd_vel_pub.publish(red_light_twist)
    if light_change_time > rospy.Time.now():
        driving_forward = not driving_forward
        light_change_time = rospy.Time.now() + rospy.Duration(3)

    rate.sleep()
```

Cuando hacemos que la opcion de `queque_size=1` estamos diciendo a `rospy` que
solo almacene una salida. En el caso de que la frecuencia de transmicion sea mas
alta que la frecuencia de recepcion entonces `rospy` puede simplemente dejar
cualquier mensaje mas alla de la `queue_size`.

El constructor de la clase `Twist` hace que los fields de ella sean por default
cero. Por ello el mensaje es implicitamente una senial de stop.

La componente `x` de la velocidad lineal en un mensaje `Twist` es por convencion
alineado en la direccion en la cual el robot tiene su frente, entonces la linea
de codigo `green_light_twist.linear.x = 0.5` significa: "ir hacia adelante a una
velocidad de 0.5 metros por segundo".

Necesitamos publicar el mensaje de velocidad continuamente ya que la mayoria de
los drivers que se usan para robots moviles cuando pasa un tiempo sin recibir
mensajes entonces pasan a un estado sleep(que casi siempre paran el robot).

Luego en `if light_change_time > rospy.Time.now():` chequeamos el tiempo del sistema
y toggleamos la luz en rojo/verde periodicamente.

Por ultimo necesitamos llamar a `rospy.sleep()` ya que sino lo hacemos vamos a
enviar muchos mensajes y la utilizacion del CPU sera intensa
