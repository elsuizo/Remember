# Resumen de las cosas mas importantes de los `services` en ROS

 - Se lo conoce como RPC(Remote Procedure Call)

Lo que significa que nos permite llamar a una funcion que esta implementada en
un `server`, permitiendo que diferentes `clients` llamen a esta funcion.

Supongamos que tenemos un Robot que tiene un `Node` llamado `robot_state` que
esta corriendo en el Robot(los estados del robot pueden ser: velocidad de las
ruedas, velocidad lineal, etc...)

Luego podemos tener en una PC un sistema que lee la telemetria del Robot
implementado que no esta conectado fisicamente con el Robot(o sea que no puede
conocer los estados...) Pero el Robot puede exponer sus estados con mediante un
`service` llamado `GetTelemetry` que le dice al `client` que le pregunta como
son sus estados por ejemplo. El `client` que llama a este `service` tiene que
cumplir con los parametros que necesita esta funcion.

```text

 +-----------------+   GetTelemetry request          +---------------+
 |  Node:          | ------------------------------> | Node:         |
 |                 |                                 |               |
 | /telemetry      | <-----------------------------  | /robot_state  |
 +-----------------+   Response: states              +---------------+
 ```

 - Un `node` puede implementar uno o mas `service`s al mismo tiempo
 - Cualquier `node` puede llamar a un `service`
 - Algo importante es que las llamadas son sincronicas/bloqueantes
   - Por ello si queremos realizar tareas que tienen mucha latencia(por ejemplo
   una tarea de planificacion de ruta) deberiamos usar otras soluciones como por
   ejemplo `actions` de ROS

## Implementando `services`

Debemos definir un `.srv` que define como es la `request` y como es la `response`
Por ejemplo

```text
string name

---

float64 distance
```

Existen muchos comandos en ROS que nos hacen la vida mas facil:

 - `rosservice list`: lista de los servicios disponibles
 - `rosservice info /some_service`: informacion de un servicio
 - `rosservice call /some_service "param1:0.0"`: llamar a un servicio
 - `rossrv show my_msgs/ServiceName`: muestra al servicio como esta representado

## Cuando utilizar un `service`

 - En muchos casos hacer una simple funcion es mejor!!!
 - Deben ser utilizadas para tareas que tengan poco procesamiento(baja latencia)
  - En lugar deberiamos utilizar `actions`
 - Nos sirve mucho cuando queremos interfacear entre lenguajes de programacion
   por ejemplo tenemos todo hecho en cpp y queremos usar algo de python que hizo
   otra persona, podemos entonces exponer los resultados con un `service`

## Ejemplo practico

Vamos a crear un package nuevo para implementar un `service` ya que de esta manera
queda aislado y podemos reutilizarlo


