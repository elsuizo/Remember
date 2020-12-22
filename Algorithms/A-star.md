# Algoritmo A*

La primera vez que se utilizo fue en 1968, para el proyecto "Shakey", que era
un robot movil que podia planear sus acciones:

https://en.wikipedia.org/wiki/Shakey_the_robot

Puede verse como una extension del algoritmo de dijstra, nada mas que este
algoritmo tiene mejor performance ya que usa una heuristica para guiar la busqueda

A* es un algoritmo de busqueda y esta formulado en terminos de grafos con peso
comenzando por un node especifico, se dice que es el que encuentra el camino con
el menor costo(menor distancia a la meta, menor tiempo, etc..)
Lo hace manteniendo un arbol de caminos originadso desde el nodo inicial y extendiendo
ese camino con un lado a la vez hasta que su criterio de finalizacion concluye

En cada iteracion de su loop principal A* necesita determinar cual es el path que
va a ser extendido, elije ese path de acuerdo a el costo del path y una estimacion
del costo que requiere extender ese path hasta la meta. Especificamente A* elige
este path que minimiza:

`f(n) = g(n) + h(n)`

Donde `n` es esl proximo nodo sobre el path, `g(n)` es el costo del path sobre
el nodo desde el comienzo del path hasta el nodo `n` y `h(n)` es una funcion
heuristica que estima el mejor path desde `n` hasta la meta. A* termina cuando
el path el path que eligio va desde el nodo inicial hasta la meta o si no hay
paths que se puedan elegir. La funcion de heuristica es especifica para cada
problema. Si la funcion de heuristica es "admisible" o sea que no sobreestima el
costo actual para llegar a la meta, A* es garantizado que retorna el path que tiene
el minimo costo desde el nodo inicial hasta la meta.

Una implementacion tipica de A* usa una queue de prioridad para hacer la seleccion
repetida de nodos de costo minimo(estimado) para expandir. En cada paso de el algoritmo
el nodo con el menor `f(n)` es removido de la queue, el valor de `f` y de `g` de
sus vecinos son actualizados respectivamente y estos vecinos son agregados a la
queue, el algoritmo continua hasta que un nodo removido(o sea el nodo con menor
valor de `f` de todos los nodos) es el nodo de la meta. El valor de `f` en el
nodo de la meta es tambien el costo total del path elegido y como vimos el valor
minimo.

El algoritmo nos da solo cuanto es el largo del camino mas corto. Para encontrar
el camino lo que tenemos que hacer es revisar los nodos ya que cada uno de ellos
tiene en cuenta a sus predecesores y asi podemos reconstruir el camino.

Como un ejemplo, cuando buscamos por la ruta mas corta sobre un map, `h(n)` puede
representar la distancia de la linea recta a la meta, dado que es fisicamente la
distancia mas corta que podemos lograr entre dos puntos. Para una grilla de un
video juego por ejemplo, usando la distancia de Manhattan puede ser mejor que la
anterior.

Si la funcion heuristica ademas satisface la condicion adicional
`h(x) <= d(x, y) + h(y)`
para cada lado `(x, y)` del grafo (donde `d` denota el largo ese lado) entonces
`h` es llamada "monotona" o "consistente". Con una heuristica consistente, A*
es garantizado que encontrara una ruta optima sin procesar ningun nodo mas de una
vez y A* es equivalente a el algoritmo de Dijkstra con el costo reducido de:

`d'(x, y) = d(x, y) + h(y) - h(x)`

En el siguiente pseudocodigo implementa el algoritmo

```text
function reconstruct_path(cameFrom, current)
    total_path := {current}
    while current in cameFrom.Keys:
        current := cameFrom[current]
        total_path.prepend(current)
    return total_path

// A* finds a path from start to goal.
// h is the heuristic function. h(n) estimates the cost to reach goal from node n.
function A_Star(start, goal, h)
    // The set of discovered nodes that may need to be (re-)expanded.
    // Initially, only the start node is known.
    // This is usually implemented as a min-heap or priority queue rather than a hash-set.
    openSet := {start}

    // For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start
    // to n currently known.
    cameFrom := an empty map

    // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore := map with default value of Infinity
    gScore[start] := 0

    // For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    // how short a path from start to finish can be if it goes through n.
    fScore := map with default value of Infinity
    fScore[start] := h(start)

    while openSet is not empty
        // This operation can occur in O(1) time if openSet is a min-heap or a priority queue
        current := the node in openSet having the lowest fScore[] value
        if current = goal
            return reconstruct_path(cameFrom, current)

        openSet.Remove(current)
        for each neighbor of current
            // d(current,neighbor) is the weight of the edge from current to neighbor
            // tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore := gScore[current] + d(current, neighbor)
            if tentative_gScore < gScore[neighbor]
                // This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] := current
                gScore[neighbor] := tentative_gScore
                fScore[neighbor] := gScore[neighbor] + h(neighbor)
                if neighbor not in openSet
                    openSet.add(neighbor)

    // Open set is empty but goal was never reached
    return failure
```

