# Resumen de Cmake

Cmake es un sistema para generar Makefiles que se organizan como un conjunto de
targets de alto nivel cada target corresponde a un ejecutable o una librería o es
un target personalizado que contiene comandos propios. Las dependencias entre
targets son expresadas en el sistema de generación para determinar el orden de
compilación y las reglas de regeneración.


## Targets binarios

Ejecutables y librerías son definidas usando los comandos: `add_executable()` y
`add_library()`. Los archivos resultantes tendrán sufijos apropiados, prefijos apropiados
y extensión para la plataforma que estamos compilando. Las dependencias entre
targets binarios son expresados usando el comando: `target_link_libraries()`
