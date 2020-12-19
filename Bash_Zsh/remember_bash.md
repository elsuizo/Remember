# Cositas para recordar de Bash

## Cuando queremos hacer un for loop en una linea

Por ejemplo con la siguiente linea creamos 24 carpetas que tienen en su nombre
el numbero correspondiente de la variable que estamos iterando

`for i in {1..24}; do mkdir lecture$i; done`
