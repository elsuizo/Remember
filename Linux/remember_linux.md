Cosas para recordar de Linux
============================

Debian related
--------------

### Agregar la architectura 32 bits a una 64

`sudo dpkg --add-architecture i386` `sudo apt-get update`
`sudo apt-get install build-essential gcc-multilib rpm libstdc++6:i386`
`libgcc1:i386 zlib1g:i386 libncurses5:i386`

### programa para saber la info de hardware

`apt-get install hardinfo`

### Para instalar Telegram

-   Descargar la app de la pagina
-   como root `mv Telegram/ /opt/`
-   enlace simbolico: `ln -s /opt/Telegram/Telegram//usr/bin/telegram`

### Para saber donde mierda esta instalada una libreria

-   `dpkg -L nombre_libreria`

General
-------

### Salir-volver de un proceso suspendido(Ctrl-z)

`fg`

1.  Tmux

    1.  Abrir sesion

        `tmux -2 new -s session_name`

    2.  reanudar sesion

        `tmux -2 attach -t session_name`

    3.  cambiar a otra sesion

        `tmux switch -t session_name`

    4.  ver todas las sesiones que hay

        `tmux list-sessions`

2.  mountar un puto .iso

    `7z x puto.iso`

3.  quemar un iso en un usb

    Tener en cuenta que debemos saber bien cual es el puerto en el que
    esta conectado nuestro usb.
    `sudo dd bs=4M if=input.iso of=/dev/sdx && sync`

4.  encriptar archivo desde la consola

    `gpg -c archivo`

5.  generar un pass piola(con pwgen)

    Por ejemplo un pass de 37 chars `pwgen 37 1`

6.  para saber como se llama internamente un programa(esto es re para
    i3)

    Esto lo usas en i3-wm cuando queres que un programa x se ejecute
    solo en un workspace(por ejemplo yo solo veo los pdf en el workspace
    4 con zathura) `xprop`

7.  convertir una frase a ASCII art

    `figlet text_a_convertir`

8.  saber el ip de una raspberry pi

    `hostname -I`

### Suspender pulseaudio para utilizar jack

`pasuspender -- jackd -rd alsa`

### Cuando tenemos una carpeta o archivo sin permisos para nada(con candado)

`sudo chmod 777 -R nombre_carpeta`

### sincronizar la hora

`timedatectl set-ntp true`

### Montar un iso

primero preparamos una carpeta: `sudo mkdir /mnt/iso`
`sudo mount /path/to/iso /mnt/iso -o loop`

### Cuando queremos saber cuales son las putas fuentes que tenemos instaladas

`fc-list`

### Cuando queremos usar neovim con el sudo y con la configuracion del user

`sudo -E nvim archivo`

### Cuando queremos escribir un comando que es largo en bash o zsh

`Ctrl-x` + `Ctrl-e` y entramos en el editor de texto que tengamos
asignado (obvio nvim)

### Cuando queremos hacer un symlink

Siempre tenemos que poner el path completo de las cosas que estamos
ligando sino vamos a tener un \"dangling\" symlink.
`sudo ln -s /home/elsuizo/Dev/julia-1.4.0/bin/julia /usr/local/bin`

Cuando queremos crear carpetas recursivamente
---------------------------------------------

-   `mkdir -p folder_name/folder_name2/forder_name3`

Saber el tipo de memoria RAM que tenemos
----------------------------------------

`sudo dmidecode --type 17`

Cambiar la imagen de fondo de lightdm
-------------------------------------

Primero debemos poner la/s imagenes que queremos en una carpeta que
tenga acceso el root, por ejemplo: `/usr/share/pixmaps` Luego agregamos
la imagen en el archivo de configuracion:
`/etc/lightdm/lightdm-gtk-greeter.conf`

Archlinux related
-----------------

### update archlinux

`sudo pacman -Syu`

### Donwgrade a package(instalar un package viejo)

-   Primero instalamos downgrade: `sudo pacman -S downgrade`
-   Luego elegimos el package que queremos instalar:

`sudo downgrade nombre_libreria`

### Agregar yaourt a pacman(yaourt es un repositorio comunitario donde los

usuarios agregan recetas para compilar paquetes, utilizan un pseudo
makefile donde se listan las dependencias y los comandos para compilar
el paquete.). Agregamos en el archivo `/etc/pacman.conf`

``` {.commonlisp org-language="emacs-lisp"}
[archlinuxfr]
SigLevel = Never
Server = http://repo.archlinux.fr/$arch
```

Luego hacemos un update de pacman e instalamos con:
`sudo pacman -Sy yaourt`

### instalar utilidades para files

`sudo pacman -S file-roller p7zip zip unzip unrar`

### instalar audio apps

`sudo pacman -S pulseaudio pavucontrol pulseaudio-alsa alsa-utils`

### Desinstalar package con pacman

`sudo pacman -Rns package`

### Solucionar el problema de arduino makefile con avrdude

Primero instalamos avrdude: `sudo pacman -S avrdude` Despues suponiendo
que tenemos a arduino ide en `/.arduino_ide`, borramos el binario que
trae `avrdude`, osea:

`rm home/elsuizo/.arduino_ide/hardware/tools/avr/bin/avrdude`

Luego \"linkeamos\" el avrdude del sistema con el que borramos(para que
utilice el del sistema y no rompa las pelotas)

`ln -s /usr/bin/avrdude /home/elsuizo/.arduino_ide/hardware/tools/avr/bin/avrdude`

### Instalar Telegram

`yaourt -S telegram-desktop-bin`

### Instalar fortran

`sudo pacman -S gcc-fortran`

### Instalar LaTeX

`sudo pacman -S texlive-most`

### Instalar complementos para cmus

`sudo pacman -S --asdeps libmad`

### Instalar Arduino Makefile para programar con editor de texto

`wget -O arduino.tar.xv http://arduino.cc/download.php\?f\=/arduino-1.6.8-linux64.tar.xz`
`mkdir ~/.arduino_ide`
`tar xf arduino.tar.xv -C ~/.arduino_ide --strip-components=1`
`git clone https://github.com/sudar/Arduino-Makefile.git ~/.arduino_mk`

-   Despues hacemos un Makefile que tenga las siguientes lineas:

    ``` {.commonlisp org-language="emacs-lisp"}
    ARDUINO_DIR = /home/elsuizo/.arduino_ide
    ARDMK_DIR = /home/elsuizo/.arduino_mk
    BOARD_TAG = uno
    include $(ARDMK_DIR)/Arduino.mk
    ```

    Donde `ARDUINO_DIR` es el path al arduino IDE `ARDMK_DIR` es el path
    al arduino makefile repo `BOARD_TAG` es el nombre de la placa que
    estamos usando

### Acceder a una red wifi sin NetworkMannager(por consola)

-   Dependencias: `sudo pacman -S iw wpa_supplicant dialog`
-   Primero accedemos al nombre del router al que queremos conectarnos
    con el comando: `ip link`
-   Luego para conectarnos: `sudo wifi-menu nombreDelRouter`

### Solucionar problema alacritty-gnome

`pacman -S fcitx fcitx-configtool`

### Saber la placa de video que tengo

-   lspci \| grep Grap

### instalar iconos piolas

`yaourt numix-icon-theme`

### problema po2man

cuando pasa esto hacemos `source /etc/profile`

### acceder a los scripts de openocd

Como openocd hay que instalarlo desde AUR parece que cambia la carpeta
donde se instala siempre, ahora por ejemplo para correr en la
\"blue-pill\" seria asi:

``` {.bash}
openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg
```

### Para iniciar Bluetooth e instalar una GUI

-   GUI: `sudo pacman -S bluez bluez-utils`

-   Iniciar el deamon: `sudo systemctl start bluetooth`

-   Si queremos que sea permanente o sea que se inicie cada vez que
    booteamos: `sudo systemctl enable bluetooth`

### Iniciar ssh daemon

`sudo systemctl start sshd`

### Reinstalar grub

`grub-install /dev/sda` `grub-mkconfig -o /boot/grub/grub.cfg`

### Para cambiar a otros themes de Gtk podemos instalar lxappearance

`sudo pacman -S lxappearance`

### Para cuando tenemos problemas al hacer un update

Generalmente se trata de que cortamos una instalacion o un update de
golpe y nos dice que no puede leer las databases, lo que se hace es rm
un archivo que hace las veces de testigo de que se esta instalando algo.
`sudo rm /var/lib/pacman/db.lck`

### Elegir el navegador preferido

-   `xdg-settings set default-web-browser firefox.desktop`

### Cuando queremos hacer un update pero ignorar algun package

-   `yay -Syyu --ignore package_name`

### Cuando queremos hacer un downgrade explicito

En algunos casos cuando se hace un update del sistema, se rompen algunas dependencias
de los programas "no encuentro tal libreria3.0.so" por ello podemos ver si tenemos
la version anterior en el cache de pacman que se encuentra:

`/var/cache/pacman/pkg/libreria2.9.so.pkg.tar.xz`

Si encontramos la version que necesitamos lo unico que nos resta es hace _downgrade_
de la misma con `pacman`:

`sudo pacman -U /var/cache/pacman/pkg/libreria2.9.so.pkg.tar.xz`

Pero luego cuando hagamos un update para que no afecte a la libreria que hicimos
esta maniobra lo que tenemos que hacer es ponerla en la lista que esta en
`/etc/pacman.conf`, con la variable `IgnorePkg=`

Nota: Si el break viene de un package que es de AUR lo que tenemos que hacer
es probar primero si compilando de nuevo anda
