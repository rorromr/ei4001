Para instalar rosserial: 

1) Nos situamos, desde un terminal, dentro de nuestro directorio de trabajo de ROS donde vayamos a descargar el paquete. En caso catkin_workspace/src.
2) Mover carpeta rosserial a esa direccion.
3) Compilamos el paquete ejecutando: catkin_make install.
4) Recompilamos la fuentes en ROS: source install/setup.bash
5) Finalmente compilamos la librerías de rosserial: rosrun rosserial_arduino make_libraries.py  

Puede que en el paso 5 pida agregar la dirección de donde se guardan las librerías de arduino (skechtbook)

Luego:

1) rospack find rosserial_arduino    (Para buscar dirección de package)
2) roscd rosserial_arduino/src
3) cp -r ros_lin /dirección de librería arduino (sketchbook)
