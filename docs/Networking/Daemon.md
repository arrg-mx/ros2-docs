# ROS2 Daemon


**daemon** es un proceso que se ejecuta en segundo plano desde el momento en que se ejecuta el primer comando de la CLI (Command Line Interface) de ROS 2. Su función es **descubrir continuamente los nodos en la red y almacenarlos**.  

## Características

- Descubrir todos los nodos en la red es un **proceso activo**.  
- Cuando se ejecutan por primera vez herramientas de línea de comandos, como `ros2 topic list`, es necesario descubrir todos los nodos. Dependiendo del número de nodos en la red, este proceso puede tardar varios segundos y puede requerir múltiples llamadas para visualizar todos los temas (*topics*).  
- Las llamadas posteriores a los comandos de ROS 2 se ejecutan rápidamente, ya que hacen referencia a la información almacenada por el daemon.  
- Si no se detiene, el daemon siempre está en ejecución en segundo plano, descubriendo constantemente nodos y generando tráfico de descubrimiento. Cuantos más nodos haya en la red, **mayor será la cantidad de datos transmitidos hacia y desde el daemon**.  
- Solo existe **un único daemon**, que es compartido por todas las terminales y herramientas de línea de comandos.  
- El daemon se crea con el entorno de la terminal que ejecutó el primer comando de ROS 2.  
- Para que los cambios en el entorno se reflejen en el daemon, este **debe reiniciarse**.