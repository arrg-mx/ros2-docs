"Este material fue desarrollado con el apoyo del PAPIME PE110923 de la UNAM."


# Notación CIDR

La **notación CIDR** (*Classless Inter-Domain Routing*, o "Enrutamiento Inter-Dominio Sin Clases") es un método para representar y asignar direcciones IP de manera más flexible que el esquema tradicional basado en clases de red (Clase A, B y C).  

CIDR se introdujo para optimizar el uso del espacio de direcciones IP y mejorar la eficiencia del enrutamiento en redes grandes y pequeñas.  

## Estructura de la Notación CIDR

Una dirección en formato CIDR se escribe como:

```
IP / Número de bits de la máscara de red
```

Ejemplo:
```
192.168.1.0/24
```

**Explicación:**

- **192.168.1.0** → Dirección base de la subred.  
- **/24** → Indica que los **primeros 24 bits** de la dirección son la **parte de la red**, y los **8 bits restantes** son para los **hosts**.

Una máscara de subred puede aplicarse a cualquier dirección IP, ya sea pública o privada. El valor `/X` no determina si una IP es pública o privada, simplemente indica cuántos bits están reservados para la identificación de la red.

Los dispositivos electrónicos procesan las direcciones IP como una secuencia de 32 bits, compuesta solo por 1's y 0's. Estos dispositivos no interpretan las direcciones en formato decimal; dicho formato se usa únicamente para facilitar su comprensión por parte de los humanos.

La notación decimal de una dirección IPv4 es una representación abreviada de su equivalente en binario. Cada uno de los cuatro números decimales corresponde a un conjunto de 8 bits, llamados "octetos". La relación entre los valores decimales y su representación en binario se muestra a continuación:

```text
1                 1                 1                 1               
2 6 3 1           2 6 3 1           2 6 3 1           2 6 3 1         
8 4 2 6 8 4 2 1 . 8 4 2 6 8 4 2 1 . 8 4 2 6 8 4 2 1 . 8 4 2 6 8 4 2 1    

--------------- . --------------- . --------------- . ---------------

1 1 0 0 0 0 0 0   1 0 1 0 1 0 0 0   0 0 0 0 0 0 0 0   0 0 0 0 0 1 0 1

--------------- . --------------- . --------------- . ---------------

192             . 168             . 0               . 5
```

### Máscaras de subred y notación CIDR

Las máscaras de subred están formadas por una secuencia continua de 1's seguidos de una secuencia de 0's. Los bits en 1 identifican la red, mientras que los bits en 0 identifican los dispositivos dentro de esa red (hosts).  

Ejemplo de una máscara de subred y su correspondencia en binario:

```text
1                 1                 1                 1               
2 6 3 1           2 6 3 1           2 6 3 1           2 6 3 1         
8 4 2 6 8 4 2 1 . 8 4 2 6 8 4 2 1 . 8 4 2 6 8 4 2 1 . 8 4 2 6 8 4 2 1 

--------------- . --------------- . --------------- . ---------------

1 1 1 1 1 1 1 1 . 1 1 1 1 1 1 1 1 . 1 1 1 1 1 1 1 1 . 0 0 0 0 0 0 0 0

--------------- . --------------- . --------------- . ---------------

255             . 255             . 255             . 0

                      1 1 1 1 1 1   1 1 1 1 2 2 2 2   2 2 2 2 2 2 3 3
0 1 2 3 4 5 6 7   8 9 0 1 2 3 4 5   6 7 8 9 0 1 2 3   4 5 6 7 8 9 0 1

--------------- . --------------- . --------------- . ---------------

- - - - - - - -   - - - - - - - -   - - - - - - - >   *
```

La notación CIDR permite expresar de manera compacta el número de bits que conforman la máscara de subred. Por ejemplo:

- La dirección `192.168.0.5/24` equivale a `192.168.0.5` con la máscara `255.255.255.0`.  
- El sufijo `/24` indica que los primeros 24 bits están reservados para la red, dejando los 8 bits restantes para los hosts.  

Este método simplifica la configuración y gestión de redes, evitando el uso de las representaciones completas de las máscaras de subred.  

### Conversión de la Notación CIDR a Máscara de Subred

El número después de la barra (`/`) representa cuántos bits están reservados para la **red**. La cantidad restante de bits determina el número de **hosts** en la subred.

| **CIDR** | **Máscara de Subred** | **Hosts Disponibles** |
|---------|------------------|------------------|
| `/8`   | 255.0.0.0  | 16,777,214 hosts |
| `/16`  | 255.255.0.0 | 65,534 hosts |
| `/24`  | 255.255.255.0 | 254 hosts |
| `/30`  | 255.255.255.252 | 2 hosts |

**Ejemplo de conversión**

Una red **192.168.1.0/24** equivale a la máscara **255.255.255.0**, lo que significa:
- **Los primeros 24 bits son para la red** (192.168.1.0).
- **Los últimos 8 bits son para los hosts** (de `192.168.1.1` a `192.168.1.254`).

## Ventajas de la Notación CIDR

- **Optimización del espacio IP**: Evita asignaciones de direcciones innecesarias.
- **Mayor flexibilidad**: Se pueden crear subredes más pequeñas o más grandes según la necesidad.
- **Reducción del tamaño de las tablas de enrutamiento**: Agrupa múltiples redes en una sola entrada de enrutamiento (*route aggregation*).

## Ejemplo Práctico en Redes

Si una empresa tiene la dirección **10.0.0.0/8**, puede dividirla en subredes más pequeñas, como:  

- **10.0.1.0/24** → Para un departamento.  
- **10.0.2.0/24** → Para otro departamento.  
- **10.0.3.0/26** → Para un grupo más reducido de dispositivos.  

En una red doméstica típica con **192.168.1.0/24**, el router asigna IPs desde **192.168.1.1** hasta **192.168.1.254**, dejando **192.168.1.255** como dirección de **broadcast**.  
