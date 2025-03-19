# Introducción a las Comunicaciones de Red

El término **Comunicaciones de Red o "Networking"** se refiere a la interconexión de dispositivos y sistemas para permitir la comunicación y el intercambio de datos entre ellos. En el ámbito de la informática y las telecomunicaciones, el **networking** abarca el diseño, implementación y gestión de redes de comunicación, incluyendo hardware, software y protocolos que facilitan la transferencia de información.  


## Definición

**Comunicaciones de Red o Networking** es el proceso de establecer y gestionar redes de comunicación entre dispositivos, ya sean computadoras, servidores, dispositivos móviles o sistemas embebidos.  

- **Objetivo:** Garantizar la transmisión eficiente y segura de datos entre dispositivos conectados.  
- **Elementos clave:** Protocolos de comunicación, infraestructura de red (routers, switches, servidores), modelos de red y tecnologías de transmisión de datos.  

## Tipos de Redes

Existen diferentes tipos de redes según su tamaño, cobertura y propósito:  

### Según la Cobertura Geográfica

| Tipo de Red | Descripción | Ejemplo |
|------------|------------|---------|
| **PAN (Personal Area Network)** | Red de corto alcance, conecta dispositivos personales. | Bluetooth entre un smartphone y auriculares. |
| **LAN (Local Area Network)** | Red local que conecta dispositivos en un espacio reducido. | Red de computadoras en una oficina. |
| **MAN (Metropolitan Area Network)** | Red que cubre una ciudad o región metropolitana. | Redes de universidades o proveedores de internet. |
| **WAN (Wide Area Network)** | Red de gran extensión que conecta múltiples LANs. | Internet global. |

### Según su Arquitectura

- **Redes Punto a Punto (P2P):** Comunicación directa entre dispositivos sin intermediarios (ej. conexión Bluetooth entre dos teléfonos).  

- **Redes Cliente-Servidor:** Un servidor gestiona y distribuye los recursos a varios clientes (ej. páginas web que se alojan en servidores).  

- **Redes en la Nube:** Los dispositivos acceden a recursos almacenados en servidores remotos (ej. servicios como Google Drive o AWS).  

## Componentes de las Redes

Para que una red funcione, se requieren diversos componentes de hardware y software:  

### Dispositivos de Red
1. **Módem** *(Hardware)*
   - Dispositivo que modula y demodula señales digitales para permitir la comunicación entre una red doméstica o empresarial y el proveedor de internet (ISP). Convierte datos digitales en señales analógicas y viceversa para la transmisión a través de líneas telefónicas, cable coaxial o fibra óptica.  

2. **Router** *(Hardware)*
   - Dispositivo encargado de dirigir los paquetes de datos entre diferentes redes, asegurando que lleguen a su destino de manera eficiente. Permite la conexión de múltiples dispositivos a internet y gestiona la comunicación dentro de una red local (LAN).  

3. **Switch** *(Hardware)* 
   - Dispositivo de red que interconecta múltiples dispositivos dentro de una misma red local (LAN), permitiendo la transferencia de datos de manera eficiente. A diferencia de un hub, un switch envía los datos solo al dispositivo destinatario, optimizando el uso del ancho de banda.  

4. **Firewall** *(Software)*
   - Sistema de seguridad que filtra y controla el tráfico de red según reglas predefinidas, protegiendo los dispositivos contra accesos no autorizados y ciberataques. Puede ser un software (ej. firewall en sistemas operativos) o un hardware dedicado.  

5. **Punto de Acceso Wi-Fi (Access Point, AP)** *(Hardware)*
   - Dispositivo que permite la conexión de dispositivos inalámbricos (Wi-Fi) a una red cableada (LAN), extendiendo la cobertura de la señal de internet en un área determinada.  

6. **Cableado de Red:** *(Hardware)* 
    - Incluye cables Ethernet, fibra óptica y coaxiales.  

## Protocolos de Comunicación

Un **protocolo de red** es un conjunto de reglas y estándares que definen cómo se deben comunicar los dispositivos dentro de una red. Estos protocolos establecen la estructura, el formato y los procedimientos que regulan la transmisión, recepción y gestión de datos en redes locales (LAN) e internet.  

Los protocolos de red garantizan que los dispositivos, sin importar su fabricante o sistema operativo, puedan intercambiar información de manera eficiente y segura.  

**Propósito de los protocolos de red:**  
- Permitir la comunicación entre dispositivos en una red.  
- Estandarizar la transmisión de datos.  
- Garantizar la seguridad y confiabilidad en la comunicación.  

### Protocolos Comunes

Los protocolos de red pueden clasificarse según su función en diferentes niveles del **modelo OSI (Open Systems Interconnection)**. A continuación, se presentan algunos de los protocolos más utilizados:  

**Protocolos de Comunicación y Transporte**

Estos protocolos permiten la transmisión de datos entre dispositivos en una red.  

| **Protocolo** | **Descripción** | **Ejemplo de uso** |
|--------------|---------------|------------------|
| **TCP (Transmission Control Protocol)** | Protocolo confiable que garantiza la entrega de datos en el orden correcto. | Transferencia de archivos, correos electrónicos, navegación web. |
| **UDP (User Datagram Protocol)** | Protocolo rápido pero sin garantía de entrega ni orden de paquetes. | Streaming de video, juegos en línea, VoIP. |
| **ICMP (Internet Control Message Protocol)** | Utilizado para enviar mensajes de error y diagnóstico en la red. | Comando `ping` para comprobar conectividad. |

**Protocolos de Internet y Direccionamiento**  

Definen cómo los dispositivos identifican y localizan a otros dentro de una red.  

| **Protocolo** | **Descripción** | **Ejemplo de uso** |
|--------------|---------------|------------------|
| **IP (Internet Protocol)** | Identifica dispositivos mediante direcciones IP y dirige los paquetes de datos. | Redes locales e internet. |
| **ARP (Address Resolution Protocol)** | Convierte direcciones IP en direcciones MAC dentro de una red local. | Comunicación entre dispositivos en una LAN. |
| **NAT (Network Address Translation)** | Traduce direcciones IP privadas en IP públicas para acceso a internet. | Conexión de múltiples dispositivos a través de un router. |

**Protocolos de Aplicación**

Permiten la interacción entre aplicaciones y servicios en internet o en redes privadas.  

| **Protocolo** | **Descripción** | **Ejemplo de uso** |
|--------------|---------------|------------------|
| **HTTP (Hypertext Transfer Protocol)** | Protocolo estándar para la navegación web. | Acceder a páginas web (`http://`). |
| **HTTPS (HTTP Secure)** | Variante segura de HTTP que cifra los datos con TLS/SSL. | Transacciones bancarias en línea, compras seguras. |
| **FTP (File Transfer Protocol)** | Permite la transferencia de archivos entre computadoras. | Subida y descarga de archivos en servidores. |
| **SMTP (Simple Mail Transfer Protocol)** | Protocolo para el envío de correos electrónicos. | Envío de emails desde clientes de correo. |
| **DNS (Domain Name System)** | Traduce nombres de dominio en direcciones IP. | `www.google.com` → `142.250.190.46`. |

**Protocolos de Seguridad**

Aseguran la privacidad, autenticidad e integridad de los datos transmitidos en una red.  

| **Protocolo** | **Descripción** | **Ejemplo de uso** |
|--------------|---------------|------------------|
| **TLS/SSL (Transport Layer Security / Secure Sockets Layer)** | Cifrado de datos en transmisiones seguras. | Comercio electrónico, banca en línea. |
| **IPSec (Internet Protocol Security)** | Proporciona seguridad en el nivel de red. | VPNs seguras para acceso remoto. |
| **SSH (Secure Shell)** | Acceso remoto seguro a servidores y dispositivos. | Administración de servidores Linux de forma remota. |

##  Conceptos Claves sobre Redes

### Puertos de Red

**Definición:**

Un **puerto de red** es un número lógico asignado a cada servicio o aplicación en un sistema para identificar procesos específicos de comunicación en una red. Los puertos permiten que múltiples aplicaciones usen la misma conexión de red sin interferencias.  

**Propósito:**  
- Facilitar la comunicación entre dispositivos y servicios en una red.  
- Permitir la organización y el control del tráfico de datos.  
- Garantizar la seguridad restringiendo el acceso a servicios no autorizados.  

**Ejemplos de Números de Puertos Conocidos:**

| **Número de Puerto** | **Protocolo/Servicio**   | **Descripción** |
|----------------------|------------------------|----------------|
| **20, 21**         | FTP (File Transfer Protocol)  | Transferencia de archivos. |
| **22**             | SSH (Secure Shell)      | Acceso remoto seguro. |
| **25**             | SMTP (Simple Mail Transfer Protocol) | Envío de correos electrónicos. |
| **53**             | DNS (Domain Name System) | Traducción de nombres de dominio. |
| **80**             | HTTP (Hypertext Transfer Protocol) | Navegación web. |
| **443**            | HTTPS (HTTP Secure) | Navegación web segura. |
| **3389**           | RDP (Remote Desktop Protocol) | Acceso remoto a escritorios Windows. |

Los puertos pueden ser **bien conocidos (0-1023)**, **registrados (1024-49151)** o **dinámicos/privados (49152-65535)**.  

### DHCP (Dynamic Host Configuration Protocol)

**Función:**  

El **DHCP** es un protocolo que asigna automáticamente direcciones **IP**, máscaras de subred, puertas de enlace y servidores DNS a dispositivos en una red.  

**Cómo Funciona:**  

- **Solicitud:** Un dispositivo se conecta a la red y envía un mensaje de "Descubrimiento DHCP" (DHCP Discover).  
- **Oferta:** El servidor DHCP responde con una oferta de una dirección IP disponible (DHCP Offer).  
- **Solicitud de IP:** El dispositivo acepta la oferta enviando un "DHCP Request".  
- **Confirmación:** El servidor DHCP confirma la asignación con un "DHCP Acknowledge".  

**Propósito:**

- Automatizar la configuración de red, evitando asignaciones manuales de IP.  
- Reducir conflictos de direcciones IP dentro de una red.  
- Facilitar la administración de redes empresariales y domésticas.  
- **Ejemplo de uso:** Un router doméstico usa DHCP para asignar automáticamente direcciones IP a teléfonos, computadoras y otros dispositivos conectados.  

### DNS (Domain Name System)

**¿Qué es?**

El **DNS** es un sistema que traduce nombres de dominio (ej. `www.google.com`) en direcciones IP (`142.250.190.46`), permitiendo que los dispositivos se comuniquen sin necesidad de recordar direcciones numéricas.  

**Cómo Funciona:**

- **Consulta:** Un usuario ingresa un dominio en el navegador.  
- **Búsqueda en Caché:** Se verifica si la IP ya está guardada en la memoria local.  
- **Consulta al Servidor DNS:** Si no está en caché, la consulta se envía a un servidor DNS.  
- **Respuesta del Servidor:** El servidor devuelve la IP correspondiente al dominio.  
- **Acceso al Sitio Web:** Con la IP obtenida, el navegador carga la página web.  

**Propósito:**

- Facilitar la navegación en internet usando nombres fáciles de recordar.  
- Optimizar la gestión de nombres de dominio en la web.  
- Permitir la distribución eficiente del tráfico en internet.  
- **Ejemplo de uso:** Cuando escribes `www.youtube.com`, el DNS convierte ese nombre en una dirección IP para que puedas acceder al sitio web.  

### Tipos de Transmisión en Redes

1. **Unicast**
    - **Definición:** Comunicación de uno a uno, donde un solo emisor envía datos a un único receptor.  
    - **Ejemplo:** Un usuario descarga un archivo de un servidor FTP.  
    - **Uso:** Navegación web, correos electrónicos, videollamadas individuales.  
1. **Multicast**
    - **Definición:** Comunicación de uno a muchos, donde un solo emisor transmite datos a múltiples receptores que están suscritos a un grupo específico.  
    - **Ejemplo:** Transmisión de video en vivo para usuarios que se han unido a un canal de streaming.  
    - **Uso:** IPTV, juegos en línea, conferencias en vivo.  
1. **Broadcast**  
    - **Definición:** Comunicación de uno a todos, donde un emisor envía datos a todos los dispositivos dentro de una red.  
    - **Ejemplo:** Un router enviando una solicitud DHCP a todos los dispositivos en una LAN.  
    - **Uso:** Mensajes de red, protocolos como ARP (Address Resolution Protocol).  

## Conceptos sobre Direccionamiento IP

### IP Address (Dirección IP)

- **Definición:** Es un identificador único asignado a cada dispositivo conectado a una red para permitir su comunicación dentro de Internet o redes privadas. Funciona como una "dirección digital" que permite enviar y recibir datos entre dispositivos.  
- **Tipos de direcciones IP:** IPv4 (formato `192.168.1.1`) e IPv6 (formato `2001:db8::ff00:42:8329`).  
- **Ejemplo de uso:** Cuando abres una página web, tu computadora usa su dirección IP para enviar una solicitud al servidor, que también tiene su propia IP.  

### Direcciones IP Estáticas vs. Dinámicas

1. IP Estática:  
    - Es una dirección IP fija que no cambia con el tiempo.  
    - Se usa en servidores, cámaras de seguridad y dispositivos que requieren accesibilidad constante desde la red.  
    - Ejemplo: Un servidor web con IP `203.0.113.45` siempre mantiene la misma dirección.  

1. IP Dinámica:
    - Es una dirección IP asignada temporalmente por un proveedor de internet (ISP) y puede cambiar en cada conexión.  
    - Se usa en la mayoría de los dispositivos domésticos para optimizar el uso de direcciones IP.  
    - Ejemplo: Un router doméstico recibe una IP diferente cada vez que se reinicia su conexión a internet.  

### IP Pública vs. IP Privada

1. IP Pública:
   - Es una dirección IP visible en internet, asignada por un **ISP** (*Internet Service Provider*) a un dispositivo o router.  
   - Permite la comunicación directa entre dispositivos de diferentes redes a través de internet.  
   - Ejemplo: Un servidor web con la IP `54.172.98.30` es accesible desde cualquier parte del mundo.  

1. IP Privada:
   - Es una dirección IP utilizada dentro de una red local (LAN), no accesible desde internet.  
   - Permite la comunicación interna entre dispositivos sin usar una IP pública.  
   - Ejemplo: Un router doméstico asigna direcciones IP privadas como `192.168.1.10` a computadoras y teléfonos dentro de una casa.  
    
    **Rangos de IP Privadas (Reservadas por la IANA):**  
    - `10.0.0.0 - 10.255.255.255`  
    - `172.16.0.0 - 172.31.255.255`  
    - `192.168.0.0 - 192.168.255.255`  

### Subnetting (Subredes)

**Definición:** 

Es la técnica de dividir una red grande en redes más pequeñas (subredes) para mejorar la organización y eficiencia del tráfico de datos.  

Se define mediante una **máscara de subred** (`Subnet Mask`), que determina cuántos bits de la dirección IP pertenecen a la red y cuántos a los dispositivos.  

**Ejemplo de máscara de subred:**  
- `255.255.255.0` → Permite 254 direcciones dentro de la subred.  
- `255.255.0.0` → Permite más dispositivos en la misma red.  

**Ejemplo de uso:** Una empresa usa subnetting para separar departamentos:  
- `192.168.1.0/24` para administración.  
- `192.168.2.0/24` para producción.  
- `192.168.3.0/24` para investigación.  

**Ventajas del Subnetting:**  
- Mejora la seguridad y segmentación de la red.  
- Reduce la congestión del tráfico.  
- Facilita la administración de direcciones IP.  

### Proxy

**Definición:**

Un **proxy** es un servidor que actúa como intermediario entre un dispositivo y el internet, filtrando solicitudes y mejorando la seguridad o el rendimiento de la red.  

**Tipos de Proxy:**
- **Proxy Directo:** Se usa para acelerar la navegación al almacenar en caché sitios web visitados con frecuencia.  
- **Proxy Inverso:** Protege servidores web al ocultar su IP y gestionar el tráfico entrante.  
- **Proxy Anónimo:** Oculta la IP real del usuario para mejorar la privacidad en internet.  

**Ejemplo de uso:**  
- En una empresa, los empleados navegan por internet a través de un **proxy** que bloquea sitios no autorizados.  
- Un usuario usa un **proxy anónimo** para acceder a contenido restringido en su país.  

## Importancia de las redes de comunicación

- **Facilita la comunicación global:** Permite acceso a internet, correo electrónico, redes sociales y servicios en la nube.  
- **Optimiza el intercambio de información:** Empresas y organizaciones dependen de redes eficientes para operar.  
- **Permite automatización y conectividad en IoT:** Dispositivos inteligentes se comunican entre sí en tiempo real.  

- **Ejemplo de aplicación:** En la robótica, **ROS 2 usa networking** para conectar múltiples robots y sensores en una misma red, permitiendo que trabajen de manera coordinada.  