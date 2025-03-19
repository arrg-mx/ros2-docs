# Protocolo RTPS (Real-Time Publish-Subscribe)

En **ROS 2**, la comunicación entre nodos se basa en el protocolo **RTPS (Real-Time Publish-Subscribe)**, el cual es una implementación específica del estándar [**DDS (Data Distribution Service)**](DDS.md). **RTPS** permite la transmisión de datos en **tiempo real**, utilizando un modelo descentralizado donde los nodos pueden intercambiar información sin depender de un servidor central.  

Este protocolo es clave para la comunicación eficiente y escalable en **ROS 2**, permitiendo el descubrimiento automático de nodos y la transmisión confiable de datos en sistemas distribuidos.  

## Estructura de RTPS

**RTPS** define un modelo de comunicación basado en **publicadores y suscriptores**, optimizado para entornos de **tiempo real**. La estructura básica de **RTPS** incluye los siguientes elementos:  

### Entidades en RTPS

- **Dominio DDS** → Define el ámbito de comunicación en el que los nodos pueden interactuar.  
- **Participant (Participante RTPS)** → Representa una aplicación en la red que puede contener múltiples publicadores y suscriptores.  
- **Publisher (Publicador)** → Envia datos a uno o más suscriptores.  
- **Subscriber (Suscriptor)** → Recibe datos de un tópico específico.  
- **DataWriter y DataReader** → Componentes internos del publicador y suscriptor que manejan la serialización y transmisión de datos.  

### Mensajes de RTPS

**RTPS** define distintos tipos de mensajes que regulan la comunicación:  

- **DATA**: Contiene los datos que se transmiten entre publicadores y suscriptores.  
- **HEARTBEAT**: Permite a un publicador notificar a los suscriptores sobre la disponibilidad de nuevos datos.  
- **ACKNACK**: Se usa para confirmar la recepción de mensajes o solicitar reenvíos en caso de pérdida de datos.  
- **GAP**: Indica que ciertos datos no están disponibles o han sido descartados.  

**Ejemplo en ROS 2:**  
Un nodo de sensores envía datos de un LIDAR mediante **DATA**, mientras que el nodo de navegación puede solicitar paquetes perdidos usando **ACKNACK**.  

## RTPS y DDS

El protocolo **RTPS es la base de DDS**, permitiendo la comunicación en sistemas distribuidos sin necesidad de un servidor central.  

### Diferencias clave entre DDS y RTPS

| **Característica** | **DDS** | **RTPS** |
|------------------|--------|--------|
| **Propósito** | Middleware de comunicación distribuida | Protocolo de transporte de datos en tiempo real |
| **Modelo de Comunicación** | Publicador-Suscriptor | Publicador-Suscriptor |
| **Transporte** | Depende de la implementación de DDS | Basado en UDP y compartición de memoria |
| **Descubrimiento** | Basado en dominios y configuraciones DDS | Implementación automática con mensajes RTPS |

**Ejemplo en ROS 2:**  
DDS define cómo los nodos deben comunicarse, mientras que RTPS maneja la transmisión física de los mensajes a través de la red.  

## RTPS Transports (Transporte de Datos en RTPS)

**RTPS** está diseñado para ser compatible con diferentes mecanismos de transmisión de datos, asegurando **baja latencia y confiabilidad** en la comunicación.  

### Métodos de Transporte en RTPS

| **Transporte** | **Descripción** | **Uso en ROS 2** |
|--------------|---------------|------------------|
| **UDP Multicast** | Envío de datos a múltiples suscriptores al mismo tiempo. | Descubrimiento automático y distribución eficiente de datos. |
| **UDP Unicast** | Comunicación directa entre dos nodos específicos. | Envío de datos críticos con baja latencia. |
| **TCP (Opcional en DDS)** | Garantiza la entrega de datos con confirmaciones. | Comunicación en redes distribuidas con alta confiabilidad. |
| **Shared Memory (Memoria Compartida)** | Transmisión directa entre procesos en la misma máquina sin usar la red. | Mejora el rendimiento en sistemas con múltiples nodos locales. |

**Ejemplo en ROS 2:**  
Un robot autónomo con sensores y cámaras puede utilizar **Shared Memory** para procesar datos internamente y **UDP Multicast** para transmitir información a otros robots en la misma red.  

## RTPS Discovery (Descubrimiento de Nodos en RTPS)

RTPS incluye un mecanismo de **descubrimiento automático**, permitiendo que los nodos encuentren y establezcan comunicación sin configuraciones manuales.  

### Tipos de Descubrimiento en RTPS

| **Modo de Descubrimiento** | **Descripción** | **Ejemplo en ROS 2** |
|---------------------------|---------------|------------------|
| **Simple Discovery** | Los nodos envían mensajes de "anuncio" en la red para descubrir otros nodos en el mismo dominio. | Un robot móvil detecta automáticamente nuevos sensores agregados al sistema. |
| **Static Discovery** | Se configuran direcciones IP específicas de los nodos participantes. | Comunicación en redes cerradas o industriales con nodos predefinidos. |
| **Cloud Discovery** | Usa un servidor externo para gestionar la conexión entre nodos distribuidos. | Control de robots desde una nube en una red global. |

**Ejemplo de Descubrimiento Automático en ROS 2:**  
1. Un nodo de navegación inicia en la red.  
1. RTPS envía un mensaje de **"Participant Discovery"** a otros nodos.  
1. Los nodos vecinos responden con su información de conexión.  
1. Se establecen los canales de comunicación entre los nodos.  

