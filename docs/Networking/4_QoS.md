# Quality of Service (QoS) (Calidad de Servicio)

## ¿Qué es QoS en ROS 2?

En **ROS 2**, **Quality of Service (QoS)** se refiere a un conjunto de configuraciones que permiten ajustar la manera en que los mensajes se transmiten entre nodos. **QoS** es una característica de **DDS (Data Distribution Service)**, el middleware que ROS 2 utiliza para la comunicación.  

**Objetivo:** Adaptar el comportamiento de la comunicación según las necesidades de cada aplicación.  

**Beneficios de QoS en ROS 2:**  
- Optimización de **rendimiento y latencia** en la transmisión de mensajes.  
- Control sobre la **confiabilidad y persistencia** de los datos.  
- Mejora en la **eficiencia de la red**, evitando congestión.  
- Soporte para sistemas **distribuidos y tolerantes a fallos**.  
- **Ejemplo de uso:**  
    - Un **sensor LIDAR** requiere **baja latencia** para enviar datos en tiempo real, incluso si se pierden algunos paquetes.  
    - Un **registro de eventos** necesita **garantizar la entrega de cada mensaje**, almacenándolo si un nodo suscriptor está desconectado.  

Para lograr estos comportamientos, **QoS define políticas específicas** llamadas **QoS Policies**, y perfiles predefinidos llamados **QoS Profiles**.  

## QoS Policies en ROS 2

Las **QoS Policies** son configuraciones individuales que controlan diferentes aspectos del comportamiento de la comunicación.  

| **Policy (Política)** | **Settings (Configuraciones Disponibles)** |
|----------------------|--------------------------------------------|
| **Reliability** (Fiabilidad) | `BEST_EFFORT`: Envía los datos sin garantizar su entrega.<br> `RELIABLE`: Asegura que los datos lleguen al destino. |
| **Durability** (Persistencia) | `VOLATILE`: Los mensajes se eliminan si no hay suscriptores.<br> `TRANSIENT_LOCAL`: Almacena mensajes para nuevos suscriptores. |
| **History** (Historial de Mensajes) | `KEEP_LAST(N)`: Solo almacena los últimos `N` mensajes.<br> `KEEP_ALL`: Guarda todos los mensajes disponibles. |
| **Deadline** (Tiempo Máximo de Entrega) | Define un tiempo límite para la entrega de cada mensaje. |
| **Liveliness** (Supervisión de la Conexión) | `AUTOMATIC`: DDS gestiona la verificación del nodo.<br> `MANUAL_BY_TOPIC`: El nodo debe notificar que sigue activo. |
| **Lifespan** (Tiempo de Vida del Mensaje) | Define cuánto tiempo un mensaje puede permanecer disponible antes de ser descartado. |

**Ejemplo de configuración en Python:**  
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```
En este ejemplo, los mensajes son **fiables**, se almacenan **los últimos 10** y si un mensaje no se recibe, se vuelve a enviar.  

## QoS Profiles

Los **QoS Profiles** son configuraciones predefinidas que combinan múltiples **QoS Policies** para distintos tipos de aplicaciones.  

### Default Profile (`rmw_qos_profile_default`)

Este es el perfil por defecto en ROS 2. Se usa en aplicaciones generales donde no se requiere una configuración especializada.  

| **Parámetro**     | **Valor** |
|------------------|----------|
| **Reliability**  | `RELIABLE` |
| **Durability**   | `VOLATILE` |
| **History**      | `KEEP_LAST (10)` |
| **Lifespan**     | No definido |
| **Deadline**     | No definido |
| **Liveliness**   | `AUTOMATIC` |

**Ejemplo de uso:**  
Un sistema de monitoreo donde la entrega confiable de mensajes es importante, pero no es necesario almacenar datos antiguos.  

### Sensor Profile (`rmw_qos_profile_sensor_data`)

Este perfil está optimizado para la transmisión de datos de sensores en tiempo real, donde es preferible reducir la latencia en lugar de garantizar la entrega de todos los mensajes.  

| **Parámetro**     | **Valor** |
|------------------|----------|
| **Reliability**  | `BEST_EFFORT` |
| **Durability**   | `VOLATILE` |
| **History**      | `KEEP_LAST (5)` |
| **Lifespan**     | No definido |
| **Deadline**     | No definido |
| **Liveliness**   | `AUTOMATIC` |

**Ejemplo de uso:**  
Un **sensor de cámara o LIDAR** que transmite datos constantemente. No es necesario garantizar que cada imagen o escaneo llegue, pero sí que la latencia sea mínima.  
