"Este material fue desarrollado con el apoyo del PAPIME PE110923 de la UNAM."

# Implementaciones RMW

## ¿Qué es RMW?  

En **ROS 2**, el middleware que gestiona la comunicación entre nodos se conoce como **RMW (ROS Middleware Interface)**. Esta capa intermedia permite que ROS 2 sea compatible con diferentes implementaciones de [**DDS (Data Distribution Service)**](DDS.md), facilitando la flexibilidad y adaptabilidad a distintos entornos y necesidades.  

La **interfaz RMW** actúa como un puente entre la API de **ROS 2** y el middleware subyacente, permitiendo a los desarrolladores elegir la implementación de DDS que mejor se adapte a su aplicación sin modificar el código de sus nodos **ROS 2**.  

## Proveedore e Implementaciones

**ROS 2** admite múltiples implementaciones de **DDS**, proporcionadas por distintos **vendors** (proveedores). Cada implementación tiene sus propias características y optimizaciones, permitiendo a los desarrolladores seleccionar la más adecuada según los requisitos de su sistema.  

### Lista de Implementaciones de RMW en ROS 2

| **Vendor**       | **Implementación DDS** | **Características Principales** |
|------------------|-----------------------|---------------------------------|
| **eProsima**     | **Fast DDS**           | Open-source, optimizado para robótica, buena latencia. |
| **ADLINK**       | **Cyclone DDS**        | Baja latencia, uso eficiente de CPU, ideal para redes locales. |
| **RTI**          | **Connext DDS**        | Versión comercial con alto rendimiento y seguridad avanzada. |
| **PrismTech**    | **OpenSplice DDS**     | Orientado a sistemas industriales y embebidos. |
| **GurumNetworks**| **GurumDDS**           | Enfocado en rendimiento y estabilidad en sistemas críticos. |

Cada implementación tiene ventajas dependiendo del contexto en el que se utilice.  

## Descripción de las Implementaciones RMW

### Fast DDS (eProsima)

- Licencia: Open-source (Apache 2.0).
- Optimizado para sistemas robóticos.  
- Configuración avanzada de QoS y seguridad.
- Buena latencia y eficiencia en redes distribuidas.
- Soportado por defecto en ROS 2 desde Foxy.

**Caso de uso:** Robots autónomos que requieren **baja latencia** y **gran flexibilidad** en la configuración de la comunicación.  

### Cyclone DDS (ADLINK)

- Optimizado para sistemas en tiempo real.
- Baja latencia y uso eficiente de recursos.
- Buena compatibilidad con sistemas embebidos y redes locales.
- Alto rendimiento en comunicación multicast.

**Caso de uso:** Aplicaciones en **sistemas industriales** y redes cerradas donde la latencia mínima es crítica.  

### Connext DDS (RTI)

- Implementación comercial con soporte técnico.
- Alta seguridad y confiabilidad en la comunicación.
- Optimizado para sistemas críticos como automoción y aviación.
- Mayor consumo de recursos en comparación con otras opciones.

**Caso de uso:** Aplicaciones **aeronáuticas, médicas y de automoción**, donde la confiabilidad es más importante que la latencia.  

### OpenSplice DDS (PrismTech)

- Diseñado para entornos industriales y embebidos.
- Compatible con sistemas de misión crítica.
- Configuraciones avanzadas de memoria compartida y redes heterogéneas.  

**Caso de uso:** **Sistemas de automatización industrial** y **procesos de manufactura** donde se requiere alta disponibilidad y tolerancia a fallos.  

### GurumDDS (GurumNetworks)

- Orientado a sistemas críticos que requieren estabilidad y alto rendimiento.
- Poco utilizado en ROS 2 en comparación con otras implementaciones.

**Caso de uso:** Aplicaciones que demandan **alta estabilidad y seguridad**, como sistemas militares o de telecomunicaciones.  

## Selección de una Implementación RMW en ROS 2

Para definir qué implementación de DDS se utilizará en ROS 2, se puede establecer la variable de entorno `RMW_IMPLEMENTATION`:  

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp  # Usar Fast DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # Usar Cyclone DDS
```

También se puede verificar qué implementación está en uso con el siguiente comando:  

```bash
ros2 doctor --report
```
