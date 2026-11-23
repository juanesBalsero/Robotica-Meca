# Entrega de trayectoria para una futura adaptación al brazo real

## Advertencia

La trayectoria incluida fue creada y validada en Gazebo. **No debe enviarse
directamente a motores reales.** Una coincidencia de nombres articulares no
garantiza que coincidan ceros, sentidos, escalas, transmisiones o límites.

## Archivos

```text
trayectorias/trayectoria_simulacion_17_3s.csv
    Trayectoria vigente: 1641 muestras y 17.3 segundos.

referencia_ros2/brazo_estesi.urdf.xacro
    Modelo y límites utilizados en la simulación.

referencia_ros2/controllers.yaml
    Configuración del controlador simulado.

referencia_ros2/ejecutar_trayectoria_csv.py
    Ejemplo de validación y envío en Gazebo. No es un driver de hardware.

DATOS_TECNICOS.yaml
    Resumen legible por personas y herramientas.

PROMPT_PARA_IA.md
    Contexto e instrucciones para continuar con otra IA.
```

## Datos que todavía deben obtenerse del robot físico

- Marca/modelo de motores y drivers.
- Tipo, resolución y conexión de cada encoder.
- Relación de transmisión de M1, M2 y M3.
- Cero mecánico, offset y sentido positivo de cada eje.
- Conversión de cuentas a radianes para M1/M2 y a metros para M3.
- Límites reales de posición, velocidad, aceleración, corriente y esfuerzo.
- Finales de carrera, homing y parada de emergencia.
- Computador o microcontrolador y protocolo de comunicación.
- Frecuencia real de control y latencia admisible.
- Masa de la carga, geometría del TCP y funcionamiento real de la pinza.

## Secuencia mínima segura de integración

1. Tener una parada de emergencia física funcional.
2. Leer encoders con los motores deshabilitados.
3. Implementar y probar homing.
4. Confirmar signo y escala de cada eje con desplazamientos pequeños.
5. Probar cada articulación por separado y sin carga.
6. Limitar inicialmente velocidad y recorrido al 10 % o menos.
7. Validar el TCP y las estaciones con mediciones reales.
8. Recién entonces convertir y probar la trayectoria completa.

El campo `pinza_cerrada` del CSV no mueve la pinza actual; solo marca los
intervalos en que debería cerrar o abrir cuando exista un actuador real.
