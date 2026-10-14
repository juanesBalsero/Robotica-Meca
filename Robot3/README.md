# Robot SCARA 

## 1. Descripción del proyecto

Este proyecto consiste en el diseño y desarrollo de un robot tipo **SCARA (Selective Compliance Assembly Robot Arm)** compuesto por cuatro eslabones, diseñado para realizar una tarea de manipulación y transporte de objetos.

La función principal del robot es **recoger una caja ubicada en un punto A y trasladarla hasta un punto B**, simulando una aplicación de automatización industrial para el transporte de piezas entre estaciones de trabajo.

El sistema permite estudiar el movimiento del manipulador, la relación entre sus eslabones y la coordinación necesaria para ejecutar una trayectoria entre dos posiciones determinadas.

## 2. Objetivo general

Desarrollar un robot SCARA de cuatro eslabones capaz de transportar una caja desde una posición inicial A hasta una posición final B, aplicando conceptos de robótica, cinemática y automatización.

## 3. Objetivos específicos

- Diseñar la estructura del robot y definir las dimensiones de sus eslabones.
- Estudiar la cinemática del manipulador para determinar la posición de su extremo.
- Establecer los movimientos necesarios para desplazarse entre los puntos A y B.
- Implementar un mecanismo de agarre para sujetar y liberar la caja.
- Verificar que el robot pueda realizar la tarea de transporte dentro de su espacio de trabajo.

## 4. Estructura del robot

El manipulador está compuesto por cuatro eslabones que forman parte de su estructura mecánica y su sistema de movimiento.

| Componente | Función |
|---|---|
| Eslabón 1 | Conecta la base con el primer segmento móvil. |
| Eslabón 2 | Permite extender el brazo y alcanzar diferentes posiciones. |
| Eslabón 3 | Forma parte del mecanismo de posicionamiento del extremo del robot. |
| Eslabón 4 | Corresponde al segmento final o mecanismo terminal destinado a manipular la caja. |

**Nota:** La distribución exacta de los grados de libertad depende del diseño implementado. En una configuración SCARA convencional, los movimientos principales se realizan mediante articulaciones rotacionales en el plano horizontal y un movimiento vertical para recoger y depositar objetos.

## 5. Funcionamiento

El ciclo de trabajo del robot se divide en las siguientes etapas:

1. **Posición inicial:** el robot permanece en su posición de reposo.
2. **Desplazamiento al punto A:** el brazo se mueve hasta la ubicación de la caja.
3. **Agarre:** el mecanismo terminal sujeta la caja.
4. **Transporte:** el robot desplaza la caja desde el punto A hasta el punto B.
5. **Liberación:** la caja se deposita en la posición de destino.
6. **Retorno:** el robot regresa a una posición inicial o queda preparado para repetir el ciclo.

## 6. Trayectoria de trabajo

La trayectoria principal conecta dos posiciones dentro del espacio de trabajo del manipulador:

- **Punto A:** posición inicial donde se encuentra la caja.
- **Punto B:** posición final donde debe depositarse la caja.

El robot debe coordinar sus articulaciones para alcanzar ambos puntos, evitando movimientos que excedan sus límites mecánicos.

## 7. Aplicaciones

Este tipo de robot puede utilizarse como referencia para estudiar aplicaciones de:

- Transporte automatizado de objetos.
- Manipulación de piezas en líneas de producción.
- Clasificación y transferencia de productos.
- Operaciones de ensamblaje.
- Sistemas de *pick and place*.

## 8. Contenido del repositorio

El repositorio está destinado a organizar los archivos relacionados con el desarrollo del proyecto, incluyendo:

- `Documentation/`: documentación técnica, informes, planos y material de referencia.
- `README.md`: descripción general, objetivos y funcionamiento del proyecto.
- Archivos de diseño, simulación y programación que se incorporen durante el desarrollo.

## 9. Resultados esperados

Se espera obtener un manipulador capaz de ejecutar el ciclo de transporte de una caja desde el punto A hasta el punto B, demostrando la coordinación de sus eslabones y el funcionamiento del mecanismo de agarre.

## 10. Autor

**Equipo 3 de trabajo**

Proyecto de robótica.
