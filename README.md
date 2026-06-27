# 08_RoboticaI_Final
Repositorio con el Trabajo Integrador Final de la cátedra "Robótica I" de la Facultad de Ingeniería de la Universidad Nacional de Cuyo.

## Descripción del Proyecto
Este proyecto implementa la simulación cinemática (directa e inversa), cálculo de trayectorias y análisis de singularidades (usando el Jacobiano) de un manipulador robótico de 6 grados de libertad. El caso de estudio principal consiste en realizar una **trayectoria de soldadura** sobre una pieza cilíndrica, evaluando tanto la cinemática articular como la cartesiana para distintas configuraciones (soldadura interna vs. externa).

## Dependencias
- **MATLAB**
- **Robotics Toolbox for MATLAB** (Peter Corke)

## Estructura del Repositorio

El código fuente principal se encuentra en la carpeta `Code/`. Los scripts están divididos de la siguiente manera:

### 1. Scripts Principales (Simulación de Tarea)
- **`main.m`**: Script principal interactivo. Ejecuta el menú de opciones que permite elegir la trayectoria a simular (Interna o Externa) y configurar qué gráficos y animaciones guardar.
- **`TrayectoriaExterior.m`**: Ejecuta la simulación de la soldadura por la cara **externa** de la pieza. Incluye maniobras de *homing*, recorrido de soldadura con movimiento oscilatorio tipo *weaving*, análisis cinemático (posición, velocidad, aceleración) y la generación de la animación 3D.
- **`TrayectoriaInterior.m`**: Ejecuta la simulación de la soldadura por la cara **interna** de la pieza, manteniendo una estructura similar a la externa pero adaptada a la geometría interior del cilindro.

### 2. Modelado y Cinemática
- **`S01_my_robot.m`**: Define los parámetros de Denavit-Hartenberg (DH) del robot de 6 GDL y construye el objeto `SerialLink` del Robotics Toolbox.
- **`CinematicaDirecta.m`**: Función para evaluar la cinemática directa (posición y orientación del actuador final a partir de los ángulos articulares).
- **`CinematicaInversa.m`**: Función que implementa un solver para obtener los valores articulares `[q1...q6]` a partir de trayectorias cartesianas (`CPosition`), configurado para partir de una "semilla" y converger a una configuración específica (Front - Elbow Up).
- **`S02_WorkSpace.m`**: Script para visualizar y analizar los límites del espacio de trabajo del manipulador.

### 3. Análisis Jacobiano y Singularidades
- **`S04_AnaliticJacobian.m` / `S05_GeometricJacobian.m` / `S06_GeometricJacobian.m`**: Scripts para el estudio analítico y geométrico de la matriz Jacobiana del robot.
- **`S07_GraphDetJ11.m`**: Analiza el comportamiento del determinante del Jacobiano en el espacio articular para encontrar, identificar y graficar las curvas de singularidades.

### 4. Archivos Auxiliares y Pruebas
- **`Test.m` / `S08_...` a `S10_...`**: Archivos de desarrollo preliminares, pruebas paramétricas y bocetos geométricos que precedieron a los scripts definitivos de trayectorias.
- **`toggleSignal.m`**: Función auxiliar de *callback* que permite ocultar o mostrar gráficas interactivamente haciendo click en las etiquetas de la leyenda de MATLAB.

## Cómo Ejecutar la Simulación

1. Asegúrate de tener instalado el **Robotics Toolbox**.
2. Abre MATLAB y navega a la carpeta `/Code` como tu directorio de trabajo actual.
3. Abre y ejecuta **`main.m`**.
4. Responde las preguntas en la *Command Window*:
   - `[IN/OUT]`: Elige qué cordón de soldadura simular.
   - `Guardar graficos cartesianos: [y/n]`
   - `Guardar graficos articulares: [y/n]`
   - `Guardar grafico de Jacobiano (Maniobrabilidad): [y/n]`
   - `Grabar simulacion: [y/n]`
5. La animación y los cálculos de derivadas se abrirán en ventanas individuales. Si el programa se pausa entre gráficos, presiona `[ENTER]` en la consola de MATLAB para continuar.
