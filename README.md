# Desarrollo de un Robot Móvil Autónomo para Entornos Logísticos en Interiores

![LogoTec](https://javier.rodriguez.org.mx/itesm/2014/tecnologico-de-monterrey-blue.png)

## Colaboradores

- [Jennifer Lizeth Avendaño Sánchez](https://github.com/jennyavsaa) (@jennyavsaa)
- [Juan Antonio Mancera Velasco.](https://github.com/Juan-117) (@Juan-117)
- [Juan Francisco García Rodríguez](https://github.com/JFranciscoGR03) (@JFranciscoGR03)

## Descripción del sistema

El sistema fue desarrollado sobre la plataforma robótica **Robocov**, un robot móvil diferencial con estructura de aluminio tipo perfil Bosch. Su forma general corresponde a un prisma rectangular de aproximadamente $0.84\,\text{m} \times 0.61\,\text{m} \times 0.25\,\text{m}$, con caras de acrílico negro que protegen los módulos internos. El acceso a componentes se realiza retirando únicamente la cara superior, lo que facilita el mantenimiento.

La locomoción se basa en dos motores *brushless* tipo hub montados directamente sobre ruedas traseras, mientras que el eje delantero está estabilizado con ruedas locas. Esta configuración diferencial resulta adecuada para entornos planos y estructurados, como pasillos logísticos. Sin embargo, la distribución de masa dominada por la batería de 48V ubicada al centro del chasis introdujo desafíos en frenado y estabilidad durante maniobras a alta velocidad.

Para cumplir con los requerimientos del caso de uso, se incorporó una caja superior para transportar paquetes con masa variable. Esta estructura se fijó a la cara superior del robot, manteniendo la estabilidad del centro de gravedad. Adicionalmente, se diseñó e imprimió un soporte en 3D para montar de forma alineada el sensor LiDAR **RPLIDAR S3** y la cámara **Logitech Brio 100** sobre el eje longitudinal del robot, a una altura de $0.62\,\text{m}$ desde el suelo, evitando interferencias con la carga.

A nivel superficial, se integró una caja de control que aloja la **ESP32**, circuitos auxiliares y una pantalla digital que permite visualizar el voltaje en tiempo real. Esta funcionalidad resultó útil para monitorear el nivel de carga de la batería de 48V y verificar la correcta entrega de energía al controlador **VESC**. Además, el sistema incluye un botón físico de paro de emergencia montado en la parte posterior, el cual desconecta exclusivamente la alimentación de los motores, preservando el estado de la **Jetson** y los sensores.

