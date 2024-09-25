# LineFollowerRobotPID_Arduino

Robot Sigue Linea con controlador PID

Demostracion: https://youtu.be/5oXEWmr-w1s

Funcionamiento:
  - Espera 2 segundos
  - Inicializa con una calibracion, girando 360 para determinar valores maximos, minimos y punto medio
  - Espera 3 segundos, y comienza el loop
  - En base al error de los sensores externos establece los valores del PID 
  - En base a los valores del PID establece cuanto gira cada motor

Tener en cuenta:
  - Los sensores miden que tan negro o que tan blanco es la superficie (no son binarios). 
  - Cuanto mas blanco mayor el error
