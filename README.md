# Sirius Firmware
[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)][gpl3]

## Introdução

Código do Sirius, robô desenvolvido no SIRLab que participou da [OBR](http://www.obr.org.br/) 2016. O robô teve os pólos da bateria unidos em um acidente, o que ocasionou em uma pequena explosão que queimou seus circuitos e de alguns componentes, impedindo sua participação na competição.

![Foto Sirius](http://i.imgur.com/nCpIops.jpg)

## Componentes

Lista de componentes utilizados no Sirius:

### [Arduino Due](https://www.arduino.cc/en/Main/ArduinoBoardDue) x1
O Arduino escolhido foi o Due, pela sua grande capacidade de processamento (84 MHz) e número de portas disponíveis.

### [QTR-8RC Reflectance Sensor Array](https://www.pololu.com/product/961) x1
Apesar da barra ter 8 sensores, foram utilizados apenas 6, excluindo os dois de cada extremidade. A barra permaneceu inteira, não removemos os sensores que não foram utilizados.

### Sensor RGB TCS230 x2
Posicionados um em cada lado da linha preta, foram utilizados para detectar a cor verde, permitindo que o robô decida qual a direção certa a seguir.

### [Hobbico CS-60](http://www.servodatabase.com/servo/hobbico/cs-60) x2 (contínuos)
Foram utilizados dois servos motores contínuos para a movimentação do robô. Uma roda foi presa em cada um dos servos, que movimentam toda a esteira. As esteiras foram escolhidas para facilitar a ultrapassagem de lombadas.

### Sensor Ultrassônico HC-SR04 x1
Localizado na parte inferior da frente do robô, foi utilizado para identificar a distância entre ele e as vítimas.

### Sensor Ultrasonico DFRobot URM 37 V3.2 x1
Esse ficou na parte de cima da frente do robô, foi utilizado para medir a distância entre ele e as paredes. Foi posicionado de forma que nem as vítimas nem o triângulo fossem detectados.

### Bateria Lipo 7.4V 2200mAh x1
Fonte de alimentação do Arduino Due e dos dois servos motores utilizados para a movimentação do robô (com regulador de 6V)

### Botões x3
Foram utilizados ao total 3 botões: um em cada ponta do robô em um no meio. O botão do meio foi utilizado para detectar obstáculos e paredes na sala de resgate. Ambos os botões laterais foram utilizados para detectar o triângulo na sala de resgate. O botão do meio foi posicionado de forma que ficasse na frente dos outros dois botões e não encostasse no triângulo (que mede cerca de 6cm de altura).

[gpl3]: http://www.gnu.org/licenses/gpl-3.0/
