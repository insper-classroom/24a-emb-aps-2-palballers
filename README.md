Palworld é um jogo de vídeo game que mistura elementos de Pokémon com mecânicas de sobrevivência e gerenciamento de recursos. O jogo permite que os jogadores capturem criaturas chamadas "Pals" e as usem para batalhar, construir, cultivar e até mesmo em combate armado. Diferentemente dos jogos tradicionais de captura de criaturas, como o famoso Pokémon, Palworld incorpora temas mais maduros, como escravidão e exploração, enquanto também apresenta um mundo aberto para exploração e combate contra inimigos humanos.

Onde comprar: https://store.steampowered.com/app/1623730/Palworld/

Link para vídeo demonstrando o funcionamento: https://www.youtube.com/watch?v=yH-ALgVcZqc

## Descrição do Projeto

Diagrama de blocos do firmware: https://drive.google.com/file/d/1xyWKp9b4O6M-3ikoxrUun7SMQVxtdiHP/view?usp=drive_link

xQueueMPU: Queue que manda informações sobre a detecção de vibração

xQueueHC: Queue que manda informações para o HC

xSemaphore_1: Semáforo para o botão 1

xSemaphore_2: Semáforo para o botão 2 

xSemaphore_3: Semáforo para o botão 3

xSemaphore_4: Semáforo para o botão 4

xSemaphore_5: Semáforo para o botão 5

xSemaphore_6: Semáforo para o botão 6


btn_callback: IRS que controla a ativação dos semáforos de cada um dos 6 botões

mpu6050_task: task que faz a leitura do MPU e envia para shake_detector_task

shake_detector_task: task que checa se houve vibração no MPU

x_task: task do joystick para o eixo x

y_task: task do joystick para o eixo y

btn_task: task que ativa quando botões são apertados

rotate_task: task que trata a rotação do scroll

hc06_task: task que envia as informações pelo bluetooth

hc_status_task: task que checa se o bluetooth está conectado


Para conectar o bluetooth no linux usar os passos descritos no site:

- https://marcqueiroz.wordpress.com/aventuras-com-arduino/configurando-hc-06-bluetooth-module-device-no-ubuntu-12-04/

