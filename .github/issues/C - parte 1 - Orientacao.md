> Issue:
>
> - Você só deve fechar as issues que foram 100% concluidas.
> - Se não concluiu 100% a issues, você pode fazer um comentário que ireimos analisar!

Vamos detectar para onde o sistema está apontando, a ideia é acender os LEDs da placa OLED da seguinte maneira:

**A referência da orientação é a posição de quando a placa liga**

Usando os LEDs da placa oled, o sistema deve:

- LED1: Apontando para esquerda
- LED2: Apontando para frente
- LED3: Apontando para direita

Para isso:

1. Crie um `enum` chamado `orientacao` com os seguintes itens: `ESQUERDA, FRENTE, DIREITA`
printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, 
1. Crie uma task (`task_orientacao`) que possui uma fila e que recebe os dados  `euler.angle.yaw`. 
   - Lembre de enivar os dados que foram calculados na `task_imu`.
1. Usando o dado calcule a `orientacao` acende o LED conforme descrição anterior.
1. Na task da IMU, detecta a orientação e envie o dado para a fila.

==A referência da orientação é a posição de quando a placa liga!==

![](https://raw.githubusercontent.com/Insper/ComputacaoEmbarcada/master/docs-src/navigation/Labs/Lab_RTOS_IMU/diagram-1.svg)
