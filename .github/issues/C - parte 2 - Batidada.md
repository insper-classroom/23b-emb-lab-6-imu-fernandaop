> Issue:
>
> - Você só deve fechar as issues que foram 100% concluidas.
> - Se não concluiu 100% a issues, você pode fazer um comentário que ireimos analisar!

**Detectando batida**

- Crie uma tarefa `task_house_down` que possui um semáforo, e que quando liberado a task pisca o led da placa
- Utilizando os dados do acelerômetro, libere o semáforo quando uma batida for detectada. 

Dica: 

- Você pode calcular o módulo $\srqt{x,y,z}$ de todos os eixos e verificar a condição, só saiba que o acelerômetro mede a gravidade, então vocês sempre vão possuir um resultado próximo de 9.0 quando a IMU estiver parada.

![](https://raw.githubusercontent.com/Insper/ComputacaoEmbarcada/master/docs-src/navigation/Labs/Lab_RTOS_IMU/diagram-2.svg)
