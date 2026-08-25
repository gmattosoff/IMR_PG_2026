O estado de um robô representa as informações necessárias para saber sua situação em determinado momento.

Nos exercícios, esse estado é definido principalmente pela posição e orientação do robô.
A POSE 2D é representada por três valores: (x), (y) e (\theta).
Os valores (x) e (y) indicam a posição do robô no plano, enquanto (\theta) representa sua orientação.
Na simulação, essas informações são atualizadas continuamente durante o movimento do robô.
A Cinemática Diferencial descreve como as velocidades das duas rodas influenciam o movimento do robô.
A velocidade linear é calculada pela média das velocidades das rodas direita e esquerda.
Já a velocidade angular depende da diferença entre essas velocidades e da distância entre as rodas.
Assim, quando as duas rodas possuem a mesma velocidade, o robô anda em linha reta.
Quando as velocidades são diferentes, o robô muda sua orientação e realiza uma curva.
Se as rodas girarem em sentidos opostos, o robô pode girar em torno do próprio eixo.

A Odometria Discreta permite atualizar a posição do robô considerando pequenos intervalos de tempo.
A cada intervalo (dt), a posição e a orientação são calculadas usando as velocidades atuais, o que faz com que o movimento contínuo seja aproximado por várias pequenas atualizações sucessivas.
Esse processo foi utilizado no exercício do quadrado, em que o robô executou movimentos programados por tempo.

Na navegação GO-TO-GOAL, o objetivo é fazer o robô alcançar uma posição determinada.
Para isso, calcula-se o ângulo desejado entre a posição atual do robô e o ponto de destino.
O erro angular é obtido pela diferença entre o ângulo desejado e a orientação atual.
No exercício do controlador proporcional, esse erro foi utilizado para calcular a velocidade angular do robô.
Quando a distância até o alvo fica menor que 10 pixels, o robô para automaticamente, concluindo a navegação.
