import pygame
import math
import numpy as np

# ============================================================
# CONFIGURAÇÕES
# ============================================================

LARGURA_TELA = 800
ALTURA_TELA = 600
FPS = 60

ESCALA = 80.0  # pixels por metro

COR_FUNDO = (30, 30, 30)
COR_ROBO = (0, 180, 255)
COR_DIRECAO = (255, 50, 50)
COR_TRAJETORIA = (100, 200, 100)
COR_ALVO = (255, 215, 0)
COR_OBSTACULO = (180, 80, 80)
COR_LIDAR = (0, 255, 0)

# ============================================================
# PARÂMETROS DA FSM
# ============================================================

ESTADO_IR_PARA_ALVO = "IR_PARA_ALVO"
ESTADO_DESVIAR = "DESVIAR_OBSTACULO"
ESTADO_ALCANCADO = "OBJETIVO_ALCANÇADO"

DISTANCIA_OBSTACULO = 0.5
DISTANCIA_ALVO = 0.2

# ============================================================
# PARÂMETROS DO CONTROLADOR
# ============================================================

KP_ANGULO = 1.5

VELOCIDADE_ALVO = 0.5

MAX_OMEGA = 1.5

OMEGA_DESVIO = 1.0

TOLERANCIA_ANGULAR = math.radians(2.0)


# ============================================================
# ROBÔ DIFERENCIAL
# ============================================================

class DiffDriveRobot:

    def __init__(
        self,
        x,
        y,
        theta=0.0,
        wheelbase=0.3
    ):

        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)

        self.L = float(wheelbase)

        self.v = 0.0
        self.omega = 0.0

        self.v_e = 0.0
        self.v_d = 0.0

        self.history = []

    # --------------------------------------------------------
    # Define comando
    # --------------------------------------------------------

    def set_direct_velocity(self, v, omega):

        self.v = float(v)
        self.omega = float(omega)

    # --------------------------------------------------------
    # Atualiza o robô
    # --------------------------------------------------------

    def update(self, dt):

        # Velocidades das rodas
        self.v_e = (
            self.v
            - (self.omega * self.L / 2.0)
        )

        self.v_d = (
            self.v
            + (self.omega * self.L / 2.0)
        )

        # Cinemática diferencial
        v = (
            self.v_e
            + self.v_d
        ) / 2.0

        omega = (
            self.v_d
            - self.v_e
        ) / self.L

        # Atualiza orientação
        self.theta += omega * dt

        self.theta = normalizar_angulo(
            self.theta
        )

        # Atualiza posição
        self.x += (
            v
            * math.cos(self.theta)
            * dt
        )

        self.y += (
            v
            * math.sin(self.theta)
            * dt
        )

        # Histórico
        if (
            len(self.history) == 0
            or np.hypot(
                self.x - self.history[-1][0],
                self.y - self.history[-1][1]
            ) > 0.03
        ):

            self.history.append(
                (
                    self.x,
                    self.y
                )
            )

            if len(self.history) > 1000:
                self.history.pop(0)

    # --------------------------------------------------------
    # Desenha robô
    # --------------------------------------------------------

    def draw(self, screen):

        # Trajetória
        if len(self.history) > 1:

            pontos = [
                (
                    int(x * ESCALA),
                    int(y * ESCALA)
                )
                for x, y in self.history
            ]

            pygame.draw.lines(
                screen,
                COR_TRAJETORIA,
                False,
                pontos,
                2
            )

        # Corpo
        pos = (
            int(self.x * ESCALA),
            int(self.y * ESCALA)
        )

        pygame.draw.circle(
            screen,
            COR_ROBO,
            pos,
            15
        )

        # Direção
        comprimento = 0.4

        frente_x = (
            self.x
            + comprimento
            * math.cos(self.theta)
        )

        frente_y = (
            self.y
            + comprimento
            * math.sin(self.theta)
        )

        pygame.draw.line(
            screen,
            COR_DIRECAO,
            pos,
            (
                int(frente_x * ESCALA),
                int(frente_y * ESCALA)
            ),
            4
        )


# ============================================================
# NORMALIZAÇÃO ANGULAR
# ============================================================

def normalizar_angulo(angulo):

    return (
        (angulo + math.pi)
        % (2 * math.pi)
    ) - math.pi


# ============================================================
# MÁQUINA DE ESTADOS FINITOS
# ============================================================

def maquina_de_estados(
    x,
    y,
    theta,
    x_alvo,
    y_alvo,
    dist_frente,
    dist_esq,
    dist_dir
):

    # ========================================================
    # DISTÂNCIA ATÉ O ALVO
    # ========================================================

    distancia_alvo = math.sqrt(
        (x_alvo - x) ** 2
        + (y_alvo - y) ** 2
    )

    # ========================================================
    # ESTADO 1
    # IR_PARA_ALVO
    # ========================================================

    # Primeiro verifica se já chegou.
    if distancia_alvo < DISTANCIA_ALVO:

        estado_atual = ESTADO_ALCANCADO

        v_cmd = 0.0
        omega_cmd = 0.0

        return (
            estado_atual,
            v_cmd,
            omega_cmd
        )

    # ========================================================
    # ESTADO 2
    # DESVIAR_OBSTACULO
    # ========================================================

    if dist_frente < DISTANCIA_OBSTACULO:

        estado_atual = ESTADO_DESVIAR

        # Para antes de girar
        v_cmd = 0.0

        # Escolhe o lado com maior espaço
        if dist_esq > dist_dir:

            omega_cmd = OMEGA_DESVIO

        elif dist_dir > dist_esq:

            omega_cmd = -OMEGA_DESVIO

        else:

            # Empate: gira para a esquerda
            omega_cmd = OMEGA_DESVIO

        return (
            estado_atual,
            v_cmd,
            omega_cmd
        )

    # ========================================================
    # ESTADO 3
    # IR_PARA_ALVO
    # ========================================================

    estado_atual = ESTADO_IR_PARA_ALVO

    # --------------------------------------------------------
    # Calcula direção do alvo
    # --------------------------------------------------------

    dx = x_alvo - x
    dy = y_alvo - y

    theta_alvo = math.atan2(
        dy,
        dx
    )

    # --------------------------------------------------------
    # Erro angular
    # --------------------------------------------------------

    erro_theta = normalizar_angulo(
        theta_alvo - theta
    )

    # --------------------------------------------------------
    # Controlador proporcional
    # --------------------------------------------------------

    omega_cmd = (
        KP_ANGULO
        * erro_theta
    )

    omega_cmd = np.clip(
        omega_cmd,
        -MAX_OMEGA,
        MAX_OMEGA
    )

    # --------------------------------------------------------
    # Velocidade linear
    # --------------------------------------------------------

    # Reduz a velocidade quando o robô
    # está muito desalinhado.

    fator_alinhamento = max(
        0.0,
        math.cos(erro_theta)
    )

    v_cmd = (
        VELOCIDADE_ALVO
        * fator_alinhamento
    )

    return (
        estado_atual,
        v_cmd,
        omega_cmd
    )


# ============================================================
# SIMULAÇÃO DO LIDAR
# ============================================================

def simular_lidar(
    robot,
    obstaculos
):

    leituras = np.full(
        360,
        5.0,
        dtype=float
    )

    for angulo in range(360):

        angulo_total = (
            robot.theta
            + math.radians(angulo)
        )

        dx = math.cos(
            angulo_total
        )

        dy = math.sin(
            angulo_total
        )

        menor_distancia = 5.0

        for ox, oy, raio in obstaculos:

            # Vetor do robô até obstáculo
            vx = ox - robot.x
            vy = oy - robot.y

            # Projeção na direção do raio
            projecao = (
                vx * dx
                + vy * dy
            )

            if projecao <= 0:
                continue

            # Distância perpendicular
            perpendicular = abs(
                vx * dy
                - vy * dx
            )

            if perpendicular <= raio:

                parte = (
                    raio ** 2
                    - perpendicular ** 2
                )

                if parte < 0:
                    continue

                distancia = (
                    projecao
                    - math.sqrt(parte)
                )

                if (
                    distancia >= 0
                    and distancia < menor_distancia
                ):

                    menor_distancia = (
                        distancia
                    )

        leituras[angulo] = (
            menor_distancia
        )

    # Pequenos ruídos
    for _ in range(5):

        indice = np.random.randint(
            0,
            360
        )

        leituras[indice] = np.random.choice(
            [
                0.0,
                np.inf,
                6.0
            ]
        )

    return leituras


# ============================================================
# PROCESSAMENTO DO SCAN
# ============================================================

def processar_scan(
    leituras_lidar
):

    leituras = np.asarray(
        leituras_lidar,
        dtype=float
    )

    if len(leituras) != 360:

        raise ValueError(
            "O LiDAR deve possuir 360 leituras."
        )

    # Filtro das leituras
    validas = np.where(
        np.isfinite(leituras)
        & (leituras >= 0.1)
        & (leituras <= 5.0),
        leituras,
        np.nan
    )

    # --------------------------------------------------------
    # Frente: 345° até 15°
    # --------------------------------------------------------

    setor_frente = np.concatenate(
        (
            validas[345:360],
            validas[0:16]
        )
    )

    # --------------------------------------------------------
    # Esquerda: 45° até 135°
    # --------------------------------------------------------

    setor_esquerda = validas[45:136]

    # --------------------------------------------------------
    # Direita: 225° até 315°
    # --------------------------------------------------------

    setor_direita = validas[225:316]

    # --------------------------------------------------------
    # Menor valor válido
    # --------------------------------------------------------

    def menor_distancia(setor):

        if np.any(
            np.isfinite(setor)
        ):

            return float(
                np.nanmin(setor)
            )

        return 5.0

    return {
        "frente": menor_distancia(
            setor_frente
        ),

        "esquerda": menor_distancia(
            setor_esquerda
        ),

        "direita": menor_distancia(
            setor_direita
        )
    }


# ============================================================
# DESENHA OBSTÁCULOS
# ============================================================

def desenhar_obstaculos(
    screen,
    obstaculos
):

    for x, y, raio in obstaculos:

        pygame.draw.circle(
            screen,
            COR_OBSTACULO,
            (
                int(x * ESCALA),
                int(y * ESCALA)
            ),
            int(raio * ESCALA)
        )


# ============================================================
# DESENHA LIDAR
# ============================================================

def desenhar_lidar(
    screen,
    robot,
    leituras
):

    origem = (
        int(robot.x * ESCALA),
        int(robot.y * ESCALA)
    )

    for angulo in range(
        0,
        360,
        5
    ):

        distancia = leituras[angulo]

        if (
            not np.isfinite(distancia)
            or distancia < 0.1
            or distancia > 5.0
        ):
            continue

        angulo_total = (
            robot.theta
            + math.radians(angulo)
        )

        x_final = (
            robot.x
            + distancia
            * math.cos(angulo_total)
        )

        y_final = (
            robot.y
            + distancia
            * math.sin(angulo_total)
        )

        pygame.draw.line(
            screen,
            COR_LIDAR,
            origem,
            (
                int(x_final * ESCALA),
                int(y_final * ESCALA)
            ),
            1
        )


# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================

def main():

    pygame.init()

    screen = pygame.display.set_mode(
        (
            LARGURA_TELA,
            ALTURA_TELA
        )
    )

    pygame.display.set_caption(
        "Exercício 5 - Máquina de Estados Finitos"
    )

    clock = pygame.time.Clock()

    font = pygame.font.SysFont(
        "monospace",
        14
    )

    # ========================================================
    # ROBÔ
    # ========================================================

    robot = DiffDriveRobot(
        x=2.0,
        y=3.75,
        theta=0.0,
        wheelbase=0.3
    )

    # ========================================================
    # ALVO
    # ========================================================

    # Alvo inicial
    alvo = (
        9.0,
        3.75
    )

    # ========================================================
    # OBSTÁCULOS
    # ========================================================

    obstaculos = [

        # Obstáculo principal
        (5.0, 3.75, 0.45),

        # Obstáculos adicionais
        (6.5, 1.5, 0.5),
        (7.0, 5.8, 0.5),
        (8.5, 2.0, 0.45),
        (8.5, 5.0, 0.45)
    ]

    running = True

    while running:

        dt = (
            clock.tick(FPS)
            / 1000.0
        )

        # ====================================================
        # EVENTOS
        # ====================================================

        for event in pygame.event.get():

            if event.type == pygame.QUIT:

                running = False

            # Clique define novo alvo
            elif (
                event.type
                == pygame.MOUSEBUTTONDOWN
                and event.button == 1
            ):

                alvo = (
                    event.pos[0] / ESCALA,
                    event.pos[1] / ESCALA
                )

                # Limita o alvo à área da simulação
                alvo = (
                    max(0.2, min(9.5, alvo[0])),
                    max(0.2, min(7.0, alvo[1]))
                )

        # ====================================================
        # LIDAR
        # ====================================================

        leituras = simular_lidar(
            robot,
            obstaculos
        )

        # ====================================================
        # PROCESSA SCAN
        # ====================================================

        distancias = processar_scan(
            leituras
        )

        # ====================================================
        # MÁQUINA DE ESTADOS
        # ====================================================

        estado, v_cmd, omega_cmd = (
            maquina_de_estados(
                robot.x,
                robot.y,
                robot.theta,
                alvo[0],
                alvo[1],
                distancias["frente"],
                distancias["esquerda"],
                distancias["direita"]
            )
        )

        # ====================================================
        # APLICA COMANDO
        # ====================================================

        robot.set_direct_velocity(
            v_cmd,
            omega_cmd
        )

        # ====================================================
        # ATUALIZA ROBÔ
        # ====================================================

        robot.update(dt)

        # ====================================================
        # DESENHO
        # ====================================================

        screen.fill(
            COR_FUNDO
        )

        # Obstáculos
        desenhar_obstaculos(
            screen,
            obstaculos
        )

        # LiDAR
        desenhar_lidar(
            screen,
            robot,
            leituras
        )

        # Alvo
        alvo_pixels = (
            int(alvo[0] * ESCALA),
            int(alvo[1] * ESCALA)
        )

        pygame.draw.circle(
            screen,
            COR_ALVO,
            alvo_pixels,
            10
        )

        # Área de chegada
        pygame.draw.circle(
            screen,
            COR_ALVO,
            alvo_pixels,
            int(DISTANCIA_ALVO * ESCALA),
            2
        )

        # Linha até o alvo
        pygame.draw.line(
            screen,
            (100, 100, 100),
            (
                int(robot.x * ESCALA),
                int(robot.y * ESCALA)
            ),
            alvo_pixels,
            1
        )

        # Robô
        robot.draw(
            screen
        )

        # ====================================================
        # TELEMETRIA
        # ====================================================

        distancia_alvo = math.sqrt(
            (alvo[0] - robot.x) ** 2
            + (alvo[1] - robot.y) ** 2
        )

        textos = [

            "EXERCICIO 5 - MAQUINA DE ESTADOS",

            "",

            f"ESTADO: {estado}",

            "",

            (
                f"Alvo: "
                f"X={alvo[0]:.2f} "
                f"Y={alvo[1]:.2f}"
            ),

            (
                f"Distancia alvo: "
                f"{distancia_alvo:.2f} m"
            ),

            "",

            (
                f"Frente: "
                f"{distancias['frente']:.2f} m"
            ),

            (
                f"Esquerda: "
                f"{distancias['esquerda']:.2f} m"
            ),

            (
                f"Direita: "
                f"{distancias['direita']:.2f} m"
            ),

            "",

            (
                f"v = "
                f"{v_cmd:.2f} m/s"
            ),

            (
                f"omega = "
                f"{omega_cmd:.2f} rad/s"
            ),

            "",

            "Clique na tela para mudar o alvo."
        ]

        for i, texto in enumerate(textos):

            renderizado = font.render(
                texto,
                True,
                (230, 230, 230)
            )

            screen.blit(
                renderizado,
                (
                    15,
                    15 + i * 20
                )
            )

        pygame.display.flip()

    pygame.quit()


# ============================================================
# EXECUÇÃO
# ============================================================

if __name__ == "__main__":
    main()
