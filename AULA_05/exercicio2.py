import pygame
import math
import numpy as np

# ============================================================
# CONFIGURAÇÕES DA JANELA
# ============================================================

LARGURA_TELA = 800
ALTURA_TELA = 600
FPS = 60

COR_FUNDO = (30, 30, 30)
COR_ROBO = (0, 180, 255)
COR_DIRECAO = (255, 50, 50)
COR_TRAJETORIA = (100, 200, 100)
COR_ALVO = (255, 215, 0)

COR_LIDAR = (0, 255, 0)
COR_OBSTACULO = (180, 80, 80)

# ============================================================
# CONFIGURAÇÕES DO ROBÔ E DO LIDAR
# ============================================================

ESCALA = 100.0       # 1 metro = 100 pixels

L = 0.3              # Distância entre rodas (m)

LIDAR_MIN = 0.1      # Distância mínima válida (m)
LIDAR_MAX = 5.0      # Distância máxima válida (m)

# ============================================================
# ROBÔ DIFERENCIAL
# ============================================================


class DiffDriveRobot:

    def __init__(
        self,
        x,
        y,
        theta=0.0,
        wheelbase=0.3,
        radius=0.15
    ):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)

        self.L = float(wheelbase)
        self.radius = float(radius)

        self.v = 0.0
        self.omega = 0.0

        self.v_e = 0.0
        self.v_d = 0.0

        self.history = []

    def set_direct_velocity(self, v, omega):
        self.v = float(v)
        self.omega = float(omega)

    def update(self, dt):

        # Cinemática diferencial
        self.v_e = self.v - (self.omega * self.L / 2.0)
        self.v_d = self.v + (self.omega * self.L / 2.0)

        # Velocidade resultante
        v = (self.v_d + self.v_e) / 2.0

        # Velocidade angular resultante
        omega = (self.v_d - self.v_e) / self.L

        # Atualiza orientação
        self.theta += omega * dt

        # Normalização angular [-pi, pi]
        self.theta = (
            (self.theta + math.pi)
            % (2 * math.pi)
        ) - math.pi

        # Atualiza posição
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        # Histórico
        if (
            len(self.history) == 0
            or np.hypot(
                self.x - self.history[-1][0],
                self.y - self.history[-1][1]
            ) > 0.05
        ):
            self.history.append(
                (self.x, self.y)
            )

            if len(self.history) > 600:
                self.history.pop(0)

    def draw(self, surface):

        # ----------------------------------------------------
        # Trajetória
        # ----------------------------------------------------

        if len(self.history) > 1:

            pontos = []

            for x, y in self.history:

                pontos.append(
                    (
                        int(x * ESCALA),
                        int(y * ESCALA)
                    )
                )

            pygame.draw.lines(
                surface,
                COR_TRAJETORIA,
                False,
                pontos,
                2
            )

        # ----------------------------------------------------
        # Corpo do robô
        # ----------------------------------------------------

        pos_int = (
            int(self.x * ESCALA),
            int(self.y * ESCALA)
        )

        raio_pixels = int(
            self.radius * ESCALA
        )

        pygame.draw.circle(
            surface,
            COR_ROBO,
            pos_int,
            raio_pixels
        )

        # ----------------------------------------------------
        # Direção
        # ----------------------------------------------------

        linha_frente_x = (
            self.x * ESCALA
            + (self.radius + 0.15)
            * ESCALA
            * math.cos(self.theta)
        )

        linha_frente_y = (
            self.y * ESCALA
            + (self.radius + 0.15)
            * ESCALA
            * math.sin(self.theta)
        )

        pygame.draw.line(
            surface,
            COR_DIRECAO,
            pos_int,
            (
                int(linha_frente_x),
                int(linha_frente_y)
            ),
            3
        )


# ============================================================
# PROCESSAMENTO DO /SCAN
# ============================================================


def processar_scan(leituras_lidar):

    """
    Recebe 360 leituras do LiDAR.

    Cada índice representa um grau:

        0°   -> frente
        90°  -> esquerda
        180° -> trás
        270° -> direita

    Retorna a menor distância válida de:

        frente   -> 345° a 15°
        esquerda -> 45° a 135°
        direita  -> 225° a 315°

    Leituras válidas:

        >= 0.1 m
        <= 5.0 m
    """

    leituras = np.asarray(
        leituras_lidar,
        dtype=float
    )

    # Verificação
    if len(leituras) != 360:
        raise ValueError(
            "O LiDAR deve possuir exatamente 360 leituras."
        )

    # --------------------------------------------------------
    # FILTRO
    # --------------------------------------------------------

    leituras_validas = np.where(
        np.isfinite(leituras)
        & (leituras >= LIDAR_MIN)
        & (leituras <= LIDAR_MAX),
        leituras,
        np.nan
    )

    # --------------------------------------------------------
    # SETOR FRENTE
    # 345° até 359°
    # 0° até 15°
    # --------------------------------------------------------

    setor_frente = np.concatenate(
        (
            leituras_validas[345:360],
            leituras_validas[0:16]
        )
    )

    # --------------------------------------------------------
    # SETOR ESQUERDA
    # 45° até 135°
    # --------------------------------------------------------

    setor_esquerda = leituras_validas[45:136]

    # --------------------------------------------------------
    # SETOR DIREITA
    # 225° até 315°
    # --------------------------------------------------------

    setor_direita = leituras_validas[225:316]

    # --------------------------------------------------------
    # MENOR DISTÂNCIA
    # --------------------------------------------------------

    min_frente = (
        np.nanmin(setor_frente)
        if np.any(np.isfinite(setor_frente))
        else LIDAR_MAX
    )

    min_esquerda = (
        np.nanmin(setor_esquerda)
        if np.any(np.isfinite(setor_esquerda))
        else LIDAR_MAX
    )

    min_direita = (
        np.nanmin(setor_direita)
        if np.any(np.isfinite(setor_direita))
        else LIDAR_MAX
    )

    return {
        'frente': float(min_frente),
        'esquerda': float(min_esquerda),
        'direita': float(min_direita)
    }


# ============================================================
# SIMULADOR DE LIDAR
# ============================================================


def simular_lidar(robot, obstaculos):

    """
    Simula 360 leituras do LiDAR.

    Cada obstáculo é representado por:
        (x, y, raio)

    O LiDAR procura a menor distância até os obstáculos.
    """

    leituras = np.full(
        360,
        LIDAR_MAX,
        dtype=float
    )

    for angulo in range(360):

        angulo_rad = math.radians(angulo)

        menor_distancia = LIDAR_MAX

        # Direção do raio do LiDAR
        dx = math.cos(
            robot.theta + angulo_rad
        )

        dy = math.sin(
            robot.theta + angulo_rad
        )

        for ox, oy, raio in obstaculos:

            # Vetor até o centro do obstáculo
            vx = ox - robot.x
            vy = oy - robot.y

            # Projeção na direção do sensor
            projecao = vx * dx + vy * dy

            if projecao <= 0:
                continue

            # Distância perpendicular
            perpendicular = abs(
                vx * dy - vy * dx
            )

            # O raio do sensor atingiu o obstáculo?
            if perpendicular <= raio:

                distancia = (
                    projecao
                    - math.sqrt(
                        max(
                            0,
                            raio ** 2
                            - perpendicular ** 2
                        )
                    )
                )

                if (
                    distancia >= 0
                    and distancia < menor_distancia
                ):
                    menor_distancia = distancia

        leituras[angulo] = menor_distancia

    # --------------------------------------------------------
    # Adiciona alguns ruídos propositalmente
    # --------------------------------------------------------

    for _ in range(10):

        indice = np.random.randint(
            0,
            360
        )

        tipo_ruido = np.random.choice(
            [
                "zero",
                "infinito",
                "fora_alcance"
            ]
        )

        if tipo_ruido == "zero":
            leituras[indice] = 0.0

        elif tipo_ruido == "infinito":
            leituras[indice] = np.inf

        elif tipo_ruido == "fora_alcance":
            leituras[indice] = 6.0

    return leituras


# ============================================================
# DESENHO DOS OBSTÁCULOS
# ============================================================


def desenhar_obstaculos(screen, obstaculos):

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
# DESENHO DAS LEITURAS DO LIDAR
# ============================================================


def desenhar_lidar(
    screen,
    robot,
    leituras_lidar
):

    pos_robot = (
        int(robot.x * ESCALA),
        int(robot.y * ESCALA)
    )

    # Mostra apenas uma leitura a cada 3 graus
    for angulo in range(0, 360, 3):

        distancia = leituras_lidar[angulo]

        # Não desenha leituras inválidas
        if (
            not np.isfinite(distancia)
            or distancia < LIDAR_MIN
            or distancia > LIDAR_MAX
        ):
            continue

        # Limita visualização
        distancia = min(
            distancia,
            LIDAR_MAX
        )

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
            pos_robot,
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
        "Exercício 2 - Nó Sensor /scan"
    )

    clock = pygame.time.Clock()

    font = pygame.font.SysFont(
        "monospace",
        14
    )

    # --------------------------------------------------------
    # Robô
    # --------------------------------------------------------

    robot = DiffDriveRobot(
        x=4.0,
        y=3.0,
        theta=0.0,
        wheelbase=L
    )

    # --------------------------------------------------------
    # Obstáculos
    # --------------------------------------------------------

    obstaculos = [

        # obstáculo na frente
        (6.0, 3.0, 0.5),

        # obstáculo à esquerda
        (4.0, 1.5, 0.4),

        # obstáculo à direita
        (4.0, 4.5, 0.4),

        # outros obstáculos
        (2.0, 2.0, 0.5),
        (7.0, 5.0, 0.6)
    ]

    running = True

    while running:

        dt = clock.tick(FPS) / 1000.0

        # ====================================================
        # EVENTOS
        # ====================================================

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                running = False

        # ====================================================
        # MOVIMENTO DO ROBÔ
        # ====================================================
        #
        # Movimento simples apenas para permitir observar
        # o sensor funcionando.
        #
        # O robô gira continuamente e avança lentamente.
        # ====================================================

        robot.set_direct_velocity(
            v=0.15,
            omega=0.25
        )

        robot.update(dt)

        # ====================================================
        # GERA SCAN
        # ====================================================

        leituras_lidar = simular_lidar(
            robot,
            obstaculos
        )

        # ====================================================
        # PROCESSA SCAN
        # ====================================================

        resultado_scan = processar_scan(
            leituras_lidar
        )

        # ====================================================
        # RENDERIZAÇÃO
        # ====================================================

        screen.fill(COR_FUNDO)

        # Obstáculos
        desenhar_obstaculos(
            screen,
            obstaculos
        )

        # LiDAR
        desenhar_lidar(
            screen,
            robot,
            leituras_lidar
        )

        # Robô
        robot.draw(screen)

        # ====================================================
        # TELEMETRIA
        # ====================================================

        info_txt = [

            "EXERCÍCIO 2 - PROCESSADOR /scan",

            (
                f"Frente:   "
                f"{resultado_scan['frente']:.2f} m"
            ),

            (
                f"Esquerda: "
                f"{resultado_scan['esquerda']:.2f} m"
            ),

            (
                f"Direita:  "
                f"{resultado_scan['direita']:.2f} m"
            ),

            "",

            (
                f"Pose: "
                f"X={robot.x:.2f} m | "
                f"Y={robot.y:.2f} m"
            ),

            (
                f"Theta: "
                f"{math.degrees(robot.theta):.1f} graus"
            ),

            "",

            "Filtro LiDAR:",

            "Minimo valido: 0.10 m",

            "Maximo valido: 5.00 m",

            "Leituras: 360"
        ]

        for i, txt in enumerate(info_txt):

            rendered = font.render(
                txt,
                True,
                (220, 220, 220)
            )

            screen.blit(
                rendered,
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
