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
COR_OBSTACULO = (180, 80, 80)
COR_LIDAR = (0, 255, 0)
COR_TEXTO = (230, 230, 230)

# ============================================================
# ROBÔ
# ============================================================

L = 0.3

class DiffDriveRobot:

    def __init__(self, x, y, theta=0.0, wheelbase=0.3):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)

        self.L = float(wheelbase)

        self.v = 0.0
        self.omega = 0.0

        self.v_e = 0.0
        self.v_d = 0.0

        self.history = []

    def set_direct_velocity(self, v, omega):
        self.v = float(v)
        self.omega = float(omega)

    def update(self, dt):

        # Cinemática do robô diferencial
        self.v_e = (
            self.v
            - (self.omega * self.L / 2.0)
        )

        self.v_d = (
            self.v
            + (self.omega * self.L / 2.0)
        )

        # Velocidade efetiva
        v = (self.v_e + self.v_d) / 2.0

        omega = (
            self.v_d - self.v_e
        ) / self.L

        # Atualiza orientação
        self.theta += omega * dt

        self.theta = (
            (self.theta + math.pi)
            % (2 * math.pi)
        ) - math.pi

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
                (self.x, self.y)
            )

            if len(self.history) > 1000:
                self.history.pop(0)

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
        frente_x = (
            self.x
            + 0.35 * math.cos(self.theta)
        )

        frente_y = (
            self.y
            + 0.35 * math.sin(self.theta)
        )

        pygame.draw.line(
            screen,
            COR_DIRECAO,
            pos,
            (
                int(frente_x * ESCALA),
                int(frente_y * ESCALA)
            ),
            3
        )


# ============================================================
# PROCESSAMENTO DO LIDAR
# ============================================================

def processar_scan(leituras_lidar):

    leituras = np.asarray(
        leituras_lidar,
        dtype=float
    )

    if len(leituras) != 360:
        raise ValueError(
            "O LiDAR deve possuir 360 leituras."
        )

    # Filtro
    validas = np.where(
        np.isfinite(leituras)
        & (leituras >= 0.1)
        & (leituras <= 5.0),
        leituras,
        np.nan
    )

    # Frente: 345° até 359° + 0° até 15°
    frente = np.concatenate(
        (
            validas[345:360],
            validas[0:16]
        )
    )

    # Esquerda
    esquerda = validas[45:136]

    # Direita
    direita = validas[225:316]

    def menor_valor(setor):
        if np.any(np.isfinite(setor)):
            return float(np.nanmin(setor))
        return 5.0

    return {
        "frente": menor_valor(frente),
        "esquerda": menor_valor(esquerda),
        "direita": menor_valor(direita)
    }


# ============================================================
# CONTROLE REATIVO
# ============================================================

def controle_reativo(distancias):

    frente = distancias["frente"]
    esquerda = distancias["esquerda"]
    direita = distancias["direita"]

    DISTANCIA_CRITICA = 0.4

    # --------------------------------------------------------
    # OBSTÁCULO À FRENTE
    # --------------------------------------------------------

    if frente < DISTANCIA_CRITICA:

        # Freia
        v = 0.0

        # Gira para o lado mais livre
        if esquerda > direita:
            omega = 1.0
        else:
            omega = -1.0

        return v, omega

    # --------------------------------------------------------
    # FRENTE LIVRE
    # --------------------------------------------------------

    v = 0.5

    # Erro lateral
    erro_lateral = esquerda - direita

    # Ganho proporcional
    KP = 0.8

    omega = KP * erro_lateral

    # Limite
    omega = np.clip(
        omega,
        -1.0,
        1.0
    )

    return v, omega


# ============================================================
# SIMULADOR DE LIDAR
# ============================================================

def simular_lidar(robot, obstaculos):

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

        dx = math.cos(angulo_total)
        dy = math.sin(angulo_total)

        menor = 5.0

        for ox, oy, raio in obstaculos:

            vx = ox - robot.x
            vy = oy - robot.y

            projecao = (
                vx * dx
                + vy * dy
            )

            if projecao <= 0:
                continue

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
                    and distancia < menor
                ):
                    menor = distancia

        leituras[angulo] = menor

    # Pequenos ruídos do sensor
    for _ in range(5):

        indice = np.random.randint(
            0,
            360
        )

        ruido = np.random.choice(
            [
                0.0,
                np.inf,
                6.0
            ]
        )

        leituras[indice] = ruido

    return leituras


# ============================================================
# DESENHA OBSTÁCULOS
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

    # Mostra somente alguns raios
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

        x = (
            robot.x
            + distancia
            * math.cos(angulo_total)
        )

        y = (
            robot.y
            + distancia
            * math.sin(angulo_total)
        )

        pygame.draw.line(
            screen,
            COR_LIDAR,
            origem,
            (
                int(x * ESCALA),
                int(y * ESCALA)
            ),
            1
        )


# ============================================================
# MAIN
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
        "Exercício 3 - Braitenberg com Trava de Segurança"
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
        wheelbase=L
    )

    # ========================================================
    # OBSTÁCULOS
    # ========================================================

    # O primeiro obstáculo está à frente,
    # mas suficientemente distante para o robô começar andando.

    obstaculos = [

        # Obstáculo principal
        (5.0, 3.75, 0.45),

        # Obstáculo superior
        (6.0, 1.5, 0.5),

        # Obstáculo inferior
        (6.5, 6.0, 0.5),

        # Outros obstáculos
        (8.0, 2.5, 0.45),
        (8.0, 5.0, 0.45)
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

        # ====================================================
        # SENSOR
        # ====================================================

        leituras = simular_lidar(
            robot,
            obstaculos
        )

        # ====================================================
        # PROCESSAMENTO
        # ====================================================

        distancias = processar_scan(
            leituras
        )

        # ====================================================
        # CONTROLE
        # ====================================================

        v_cmd, omega_cmd = (
            controle_reativo(
                distancias
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
        # ATUALIZA
        # ====================================================

        robot.update(dt)

        # ====================================================
        # DESENHO
        # ====================================================

        screen.fill(
            COR_FUNDO
        )

        desenhar_obstaculos(
            screen,
            obstaculos
        )

        desenhar_lidar(
            screen,
            robot,
            leituras
        )

        robot.draw(
            screen
        )

        # ====================================================
        # TELEMETRIA
        # ====================================================

        if (
            distancias["frente"]
            < 0.4
        ):
            estado = "FREANDO / GIRANDO"

        else:
            estado = "AVANCANDO"

        textos = [

            "EXERCICIO 3 - CONTROLE REATIVO",

            "",

            f"Frente:   "
            f"{distancias['frente']:.2f} m",

            f"Esquerda: "
            f"{distancias['esquerda']:.2f} m",

            f"Direita:  "
            f"{distancias['direita']:.2f} m",

            "",

            f"v = "
            f"{v_cmd:.2f} m/s",

            f"omega = "
            f"{omega_cmd:.2f} rad/s",

            "",

            f"Estado: {estado}",

            "",

            f"X = {robot.x:.2f} m",

            f"Y = {robot.y:.2f} m",

            f"Theta = "
            f"{math.degrees(robot.theta):.1f} graus"
        ]

        for i, texto in enumerate(textos):

            renderizado = font.render(
                texto,
                True,
                COR_TEXTO
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
