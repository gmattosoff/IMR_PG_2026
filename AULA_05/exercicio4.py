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
COR_LINHA_ALVO = (120, 120, 120)

ESCALA = 80.0

# ============================================================
# PARÂMETROS DO CONTROLADOR
# ============================================================

KP = 1.5

MAX_OMEGA = 2.0

TOLERANCIA_ANGULAR = math.radians(1.0)

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
    # Recebe velocidade linear e angular
    # --------------------------------------------------------

    def set_direct_velocity(self, v, omega):

        self.v = float(v)
        self.omega = float(omega)

    # --------------------------------------------------------
    # Atualização da pose
    # --------------------------------------------------------

    def update(self, dt):

        # Para este exercício:
        # o robô permanece no lugar e somente gira.

        self.v_e = (
            self.v
            - (self.omega * self.L / 2.0)
        )

        self.v_d = (
            self.v
            + (self.omega * self.L / 2.0)
        )

        # Atualiza orientação
        self.theta += self.omega * dt

        # Normalização angular
        self.theta = normalizar_angulo(
            self.theta
        )

        # Histórico da posição
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

            if len(self.history) > 600:
                self.history.pop(0)

    # --------------------------------------------------------
    # Desenho
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

        # Posição
        pos = (
            int(self.x * ESCALA),
            int(self.y * ESCALA)
        )

        # Corpo
        pygame.draw.circle(
            screen,
            COR_ROBO,
            pos,
            15
        )

        # ----------------------------------------------------
        # Indicador de orientação
        # ----------------------------------------------------

        comprimento = 0.45

        frente_x = (
            self.x
            + comprimento * math.cos(self.theta)
        )

        frente_y = (
            self.y
            + comprimento * math.sin(self.theta)
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

    """
    Normaliza um ângulo para o intervalo [-pi, pi].
    """

    return (
        (angulo + math.pi)
        % (2 * math.pi)
    ) - math.pi


# ============================================================
# CONTROLADOR PROPORCIONAL
# ============================================================

def calcular_orientacao_alvo(
    x,
    y,
    theta,
    x_alvo,
    y_alvo,
    Kp=1.5
):

    # ========================================================
    # 1. ÂNGULO DESEJADO
    # ========================================================

    dx = x_alvo - x
    dy = y_alvo - y

    theta_alvo = math.atan2(
        dy,
        dx
    )

    # ========================================================
    # 2. ERRO DE ORIENTAÇÃO
    # ========================================================

    e_theta = (
        theta_alvo - theta
    )

    # Normalização [-pi, pi]
    e_theta = normalizar_angulo(
        e_theta
    )

    # ========================================================
    # 3. CONTROLADOR PROPORCIONAL
    # ========================================================

    omega = Kp * e_theta

    # Saturação
    omega = np.clip(
        omega,
        -MAX_OMEGA,
        MAX_OMEGA
    )

    return omega


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
        "Exercício 4 - Controlador P para Orientação"
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
        x=4.0,
        y=3.75,
        theta=0.0,
        wheelbase=0.3
    )

    # ========================================================
    # ALVO
    # ========================================================

    alvo = None

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

            # ------------------------------------------------
            # Clique esquerdo define alvo
            # ------------------------------------------------

            elif (
                event.type
                == pygame.MOUSEBUTTONDOWN
                and event.button == 1
            ):

                alvo = event.pos

        # ====================================================
        # CONTROLE
        # ====================================================

        omega_cmd = 0.0

        theta_alvo = None
        erro_theta = None

        if alvo is not None:

            # Converte coordenadas da tela
            # para metros

            x_alvo = (
                alvo[0] / ESCALA
            )

            y_alvo = (
                alvo[1] / ESCALA
            )

            # ------------------------------------------------
            # Calcula orientação desejada
            # ------------------------------------------------

            dx = x_alvo - robot.x
            dy = y_alvo - robot.y

            theta_alvo = math.atan2(
                dy,
                dx
            )

            # ------------------------------------------------
            # Erro angular
            # ------------------------------------------------

            erro_theta = normalizar_angulo(
                theta_alvo - robot.theta
            )

            # ------------------------------------------------
            # Controlador P
            # ------------------------------------------------

            omega_cmd = (
                KP
                * erro_theta
            )

            # Limite físico
            omega_cmd = np.clip(
                omega_cmd,
                -MAX_OMEGA,
                MAX_OMEGA
            )

            # ------------------------------------------------
            # Pequena tolerância
            # ------------------------------------------------

            if abs(erro_theta) < TOLERANCIA_ANGULAR:

                omega_cmd = 0.0

        # ====================================================
        # APLICA COMANDO
        # ====================================================

        # Não existe avanço neste exercício.
        # O objetivo é somente alinhar a orientação.

        robot.set_direct_velocity(
            0.0,
            omega_cmd
        )

        # ====================================================
        # ATUALIZA ROBÔ
        # ====================================================

        robot.update(dt)

        # ====================================================
        # RENDERIZAÇÃO
        # ====================================================

        screen.fill(
            COR_FUNDO
        )

        # ----------------------------------------------------
        # Alvo
        # ----------------------------------------------------

        if alvo is not None:

            pygame.draw.circle(
                screen,
                COR_ALVO,
                alvo,
                10
            )

            # Linha entre robô e alvo
            pygame.draw.line(
                screen,
                COR_LINHA_ALVO,
                (
                    int(robot.x * ESCALA),
                    int(robot.y * ESCALA)
                ),
                alvo,
                1
            )

        # ----------------------------------------------------
        # Robô
        # ----------------------------------------------------

        robot.draw(
            screen
        )

        # ====================================================
        # TELEMETRIA
        # ====================================================

        if theta_alvo is not None:

            theta_alvo_graus = (
                math.degrees(theta_alvo)
            )

        else:

            theta_alvo_graus = 0.0

        if erro_theta is not None:

            erro_graus = (
                math.degrees(erro_theta)
            )

        else:

            erro_graus = 0.0

        info_txt = [

            "EXERCÍCIO 4 - CONTROLADOR P",

            "",

            (
                f"Pose: "
                f"X={robot.x:.2f} m | "
                f"Y={robot.y:.2f} m"
            ),

            (
                f"Theta atual: "
                f"{math.degrees(robot.theta):.2f} graus"
            ),

            (
                f"Theta alvo: "
                f"{theta_alvo_graus:.2f} graus"
            ),

            (
                f"Erro theta: "
                f"{erro_graus:.2f} graus"
            ),

            "",

            (
                f"Kp = {KP:.2f}"
            ),

            (
                f"omega = "
                f"{omega_cmd:.3f} rad/s"
            ),

            "",

            "Clique com o botao esquerdo",
            "para definir um novo alvo."
        ]

        for i, texto in enumerate(
            info_txt
        ):

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
