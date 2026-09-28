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
COR_RODA = (80, 80, 80)


# ============================================================
# CONVERSOR /cmd_vel -> VELOCIDADES DAS RODAS
# ============================================================

def converter_cmd_vel(v, omega, L=0.3, max_wheel_speed=1.5):
    """
    Converte velocidade linear e angular do robô em
    velocidades das rodas esquerda e direita.

    v      -> velocidade linear (m/s)
    omega  -> velocidade angular (rad/s)
    L      -> distância entre as rodas (m)

    Retorna:
        v_e -> velocidade da roda esquerda (m/s)
        v_d -> velocidade da roda direita (m/s)
    """

    # 1. Calcula as velocidades brutas das rodas
    v_e = v - (omega * L / 2)
    v_d = v + (omega * L / 2)

    # 2. Verifica se alguma roda ultrapassou o limite
    maior_velocidade = max(abs(v_e), abs(v_d))

    # 3. Saturação proporcional
    if maior_velocidade > max_wheel_speed:

        fator = max_wheel_speed / maior_velocidade

        v_e *= fator
        v_d *= fator

    return v_e, v_d


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
        radius=15.0
    ):

        # Posição em metros
        self.x = float(x)
        self.y = float(y)

        # Orientação
        self.theta = float(theta)

        # Distância entre as rodas
        self.L = float(wheelbase)

        # Raio apenas para desenho
        self.radius = float(radius)

        # Comandos do robô
        self.v = 0.0
        self.omega = 0.0

        # Velocidades das rodas
        self.v_e = 0.0
        self.v_d = 0.0

        # Histórico da trajetória
        self.history = []

    def set_cmd_vel(self, v, omega):

        # Guarda o comando recebido
        self.v = v
        self.omega = omega

        # Converte cmd_vel para velocidades das rodas
        self.v_e, self.v_d = converter_cmd_vel(
            v,
            omega,
            self.L,
            1.5
        )

    def update(self, dt):

        # ----------------------------------------------------
        # Cinemática do robô diferencial
        # ----------------------------------------------------

        # Velocidade linear resultante
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

        # Atualiza os valores efetivamente aplicados
        self.v = v
        self.omega = omega

        # ----------------------------------------------------
        # Histórico
        # ----------------------------------------------------

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

    def draw(self, surface, escala):

        # Desenha trajetória
        if len(self.history) > 1:

            pontos = []

            for x, y in self.history:
                pontos.append(
                    (
                        int(x * escala),
                        int(y * escala)
                    )
                )

            pygame.draw.lines(
                surface,
                COR_TRAJETORIA,
                False,
                pontos,
                2
            )

        # Converte posição do robô para pixels
        pos_int = (
            int(self.x * escala),
            int(self.y * escala)
        )

        # Desenha corpo
        pygame.draw.circle(
            surface,
            COR_ROBO,
            pos_int,
            int(self.radius)
        )

        # Desenha direção
        linha_frente_x = (
            self.x * escala
            + (self.radius + 10)
            * math.cos(self.theta)
        )

        linha_frente_y = (
            self.y * escala
            + (self.radius + 10)
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
# NORMALIZAÇÃO ANGULAR
# ============================================================

def normalizar_angulo(angulo):

    return (
        (angulo + math.pi)
        % (2 * math.pi)
    ) - math.pi


# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================

def main():

    pygame.init()

    screen = pygame.display.set_mode(
        (LARGURA_TELA, ALTURA_TELA)
    )

    pygame.display.set_caption(
        "Exercício 1 - Nó Atuador /cmd_vel"
    )

    clock = pygame.time.Clock()

    font = pygame.font.SysFont(
        "monospace",
        14
    )

    # --------------------------------------------------------
    # ESCALA
    # --------------------------------------------------------
    # 1 metro = 250 pixels
    # --------------------------------------------------------

    ESCALA = 250

    # --------------------------------------------------------
    # Robô
    # --------------------------------------------------------

    robot = DiffDriveRobot(
        x=400 / ESCALA,
        y=300 / ESCALA,
        theta=0.0,
        wheelbase=0.3
    )

    # --------------------------------------------------------
    # Alvo
    # --------------------------------------------------------

    target_pos = None

    # --------------------------------------------------------
    # Controlador P
    # --------------------------------------------------------

    KP_DISTANCIA = 1.2
    KP_ANGULO = 4.0

    # Limites do robô
    MAX_V = 1.5
    MAX_OMEGA = 3.5

    TOLERANCIA_CHEGADA = 0.03

    running = True

    while running:

        dt = clock.tick(FPS) / 1000.0

        # ====================================================
        # EVENTOS
        # ====================================================

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                running = False

            elif (
                event.type == pygame.MOUSEBUTTONDOWN
                and event.button == 1
            ):

                target_pos = event.pos

        # ====================================================
        # CONTROLADOR
        # ====================================================

        v_cmd = 0.0
        omega_cmd = 0.0

        if target_pos is not None:

            # Converte alvo de pixels para metros
            alvo_x = target_pos[0] / ESCALA
            alvo_y = target_pos[1] / ESCALA

            # Erros de posição
            dx = alvo_x - robot.x
            dy = alvo_y - robot.y

            distancia = math.hypot(dx, dy)

            if distancia > TOLERANCIA_CHEGADA:

                # Ângulo desejado
                theta_desejado = math.atan2(
                    dy,
                    dx
                )

                # Erro angular
                erro_theta = normalizar_angulo(
                    theta_desejado - robot.theta
                )

                # Controle angular
                omega_cmd = (
                    KP_ANGULO
                    * erro_theta
                )

                omega_cmd = np.clip(
                    omega_cmd,
                    -MAX_OMEGA,
                    MAX_OMEGA
                )

                # Controle linear
                fator_alinhamento = max(
                    0.0,
                    math.cos(erro_theta)
                )

                v_cmd = (
                    KP_DISTANCIA
                    * distancia
                    * fator_alinhamento
                )

                v_cmd = np.clip(
                    v_cmd,
                    0.0,
                    MAX_V
                )

        # ====================================================
        # ENVIA /cmd_vel PARA O ROBÔ
        # ====================================================

        robot.set_cmd_vel(
            v_cmd,
            omega_cmd
        )

        # ====================================================
        # ATUALIZA ROBÔ
        # ====================================================

        robot.update(dt)

        # ====================================================
        # RENDERIZAÇÃO
        # ====================================================

        screen.fill(COR_FUNDO)

        # ----------------------------------------------------
        # Desenha alvo
        # ----------------------------------------------------

        if target_pos is not None:

            pygame.draw.circle(
                screen,
                COR_ALVO,
                target_pos,
                8
            )

            pygame.draw.circle(
                screen,
                COR_ALVO,
                target_pos,
                int(TOLERANCIA_CHEGADA * ESCALA),
                1
            )

        # ----------------------------------------------------
        # Desenha robô
        # ----------------------------------------------------

        robot.draw(
            screen,
            ESCALA
        )

        # ====================================================
        # TELEMETRIA
        # ====================================================

        info_txt = [

            f"Pose: X={robot.x:.2f} m | "
            f"Y={robot.y:.2f} m | "
            f"Theta={math.degrees(robot.theta):.1f} deg",

            f"/cmd_vel: "
            f"v={v_cmd:.2f} m/s | "
            f"omega={omega_cmd:.2f} rad/s",

            f"Roda esquerda: "
            f"{robot.v_e:.2f} m/s",

            f"Roda direita: "
            f"{robot.v_d:.2f} m/s",

            "Limite das rodas: 1.50 m/s",

            f"Alvo: "
            f"{target_pos if target_pos else 'Clique na tela'}"
        ]

        for i, txt in enumerate(info_txt):

            rendered = font.render(
                txt,
                True,
                (220, 220, 220)
            )

            screen.blit(
                rendered,
                (15, 15 + i * 20)
            )

        pygame.display.flip()

    pygame.quit()


# ============================================================
# EXECUÇÃO
# ============================================================

if __name__ == "__main__":
    main()

