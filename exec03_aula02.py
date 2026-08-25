import pygame
import math
import numpy as np

LARGURA_TELA = 800
ALTURA_TELA = 600
FPS = 60
COR_FUNDO = (30, 30, 30)
COR_ROBO = (0, 180, 255)
COR_DIRECAO = (255, 50, 50)
COR_TRAJETORIA = (100, 200, 100)
COR_ALVO = (255, 255, 0)

KP = 3.0
VELOCIDADE = 100.0
DISTANCIA_PARADA = 10.0

class DiffDriveRobot:
    def __init__(self, x, y, theta=0.0, wheelbase=30.0, radius=15.0):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.L = float(wheelbase)
        self.radius = float(radius)

        self.v = 0.0
        self.omega = 0.0

        self.history = []

    def set_wheel_velocities(self, v_left, v_right):
        self.v = (v_right + v_left) / 2.0
        self.omega = (v_right - v_left) / self.L

    def set_direct_velocity(self, v, omega):
        self.v = v
        self.omega = omega

    def update(self, dt):
        self.theta += self.omega * dt
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        if len(self.history) == 0 or np.hypot(
            self.x - self.history[-1][0],
            self.y - self.history[-1][1]
        ) > 5:
            self.history.append((self.x, self.y))

            if len(self.history) > 500:
                self.history.pop(0)

    def draw(self, surface):
        if len(self.history) > 1:
            pygame.draw.lines(
                surface,
                COR_TRAJETORIA,
                False,
                self.history,
                2
            )

        pos_int = (int(self.x), int(self.y))

        pygame.draw.circle(
            surface,
            COR_ROBO,
            pos_int,
            int(self.radius)
        )

        linha_frente_x = self.x + (self.radius + 10) * math.cos(self.theta)
        linha_frente_y = self.y + (self.radius + 10) * math.sin(self.theta)

        pygame.draw.line(
            surface,
            COR_DIRECAO,
            pos_int,
            (int(linha_frente_x), int(linha_frente_y)),
            3
        )

def normalizar_angulo(angulo):
    return (angulo + math.pi) % (2 * math.pi) - math.pi

def main():
    pygame.init()

    screen = pygame.display.set_mode(
        (LARGURA_TELA, ALTURA_TELA)
    )

    pygame.display.set_caption(
        "Exercício 3: Controle Proporcional"
    )

    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 14)

    robot = DiffDriveRobot(
        x=LARGURA_TELA // 2,
        y=ALTURA_TELA // 2,
        theta=0.0
    )

    alvo = None

    running = True

    while running:
        dt = clock.tick(FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    alvo = event.pos

        v_cmd = 0.0
        omega_cmd = 0.0
        distancia = 0.0
        theta_desejado = robot.theta
        erro_theta = 0.0

        if alvo is not None:
            x_alvo, y_alvo = alvo

            dx = x_alvo - robot.x
            dy = y_alvo - robot.y

            distancia = math.hypot(dx, dy)

            if distancia > DISTANCIA_PARADA:
                theta_desejado = math.atan2(dy, dx)

                erro_theta = normalizar_angulo(
                    theta_desejado - robot.theta
                )

                omega_cmd = KP * erro_theta
                v_cmd = VELOCIDADE
            else:
                v_cmd = 0.0
                omega_cmd = 0.0

        robot.set_direct_velocity(
            v_cmd,
            omega_cmd
        )

        robot.update(dt)

        screen.fill(COR_FUNDO)

        if alvo is not None:
            pygame.draw.circle(
                screen,
                COR_ALVO,
                alvo,
                10
            )

            pygame.draw.circle(
                screen,
                COR_ALVO,
                alvo,
                10,
                2
            )

        robot.draw(screen)

        info_txt = [
            f"Pose X: {robot.x:.1f} px | Y: {robot.y:.1f} px | Theta: {math.degrees(robot.theta):.1f} deg",
            f"Alvo: {alvo if alvo is not None else 'Nenhum'}",
            f"Distancia: {distancia:.1f} px",
            f"Theta desejado: {math.degrees(theta_desejado):.1f} deg",
            f"Erro theta: {math.degrees(erro_theta):.1f} deg",
            f"v = {robot.v:.1f} px/s | omega = {robot.omega:.2f} rad/s",
            "Clique com o botao esquerdo do mouse para definir o alvo"
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

if __name__ == "__main__":
    main()