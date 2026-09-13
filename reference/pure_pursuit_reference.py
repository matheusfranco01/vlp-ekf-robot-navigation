import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation

plt.rcParams.update({"font.size": 15})

def T2D(M1, dx, dy):
    """Aplica translacao 2D em coordenadas homogeneas."""
    T = np.array([
    [1.0, 0.0, dx],
    [0.0, 1.0, dy],
    [0.0, 0.0, 1.0]
    ])
    return T @ M1

def Rz2D(M1, th):
    """Aplica rotacao no plano em coordenadas homogeneas."""
    th = float(np.asarray(th).ravel()[0])
    Rz = np.array([
    [np.cos(th), -np.sin(th), 0.0],
    [np.sin(th), np.cos(th), 0.0],
    [0.0, 0.0, 1.0]
    ])
    return Rz @ M1

class differentialRobot:
    """
    Classe para representar um robô diferencial.
    """
    def __init__(self, x0=0.0, y0=0.0, theta0=0.0, vl=0.4):
        self.x = x0
        self.y = y0
        self.theta = theta0
        self.vl = vl

    def closest_point(self, path):
        """Encontra o ponto mais próximo do robô no caminho."""
        # Deslocamento do robô para cada ponto do caminho
        dx = path.cx - self.x
        dy = path.cy - self.y

        # Distância euclidiana - Look ahead distance
        dists = np.hypot(dx, dy) 
        closest_index = np.argmin(dists)
        return closest_index
    
    def goal_point(self, path, look_ahead_distance):
        """Encontra o ponto-alvo do Pure Pursuit."""
        closest_index = self.closest_point(path)
        look_ahead_index = closest_index

        # Avança no caminho até encontrar um ponto a uma distância L - look ahead
        while look_ahead_index < len(path.cx):
            dx = path.cx[look_ahead_index] - self.x
            dy = path.cy[look_ahead_index] - self.y
            dist = np.hypot(dx, dy)
            if dist >= look_ahead_distance:
                break
            look_ahead_index += 1

        if look_ahead_index >= len(path.cx):
            look_ahead_index = len(path.cx) - 1

        return look_ahead_index
        
    def drive_robot(self, path, look_ahead_distance):   
        """Calcula os comandos de controle usando Pure Pursuit."""
        goal_index = self.goal_point(path, look_ahead_distance)
        goal_x = path.cx[goal_index]
        goal_y = path.cy[goal_index]

        # Transformação para o frame do robô
        dx = goal_x - self.x
        dy = goal_y - self.y

        # Rotação por -theta
        localP = Rz2D(np.array([[dx], [dy], [1.0]]), -self.theta)
            
        curvature = (2.0 * localP[1, 0]) / (look_ahead_distance**2)
        omega = curvature * self.vl

        return omega

    def update_pose(self, omega, dt):
        """Atualiza a pose do robô usando cinemática diferencial."""
        self.x += self.vl * np.cos(self.theta) * dt
        self.y += self.vl * np.sin(self.theta) * dt
        self.theta += omega * dt

    def plot(self, ax, color="red"):
        """
        Desenha o robô no frame global.
        """
        body = np.array([
            [100.0, 227.5, 227.5, 100.0, -200.0, -227.5, -227.5, -200.0],
            [-190.5, -50.0, 50.0, 190.5, 190.5, 163.0, -163.0, -190.5],
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
            ])
        body[:2, :] /= 1000.0

        left_wheel = np.array([
            [ 97.5, 97.5, -97.5, -97.5],
            [170.5, 210.5, 210.5, 170.5],
                [ 1.0, 1.0, 1.0, 1.0]
                ])
        left_wheel[:2, :] /= 1000.0

        right_wheel = np.array([
            [ 97.5, 97.5, -97.5, -97.5],
            [-170.5, -210.5, -210.5, -170.5],
            [ 1.0, 1.0, 1.0, 1.0]
            ])
        right_wheel[:2, :] /= 1000.0

        robot_c = T2D(Rz2D(body, self.theta), self.x, self.y)
        robot_e = T2D(Rz2D(left_wheel, self.theta), self.x, self.y)
        robot_d = T2D(Rz2D(right_wheel, self.theta), self.x, self.y)
        ax.fill(robot_c[0, :], robot_c[1, :], color, edgecolor="k", alpha=0.7)
        ax.fill(robot_e[0, :], robot_e[1, :], color, edgecolor="k", alpha=0.7)
        ax.fill(robot_d[0, :], robot_d[1, :], color, edgecolor="k", alpha=0.7)
        ax.plot(self.x, self.y, "or", markersize=6)
        ax.plot([self.x, self.x + 0.12 * np.cos(self.theta)],
        [self.y, self.y + 0.12 * np.sin(self.theta)],
        "r", linewidth=2)

class pathCreator:
    """
    Classe para definir o caminho a ser seguido pelo robô.
    """

    def __init__(self, x0, xf, y0, yf):
        # Ponto inicial do caminho no frame global
        self.x0 = x0
        self.y0 = y0
        
        # Ponto final do caminho no frame global
        self.xf = xf
        self.yf = yf
        
        self.cx = None
        self.cy = None

    def create_line_path(self, steps=100):
        """Cria um caminho retilíneo entre o ponto inicial e o final."""
        self.cx = np.linspace(self.x0, self.xf, steps)
        self.cy = np.linspace(self.y0, self.yf, steps)
        return self.cx, self.cy 
    
    def create_circular_path(self, steps=100):
        """Cria um arco circular entre o ponto inicial e o ponto final."""
        # Deslocamento
        dx = self.xf - self.x0
        dy = self.yf - self.y0

        # Circunferência
        r = (dx**2 + dy**2) / (2 * dx) # Raio
        cnt_x =  self.x0 + r # Centro x
        cnt_y = self.y0     # Centro y

        # Angulos inicial e final do arco
        theta_i = np.pi
        theta_f = np.arctan2(self.yf - cnt_y, self.xf - cnt_x)

        if theta_f > theta_i:
            theta_f -= 2 * np.pi

        # Gerar os pontos do arco
        angles = np.linspace(theta_i, theta_f, steps) 
        self.cx = cnt_x + r * np.cos(angles)
        self.cy = cnt_y + r * np.sin(angles)

        return self.cx, self.cy

    def create_custom_path(self, points):
        """Cria um caminho personalizado a partir de uma lista de pontos."""
        self.cx = np.asarray([p[0] for p in points], dtype=float)
        self.cy = np.asarray([p[1] for p in points], dtype=float)
        return self.cx, self.cy

def simulate(robots, path, dt=0.1, look_ahead_distance=0.8, max_steps=2000):
    """Executa a simulação do Pure Pursuit para múltiplos robôs"""
    trajectories = []
    
    for robot in robots:
        history_x = [robot.x]
        history_y = [robot.y]
        history_theta = [robot.theta]
        history_steer = []

        for _ in range(max_steps):
            omega = robot.drive_robot(path, look_ahead_distance)
            robot.update_pose(omega, dt)

            history_x.append(robot.x)
            history_y.append(robot.y)
            history_theta.append(robot.theta)
            history_steer.append(omega)

            goal_x = path.cx[-1]
            goal_y = path.cy[-1]

            if np.hypot(robot.x - goal_x, robot.y - goal_y) <= 0.1:
                break

        trajectories.append({
            "x": np.asarray(history_x),
            "y": np.asarray(history_y),
            "theta": np.asarray(history_theta),
            "steer": np.asarray(history_steer),
        })
        
    return trajectories

def animate_simulation(path, trajectories, robots, colors, look_ahead_distance=0.5, interval=50):
    """Cria uma animação das trajetórias de múltiplos robôs seguindo o caminho."""
    fig, ax = plt.subplots()
    
    all_x = np.concatenate([path.cx] + [traj["x"] for traj in trajectories])
    all_y = np.concatenate([path.cy] + [traj["y"] for traj in trajectories])
    
    x_min, x_max = np.min(all_x) - 0.5, np.max(all_x) + 0.5
    y_min, y_max = np.min(all_y) - 0.5, np.max(all_y) + 0.5

    max_frames = max(len(traj["x"]) for traj in trajectories)

    def update(frame):
        ax.cla()
        ax.plot(path.cx, path.cy, "--", color="gray", label="Caminho Alvo")

        for i, (robot, traj, color) in enumerate(zip(robots, trajectories, colors)):
            idx = min(frame, len(traj["x"]) - 1)
            
            ax.plot(traj["x"][: idx + 1], traj["y"][: idx + 1], color=color, alpha=0.5, label=f"Robô {i+1}")

            robot.x = traj["x"][idx]
            robot.y = traj["y"][idx]
            robot.theta = traj["theta"][idx]
            robot.plot(ax, color=color)

            goal_index = robot.goal_point(path, look_ahead_distance)
            label = "Waypoint" if i == 0 else None
            ax.plot(path.cx[goal_index], path.cy[goal_index], "x", color=color, markersize=6, label=label)

        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.grid(True)
        ax.legend(loc="best")

    animation = FuncAnimation(
        fig,
        update,
        frames=max_frames,
        interval=interval,
        repeat=False,
    )

    plt.show()
    return animation

def plot_tracking_error(path, trajectories, dt=0.1):
    """Plota o erro de rastreamento ao longo do tempo"""
    fig, ax = plt.subplots()

    for i, traj in enumerate(trajectories):
        traj_x = traj["x"]
        traj_y = traj["y"]
        errors = []

        for x, y in zip(traj_x, traj_y):
            dx = path.cx - x
            dy = path.cy - y
            errors.append(np.min(np.hypot(dx, dy)))

        time = np.arange(len(errors)) * dt
        ax.plot(time, errors, label=f"Robô {i+1}")

    ax.set_xlabel("Tempo [s]")
    ax.set_ylabel("Erro [m]")
    ax.grid(True)
    ax.legend(loc="best")
    plt.show()
    

path = pathCreator(x0=0.0, xf=10.0, y0=0.0, yf=4.0)
path.create_circular_path(steps=100)

robots = [
    differentialRobot(x0=0.0, y0=0.0, theta0=0.0, vl=1),
    #differentialRobot(x0=2.0, y0=6.0, theta0=-np.pi/4, vl=0.5),
    #differentialRobot(x0=2.0, y0=0.0, theta0=np.pi/2, vl=0.3),
    #differentialRobot(x0=3.0, y0=7.0, theta0=np.pi/2, vl=1)   
]

colors = [
          "red", 
          "blue", 
          "green", 
          #"orange"
          ]

trajectories = simulate(robots, path, dt=0.1, look_ahead_distance=.7)
animate_simulation(path, trajectories, robots, colors, look_ahead_distance=.7)
plot_tracking_error(path, trajectories, dt=0.1)



