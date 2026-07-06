import numpy as np
import matplotlib.pyplot as plt
from transforms2d import Pose2D
from matplotlib.animation import FuncAnimation  

DT = 0.05                                          # durée d'un pas de temps (s)
POSE_CAPTEUR_ROBOT = Pose2D(0.5, 0.0, np.pi/6)     # calibration, fixée une fois
BALISE_MONDE = np.array([1.0, -2.0])


def draw_frame(ax, pose, label, scale=0.5):
    """Dessine un repère : flèche x (rouge), flèche y (verte), label."""
    origine = np.array([pose.x, pose.y])
    direction_x = pose.matrix[:2, 0]     # colonne 0 du bloc rotation = axe x du repère
    direction_y = pose.matrix[:2, 1]     # colonne 1 = axe y
    ax.arrow(*origine, *(scale * direction_x), head_width=0.08, color="red")
    ax.arrow(*origine, *(scale * direction_y), head_width=0.08, color="green")
    ax.text(pose.x + 0.1, pose.y + 0.1, label)

def pose_robot_at(t):
    angle = 0.5 * t
    x = 3.0 * np.cos(angle)
    y = 3.0 * np.sin(angle)
    theta = angle + np.pi/2

    return Pose2D(x, y, theta)


def update(frame):
    """Dessine l'image numéro `frame`. Appelée par FuncAnimation."""
    t = frame * DT                                  # numéro d'image -> temps simulé

    ax.clear()                                      # efface TOUT (dessins ET réglages !)
    ax.set_aspect("equal")                          # ... donc on re-règle la scène
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.grid(True)

    pose_robot_monde = pose_robot_at(t)
    pose_capteur_monde = pose_robot_monde.compose(POSE_CAPTEUR_ROBOT)

    mesure = pose_capteur_monde.inverse().transform_point(BALISE_MONDE)
    point_robot = POSE_CAPTEUR_ROBOT.transform_point(mesure)           
    reprojection = pose_robot_monde.transform_point(point_robot)

    ax.plot(*BALISE_MONDE, "k+", markersize=12)                           
    ax.plot(*reprojection, "bo", markersize=5)

    draw_frame(ax, Pose2D(0.0, 0.0, 0.0), "monde")
    draw_frame(ax, pose_robot_monde, "robot")
    draw_frame(ax, pose_capteur_monde, "capteur", scale=0.3)    



if __name__ == "__main__":
    fig, ax = plt.subplots()
    anim = FuncAnimation(fig, update, frames=200, interval=50)
    anim.save("demo.gif", writer="pillow", fps=20)
    
    plt.show()


