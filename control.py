from math import sin, cos, atan2, acos, pi, tan, sqrt
import threading
import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
import meshcat.transformations as tf
import termios
import tty

# ---------------------------------------------
import time
from dynamixel_sdk import *

enable_real = True
enable_print = False

if enable_real:
    portHandler = PortHandler("/dev/ttyUSB1")
    packetHandler = PacketHandler(1.0)

    portHandler.openPort()
    portHandler.setBaudRate(1000000)

    ADDR_GOAL_POSITION = 30

    ids = []
    for id in range(0, 100):
        model_number, result, error = packetHandler.ping(portHandler, id)
        if model_number == 12:
            print(f"Found AX-12 with id: {id}")
            ids.append(id)
    print("Found motors: {}".format(ids))
    for id in ids:
        packetHandler.write2ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, 512)
# ---------------------------------------------

# Dimensions (mm)
l1, l2, l3 = 0.045, 0.065, 0.087  # Utilisés pour direct()
xoffset_3_6 = 0.091
xoffset_1245 = 0.0305
yoffset_1245 = 0.0753

# Angle de repos de chaque patte 
alph1, alph2, alph3, alph4, alph5, alph6 = -pi / 2, -pi / 2, pi, pi / 2, pi / 2, 0

# Positions de repos pour chaque patte
idle_z = -0.08
idle_x_1245 = 0.07
idle_y_1245 = 0.18
idle_x_36 = 0.22
idles = [np.array([idle_x_1245, idle_y_1245, idle_z]),
         np.array([-idle_x_1245, idle_y_1245, idle_z]),
         np.array([-idle_x_36, 0, idle_z]),
         np.array([-idle_x_1245, -idle_y_1245, idle_z]),
         np.array([idle_x_1245, -idle_y_1245, idle_z]),
         np.array([idle_x_36, 0, idle_z])
         ]

air = np.array([0., 0., 0.02])
origin_to_idle_ground = (2 * 0.13 ** 2) ** .5
radius = 0.05
ground_time = 0.25


def map_angles(values, from_low=pi, from_high=-pi, to_low=0, to_high=1023):
    values = [value - 2 * bool((i - 1) % 3 <= 1) * value for i, value in
              enumerate(values)]  # Calcule les opposés des angles des moteurs 2 et 3 de chaque patte

    mapped_angles = []
    for value in values:
        mapped_angles += [int((value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low)]
    return mapped_angles


def send_angles(angles):
    d = {
        "31": 0,
        "32": 1,
        "1": 2,
        "21": 3,
        "22": 4,
        "23": 5,
        "11": 6,
        "12": 7,
        "13": 8,
        "61": 9,
        "62": 10,
        "63": 11,
        "51": 12,
        "52": 13,
        "53": 14,
        "41": 15,
        "42": 16,
        "43": 17
    }
    for id in ids:
        packetHandler.write2ByteTxOnly(portHandler, id, ADDR_GOAL_POSITION, angles[d[str(id)]])


def sandbox(t):
    """
    python simulator.py -m sandbox

    Un premier bac à sable pour faire des expériences

    La fonction reçoit le temps écoulé depuis le début (t) et retourne une position cible
    pour les angles des 12 moteurs

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """

    # Par exemple, on envoie un mouvement sinusoidal
    targets = [0] * 12
    targets[0] = np.sin(t)

    return targets


# Ne fonctionne certainement pas
def direct(alpha, beta, gamma):
    """
    python simulator.py -m direct

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument la cible (alpha, beta, gamma) des degrés de liberté de la patte, et produit
    la position (x, y, z) atteinte par le bout de la patte

    - Sliders: les angles des trois moteurs (alpha, beta, gamma)
    - Entrées: alpha, beta, gamma, la cible (radians) des moteurs
    - Sortie: un tableau contenant la position atteinte par le bout de la patte (en mètres)
    """
    xp = l1 + cos(beta) * l2 + cos(beta + gamma) * l3
    yp = sin(beta) * l2 + sin(beta + gamma) * l3

    x = cos(alpha) * xp
    y = sin(alpha) * xp
    z = yp
    return [x, y, z]


def inverse(x, y, z):
    """
    python simulator.py -m inverse

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument une position cible (x, y, z) pour le bout de la patte, et produit les angles
    (alpha, beta, gamma) pour que la patte atteigne cet objectif

    - Sliders: la position cible x, y, z du bout de la patte
    - Entrée: x, y, z, une position cible dans le repère de la patte (mètres), provenant du slider
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """
    l1x = 0.0558
    l1z = -0.016
    l2x = 0.068
    l2z = -0.0229
    l3x = 0.014
    l3z = -0.093
    l3h = sqrt(l2x ** 2 + l2z ** 2)
    l4h = sqrt(l3x ** 2 + l3z ** 2)

    alpha = -atan2(y, x)
    d = sqrt(x ** 2 + y ** 2)
    z -= l1z

    AC = sqrt((d - l1x) ** 2 + z ** 2)

    teta3 = atan2(z, d - l1x)
    beta = al_kashi(l4h, AC, l3h) + teta3
    gamma = pi - al_kashi(AC, l4h, l3h)

    return [alpha, beta - atan2(l2z, l2x), gamma + atan2(l3z, l3x)]


def al_kashi(a, b, c):
    return acos(min(1, max(-1, (b ** 2 + c ** 2 - a ** 2) / (2 * b * c))))


def inverse_hexapod(x, y, z):
    """
    python simulator.py -m inverse -r hexapod

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument une position cible (x, y, z) pour le bout de la patte, et produit les angles
    (alpha, beta, gamma) pour que la patte atteigne cet objectif

    - Sliders: la position cible x, y, z du bout de la patte
    - Entrée: x, y, z, une position cible dans le repère de la patte (mètres), provenant du slider
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """
    l2h = 49e-3
    l3h = 64.5e-3  # -60.5 0 22.5 par rapport à l2h
    l4h = 90.3e-3  # -10 -11.5 89 par rapport à l3h

    d = sqrt(x ** 2 + y ** 2)
    AC = sqrt((d - l2h) ** 2 + z ** 2)
    teta3 = atan2(z, d - l2h)

    alpha = atan2(y, x)
    beta = al_kashi(l4h, AC, l3h) + teta3
    gamma = pi - al_kashi(AC, l4h, l3h)
    return [alpha, beta, gamma]


def draw(t):
    """
    python simulator.py -m draw

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Le but est, à partir du temps donné, de suivre une trajectoire de turn_triangle. Pour ce faire, on
    utilisera une interpolation linéaire entre trois points, et la fonction inverse précédente.

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """
    pass


def walk_triangle(x_speed, y_speed):
    point = np.array([0., 0., 0.])
    if not np.isclose(x_speed + 1, 1):
        point[0] += x_speed * ground_time
    if not np.isclose(y_speed + 1, 1):
        point[1] += y_speed * ground_time
    return point, -point


def turn_triangle(turn_speed):
    point = np.array([0., 0., 0.])
    if not np.isclose(turn_speed + 1, 1):
        point[1] += tan(turn_speed * ground_time / 1) * origin_to_idle_ground

    return point, -point


def legs(targets_robot):
    """
    python simulator.py -m legs

    Le robot est figé en l'air, on contrôle toute les pattes

    - Sliders: les 12 coordonnées (x, y, z) du bout des 4 pattes
    - Entrée: des positions cibles (tuples (x, y, z)) pour le bout des 4 pattes
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """

    x1, y1, z1 = targets_robot[0]
    x2, y2, z2 = targets_robot[1]
    x3, y3, z3 = targets_robot[2]
    x4, y4, z4 = targets_robot[3]
    x5, y5, z5 = targets_robot[4]
    x6, y6, z6 = targets_robot[5]

    x1 -= xoffset_1245
    x2 += xoffset_1245
    x3 += xoffset_3_6
    x4 += xoffset_1245
    x5 -= xoffset_1245
    x6 -= xoffset_3_6

    y1 -= yoffset_1245
    y2 -= yoffset_1245
    y4 += yoffset_1245
    y5 += yoffset_1245

    target1 = [x1 * cos(alph1) - y1 * sin(alph1), x1 * sin(alph1) + y1 * cos(alph1), z1]
    target2 = [x2 * cos(alph2) - y2 * sin(alph2), x2 * sin(alph2) + y2 * cos(alph2), z2]
    target3 = [x3 * cos(alph3) - y3 * sin(alph3), x3 * sin(alph3) + y3 * cos(alph3), z3]
    target4 = [x4 * cos(alph4) - y4 * sin(alph4), x4 * sin(alph4) + y4 * cos(alph4), z4]
    target5 = [x5 * cos(alph5) - y5 * sin(alph5), x5 * sin(alph5) + y5 * cos(alph5), z5]
    target6 = [x6 * cos(alph6) - y6 * sin(alph6), x6 * sin(alph6) + y6 * cos(alph6), z6]

    return inverse(*target1) + inverse(*target2) + inverse(*target3) + inverse(*target4) + inverse(*target5) + inverse(
        *target6)


def walk(t, speed_x, speed_y, speed_rotation):
    """
    python simulator.py -m walk

    Le but est d'intégrer tout ce que nous avons vu ici pour faire marcher le robot

    - Sliders: speed_x, speed_y, speed_rotation, la vitesse cible du robot
    - Entrée: t, le temps (secondes écoulées depuis le début)
            speed_x, speed_y, et speed_rotation, vitesses cibles contrôlées par les sliders
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """
    # Trans is for walking, turn is for turning
    trans_point1, trans_point2 = walk_triangle(speed_x, speed_y)  #  Calculates the 2 ground points to reach in order to walk in the referential of one leg
    turn_point1, turn_point2 = turn_triangle(speed_rotation)  #  Calculates the 2 ground points to reach in order to turn in the referential of one leg
    trans_points, turn_points = [], []  # The ground points in order to walk and turn for one leg

    cur_t = 0
    trans_points.append([cur_t] + trans_point1.tolist())
    turn_points.append([cur_t] + turn_point1.tolist())

    cur_t += ground_time
    trans_points.append([cur_t] + trans_point2.tolist())
    turn_points.append([cur_t] + turn_point2.tolist())

    cur_t += ground_time / 2
    trans_points.append([cur_t] + air.tolist())
    turn_points.append([cur_t] + np.zeros(3).tolist())

    cur_t += ground_time / 2
    trans_points.append([cur_t] + trans_point1.tolist())
    turn_points.append([cur_t] + turn_point1.tolist())

    # Make the two groups out of phase (one group in the air while the other on the ground)
    group1_trans = interpolate3d(trans_points, t % (2 * ground_time))
    group2_trans = interpolate3d(trans_points, (t + ground_time) % (2 * ground_time))
    group1_turn = interpolate3d(turn_points, t % (2 * ground_time))
    group2_turn = interpolate3d(turn_points, (t + ground_time) % (2 * ground_time))

    # The turn points need to be rotated for each leg
    rot1 = np.array([[cos(alph1), sin(alph1), 0], [-sin(alph1), cos(alph1), 0], [0, 0, 0]]) @ group1_turn
    rot2 = np.array([[cos(alph2), sin(alph2), 0], [-sin(alph2), cos(alph2), 0], [0, 0, 0]]) @ group2_turn
    rot3 = np.array([[cos(alph3), sin(alph3), 0], [-sin(alph3), cos(alph3), 0], [0, 0, 0]]) @ group1_turn
    rot4 = np.array([[cos(alph4), sin(alph4), 0], [-sin(alph4), cos(alph4), 0], [0, 0, 0]]) @ group2_turn
    rot5 = np.array([[cos(alph5), sin(alph5), 0], [-sin(alph5), cos(alph5), 0], [0, 0, 0]]) @ group1_turn
    rot6 = np.array([[cos(alph6), sin(alph6), 0], [-sin(alph6), cos(alph6), 0], [0, 0, 0]]) @ group2_turn

    # Compute the angles given the points for each leg at time t
    angles = legs([group1_trans + idles[0] + rot1, group2_trans + idles[1] + rot2, group1_trans + idles[2] + rot3,
                   group2_trans + idles[3] + rot4, group1_trans + idles[4] + rot5, group2_trans + idles[5] + rot6])
    if enable_print:
        print(angles)
    mapped = map_angles(angles)
    if enable_real:
        send_angles(mapped)
    return angles


def interpolate(values, t):
    if t < values[0][0]:
        return values[0][1]
    if t > values[-1][0]:
        return values[-1][1]

    tmin = values[0][0]
    tnext = values[1][0]
    i = 1
    while values[i][0] < t and i < len(values) - 1:
        tmin = values[i][0]
        tnext = values[i + 1][0]
        i += 1
    return values[i - 1][1] + (t - tmin) * (values[i][1] - values[i - 1][1]) / (tnext - tmin)


def interpolate3d(values, t):
    X = [(e[0], e[1]) for e in values]
    Y = [(e[0], e[2]) for e in values]
    Z = [(e[0], e[3]) for e in values]
    return interpolate(X, t), interpolate(Y, t), interpolate(Z, t)


def detect_keypress():
    global key
    while True:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            key = sys.stdin.read(1)
            if key == 'z':
                walk(time.time(), 0.1, 0, 0)
            elif key == 's':
                walk(time.time(), -0.1, 0, 0)
            elif key == 'q':
                walk(time.time(), 0, 0.1, 0)
            elif key == 'd':
                walk(time.time(), 0, -0.1, 0)
            elif key == 'a':
                walk(time.time(), 0, 0, 0.2)
            elif key == 'e':
                walk(time.time(), 0, 0, -0.2)
            elif key == 'r':
                for id in ids:
                    packetHandler.write2ByteTxOnly(portHandler, id, ADDR_GOAL_POSITION, 512)
            elif key == 'x':
                break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    print("N'exécutez pas ce fichier, mais simulator.py")
    # if enable_real:
    #     while True:
    #         for id in ids: 
    #             packetHandler.write2ByteTxOnly(portHandler, id, ADDR_GOAL_POSITION, 512)
    #         time.sleep(0.1)
    # while True:
    #     walk(time.time(), 0, 0, -0.2)
    key = None
    threading.Thread(target=detect_keypress).start()
