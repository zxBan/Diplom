import numpy as np
import matplotlib.pyplot as plt
from structs import Circle
import math

def DrawCurve(edge, ax, color):
    p1 = (edge.V1.x, edge.V1.y)
    p2 = (edge.V2.x, edge.V2.y)
    center = (edge.circle.x, edge.circle.y)
    radius = np.linalg.norm(np.array(center) - np.array(p1))
    angle_p1 = np.arctan2(p1[1] - center[1], p1[0] - center[0])
    angle_p2 = np.arctan2(p2[1] - center[1], p2[0] - center[0])
    if angle_p1 < 0:
        angle_p1 = 2 * np.pi + angle_p1
    if angle_p2 < 0:
        angle_p2 = 2 * np.pi + angle_p2
    if angle_p1 > angle_p2:
        angles = np.linspace(angle_p2, angle_p1, 20)
    else:
        angles = np.linspace(angle_p1, angle_p2, 20)
    x_points = center[0] + radius * np.cos(angles)
    y_points = center[1] + radius * np.sin(angles)
    ax.plot(x_points, y_points, color=color)


def PointTangent(v, ax):  # отрисовка касательных
    for i in v.edges:
        if i.circle is not None:
            DrawCurve(i, ax, "black")
        else:
            ax.plot([i.V1.x, i.V2.x], [i.V1.y, i.V2.y], color=[0, 0, 0])

def Draw(vertexes, ax):  # отрисовка
    for v in vertexes:
        PointTangent(v, ax)


def DrawCircles(circles, ax, isShowVirtual=False):  # отрисовка окружностей
    for circle in circles:
        if circle.first:
            continue
        if not isShowVirtual and circle.virtual:
            continue
        if circle.virtual:
            color = "red"
        else:
            color = "blue"
        circle1 = plt.Circle((circle.x, circle.y), circle.R, fill=False, color=color)
        ax.add_patch(circle1)


def DrawWay(result, ax):  # отрисовка А*
    for edge in result:
        if edge.circle is not None:
            DrawCurve(edge, ax, "green")
            if edge.circle.virtual or edge.circle.first:
                circle1 = plt.Circle((edge.circle.x, edge.circle.y), edge.circle.R, fill=False, color="red")
                ax.add_patch(circle1)
        else:
            ax.plot([edge.V1.x, edge.V2.x], [edge.V1.y, edge.V2.y], color="green")
        ax.plot(edge.V1.x, edge.V1.y, 'o', color=[1, 0, 0])
        ax.plot(edge.V2.x, edge.V2.y, 'o', color=[1, 0, 0])

