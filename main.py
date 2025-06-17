import math
import numpy as np
import matplotlib.pyplot as plt
import pathfinder
import drawer
from structs import Vertex, Edge, Circle

xA = -100
yA = 190

xF = 350
yF = 180

x_circle = 25
y_circle = 200
R = 35

R_min = 40
al = 60


def ReserchWay(x, y, v1, v2):  # нужна при попадении отрезка на окружность
    return (v1.x <= x <= v2.x or v2.x <= x <= v1.x) and (v1.y <= y <= v2.y or v2.y <= y <= v1.y)


def Lenght(v1, v2):
    return math.sqrt((v1.x - v2.x) ** 2 + (v1.y - v2.y) ** 2)


def LenghtCicles(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def CreateVirtualCircles(circles):
    for circle in circles:
        if circle.R >= R_min or circle.virtual:
            pass
        else:
            phi = 0
            step = 2 * math.pi / 20
            while phi <= 2 * math.pi:
                v_circle = Circle(circle.x + (R_min - circle.R) * math.cos(phi),
                                  circle.y + (R_min - circle.R) * math.sin(phi), R_min, True)
                circles.append(v_circle)
                phi += step


def determine_direction(x, y, x2, y2, x3, y3):
    # Вектор от центра окружности к началу вектора
    vector1 = (x2 - x, y2 - y)

    # Вектор от центра окружности к концу вектора
    vector2 = (x3 - x, y3 - y)

    # Векторное произведение (cross product)
    cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0]

    if cross_product > 0:
        return False
    else:
        return True


def TangentStart(circles, v, vertexes):  # касательные для  начальной окружности к окружности с учетом поворота
    for circle1 in circles:
        if circle1.first:
            for circle2 in circles:
                if not circle2.first:
                    b = circle1.R + circle2.R
                    d = LenghtCicles(circle1.x, circle1.y, circle2.x, circle2.y)
                    try:
                        tetha = math.acos(b / d)  # угол между прямой ,соединяющая центры,и касательной
                    except Exception as E:
                        pass

                    alpha = math.atan2(circle2.y - circle1.y,
                                       circle2.x - circle1.x)  # угол,на которое происходит смещение относительно горизотальнгой линии центра одного из окружностей.
                    # координаты точки касания для 1 окружности
                    xT1 = circle1.x + circle1.R * math.cos(tetha + alpha)
                    yT1 = circle1.y + circle1.R * math.sin(tetha + alpha)

                    xT2 = circle1.x + circle1.R * math.cos(-tetha + alpha)
                    yT2 = circle1.y + circle1.R * math.sin(-tetha + alpha)

                    alpha = math.atan2(circle1.y - circle2.y, circle1.x - circle2.x)
                    x1 = circle2.x + circle2.R * math.cos(tetha + alpha)
                    y1 = circle2.y + circle2.R * math.sin(tetha + alpha)

                    x2 = circle2.x + circle2.R * math.cos(-tetha + alpha)
                    y2 = circle2.y + circle2.R * math.sin(-tetha + alpha)

                    V1 = Vertex(xT1, yT1)
                    V2 = Vertex(xT2, yT2)
                    V3 = Vertex(x1, y1)
                    V4 = Vertex(x2, y2)

                    if determine_direction(circle1.x, circle1.y, V1.x, V1.y, V3.x,
                                           V3.y) == circle1.clockwise:
                        if Point(V1, V3, circles, circle1, circle2):
                            circle1.append_vertex(V1)
                            circle2.append_vertex(V3)
                            vertexes.append(V1)
                            vertexes.append(V3)
                            # построение дуги
                            alpha = ReserchAlpha(circle1.x, v.x, V1.x, circle1.y, v.y, V1.y)
                            L = (alpha / 180) * math.pi * circle1.R
                            E = Edge(v, V1, L, circle1)
                            V1.append_edge(E)
                            v.append_edge(E)

                        if Point(V2, V4, circles, circle1, circle2):
                            circle1.append_vertex(V2)
                            circle2.append_vertex(V4)
                            vertexes.append(V2)
                            vertexes.append(V4)
                            alpha = ReserchAlpha(circle1.x, v.x, V2.x, circle1.y, v.y, V2.y)
                            L = (alpha / 180) * math.pi * circle1.R
                            E = Edge(v, V2, L, circle1)
                            V2.append_edge(E)
                            v.append_edge(E)

                    b1 = abs(circle1.R - circle2.R)
                    tetha1 = math.acos(b1 / d)
                    alpha = math.atan2(circle2.y - circle1.y,
                                       circle2.x - circle1.x)  # угол,на которое происходит смещение относительно горизотальнгой линии центра одного из окружностей.
                    # координаты точки касания для 1 окружности
                    xT1 = circle1.x + circle1.R * math.cos(tetha1 + alpha)
                    yT1 = circle1.y + circle1.R * math.sin(tetha1 + alpha)

                    xT2 = circle1.x + circle1.R * math.cos(-tetha1 + alpha)
                    yT2 = circle1.y + circle1.R * math.sin(-tetha1 + alpha)

                    alpha = math.atan2(circle1.y - circle2.y, circle1.x - circle2.x)
                    x1 = circle2.x + circle2.R * math.cos(tetha1 + alpha)
                    y1 = circle2.y + circle2.R * math.sin(tetha1 + alpha)

                    x2 = circle2.x + circle2.R * math.cos(-tetha1 + alpha)
                    y2 = circle2.y + circle2.R * math.sin(-tetha1 + alpha)

                    V1 = Vertex(xT1, yT1)
                    V2 = Vertex(xT2, yT2)
                    V3 = Vertex(x1, y1)
                    V4 = Vertex(x2, y2)

                    if determine_direction(circle1.x, circle1.y, V1.x, V1.y, V3.x,
                                           V3.y) == circle1.clockwise:
                        if Point(V1, V4, circles, circle1, circle2):
                            circle1.append_vertex(V1)
                            circle2.append_vertex(V4)
                            vertexes.append(V1)
                            vertexes.append(V4)
                            # построение дуги
                            alpha = ReserchAlpha(circle1.x, v.x, V1.x, circle1.y, v.y, V1.y)
                            L = (alpha / 180) * math.pi * circle1.R
                            E = Edge(v, V1, L, circle1)
                            V1.append_edge(E)
                            v.append_edge(E)

                        if Point(V2, V3, circles, circle1, circle2):
                            circle1.append_vertex(V2)
                            circle2.append_vertex(V3)
                            vertexes.append(V2)
                            vertexes.append(V3)
                            # построение дуги
                            alpha = ReserchAlpha(circle1.x, v.x, V2.x, circle1.y, v.y, V2.y)
                            L = (alpha / 180) * math.pi * circle1.R
                            E = Edge(v, V2, L, circle1)
                            V2.append_edge(E)
                            v.append_edge(E)


def IntersectionCircle(circles):
    for i in range(0, len(circles)):
        for j in range(i + 1, len(circles)):
            d = math.sqrt((circles[j].x - circles[i].x) ** 2 + (circles[j].y - circles[i].y) ** 2)
            if d > circles[i].R + circles[j].R:  # не пересекаются
                TangentCircle(circles[i], circles[j], vertexes, circles)
            else:
                pass


def TangentFinish(circles, v, vertexes):  # касательные для конечной точки и окружности
    for circle in circles:
        if circle.R < R_min:
            continue
        b = math.sqrt((v.x - circle.x) ** 2 + (v.y - circle.y) ** 2)
        th = math.acos(circle.R / b)
        d = math.atan2(v.y - circle.y, v.x - circle.x)
        d1 = d + th
        d2 = d - th
        T1x = circle.x + circle.R * math.cos(d1)
        T1y = circle.y + circle.R * math.sin(d1)
        T2x = circle.x + circle.R * math.cos(d2)
        T2y = circle.y + circle.R * math.sin(d2)

        V1 = Vertex(T1x, T1y)
        V2 = Vertex(T2x, T2y)
        if Point(v, V1, circles, circle):
            vertexes.append(V1)
            circle.append_vertex(V1)

        if Point(v, V2, circles, circle):
            vertexes.append(V2)
            circle.append_vertex(V2)


def TangentCircle(circle1, circle2, vertexes, circles):  # касательные для окружностей
    if circle1.first or circle2.first:
        return
    b = circle1.R + circle2.R
    d = LenghtCicles(circle1.x, circle1.y, circle2.x, circle2.y)
    tetha = math.acos(b / d)  # угол между прямой ,соединяющая центры,и касательной

    alpha = math.atan2(circle2.y - circle1.y,
                       circle2.x - circle1.x)  # угол,на которое происходит смещение относительно горизотальнгой линии центра одного из окружностей.
    # координаты точки касания для 1 окружности
    xT1 = circle1.x + circle1.R * math.cos(tetha + alpha)
    yT1 = circle1.y + circle1.R * math.sin(tetha + alpha)

    xT2 = circle1.x + circle1.R * math.cos(-tetha + alpha)
    yT2 = circle1.y + circle1.R * math.sin(-tetha + alpha)

    alpha = math.atan2(circle1.y - circle2.y, circle1.x - circle2.x)
    x1 = circle2.x + circle2.R * math.cos(tetha + alpha)
    y1 = circle2.y + circle2.R * math.sin(tetha + alpha)

    x2 = circle2.x + circle2.R * math.cos(-tetha + alpha)
    y2 = circle2.y + circle2.R * math.sin(-tetha + alpha)

    V1 = Vertex(xT1, yT1)
    V2 = Vertex(xT2, yT2)
    V3 = Vertex(x1, y1)
    V4 = Vertex(x2, y2)

    if Point(V1, V3, circles, circle1, circle2):
        circle1.append_vertex(V1)
        circle2.append_vertex(V3)
        vertexes.append(V1)
        vertexes.append(V3)

    if Point(V2, V4, circles, circle1, circle2):
        circle1.append_vertex(V2)
        circle2.append_vertex(V4)
        vertexes.append(V2)
        vertexes.append(V4)

    b1 = abs(circle1.R - circle2.R)
    tetha1 = math.acos(b1 / d)
    alpha = math.atan2(circle2.y - circle1.y,
                       circle2.x - circle1.x)  # угол,на которое происходит смещение относительно горизотальнгой линии центра одного из окружностей.
    # координаты точки касания для 1 окружности
    xT1 = circle1.x + circle1.R * math.cos(tetha1 + alpha)
    yT1 = circle1.y + circle1.R * math.sin(tetha1 + alpha)

    xT2 = circle1.x + circle1.R * math.cos(-tetha1 + alpha)
    yT2 = circle1.y + circle1.R * math.sin(-tetha1 + alpha)

    alpha = math.atan2(circle1.y - circle2.y, circle1.x - circle2.x)
    x1 = circle2.x + circle2.R * math.cos(tetha1 + alpha)
    y1 = circle2.y + circle2.R * math.sin(tetha1 + alpha)

    x2 = circle2.x + circle2.R * math.cos(-tetha1 + alpha)
    y2 = circle2.y + circle2.R * math.sin(-tetha1 + alpha)

    V1 = Vertex(xT1, yT1)
    V2 = Vertex(xT2, yT2)
    V3 = Vertex(x1, y1)
    V4 = Vertex(x2, y2)
    if Point(V1, V4, circles, circle1, circle2):
        circle1.append_vertex(V1)
        circle2.append_vertex(V4)
        vertexes.append(V1)
        vertexes.append(V4)

    if Point(V2, V3, circles, circle1, circle2):
        circle1.append_vertex(V2)
        circle2.append_vertex(V3)
        vertexes.append(V2)
        vertexes.append(V3)


def CircleT(circles, vertexes):
    for i in range(0, len(circles)):
        for j in range(i + 1, len(circles)):
            TangentCircle(circles[i], circles[j], vertexes, circles)


def ReserchAlpha(x1, x2, x3, y1, y2, y3):  # нахождение угла на окружности по 3м точкам
    a = ((x2 - x1) * (x3 - x1) + (y2 - y1) * (y3 - y1))
    b1 = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    b2 = math.sqrt((x3 - x1) ** 2 + (y3 - y1) ** 2)
    return math.acos(a / (b1 * b2))


def Curve(circles):  # кривые на окружности
    for circle in circles:
        if circle.first:
            continue
        if circle.R < R_min:
            continue
        for i in range(0, len(circle.circle_vertexes)):
            for j in range(i + 1, len(circle.circle_vertexes)):
                v1 = circle.circle_vertexes[i]
                v2 = circle.circle_vertexes[j]
                try:
                    alpha = ReserchAlpha(circle.x, v1.x, v2.x, circle.y, v1.y, v2.y)
                    L = (alpha / 180) * math.pi * circle.R
                    E = Edge(v1, v2, L, circle)
                    v1.append_edge(E)
                    v2.append_edge(E)
                except Exception as E:
                    pass


def Point(v1, v2, circles, circle1=None, circle2=None):  # прямая для 2х точек
    s = True
    for i in circles:
        if i == circle1 or i == circle2 or i.virtual:
            continue
        a = i.x
        b = i.y
        if (abs(v1.x - v2.x) < 0.00001):
            k = 10000000000
            c = 10000000000
        else:
            k = (v1.y - v2.y) / (v1.x - v2.x)
            c = (v1.x * v2.y - v2.x * v1.y) / (v1.x - v2.x)
        f = -8 * a * k * c + 8 * a * k * b - 4 * (c ** 2) - 4 * (b ** 2) + 8 * c * b + 4 * (i.R ** 2) - 4 * (k ** 2) * (
                a ** 2) + 4 * (k ** 2) * (i.R ** 2)
        if f >= 0:
            x1 = (2 * (a - k * c + k * b) + math.sqrt(f)) / (2 * (1 + k ** 2))
            x2 = (2 * (a - k * c + k * b) - math.sqrt(f)) / (2 * (1 + k ** 2))
            y1 = k * x1 + c
            y2 = k * x2 + c
            if ReserchWay(x1, y1, v1, v2) or ReserchWay(x2, y2, v1, v2):
                s = False
                break
    if s is True:
        L = Lenght(v1, v2)
        E = Edge(v1, v2, L)
        v1.append_edge(E)
        v2.append_edge(E)
    return (s)


A = Vertex(xA, yA)
F = Vertex(xF, yF)
Circle0_1 = Circle((A.x + R_min * math.cos(((90 + al) * math.pi) / 180)), (A.y + R_min * math.sin(((90 + al) * math.pi) / 180)),
                   R_min, False, first=True, clockwise=False)
Circle0_2 = Circle((A.x - R_min * math.cos(((90 + al) * math.pi) / 180)), (A.y - R_min * math.sin(((90 + al) * math.pi) / 180)),
                   R_min, False, first=True, clockwise=True)
Circle1 = Circle(x_circle, y_circle, R, False)
Circle2 = Circle(40, 230, 40, False)
Circle3 = Circle(160, 175, 60, False)
Circle4 = Circle(290, 225, 55, False)
circles = [Circle0_1, Circle0_2, Circle2, Circle3, Circle4]

vertexes = [A, F]
CreateVirtualCircles(circles)
TangentStart(circles, A, vertexes)  # касательные от старта
TangentFinish(circles, F, vertexes)  # касательные к финишу
IntersectionCircle(circles)
Curve(circles)  # дуги
Point(A, F, circles)  # прямая между стартом и финишем
Result_Vertexes, Result_Edges = pathfinder.A_star(A, F)  # поиск пути

alpha = (al * math.pi) / 180
x2 = A.x + 20 * math.cos(alpha)  # стрелка
y2 = A.y + 20 * math.sin(alpha)

fig, ax = plt.subplots(figsize=(10, 10))
ax = plt.gca()
ax.set_aspect('equal')
drawer.DrawCircles(circles, ax)
a = ax.plot(A.x, A.y, 'o', color=[0, 0, 0])[0]
f = ax.plot(F.x, F.y, 'o', color=[0, 0, 0])[0]
ax.plot([A.x, x2], [A.y, y2], color=[0, 0, 0])
drawer.Draw(vertexes, ax)
drawer.DrawWay(Result_Edges, ax)
plt.show()
