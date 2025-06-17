class Edge:
    def __init__(self, V1, V2, l, circle=None):
        self.V1 = V1
        self.V2 = V2
        self.l = l
        self.circle = circle

class Vertex:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.edges = []
        self.cost = float('inf')
        self.previous = None

    def append_edge(self, edge):
        self.edges.append(edge)

class Circle:
    def __init__(self,x,y,R, virtual,first=False, clockwise=None):
        self.x = x
        self.y = y
        self.R = R
        self.circle_vertexes=[]
        self.virtual = virtual
        self.first = first
        self.clockwise = clockwise # true если движемся только по часовой


    def append_vertex(self, vertex):
        self.circle_vertexes.append(vertex)