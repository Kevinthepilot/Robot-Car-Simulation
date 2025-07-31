import heapq

def manhattan_distance(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)

class Algo:
    def __init__(self):
        self.arr = []
        self.filename = "map.txt"
        self.movePath = []

    def readTest(self):
        with open(self.filename, "r") as file:
            lines = file.readlines()

        header = lines[0].strip().split()
        rows, cols = map(int, header)

        for line in lines[1:]:
            row = list(map(float, line.strip().split()))
            self.arr.append(row)

        # for row in self.arr:
            # print(row)

    def translate(self, x, y):
        if x == y: return 'U'
        if (
            (x == 'U' and y == 'R') or
            (x == 'R' and y == 'D') or
            (x == 'D' and y == 'L') or
            (x == 'L' and y == 'U')
        ): return 'R'
        if (
            (x == 'U' and y == 'L') or
            (x == 'L' and y == 'D') or
            (x == 'D' and y == 'R') or
            (x == 'R' and y == 'U')
        ): return 'L'

    def a_star(self, start, end, k_value):
        xd = [0, 0, 1, -1]
        yd = [1, -1, 0, 0]
        dirChar = ['U', 'D', 'L', 'R']

        rows = len(self.arr)
        cols = len(self.arr[0])

        open_set = []
        heapq.heappush(open_set, (0 + manhattan_distance(*start, *end), 0, start))

        came_from = {}
        direction = {}

        g_score = [[float('inf')] * cols for _ in range(rows)]
        g_score[start[1]][start[0]] = 0

        while open_set:
            _, cost, (x, y) = heapq.heappop(open_set)

            if (x, y) == end:
                path = []
                while (x, y) != start:
                    path.append(direction[(x, y)])
                    x, y = came_from[(x, y)]
                path.reverse()
                self.movePath = path
                return

            for i in range(4):
                newX, newY = x + xd[i], y + yd[i]

                if 0 <= newX < rows and 0 <= newY < cols and self.arr[newY][newX] < k_value:
                    tentative_g = cost + 1
                    if tentative_g < g_score[newY][newX]:
                        g_score[newY][newX] = tentative_g
                        f_score = tentative_g + manhattan_distance(newX, newY, *end)
                        heapq.heappush(open_set, (f_score, tentative_g, (newX, newY)))
                        came_from[(newX, newY)] = (x, y)
                        direction[(newX, newY)] = dirChar[i]

        print("No path to destination.")
