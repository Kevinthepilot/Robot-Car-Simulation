from movement import RobotMovement

class Algo:
    def __init__(self):
        self.arr = []
        self.filename = "testcase.txt"
        
        self.movePath = []
        
    def readTest(self):
        with open(self.filename, "r") as file:
            lines = file.readlines()
       
        header = lines[0].strip().split()
        rows, cols = map(int, header)
        
        for line in lines[1:]:
            row = list(map(int, line.strip().split()))
            self.arr.append(row)
        
        for row in self.arr: print(row)
    
    
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
            
    
    def bfs(self):
        xd = [0, 0, 1, -1]
        yd = [1, -1, 0, 0]
        dirChar = ['R', 'L', 'D', 'U']
    
        rows = len(self.arr)
        cols = len(self.arr[0])
    
        # Find start
        startX = startY = None
        for i, row in enumerate(self.arr):
            for j, val in enumerate(row):
                if val == -1:
                    startX, startY = i, j
                    break
            if startX is not None:
                break
    
        # Proper matrix initialization
        visited = [[False for _ in range(cols)] for _ in range(rows)]
        count = [[0 for _ in range(cols)] for _ in range(rows)]
        parent = [[None for _ in range(cols)] for _ in range(rows)]
        direction = [['' for _ in range(cols)] for _ in range(rows)]
    
        queue = [[startX, startY]]
        visited[startX][startY] = True
    
        while queue:
            x, y = queue.pop(0)
            if self.arr[x][y] == 2:
                path = []
                while (x, y) != (startX, startY):
                    path.append(direction[x][y])
                    x, y = parent[x][y]
                path.reverse()
                self.movePath = path
                return
    
            for i in range(4):
                newX = x + xd[i]
                newY = y + yd[i]
                if 0 <= newX < rows and 0 <= newY < cols and not visited[newX][newY] and self.arr[newX][newY] != 1:
                    visited[newX][newY] = True
                    count[newX][newY] = count[x][y] + 1
                    queue.append([newX, newY])
                    parent[newX][newY] = (x, y)
                    direction[newX][newY] = dirChar[i]
        
        print("No path to destination.")
        
        
        