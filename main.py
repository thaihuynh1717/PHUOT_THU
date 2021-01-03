import heapq

class Graph: 
    def __init__(self,N, M, A): 
        self.A = A
        self.N = N
        self.M = M
    # CACH VI TRI DI CHUYEN CUA PHUOT THU
    def neighbors(self, cur):
        curX , curY = cur
        l = []
        heapq.heappush(l, (curX-1, curY))
        heapq.heappush(l, (curX+1, curY))
        heapq.heappush(l, (curX, curY-1))
        heapq.heappush(l, (curX, curY+1))
        return l
    # IN CAC DIEM DA DEN
    def printWay(self, came_from):
        p = {}
        for p in self.came_from: p = p 
        l=[[p[0],p[1]]]
        while(p != (0, 0)):   
            x, y = self.came_from[p]
            l.insert(0, [x, y])
            p=self.came_from[p]
        for i in range(self.M):
            arrow = ""
            for j in range(self.M):
                if self.A[i][j] == 1 :
                    if [i, j] in l: arrow+="X "
                    if [i, j] not in l : arrow +="O "
                if self.A[i][j]==2: arrow+="G "
                if self.A[i][j]==0: arrow+="# "
            print(arrow)
        print()

            
    # KIEM TRA TOA DO
    def checkMap(self, next_nodeX, next_nodeY):
        if(next_nodeX in range(self.M) and next_nodeY in range(self.M)): 
            return True
        return False
    # THUAT TOAN BFS
    def BFS(self, startX, startY, goalX, goalY):
        stack = []
        heapq.heappush(stack, (startX, startY))
        #
        visited=[]
        for i in range(self.M): visited.append([False]*self.M)
        visited[startX][startY]=True
        #
        self.came_from = {}
        self.gas = self.N
        #
        while (len(stack)>0 and self.gas > 0):
            cur=heapq.heappop(stack)
            self.gas-=1
            curX, curY = cur
            if self.A[curX][curY] == 2: 
                self.gas=self.N
            for next_node in self.neighbors(cur) :
                nextX, nextY = next_node
                if(nextX in range(self.M) and nextY in range(self.M)):
                    if self.A[nextX][nextY] > 0:
                        if nextX==goalX and nextY==goalY:
                            print("Da tim thay")
                            self.came_from[next_node]=cur
                            self.printWay(self.came_from)
                            return True
                        elif  visited[nextX][nextY] is False:
                            self.came_from[next_node]=cur
                            stack.append(next_node)
                            visited[nextX][nextY]=True
        self.printWay(self.came_from)    
        print("Không tìm thấy")
        return False
    # THUAT TOAN DFS
    def DFS(self, startX, startY, goalX, goalY):
        stack = []
        heapq.heappush(stack, (startX, startY))
        #
        visited=[]
        for i in range(self.M): visited.append([False]*self.M)
        visited[startX][startY]=True
        #
        self.came_from = {}
        self.gas = self.N
        #
        while (len(stack)>0 and self.gas > 0):
            cur=stack.pop(len(stack)-1)
            self.gas-=1
            curX, curY = cur
            if self.A[curX][curY] == 2: 
                self.gas=self.N
            for next_node in self.neighbors(cur) :
                nextX, nextY = next_node
                if(nextX in range(self.M) and nextY in range(self.M)):
                    if self.A[nextX][nextY] > 0:
                        if nextX==goalX and nextY==goalY:
                            print("Da tim thay")
                            self.came_from[next_node]=cur
                            self.printWay(self.came_from)
                            return True
                        elif  visited[nextX][nextY] is False:
                            self.came_from[next_node]=cur
                            stack.append(next_node)
                            visited[nextX][nextY]=True
        self.printWay(self.came_from)    
        print("Không tìm thấy")
        return False
    # THUAT TOAN UCS
    def UCS(self, startX, startY, goalX, goalY):
        queue=[]
        heapq.heappush(queue, (0,  (startX, startY)))
        cost_so_far = {(startX,startY): 0}
        self.came_from = {}
        self.gas=self.N
        next_node = (startX, startY)
        while (len(queue)>0 and self.gas>0):
            cur = heapq.heappop(queue)
            self.gas-=1
            cur_cost, cur_node = cur
            if cur_node == (goalX, goalY):
                print(f"Da tim thay duong di voi chi phi: {cost_so_far[cur_node]}")
                self.came_from[next_node]=cur_node
                self.printWay(self.came_from)
                return True
            for next_node in self.neighbors(cur_node):
                nextX, nextY = next_node
                if(nextX in range(self.M) and nextY in range(self.M)):
                    if self.A[nextX][nextY] > 0:
                        new_cost = cur_cost + 1
                        if (next_node not in cost_so_far) or (new_cost < cost_so_far[next_node]):
                            if self.A[next_node[0]][next_node[1]] == 2:
                                self.gas = self.N
                            heapq.heappush(queue, (new_cost, next_node))
                            cost_so_far[next_node]=new_cost
                            self.came_from[next_node]=cur_node
        self.printWay(self.came_from)    
        print("Không tìm thấy")
        return False

    def heuristic(self, p1, p2, heu_type ="Manhattan"):
        if heu_type == "Manhattan":
            return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])
        elif heu_type == "Euclidean":
            # TODO: pass
            return math.sqrt(math.pow(p2[0]-p1[0],2)+ math.pow(p2[1]-p1[1]),2)
            pass
        return sys.maxsize # return maximun number

    def Astar(self, startX, startY, goalX, goalY):
        queue=[]
        close_list={(startX, startY): 0}
        f = self.heuristic((startX, startY), (goalX, goalY))
        heapq.heappush(queue, (f,  (startX, startY)))
        self.came_from = {}
        self.gas=self.N

        while (len(queue)>0 and self.gas>0):
            cur = heapq.heappop(queue)
            self.gas-=1
            cur_cost, cur_node = cur
            if cur_node == (goalX, goalY):
                self.came_from[next_node]=cur_node
                self.printWay(self.came_from)
                return True
            for next_node in self.neighbors(cur_node):
                nextX, nextY = next_node
                if(nextX not in range(self.M) or nextY not in range(self.M)):
                    continue
                if self.A[nextX][nextY] == 0:
                    continue
                
                if self.A[next_node[0]][next_node[1]] == 2:
                        self.gas = self.N   
                new_cost = cur_cost + 1
                

                if next_node not in close_list or new_cost < close_list[next_node]:
                    close_list[next_node]=new_cost
                    new_f = new_cost +self.heuristic(next_node,(goalX, goalY),"Manhattan")
                    heapq.heappush(queue, (new_f, next_node))
                    self.came_from[next_node]=cur_node

        print("Khong tim thay.")
        return False
                            




# doc du lieu tu map.txt
f=open('map.txt','r')
N = int(f.readline())
M = int(f.readline())
A = []
for i in range(M):
    arrow = f.readline().split(" ")
    d = []
    for x in arrow: d.append(int(x))
    A.append(d)
# do thi A
g=Graph(N, M, A)
# g.BFS(0,0,9,9)
# g.DFS(0,0,9,9)
# g.UCS(0,0,9,9)
g.Astar(0,0,9,9)
print("DONE!")

