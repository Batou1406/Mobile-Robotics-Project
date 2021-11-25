import numpy as np
import heapq

class ShorthestPath(object):
    def __init__(self, startPos, goalPos, obstaclesList, widthLen, heightLen):
        self.start=startPos #in (x,y) form
        self.goal=goalPos #in (x,y) form
        self.obstacles=obstaclesList #List of obstacles
        self.width=widthLen
        self.height=heightLen

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def generateGrid(self):
        grid=np.zeros((self.height,self.width))
        for i in range(len(self.obstacles)):
            grid[self.obstacles[i][0]][self.obstacles[i][1]]=1
        return grid


    def astar(self):
        array=self.generateGrid()
        neighbors=[(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        close_set=set()
        came_from={}
        gscore={self.start:0}
        fscore={self.start:self.heuristic(self.start, self.goal)}
        oheap=[]
        heapq.heappush(oheap, (fscore[self.start], self.start))

        while oheap:
            current=heapq.heappop(oheap)[1]
            if current==self.goal:
                data=[]
                while current in came_from:
                    data.append(current)
                    current=came_from[current]
                return data
            close_set.add(current)

            for i, j in neighbors:
                neighbor=current[0]+i, current[1]+j
                tentative_g_score=gscore[current]+self.heuristic(current, neighbor)
                if 0 <= neighbor[0] < array.shape[0]:
                    if 0 <= neighbor[1] < array.shape[1]:
                        if array[neighbor[0]][neighbor[1]]==1:
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                    came_from[neighbor]=current
                    gscore[neighbor]=tentative_g_score
                    fscore[neighbor]=tentative_g_score + self.heuristic(neighbor, self.goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        return False
