import numpy as np
from collision import aabb_intersect
class Environment:
    def __init__(self, boundary, blocks, goal,step_size =0.5,goal_sample_rate =0.05):
        self.boundary = boundary
        self.blocks = blocks
        self.goal = goal
        self.dirs = []
        for dx in (-1, 0, 1):         
            for dy in (-1, 0, 1):      
                for dz in (-1, 0, 1):  
                    if not (dx == 0 and dy == 0 and dz == 0):
                        self.dirs.append((dx, dy, dz))  
        self.step_size = step_size
        self.goal_sample_rate= goal_sample_rate
        
    def _in_bounds(self, coord):
        x, y, z = coord
        b = self.boundary[0]        
        bx0, by0, bz0 = b[0:3]       
        bx1, by1, bz1 = b[3:6]       
        return (bx0 <= x <= bx1 and
                by0 <= y <= by1 and
                bz0 <= z <= bz1)
        
    def getHeuristic(self, coord):
        return np.linalg.norm(coord - self.goal)
    
    def isGoal(self,coord):
        return np.allclose(coord,self.goal)
    
    def getCost(self, from_coord, to_coord):
        return np.linalg.norm(to_coord - from_coord)
    
    def collision_free(self, p0, p1):
        p0 = np.array(p0)
        p1 = np.array(p1)
        if not (self._in_bounds(p0) and self._in_bounds(p1)):
            return False
        for blk in self.blocks:
            aabb_min = blk[:3]
            aabb_max = blk[3:6]
            if aabb_intersect(p0, p1, np.array(aabb_min), np.array(aabb_max)):
                return False
        return True
        
    def getNeighbors(self, coord):
        nbrs = []
        coord = np.array(coord,dtype=float)
        for d in self.dirs:
            next_coord = coord + np.array(d)
            if self.collision_free(coord, next_coord):
                nbrs.append((d, next_coord))
            if self.collision_free(coord, self.goal):
                action = tuple(self.goal - coord)
                nbrs.append((action, self.goal))
        return nbrs
    
    def rrt_sample(self):
        if np.random.rand() < self.goal_sample_rate:
            return self.goal
        b = self.boundary[0]
        xmin,ymin,zmin, xmax,ymax,zmax = b[:6]
        return np.array([
               np.random.uniform(xmin, xmax),
               np.random.uniform(ymin, ymax),
               np.random.uniform(zmin, zmax),
               ], dtype=float)
    
    def rrt_nearest(self,nodes,rnd):
        dists =[]
        for n in nodes:
            dists.append(np.linalg.norm(n.coord- rnd))
        idx   = int(np.argmin(dists))
        return nodes[idx]
    
    def rrt_steer(self,nearest_node, rnd):
        vec = rnd - nearest_node.coord 
        dist = np.linalg.norm(vec)
        if dist <= self.step_size:
            return rnd
        return nearest_node.coord + (vec / dist) * self.step_size
        

        