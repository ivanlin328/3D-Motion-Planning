import numpy as np

class RRTNode(object):
  def __init__(self,coord,parent= None):
    self.coord = coord
    self.parent_node = parent
  
class RRT(object):
    @staticmethod
    def plan(start_coord, environment,max_iterations):
        start_node = RRTNode(start_coord)
        nodes = [start_node]
        nodes_considered = 0
        for  i in range (max_iterations):
            # 1)  random sample
            rnd = environment.rrt_sample()                              
            # 2) Find nearest node                        
            nearest_node = environment.rrt_nearest(nodes, rnd)        
            # 3) extend
            new_pt = environment.rrt_steer(nearest_node, rnd)
            
            if environment.collision_free(nearest_node.coord, new_pt):
                new_node = RRTNode(new_pt)
                new_node.parent_node = nearest_node
                nodes.append(new_node)
                nodes_considered += 1
                
                if np.linalg.norm(new_pt - environment.goal) <= environment.step_size:
                    if environment.collision_free(new_pt, environment.goal):
                        goal_node = RRTNode(environment.goal,parent=new_node)
                        print(f"Number of nodes considered: {nodes_considered}")
                        return RRT._reconstruct_path(goal_node)
        return None          
    @staticmethod
    def _reconstruct_path(goal_node):
        path = []
        node = goal_node 
        while node is not None:
            path.append(node.coord)
            node = node.parent_node
        path.reverse()
        return path
        