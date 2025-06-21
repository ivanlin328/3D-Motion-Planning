
# priority queue for OPEN list
from pqdict import pqdict
import math
class AStarNode(object):
  def __init__(self, pqkey, coord, hval):
    self.pqkey = pqkey
    self.coord = coord
    self.g = math.inf
    self.h = hval
    self.parent_node = None
    self.parent_action = None
    self.closed = False
  def __lt__(self, other):
    return self.g < other.g     


class AStar(object):
  @staticmethod
  def plan(start_coord, environment, epsilon = 1):
    # Initialize the graph and open list
    Graph = {}
    OPEN = pqdict()
    CLOSED = set() 
    
    # start node
    start_key = tuple(start_coord)
    start_node = AStarNode(start_key, start_coord, environment.getHeuristic(start_coord))    # pqkey, coord, h
    start_node.g = 0
    OPEN[tuple(start_coord)] = start_node.g + epsilon * start_node.h
    Graph[start_key] = start_node
    nodes_considered = 0
    # while τ ∉ CLOSED do
    while OPEN:
      # Remove i with smallest f_i from OPEN
      curr_key, curr_f = OPEN.popitem()
      nodes_considered += 1
      # Insert i into CLOSED
      CLOSED.add(curr_key)
      
      curr_node = Graph[curr_key]
      if environment.isGoal(curr_node.coord):
        print(f"Number of nodes considered: {nodes_considered}")
        return AStar._reconstruct_path(curr_node)
      
      # for j ∈ Children(i) and j ∉ CLOSED do
      for action, nbr_coord in environment.getNeighbors(curr_node.coord):
              nbr_key = tuple(nbr_coord)
              if nbr_key in CLOSED:
                  continue
              cij = environment.getCost(curr_node.coord, nbr_coord)
              tentative_g = curr_node.g + cij
              if nbr_key not in Graph:
                nbr_node = AStarNode(nbr_key,nbr_coord,environment.getHeuristic(nbr_coord))
                Graph[nbr_key] = nbr_node
              nbr_node = Graph[nbr_key]
              
              #if g_j > g_i + c_ij then
              if nbr_node.g > tentative_g  :
                  # 8:   g_j ← g_i + c_ij
                  nbr_node.g = tentative_g
                  # 9:   Parent(j) ← i
                  nbr_node.parent_node = curr_node
                  # new f_j
                  f_j = nbr_node.g + epsilon * nbr_node.h
                  OPEN[nbr_key] = f_j
    return []       
  @staticmethod
  def _reconstruct_path(node):
      path = []
      while node is not None:
          path.append(node.coord)
          node = node.parent_node
      path.reverse()
      return path
        
