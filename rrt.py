from __future__ import annotations
import math
import random as rand
from dataclasses import dataclass

@dataclass
class State:
    """Struct for the state/configuration representation"""
    x: float = 0
    y: float = 0
    dx: float = 0
    dy: float = 0

    def get_location(self):
        return [self.x,self.y]

class Graph():
    """A graph class for the motion planner's tree"""
    #tree = {}?

    def __init__(self,initial_state):
        self.v = set() #Set of vertices in the tree
        self.e = set() #Set of tuples of edges = (v1,v2)
        self.tree = {initial_state:[]} #Adjacency list, vertex maps to list of connections
    
    def add_vertex(self,v):
        self.tree[v] = []

    def add_edge(self,v1,v2):
        self.tree[v1].append(v2)

    def find_nearest(self,q):
        distances = {k:v for (k,v) in zip(self.tree.keys(), map(lambda k: math.dist(k,q),self.tree.keys()))}
        return min(distances,key=distances.get)

    def trajectory(self):
        """Returns the length of the ship's trajectory as a list of
        (state,control) tuples"""
        pass

    def motion_tree(self):
        """Return the length of the motion tree and the list of edges
        Consisting of the state and the control to be applied"""
        pass


class RRTRobot():    
    def __init__(self,ncol,nrow,obs,qs,qg):
        self.state = State(qs)
        self.qg = qg
        self.ncol = ncol
        self.nrow = nrow
        self.obstacles = obs
        self.acc = {'dir':0,'mag':0}

        
    def __call__(self):
        """Find a path using RRT, then return the path trajectory and
        motion tree"""
        sol = self.rrt(self.state) #returns the entire rrt graph/tree
        return sol if sol is not None else "no solution found"


    def goal_test(self):
        """Goal is considered reached when we're within 0.1 map units of it"""
        return 1 if math.dist(self.state.get_location(),self.qg) <= 0.1 else 0

    #Check is state colliding with an obstacle, done every 0.05 path units/timestep
    def collision_checker(self, q):
        """Maps a configuration to 1 if it collides with an obstacle, otw. 0"""
        for obs in self.obsacles:
            if(obs[0] <= q['x'] <= obs[0]+1 and
                obs[1] <= q['y'] <= obs[1]+1):
                return 1
        return 0
    
    def sample(self,goal_bias=0.05):
        """Sample and return a random point in C_free+qg+qs"""
        while True:
            if rand.random() <= goal_bias:
                return self.qg
            else:
                milestone = [rand.uniform(0,self.ncol),rand.uniform(0,self.nrow)]
                if not self.collision_checker(milestone):
                    return milestone

    def generate_random_control(self,m=10):
        """Generate m random controls to simulate steering.
        A control is def as a dir and magnitude"""
        controls = []
        for m in range(m):
            theta = 2*math.pi*rand.random() #math.radians()*rand.random()
            mag = rand.random()*0.5
            controls.append({'dir':theta,'mag':mag})
        return controls
    
    def update_state(self,state,control,dt=0.05):
        """Update the ship's state at each time slice
        using vector addition, as well as the acceleration"""
        state.dx += dt*control['mag']*math.cos(control['dir'])
        state.dy += dt*control['mag']*math.sin(control['dir'])
        state.x += dt*state.dx
        state.y += dt*state.dy
        return state
    
    def simulate_path(self,milestone,control,time_slices=20):
        """Simulate the curvature/path to a milestone for a given control
        using 20 time slices per step for integration (ie dt=0.05)"""
        for step in range(time_slices):
            control['dir'] += math.atan2(self.dy,self.dx)
            state = self.update_state(state,control,1/time_slices)
            if self.collision_checker(state):
                return None
        return 
            
        

    def rrt(self,qs):
        tree = Graph(qs)
        while not self.goal_test():
            milestone = self.sample()
            controls = self.generate_random_control()
            sim_paths = [{'path':self.simulate_path(milestone,ctrl), 'control':ctrl} for ctrl in controls]
            #nearest_config = min(math.dist(self.state.get_location(),paths)) 
            #[math.dist(self.state.get_location(),path) for path in paths if path is not None]
        return tree


"""
if path was feasible it should return final state and control used
for closest path need list index to get back the control/updated state
TODO: make sure that the update state is acting appropriately
fix the collision checker? idrk
also make sure simulate path returns updated values

motion tree uses the control to be applied so not the final value, store initial.
"""

"""
algo:
each steering/trajectory control = a different curve to the path
until goal is reached:
simulate 1 timestep aka iteration:
    generate a random target point
    generate 10 random controls ie curvatures/paths to get there
    divide the path into 20 slices
    update the ship trajectory for each control so need up to 10 copies?
    check for collisions at the ship's location (ie after each slice)
    if a control collides remove it from consideration
    after the full timestep, pick the control that got the ship
    closest to the target location. If none, nothing added to the tree

afaik we're NOT adding every .05 path units to the tree, just final goal+control

-collision checker needs to check that entire curve isn't colliding,
from start vertex to goal vertex.
-distance growth limit = 0.05 in essense, b/c collision check every 0.05
-potentially move collision check from sample to rrt()?

-want to update state at end of iters, but spec says check for collision
after each slice, so... actually those aren't contradictory i guess

-what's the difference btwn doing "while not a goal" and "for k=1:K"?
collision goes from source to dest for each edge

"""


#OUTDATED/DEPRECATED STUFF:
"""
nearest_state = tree.find_nearest(milestone)
maybe not doing nearest state, doing nearest steering.
trajectory_control = self.generate_random_control(milestone)
#don't do this if steering not possible.
tree.add_vertex(milestone)
tree.add_edge(nearest_state,milestone)
"""

"""timestep explanation:
path is a parameterized curve T(t), where T(0)=qs and T(1)=qg.
T(t) is some point in C_free.
That means t parameterizes how far along the path we are, and thus
acts like time in that as t increases, the distance along the path
increases. Note that t is a point in [0,1] though, not measured
in seconds.
"""

"""
self.acc['dir'] += math.atan2(self.dy,self.dx)
self.dx += dt*self.acc['mag']*math.cos(self.acc['dir'])
self.dy += dt*self.acc['mag']*math.sin(self.acc['dir'])
self.x += dt*self.dx
self.y += dt*self.dy
"""