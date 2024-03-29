import sys
import rrt

#World's symbolic representation
blocked = '#'

def main(*args):
    ncol = int(sys.stdin.readline())
    nrow = int(sys.stdin.readline())

    obstacles = set()
    
    #Grid defined 0,0 to be in the bottom left, but input is read from top left,
    #so flip the row coordinates
    for row in range(nrow):
        for col in range(ncol):
            cell = sys.stdin.read(1)
            if cell == '\n':
                cell = sys.stdin.read(1)
            elif cell == blocked:
                obstacles.add(((nrow-1)-row,col))
    
    #The initial angle and speed are assumed to be 0, so dx and dy = 0
    initial = [float(sys.stdin.readline()),float(sys.stdin.readline()),0,0]
    goal = [float(sys.stdin.readline()),float(sys.stdin.readline())]

    
    #Call the agent and get solution, or none if DNE
    agent = rrt.RRTRobot(ncol,nrow,obstacles,initial,goal)
    solution = agent() #returns the final rrt tree
    if solution is not None:
        print(*solution.trajectory(),sep="\n")  # prints the lenth of the trajectory, then the trajectory
        print(*solution.motiontree(),sep='\n')  # prints the length of the motrion tree, then the tree
    else:
        print("No solution found")

if __name__ == "__main__":
    main()