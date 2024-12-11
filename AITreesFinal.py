import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches



'''
This should be the program for the final submission for the AI final project
'''

# Define the Node class
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

# Define the RRT class
class RRT:
    def __init__(self, start, goal, obstacleList, max_iter=11000, max_step_size=0.05, goal_threshold=0.05):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacleList = obstacleList
        self.max_iter = max_iter
        self.max_step_size = max_step_size
        self.goal_threshold = goal_threshold




    #Computes the distance with the pythagorean theorem
    def computeDistance(self, node1, node2):
        return np.linalg.norm([node1.x - node2.x, node1.y - node2.y])


    #draws the lines once the random values are given.
    def drawLines(self, current, final):
        return plt.plot([current.x, final.x], [current.y, final.y], 'c-', linewidth=1)
    

    #checking to ensure that the new point will not collide with any obstacles.
    def checkCollisions(self, newNode, obstacleList):
        #creating new points
        x = newNode.x
        y = newNode.y

        #Loop that scans through the obstacles to see if there is a collision
        for obstacle in obstacleList:
            obsX, obsY, width, height = obstacle
            if (obsX <= x <= obsX + width) and (obsY <= y <= obsY + height):
                return True
        return False

    #creating fandom values between 0 and 1 for x and y (Within the environment)
    def generateRandomNode(self):
        return Node(np.random.uniform(0, 1), np.random.uniform(0, 1))
    


    
    def nearestNode(self, nodeList, randomNode):
            distances = [self.computeDistance(node, randomNode) for node in nodeList]
            nearest_index = np.argmin(distances)
            return nodeList[nearest_index]  
    
    def newNode(self, nearestNode, randomNode, maxStepSize):
        dist = self.computeDistance(nearestNode, randomNode)
        if dist < maxStepSize:
            return randomNode  # New node is the random node if within max step size
        else:
            scale = maxStepSize / dist
            x = nearestNode.x + (randomNode.x - nearestNode.x) * scale
            y = nearestNode.y + (randomNode.y - nearestNode.y) * scale
            return Node(x, y)
        

    def generatePath(self, nodes, goalNode):
        path = []
        current = goalNode
        while current.parent is not None:
            path.append((current.x, current.y))
            current = current.parent
        path.append((self.start.x, self.start.y))
        path.reverse()
        return path

    
    def plan(self):
        nodes = [self.start]  #Start with the initial node
        nodeCount = 0
        for i in range(self.max_iter):
            randomNode = self.generateRandomNode()
            nearest = self.nearestNode(nodes, randomNode)
            newNode = self.newNode(nearest, randomNode, self.max_step_size)

            #Create a new node if there are no collisions
            if not self.checkCollisions(newNode, self.obstacleList):
                newNode.parent = nearest
                nodes.append(newNode)
                self.drawLines(nearest, newNode)
                nodeCount += 1
                
                #Check if the new node is within the goal threshold
                if self.computeDistance(newNode, self.goal) <= self.goal_threshold:
                    #Connects to the goal if it is in range
                    finalNode = Node(self.goal.x, self.goal.y)
                    finalNode.parent = newNode
                    nodes.append(finalNode)
                    self.drawLines(newNode, finalNode)
                    print('Number of nodes ' , nodeCount)
                    return self.generatePath(nodes, finalNode)  #returns the path

        return None  #No valid path found





        
#plotting all the necessary values into the image
def plotEnvironment(start, goal, obstaclelist, path=None, plt=None):
    if plt is None:
        plt.figure(figsize=(8, 8))
        plt.axis([0, 1, 0, 1])
        plt.grid(True)

    # Plot obstacles as rectangles
    for obstacle in obstaclelist:
        x, y, width, height = obstacle
        rect = patches.Rectangle((x, y), width, height, color='black')
        plt.gca().add_patch(rect)

    # Plot start and goal points
    plt.plot(start[0], start[1], 'go', markersize=10, label='Start')
    plt.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')

    # Plot path if provided
    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, 'b-', linewidth=2, label='Path')  # Specify color and line style for path

    plt.legend(loc='upper right', bbox_to_anchor=(0.15, 1.15))

    # Show plot if plt was not provided (only show when a new plot is created)
    if plt is None:
        plt.show()






# Function to select obstacle lists
def obstacleSelection(listNum):
    obstacle_lists = {
        #x, y, width, height *First 4 sections are for drawing the borders
        #simple starter program
        1: [(1, 0, 0.001, 1), (0,0, 0.001, 1), (0, 0, 1, 0.001), (0, 1, 1, 0.001),    (0.2, 0.075, 0.075, 0.8), (0.075, 0.2, 0.8, 0.075), (0.45, 0.05, 0.075, 0.9), (0.05, 0.45, 0.9, 0.075), (0.725, 0.075, 0.075, 0.8), (0.075, 0.725, 0.8, 0.075)],
        
        #horizontal lines back and fourth
        2: [(1, 0, 0.001, 1), (0,0, 0.001, 1), (0, 0, 1, 0.001), (0, 1, 1, 0.001),    (0.0, 0.2, 0.9, 0.1), (0.0, 0.5, 0.4, 0.2), (0.45, 0.5, 0.55, 0.2), (0.45, 0.85, 0.1, 0.15), (0.45, 0.7, 0.1, 0.1), (0.7, 0.75, 0.1, 0.25)],
    
        #Cross with end obstacles
        3: [(1, 0, 0.001, 1), (0,0, 0.001, 1), (0, 0, 1, 0.001), (0, 1, 1, 0.001),    (0.45, 0.05, 0.075, 0.9), (0.05, 0.45, 0.9, 0.075), (0.05, 0.2, 0.075, 0.6), (0.875, 0.2, 0.075, 0.6), (0.2, 0.05, 0.6, 0.075), (0.2, 0.875, 0.6, 0.075)],
        
        #Complicated version of 3
        4: [(1, 0, 0.001, 1), (0,0, 0.001, 1), (0, 0, 1, 0.001), (0, 1, 1, 0.001),    (0.45, 0.05, 0.075, 0.475), (0.05, 0.45, 0.475, 0.075), (0.2, 0.05, 0.6, 0.075), (0.05, 0.2, 0.075, 0.6), (0.575, 0.2, 0.075, 0.3), (0.2, 0.575, 0.625, 0.075), (0.175, 0.575, 0.075, 0.425), (0.575, 0.175, 0.425, 0.075), (0.3, 0.7, 0.7, 0.075), (0.3, 0.7, 0.075, 0.2)],
    
        #Back and fourth lines
        5: [(1, 0, 0.001, 1), (0,0, 0.001, 1), (0, 0, 1, 0.001), (0, 1, 1, 0.001),    (0, 0.6, 0.9, 0.075), (0.1, 0.4, 0.9, 0.075), (0, 0.2, 0.9, 0.075), (0.1, 0.8, 0.9, 0.075)],

       }

    if listNum in obstacle_lists:
        return obstacle_lists[listNum]
    else:
        print("Invalid selection. Please choose a number between 1 and 5.")
        return None




def main():
    start = (0.1, 0.1)
    goal = (0.9, 0.9)

    # Prompt user for obstacle selection
    while True:
        try:
            listSelection = int(input("\nPlease input the test case you would like to use (1-5): "))
            if 1 <= listSelection <= 5:
                break
            else:
                print("Invalid selection. Please choose a number between 1 and 5.")
        except ValueError:
            print("Invalid input. Please enter a valid number.")

    obstacle_list = obstacleSelection(listSelection)

    if obstacle_list is None:
        return

    # Create a single figure and pass it to plotEnvironment
    plt.figure(figsize=(8, 8))  # Create a new figure
    plotEnvironment(start, goal, obstacle_list, plt=plt)  # Pass plt object

    # Initialize RRT and find path
    rrt = RRT(start, goal, obstacle_list)
    path = rrt.plan()

    # Print statements for success/failure
    if path:
        print("Path found!")
        plotEnvironment(start, goal, obstacle_list, path, plt=plt)  # Pass plt object
    else:
        print("Failed to find a path.")

    plt.show()  # Ensure the plot is displayed

    print("Finished with Program")

if __name__ == "__main__":
    main()
