# ----------
# User Instructions:
#
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal.
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
goal = [len(grid) - 1, len(grid[0]) - 1]
cost = 1  # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def compute_value(grid, goal, cost):
    # ----------------------------------------
    # insert code below
    # ----------------------------------------

    # make sure your function returns a grid of values as
    # demonstrated in the previous video.
    result = [[99 for col in range(len(grid[0]))] for row in range(len(grid))]
    open = []
    open.append([0, goal[0], goal[1]])
    result[goal[0]][goal[1]] = 0

    while len(open) > 0:
        open.sort()
        open.reverse()
        next = open.pop()

        g = next[0]
        x = next[1]
        y = next[2]

        for i in range(len(delta)):
            x2 = x + delta[i][0]
            y2 = y + delta[i][1]
            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                if result[x2][y2] == 99 and grid[x2][y2] == 0:
                    g2 = g + cost
                    open.append([g2, x2, y2])
                    result[x2][y2] = g2
    return result

res = compute_value(grid, goal, cost)


print("Goal: {}".format(goal))
for r in range(len(res)):
    for c in range(len(res[0])):
        print(res[r][c], end = ' ')
        #print(" ", "","")
    print("")