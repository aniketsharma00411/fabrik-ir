"""
Submission for Assignment of Introduction to Robotics (BCCS-9402) course

Submitted by: Aniket Sharma (2019BCS-008)

Problem statement:
Write a program to calculate Inverse Kinematics of a three-link manipulator robot using Fabrik technique.

INPUT:

It takes 6 values from user which are as follows:
    1. (space-seprated float) Initial position of joint 0
    2. (space-seprated float) Initial position of joint 1
    3. (space-seprated float) Initial position of joint 2
    4. (space-seprated float) Initial position of joint 3
    5. (space-seprated float) Goal position of end-effector
    6. (float) Tolerence

OUTPUT:

    1. (int) number of iterations taken by algorithm
    2. (list) New positions of joints
    3. (matplotlib plot) Graph showing initial joint positions, final joint positions and goal state

Example:

Enter initial position of joint 0: 0 0
Enter initial position of joint 1: 1.55 1.26
Enter initial position of joint 2: 1.62 2.48
Enter initial position of joint 3: 1.97 3.26
Enter goal position of end effector: 2 3
Enter tolerance: 0

Number of iterations = 15
New Joint positions are:
(0.0, 0.0)
(1.6506993700365413, 1.1248518079111427)
(1.4655173683767273, 2.332745688291892)
(2.0, 3.0)
"""

import matplotlib.pyplot as plt


def eucledian_distance(p1, p2):
    """
    Method to calculate eucledian distance between two points.

    INPUT:

    1. (tuple of float) Point 1
    2. (tuple of float) Point 2

    OUTPUT:

    1. (float) eucledian distance between the two points
    """
    return (((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2))**0.5


def fabrik_solver(p, goal, tolerence):
    """
    Method to use fabrik technique to reach goal state.

    INPUT:

    1. (list) List of all positions of joints
    2. (tuple of float) Goal state
    3. (float) Tolerence 

    OUTPUT:

    1. (list) New positions of joints
    2. (int) number of iterations
    """
    b = p[0]
    diff = eucledian_distance(p[-1], goal)

    number_iterations = 0

    while diff > tolerence:
        # forward reaching
        p[-1] = goal
        for i in range(len(p)-2, -1, -1):
            r = eucledian_distance(p[i+1], p[i])
            lam = l[i]/r
            p[i] = ((1-lam)*p[i+1][0] + lam*p[i][0],
                    (1-lam)*p[i+1][1] + lam*p[i][1])

        # backward reaching
        p[0] = b
        for i in range(len(p)-1):
            r = eucledian_distance(p[i+1], p[i])
            lam = l[i]/r
            p[i+1] = ((1-lam)*p[i][0] + lam*p[i+1][0],
                      (1-lam)*p[i][1] + lam*p[i+1][1])

        diff = eucledian_distance(p[-1], goal)

        number_iterations += 1

    return p, number_iterations


def print_solution(p, number_iterations):
    """
    Method to print the answer.

    INPUT:

    1. (list) New positions of joints
    2. (int) Number of iterations
    """
    print('\nNumber of iterations = {}'.format(number_iterations))

    print('New Joint positions are:')
    for i in range(len(p)):
        print(p[i])


def plot_graph(p, new_p, goal):
    """
    Method to print the graph.

    INPUT:

    1. (list) List of all positions of joints
    2. (list) New positions of joints
    3. (tuple of float) Goal state

    OUTPUT:

    1. (matplotlib plot) Graph showing initial joint positions, final joint positions and goal state
    """
    p_x = []
    p_y = []
    for coord in p:
        p_x.append(coord[0])
        p_y.append(coord[1])

    new_p_x = []
    new_p_y = []
    for coord in new_p:
        new_p_x.append(coord[0])
        new_p_y.append(coord[1])

    plt.plot(p_x, p_y, label='Original Positions', marker='o')
    plt.plot(new_p_x, new_p_y, label='New Positions', marker='o')
    plt.scatter([goal[0]], [goal[1]], s=400, marker='*', label='Goal')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    # Taking input
    p = []
    for i in range(4):
        temp = tuple(
            map(float, input('Enter initial position of joint {}: '.format(i)).split()))
        p.append(temp)

    goal = tuple(
        map(float, input('Enter goal position of end effector: ').split()))

    tolerence = float(input('Enter tolerance: '))

    dist = eucledian_distance(p[0], goal)

    l = []
    for i in range(len(p)-1):
        l.append(eucledian_distance(p[i], p[i+1]))

    # Checking if the goal state is reachable
    if dist > sum(l):
        print('Goal unreachable')
        exit()

    new_p, number_iterations = fabrik_solver(p.copy(), goal, tolerence)

    print_solution(new_p, number_iterations)

    plot_graph(p, new_p, goal)
