README
------------------------
Title: CS365 Lab A - Search and Pathfinding

Authors: Gordian Bruns, Dipesh Poudel

Programming Language: Python 3.7.4
------------------------
I. File list
------------
  - mouse.py	implementation of the agent + goal test
  - main.py		implementation of state representation + search algorithms + transition function
(store the files in the same directory)

II. Usage
------------
goal test:

	1. Read in a file to obtain the maze as a nested list
	2. Call the state representation function on the nested list to obtain a graph (networkx graph)
	3. Call the transition function, which returns a mouse that got moved according to the path and collected the goals
	4. Now, you can call the goal_test function on the mouse and if the mouse collected all goals, it is going to return True, otherwise False


depth-first search:

	1. Use a command line and call it by typing 'python main.py [filename] dfs'
	   -> note that [filename] has to be the name of a file that is in the same directory
	2. The program is going to return a displayed maze with the path as "#", the cost of the path, and how many nodes were expanded
	3. Example of how to run it:
	    Command line: python main.py 1prize-open.txt dfs (given the 1prize-open.txt file on tools)


breadth-first search:

	1. Use a command line and call it by typing 'python main.py [filename] bfs'
	   -> note that [filename] has to be the name of a file that is in the same directory
	2. The program is going to return a displayed maze with the path as "#", the cost of the path, and how many nodes were expanded
	3. Example of how to run it:
	    Command line: python main.py 1prize-open.txt gfs (given the 1prize-open.txt file on tools)


greedy best-first search:

	1. Use a command line and call it by typing 'python main.py [filename] gbfs'
	   -> note that [filename] has to be the name of a file that is in the same directory
	2. The program is going to return a displayed maze with the path as "#", the cost of the path, and how many nodes were expanded
	3. Example of how to run it:
	    Command line: python main.py 1prize-open.txt gbfs (given the 1prize-open.txt file on tools)


A* search:

	1. Use a command line and call it by typing 'python main.py [filename] astar'
	   -> note that [filename] has to be the name of a file that is in the same directory
	2. The program is going to return a displayed maze with the path as "#", the cost of the path, and how many nodes were expanded
	3. Example of how to run it:
	    Command line: python main.py 1prize-open.txt astar (given the 1prize-open.txt file on tools)


multiprize A* search:

	1. Use a command line and call it by typing 'python main.py [filename] m_astar'
	   -> note that [filename] has to be the name of a file that is in the same directory
	2. The program is going to return a displayed maze with the goals replaced by numbers (letters if > 9) in which order they were obtained, the cost of the path, and how many nodes were expanded
	3. Example of how to run it:
	    Command line: python main.py multiprize-micro.txt m_astar (given the multiprize-micro.txt file on tools)
