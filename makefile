
CC = g++
CFLAGS = -g -Wall


Robot_IK_Solver: main.cpp Vec.cpp Matrix.cpp Robot_IK_Solver.cpp
	$(CC) main.cpp Vec.cpp Matrix.cpp Robot_IK_Solver.cpp $(CFLAGS) -o Robot_IK_Solver

clean:
	rm Robot_IK_Solver
