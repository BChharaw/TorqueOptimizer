# Robotic Assembly Torque Analysis and Optimization Program

This C++ program is designed to analyze the effective torque at various joints within a robotic assembly and provide an optimized design for the assembly that minimizes torque, thereby reducing energy consumption and stress on the assembly. The program is developed for the MTE 119 course project alongside my classmate Samantha Chong

## Usage
1. Clone the repository or download the source code.
2. Compile the program using a C++ compiler.
3. Run the compiled executable.

## Algorithm Overview
The program utilizes the following main components:

### Structures:
- AngleOfMembs: Stores required angles of members based on the parallelogram concept.
- TestAngles: Stores final angles for all three test cases.
- LengthOfMembs: Stores the length of members.
- Point: Represents a 2D point.

### Functions:
- bool testValues(...): Takes lengths and angles as input and checks their validity.
- AngleOfMembs anglesReq(...): Calculates the required angles for a given set of lengths and coordinates.
- double torque(...): Calculates the torque at the base motor based on lengths and angles.
- double calculateCMPTorque(...): Performs torque calculations for different scenarios and minimizes torque.
- int main(): The main function that executes the optimization process and displays results.

### Optimization:
The program employs an optimization strategy to find the combination of member lengths that minimizes the torque. It iterates through various member lengths and angles, testing each combination for validity and computing the torque. The optimization process narrows down the search space based on a predefined center and radius of convergence.
