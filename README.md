# Project-Sentinel_F

# Priority-Aware Sensor Deployment using Discrete Optimization

This project implements a priority-aware sensor deployment system that optimizes sensor placement to maximize coverage of important regions while minimizing redundant overlap and sensor usage. The system is designed to help visualize and understand how discrete optimization techniques can be applied to real-world sensor deployment problems.

The project combines a constraint-based optimization backend with an interactive web-based visualization interface.

---

## Problem Overview

In many applications such as environmental monitoring, surveillance, and IoT systems, sensors are limited in number due to cost, energy, and deployment constraints. Poor sensor placement can lead to uncovered regions or unnecessary overlap, reducing overall efficiency.

This project addresses the problem by:
- Prioritizing important regions within the deployment area
- Maximizing coverage of these regions
- Limiting the number of sensors used
- Reducing redundant sensing overlap

---

## Key Features

- Priority-aware sensor deployment optimization
- Discretization of continuous priority regions into sub-target points
- Grid-based candidate sensor placement
- Constraint-based optimization using Google OR-Tools (CP-SAT)
- Interactive web-based visualization
- Real-time experimentation with sensor parameters

---

## System Architecture

The system follows a client–server architecture:

### Frontend
- Web-based interactive UI
- Allows users to:
  - Define environment size
  - Specify priority regions
  - Adjust sensor range and sensor budget
  - Visualize optimized sensor deployments in real time

### Backend
- Implemented using Python and Flask
- Uses Google OR-Tools CP-SAT solver
- Handles:
  - Environment discretization
  - Coverage computation
  - Optimization modeling and solving
- Returns optimized sensor placement and coverage results via API

---

## Optimization Approach

### 1. Pre-Processing: Modeling the Space
- **Environment Discretization:** Priority regions are discretized into a set of sub-target points. Each sub-target represents a specific coordinate that must be covered.
- **Candidate Sensor Locations:** Possible sensor placements are restricted to predefined grid locations across the environment.
- **Coverage Pre-calculation:** The system pre-calculates a boolean coverage matrix, determining mathematically which grid locations can cover which sub-target points based on the defined sensor range.

### 2. The ILP Model (Constraint Programming)
The problem is formulated as an Integer Linear Programming (ILP) model, defining decisions as mathematical variables and logical constraints:
- **Decision Variables:**
  - Binary variables ($x_i \in \{0, 1\}$) determine if a sensor is placed at grid location $i$.
  - Binary variables ($y_j \in \{0, 1\}$) determine if sub-target $j$ is covered.
  - Integer variables define the total count of sensors covering sub-target $j$ to detect overlap.
- **Logical Constraints:**
  - **Inventory Constraint:** $\sum x_i \leq \text{Budget}$. The total number of deployed sensors cannot exceed the budget.
  - **Coverage Activation:** $y_j$ is forced to $1$ **only if** $\sum x_{i \text{ covering } j} \geq 1$.
- **Objective Function (Maximization):**
  The solver is tasked with maximizing a linear function, balancing competing priorities via weighting factors:
  $$ \text{Maximize} = \sum (y_j \cdot W_{\text{coverage}}) - \sum (\text{overlap}_j \cdot W_{\text{overlap}}) - \sum (x_i \cdot W_{\text{cost}}) $$
  - **$(+)$ Reward for Coverage:** Maximize the number of priority targets covered ($y_j$).
  - **$(-)$ Penalty for Overlap:** Minimize the redundant overlap (multiple sensors seeing the same spot) to encourage spreading assets out.
  - **$(-)$ Penalty for Usage:** Minimize the total count of sensors used ($x_i$).

### 3. The Solver Mechanism (`cp_sat.Solve(model)`)
When the optimization engine calls `Solve()`, Google OR-Tools CP-SAT explores the massive combinatorial search space using a technique called **Branch and Bound**. Here is a brief overview of the process:

1.  **Relaxation:** The solver first solves a simplified version of the problem where variables can be fractions (e.g., placing $0.5$ of a sensor). This gives a rapid theoretical **Upper Bound** (Primal Bound) of the best possible objective score.
2.  **Branching:** The solver takes a fractional decision and forces a binary choice (e.g., "Sensor at Location X is now $1$"). This splits the problem into two sub-problems (branches), creating a tree structure.
3.  **Bounding:** For each new branch, the solver finds a real integer solution (e.g., a real map configuration). This gives a **Lower Bound** (Dual Bound) – the best feasible solution found so far.
4.  **Maximization vs. Minimization Loop:** The solver’s goal is to close the "Gap" between the Upper and Lower bounds:
    - **Maximizing Feasible Solutions:** CP-SAT uses heuristics to constantly search for better real map layouts, pushing the **Lower Bound up**.
    - **Minimizing Search Space:** As branching continues, if the fractional Upper Bound of a sub-problem drops below the current real Lower Bound, that entire branch is mathematically eliminated ("pruned") because it can never contain the optimal solution. This constantly pulls the **Upper Bound down**.
5.  **Termination:** The solve completes when the **Gap is zero**, meaning the best feasible solution found (Maximized) is mathematically proven to be equal to the tightened theoretical limit (Minimized).

---

## Technologies Used

- Python
- Flask
- Google OR-Tools (CP-SAT Solver)
- NumPy
- HTML, CSS, JavaScript (Frontend Visualization)

---

## How to Run the Project

### Prerequisites
- Python 3.x
- pip

### 1. Install Dependencies
```bash
pip install flask flask-cors numpy ortools
