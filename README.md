
# 3D Motion Planning with Search-based and Sampling-based Algorithms

This project implements and compares two classes of motion planners — **Weighted A**\* and **Rapidly-Exploring Random Trees (RRT)** — for 3D environments with axis-aligned box obstacles. The planners compute **collision-free trajectories** for an autonomous agent navigating continuous 3D space, using a robust collision checker for line-segment vs. AABB intersections.



## Project Structure

```
.
├── collision.py          # Line-segment vs. AABB collision checker
├── environment.py        # 3D map environment setup (boundaries, obstacles, start/goal)
├── astar.py              # Weighted A* planner implementation
├── rrt.py                # RRT planner implementation
├── Planner.py            # Planner class wrapper to switch between algorithms
├── main.py               # Entry point: run planner on selected map
└── README.md             # Project description
```

## Installation

```bash
git clone https://github.com/ivanlin328/3D-Motion-Planning.git
cd 3D-Motion-Planning
pip install -r requirements.txt   # (if provided, otherwise standard Python 3.x packages)
```

## Usage

```bash
python main.py
```

Options (modify in `main.py` or Planner interface):

* Select planner: `'astar'` or `'rrt'`
* Set heuristic inflation factor (ϵ) for Weighted A\*
* Configure RRT parameters: max iterations, step size δ, goal bias probability pgoal
* Choose test environment (7 predefined maps)

## Examples

### Pillars Environment

<table>
  <tr>
    <th>Weighted A* (ϵ=1)</th>
    <th>Weighted A* (ϵ=5)</th>
    <th>RRT</th>
  </tr>
  <tr>
    <td><img src="Result/A*-1/Pillars/Pillars_2.png" width="300"></td>
    <td><img src="Result/A*-5/Pillars/Pillars_2.png" width="300"></td>
    <td><img src="Result/RRT/Pillars/Pillars_2.png" width="300"></td>
  </tr>
</table>

### Maze Environment

<table>
  <tr>
    <th>Weighted A* (ϵ=1)</th>
    <th>Weighted A* (ϵ=5)</th>
    <th>RRT</th>
  </tr>
  <tr>
    <td><img src="Result/A*-1/Maze/Maze2.png" width="300"></td>
    <td><img src="Result/A*-5/Maze/Maze2.png" width="300"></td>
    <td><img src="Result/RRT/Maze/Maze2.png" width="300"></td>
  </tr>
</table>

*... more examples for Flappy Bird, Pillars, Window, Tower, Room ...*


## Problem Formulation

* **Space:** 3D Euclidean space bounded by rectangular volume
* **Obstacles:** static, axis-aligned rectangular boxes (AABBs)
* **State:** continuous robot position in R³
* **Goal:** compute minimum-cost collision-free path from start to goal
* **Cost function:** Euclidean arc-length of the path

---

## Quantitative Results

### Table 1: Path Length Comparison

| Environment | A\* (ϵ = 1) | A\* (ϵ = 5) | RRT    |
| ----------- | ----------- | ----------- | ------ |
| Single Cube | 8.12        | 9.20        | 10.52  |
| Maze        | 85.73       | 92.86       | 111.95 |
| Flappy Bird | 30.74       | 32.00       | 45.41  |
| Pillars     | 30.41       | 30.70       | 45.37  |
| Window      | 28.21       | 29.92       | 34.34  |
| Tower       | 34.87       | 37.82       | 41.80  |
| Room        | 19.07       | 21.56       | 22.53  |

---

### Table 2: Planning Time Comparison (seconds)

| Environment | A\* (ϵ = 1) | A\* (ϵ = 5) | RRT       |
| ----------- | ----------- | ----------- | --------- |
| Single Cube | 0.00210     | 0.00209     | 0.00361   |
| Maze        | 2.38972     | 2.33748     | 511.83538 |
| Flappy Bird | 0.29267     | 0.05735     | 1.33716   |
| Pillars     | 0.85290     | 0.03930     | 1.31746   |
| Window      | 1.03647     | 0.03092     | 1.33194   |
| Tower       | 0.52314     | 0.09211     | 2.41289   |
| Room        | 0.18256     | 0.09472     | 0.08273   |

---

### Table 3: Number of Nodes Considered

| Environment | A\* (ϵ = 1) | A\* (ϵ = 5) | RRT   |
| ----------- | ----------- | ----------- | ----- |
| Single Cube | 5           | 5           | 62    |
| Maze        | 2240        | 2170        | 14757 |
| Flappy Bird | 411         | 71          | 1290  |
| Pillars     | 535         | 22          | 1165  |
| Window      | 930         | 26          | 1429  |
| Tower       | 474         | 78          | 1442  |
| Room        | 125         | 66          | 225   |

---


