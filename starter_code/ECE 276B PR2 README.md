# ECE 276B PR2: 3D Motion Planning

## Description

This repository contains an implementation and comparison of search-based and sampling-based motion planning algorithms in 3-D environments. Specifically, it provides:

- **Collision checking** between line segments and axis-aligned bounding boxes (AABBs) (`collision.py`).
- **Search-based planner**: A* (and weighted A*) implemented in `astar.py`.
- **Sampling-based planners**: RRT and RRT-Connect implemented in `rrt.py`.
- **Baseline planner** (greedy) in `Planner.py` for reference.
- **Environment loader** and utility functions in `environment.py`.
- **Visualization** and example scripts in `main.py`.
- **Test maps** under the `maps/` directory.

## Repository Structure

```
.
├── README.md           # Project overview and usage instructions
├── main.py             # Entry point: load a map, run a planner, and plot the path
├── collision.py        # Part 1: segment–AABB collision checking
├── astar.py            # Part 2: A* (weighted) search-based planner
├── rrt.py              # Part 3: RRT and RRT-Connect sampling-based planners
├── Planner.py          # Baseline planner to be replaced or extended
├── environment.py      # Loads map files and provides sampling/collision interfaces
├── maps/               # 3-D environment descriptions (boundary, obstacles, start, goal)
│   ├── cube.txt
│   ├── maze.txt
│   └── ...
└── results/            # (Optional) directory for generated plots and logs
```

## Requirements

- Python 3.8 or higher
- NumPy
- Matplotlib
- tqdm (optional, for progress bars)

Install the required packages via:

```bash
pip install numpy matplotlib tqdm
```

> *Note:* If you choose to integrate OMPL for Part 3 rather than our RRT implementation, follow the instructions at [https://ompl.kavrakilab.org/installation.html](https://ompl.kavrakilab.org/installation.html) and ensure the `ompl` Python package is available.

## Usage

All planners are invoked through `main.py`. The general syntax is:

```bash
python main.py --planner <planner_name> --map <map_file> [options]
```

### Common Arguments

- `--planner`: Chooses the planner to run. Options:
  - `baseline` (from `Planner.py`)
  - `astar` (search-based)
  - `rrt` (sampling-based)
- `--map`: Path to a map file in `maps/` (e.g., `maps/window.txt`).
- `--output`: *(Optional)* Path to save the plotted result (e.g., `results/astar_window.png`).

### Planner-Specific Options

- **A***

  - `--heuristic_weight`: Weight for weighted A* (default: `1.0`).

- **RRT**

  - `--max_iter`: Maximum number of iterations (default: `10000`).
  - `--step_size`: Maximum extension distance `δ` (default: `1.0`).

### Examples

```bash
# Run weighted A* on the 'window' environment
python main.py --planner astar --map maps/window.txt --heuristic_weight 1.5 --output results/astar_window.png

# Run RRT on the 'maze' environment
python main.py --planner rrt --map maps/maze.txt --max_iter 5000 --output results/rrt_maze.png
```

## Results & Report

- Generated plots and performance logs can be found in the `results/` directory.
- A detailed write-up, including comparisons of path quality, node counts, and parameter studies, is provided in `report.pdf`.

---

*Prepared for ECE 276B: Planning & Learning in Robotics (Spring 2025)*