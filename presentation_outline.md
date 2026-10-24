# PRESENTATION OUTLINE (10 SLIDES)

## Project: Robot Path Planning Using the A* Search Algorithm

Designed for PowerPoint / Google Slides / Keynote presentation defense.

---

### SLIDE 1: Title Slide
- **Slide Title**: Robot Path Planning Using the A* Search Algorithm
- **Subtitle**: An Interactive 2D Grid Simulation & Algorithmic Analysis
- **Presenter / Student Name**: `[Student Name]`
- **Course**: Design & Analysis of Algorithms (DAA) / Artificial Intelligence
- **Department**: Department of Computer Science & Engineering
- **Institution**: `[College / University Name]`
- **Academic Year**: 2025 – 2026

---

### SLIDE 2: Abstract & Motivation
- **The Core Problem**: Mobile robots must navigate complex environments from an initial position to a destination without colliding with obstacles.
- **Why Path Planning Matters**: Autonomous vacuum cleaners, warehouse logistics (e.g. Amazon Kiva robots), planetary rovers, and self-driving cars all rely on shortest-path graph search.
- **Project Motivation**:
  - Build a visual, interactive platform to demystify heuristic search algorithms.
  - Bridge theoretical algorithm mechanics ($f = g + h$) with live graphical simulation.
  - Empirically compare heuristic guidance (A*) against uninformed search (Dijkstra).

---

### SLIDE 3: Problem Statement & Objectives
- **Problem Statement**:
  - Given a 2D uniform-cost grid with static obstacles, a Start cell $S$, and a Goal cell $G$.
  - Find an optimal, collision-free orthogonal path $\mathcal{P}$ minimizing total moves.
- **Key Objectives**:
  - Implement a decoupled, independent A* pathfinding engine in Python.
  - Formulate an admissible & consistent Manhattan distance heuristic.
  - Build a responsive, dark-themed Pygame desktop application for interactive demonstration.
  - Provide live step-by-step exploration playback and animated robot travel.
  - Verify correctness and optimality across 15 automated unit test cases.

---

### SLIDE 4: Theoretical Background & The A* Evaluation Function
- **Evaluation Function**:
  $$f(n) = g(n) + h(n)$$
- **Components**:
  - $g(n)$: Exact movement cost from Start $S$ to current node $n$ (accumulated steps).
  - $h(n)$: Estimated cost from node $n$ to Goal $G$ (Manhattan distance).
  - $f(n)$: Total estimated cost of path through node $n$.
- **Heuristic Properties**:
  - **Admissible**: $h(n) \le h^*(n)$ (never overestimates true cost).
  - **Consistent**: $h(n) \le c(n, n') + h(n')$ (satisfies triangle inequality).
- **Manhattan Distance Formula**:
  $$h((r_1, c_1), (r_2, c_2)) = |r_1 - r_2| + |c_1 - c_2|$$
  - Perfect for 4-directional grids with unit costs.

---

### SLIDE 5: Proposed System Architecture
- **Software Design Pattern**: Decoupled Model-View-Controller (MVC) architecture.
- **Component Breakdown**:
  - **Engine (`astar.py`)**: Priority queue (`heapq`), parent tracking, state evaluation, trace generation.
  - **Model (`grid.py`)**: Spatial topology, obstacle sets, start/goal boundaries, state invalidation.
  - **View (`renderer.py`)**: Dark-themed Pygame rendering, vector graphics, animated robot sprite, metric cards.
  - **Controller (`main.py`)**: Event loop, user interaction dispatch, simulation state machine.
- **Key Advantage**: The algorithm is 100% decoupled from the UI, allowing pure benchmark timing via `time.perf_counter()`.

---

### SLIDE 6: Algorithm Workflow & Priority Queue Mechanics
- **Step 1: Initialization**:
  - Insert Start node into Min-Heap with $f = h(\text{start}, \text{goal})$, $g = 0$.
- **Step 2: Min-Heap Extraction**:
  - Pop node with lowest $f(n)$. (Tie-breaking by lower $h(n)$ pulls search toward goal).
- **Step 3: Goal Test**:
  - If popped node is Goal, terminate and reconstruct path.
- **Step 4: Neighbor Expansion**:
  - Examine 4 orthogonal neighbors (North, South, East, West).
  - Skip obstacles, out-of-bounds cells, and closed set nodes.
  - If a shorter $g(n)$ path is found, update parent and push new $(f, h, \dots)$ entry into Min-Heap.
- **Step 5: Trace Recording**:
  - Record discrete event timestamps for visual replay.

---

### SLIDE 7: Application Interface & Interactive Controls
- **User Interface Layout**:
  - **Header Bar**: Project title, subtitle, live status badge.
  - **Main Canvas**: $25 \times 35$ interactive grid showing Start (Green), Goal (Red), Obstacles (Slate), Explored nodes (Blue), Frontier nodes (Amber), and Path (Cyan).
  - **Sidebar**: Environment configuration, live performance metrics, color legend, keyboard shortcuts.
  - **Control Panel**: Run/Pause, Step, Reset, Clear, Random Map, Compare, Mode toggles, Speed selector.
- **Interactive Capabilities**:
  - Paint obstacles with Left-Click & Drag; erase with Right-Click.
  - One-click repositioning of Start and Goal.
  - Step-by-step forward debugging via `[⏭ Step]` button or `S` key.
- *Instructions for Capturing Live Screenshots*:
  1. Launch the app: `python main.py`.
  2. Click `[🎲 Random]` to generate obstacles.
  3. Press `[▶ Run A*]` and take a system screenshot (`Cmd+Shift+4` on macOS, `Win+Shift+S` on Windows).
  4. Click `[⚖ Compare]` to show the side-by-side comparison panel and capture.

---

### SLIDE 8: Test Cases & Empirical Results
- **Automated Verification**:
  - 15 unit tests executed using `unittest` in 0.003 seconds — **100% Pass Rate**.
  - Verified against independent Breadth-First Search (BFS) reference for absolute optimality.
- **Empirical Benchmark: A* vs. Dijkstra**:
  - Tested on identical $25 \times 35$ map with 25% obstacle density:
  - **Optimal Path Length**: 26 moves (Identical for both).
  - **Nodes Explored by A***: **27 nodes**.
  - **Nodes Explored by Dijkstra**: **619 nodes**.
  - **Result**: A* explored **95.6% fewer nodes** and executed over 7 times faster!

---

### SLIDE 9: Limitations & Future Scope
- **Current Limitations**:
  - Discretized orthogonal movement produces $90^\circ$ turns rather than smooth curves.
  - Point-mass assumption does not model robot dimensions or turning radius.
  - Static map assumes obstacles do not move during execution.
- **Future Enhancements**:
  - **8-Directional Movement**: Support diagonals with Octile distance heuristic.
  - **Any-Angle Path Smoothing (Theta*)**: Post-process paths with line-of-sight raycasting.
  - **Dynamic Re-planning ($D^*$ Lite)**: Real-time adaptation when sensors detect moving obstacles.
  - **Hardware Deployment**: Export waypoint sequences to ROS-based mobile robots.

---

### SLIDE 10: Conclusion & Questions
- **Summary**:
  - Successfully designed, implemented, and verified an interactive A* robot path planning simulator.
  - Proved that heuristic guidance dramatically optimizes search efficiency while preserving strict shortest-path optimality.
  - Delivered a clean, modular, and academic-grade project suitable for laboratory demonstrations and college viva.
- **Thank You!**
- **Questions & Answers**: Open floor for examiners and faculty.
