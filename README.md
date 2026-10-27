# Informed Search Algorithm Simulation (Surigao del Norte Map)

An interactive, visual simulation for **Informed Search Algorithms** built purely with **Python's built-in `tkinter`** library.

This simulation renders the satellite map of **Surigao del Norte and Siargao Island**, demonstrating the step-by-step execution, node expansions, priority queue management, and pathfinding logic of:
1. **A\* Search**: $f(n) = g(n) + h(n)$ (guaranteed shortest road cost)
2. **Greedy Best-First Search (GBFS)**: $f(n) = h(n)$ (heuristic-driven search)

---

## Key Features

- **Tkinter Only**: Zero external pip dependencies; uses Python's standard library.
- **Exact Map Overlay**: Renders directly on top of the satellite map (`map.png`, 1024x509) matching exact town pinpoints.
- **Strict Heuristic Requirement**: The goal node always has **zero heuristic** ($h(\text{Goal}) = 0.00\text{ km}$), and straight-line distances (SLD) are dynamically calculated for every town relative to whichever goal node is selected.
- **Interactive Map Controls & Visual Feedback**:
  - **Dashed Lines on Node Visitation Order**: As each node is visited in sequence (e.g., Start at **Surigao City** → visits **Tagana-an** → visits **San Francisco** → visits **Sison**), a prominent dashed line (`dash=(8, 4)`) is drawn directly from the previously visited node to the newly visited node, and **stays permanently on the map** as the search progresses.
  - **Dashed Halo Rings**: Every visited node has a dashed outline ring indicating it has been explored.
  - **Final Path Highlighting**: Only when the Goal is reached does the final optimal path get drawn in a solid glowing emerald green stroke over the search tree.
  - **Left-Click** any town on the map to set it as **Start Node**.
  - **Right-Click** or **Shift + Left-Click** any town on the map to set it as **Goal Node**.
  - **Hover** over any town to see an instant tooltip with its coordinates, heuristic to goal, state, and connected roads.
- **Step-by-Step Playback**:
  - `Play` / `Pause` with dynamic animation speed control (100ms - 1000ms).
  - `Step Forward` and `Step Backward` to inspect every single priority queue expansion.
  - `First Step` and `Finish` to instantly view the initial state or the final optimal path.
- **Deep-Dive Inspection Tabs**:
  1. **Frontier (Open List)**: Live priority queue table sorted by evaluation score $f(n)$, showing $g(n)$, $h(n)$, and parent pointers.
  2. **Heuristic Table $h(n)$**: Complete table showing every town's straight-line distance to the current goal, with the goal highlighted at $h=0.00\text{ km}$.
  3. **Search Trace Log**: Chronological narrative log explaining every node popped, skipped, or added.
  4. **Map Edges & Costs**: All 23 connections with their Step Costs and Straight-Line Distances.
- **⚔ Side-by-Side Comparison Modal**:
  - Compare A* vs Greedy Best-First Search on the exact same route.
  - Compares Total Step Cost, Nodes Explored, Simulation Steps, and Path Choice.
  - Clearly demonstrates how Greedy can take suboptimal detours (e.g., `Malimono → Tagana-an`: Greedy takes 96 km while A* finds the optimal 55 km path!).
- **Preset Demonstrations**:
  - `★ Malimono -> Tagana-an (Greedy Suboptimal Demo: 96km vs 55km)`
  - `Surigao City -> Dapa (Mainland to Siargao Island)`
  - `Surigao City -> General Luna (Island Crossing)`
  - `Santa Monica -> Alegria (North to South)`
  - `Del Carmen -> Burgos (Siargao Local)`

---

## Dataset (21 Towns & 23 Connections)

| Edge | Straight Line Distance (SLD) | Step Cost (Road) |
|---|---|---|
| Surigao City ↔ Tagana-an | 14.12 km | 23.0 km |
| Tagana-an ↔ Sison | 7.31 km | 15.0 km |
| Sison ↔ Placer | 8.00 km | 20.0 km |
| Placer ↔ Bacuag | 6.75 km | 15.0 km |
| Bacuag ↔ Gigaquit | 6.70 km | 12.0 km |
| Bacuag ↔ Claver | 11.10 km | 14.0 km |
| Claver ↔ Socorro | 26.00 km | 37.0 km |
| Socorro ↔ Dapa | 18.00 km | 30.0 km |
| Dapa ↔ General Luna | 11.86 km | 15.0 km |
| Dapa ↔ Pilar | 13.00 km | 26.0 km |
| Pilar ↔ San Isidro | 8.10 km | 12.0 km |
| San Isidro ↔ Burgos | 9.06 km | 12.0 km |
| Burgos ↔ Santa Monica | 4.15 km | 11.0 km |
| Santa Monica ↔ San Benito | 7.39 km | 8.0 km |
| San Benito ↔ Del Carmen | 10.57 km | 13.0 km |
| Del Carmen ↔ San Isidro | 14.77 km | 16.0 km |
| Del Carmen ↔ Dapa | 15.19 km | 25.0 km |
| Placer ↔ Tubod | 11.67 km | 16.0 km |
| Tubod ↔ Alegria | 9.61 km | 11.0 km |
| Tubod ↔ Mainit | 5.42 km | 7.0 km |
| Mainit ↔ Malimono | 15.87 km | 38.0 km |
| Malimono ↔ San Francisco | 17.80 km | 21.0 km |
| San Francisco ↔ Surigao City | 7.59 km | 11.0 km |

---

## How to Run

### Option 1: Using Python
Open your terminal in this directory and run:
```bash
python main.py
```
*(or `python app.py`)*

### Option 2: Using the Windows Batch Launcher
Double-click `run.bat` in the project folder.

---

## Keyboard Shortcuts

- `Space`: Play / Pause auto-simulation
- `Right Arrow (→)`: Step Forward (advance 1 step)
- `Left Arrow (←)`: Step Backward (rewind 1 step)
- `Home`: Jump to Initial Step (Step 0)
- `End`: Jump to Final Step (Goal reached)
- `R` or `r`: Reset simulation
