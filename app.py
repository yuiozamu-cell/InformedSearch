"""
Surigao Informed Search Simulation Application
Implements A* Search and Greedy Best-First Search using Tkinter only.
Uses real map coordinates and distances for Surigao del Norte & Siargao Island.
"""

import math
import heapq
import os
import tkinter as tk
from tkinter import ttk, messagebox

# ---------------------------------------------------------
# GRAPH DATA DEFINITION
# ---------------------------------------------------------
# Calibrated 2D coordinates in kilometers (matches all straight-line distances)
COORDINATES_KM = {
    'Malimono': (10.969, 21.371),
    'San Francisco': (19.319, 5.651),
    'Mainit': (20.145, 34.320),
    'Alegria': (22.931, 43.484),
    'Sison': (25.329, 21.935),
    'Tubod': (25.564, 34.241),
    'Surigao City': (26.739, 7.248),
    'Tagana-an': (32.424, 20.173),
    'Placer': (32.712, 25.015),
    'Bacuag': (34.570, 31.504),
    'Gigaquit': (40.110, 35.271),
    'Claver': (42.926, 38.809),
    'Socorro': (68.584, 43.008),
    'Del Carmen': (78.828, 17.209),
    'Dapa': (82.737, 31.888),
    'San Benito': (86.044, 9.486),
    'Santa Monica': (91.316, 4.307),
    'Pilar': (91.811, 22.579),
    'San Isidro': (93.371, 14.631),
    'General Luna': (94.522, 33.223),
    'Burgos': (95.204, 5.758),
}

# Exact pixel coordinates on the 1024x509 map image
PIN_PIXELS = {
    'Malimono': (104, 189),
    'San Francisco': (204, 51),
    'Mainit': (176, 321),
    'Alegria': (192, 414),
    'Tubod': (229, 325),
    'Sison': (240, 198),
    'Surigao City': (270, 69),
    'Tagana-an': (312, 194),
    'Placer': (310, 239),
    'Bacuag': (322, 304),
    'Gigaquit': (371, 346),
    'Claver': (394, 381),
    'Socorro': (644, 445),
    'Del Carmen': (760, 203),
    'Dapa': (791, 346),
    'San Benito': (832, 135),
    'Santa Monica': (885, 91),
    'Burgos': (921, 106),
    'San Isidro': (901, 188),
    'Pilar': (884, 263),
    'General Luna': (906, 368),
}

# Edges: (City1, City2, Straight-Line Distance km, Step Cost km)
EDGES_RAW = [
    ('Surigao City', 'Tagana-an', 14.12, 23.0),
    ('Tagana-an', 'Sison', 7.31, 15.0),
    ('Sison', 'Placer', 8.00, 20.0),
    ('Placer', 'Bacuag', 6.75, 15.0),
    ('Bacuag', 'Gigaquit', 6.70, 12.0),
    ('Bacuag', 'Claver', 11.10, 14.0),
    ('Claver', 'Socorro', 26.00, 37.0),
    ('Socorro', 'Dapa', 18.00, 30.0),
    ('Dapa', 'General Luna', 11.86, 15.0),
    ('Dapa', 'Pilar', 13.00, 26.0),
    ('Pilar', 'San Isidro', 8.10, 12.0),
    ('San Isidro', 'Burgos', 9.06, 12.0),
    ('Burgos', 'Santa Monica', 4.15, 11.0),
    ('Santa Monica', 'San Benito', 7.39, 8.0),
    ('San Benito', 'Del Carmen', 10.57, 13.0),
    ('Del Carmen', 'San Isidro', 14.77, 16.0),
    ('Del Carmen', 'Dapa', 15.19, 25.0),
    ('Placer', 'Tubod', 11.67, 16.0),
    ('Tubod', 'Alegria', 9.61, 11.0),
    ('Tubod', 'Mainit', 5.42, 7.0),
    ('Mainit', 'Malimono', 15.87, 38.0),
    ('Malimono', 'San Francisco', 17.80, 21.0),
    ('San Francisco', 'Surigao City', 7.59, 11.0),
]

# Build adjacency lookup
ADJACENCY = {city: [] for city in COORDINATES_KM}
EDGE_MAP = {}
for u, v, sld, cost in EDGES_RAW:
    ADJACENCY[u].append((v, cost, sld))
    ADJACENCY[v].append((u, cost, sld))
    EDGE_MAP[(u, v)] = (cost, sld)
    EDGE_MAP[(v, u)] = (cost, sld)


def get_heuristic(city, goal):
    """
    Computes Straight Line Distance (SLD) in km from city to goal.
    Crucial requirement: h(goal) is strictly 0.00 km.
    """
    if city == goal:
        return 0.0
    x1, y1 = COORDINATES_KM[city]
    x2, y2 = COORDINATES_KM[goal]
    return round(math.hypot(x1 - x2, y1 - y2), 2)


# ---------------------------------------------------------
# INFORMED SEARCH SIMULATION ENGINE (SNAPSHOT GENERATOR)
# ---------------------------------------------------------
def generate_search_trace(start_node, goal_node, algorithm='A*'):
    """
    Precomputes step-by-step search execution states for full interactive playback.
    Supports forward, backward, pause, and jump-to-step without state mutation bugs.
    """
    snapshots = []
    h_start = get_heuristic(start_node, goal_node)
    g_start = 0.0
    f_start = (g_start + h_start) if algorithm == 'A*' else h_start

    # Priority queue format: (f_score, h_score, tie_counter, current_node, g_score, path)
    counter = 0
    pq = [(f_start, h_start, counter, start_node, g_start, [start_node])]
    explored = {}  # node -> best g_score found
    visited_sequence = [start_node]  # sequence of nodes visited in chronological order

    # Step 0: Initial State
    snapshots.append({
        'step_num': 0,
        'action': 'INITIAL',
        'current_node': None,
        'f_score': None,
        'g_score': None,
        'h_score': None,
        'current_path': [],
        'frontier': [(start_node, f_start, g_start, h_start, None, [start_node])],
        'explored': {},
        'visited_edges': [],
        'description': (
            f"Initialized {algorithm} search from '{start_node}' to '{goal_node}'. "
            f"Frontier contains Start node with h={h_start:.2f} km."
        ),
        'final_path': None,
        'total_cost': None,
        'newly_added': [start_node],
    })

    step_counter = 0
    goal_found = False

    while pq:
        f_curr, h_curr, _, curr_node, g_curr, curr_path = heapq.heappop(pq)
        step_counter += 1

        # In A* graph search, skip if a cheaper path to curr_node was already expanded
        if curr_node in explored and explored[curr_node] <= g_curr:
            frontier_repr = [
                (item[3], item[0], item[4], item[1], item[5][-2] if len(item[5]) > 1 else None, item[5])
                for item in sorted(pq)
            ]
            current_edges = [(visited_sequence[i], visited_sequence[i + 1]) for i in range(len(visited_sequence) - 1)]
            snapshots.append({
                'step_num': step_counter,
                'action': 'SKIP',
                'current_node': curr_node,
                'f_score': f_curr,
                'g_score': g_curr,
                'h_score': h_curr,
                'current_path': curr_path,
                'frontier': frontier_repr,
                'explored': dict(explored),
                'visited_edges': current_edges,
                'description': (
                    f"Skipping '{curr_node}' (popped with g={g_curr:.1f}): "
                    f"Already explored with lower/equal cost g={explored[curr_node]:.1f} km."
                ),
                'final_path': None,
                'total_cost': None,
                'newly_added': [],
            })
            continue

        explored[curr_node] = g_curr
        if not visited_sequence or visited_sequence[-1] != curr_node:
            visited_sequence.append(curr_node)
        current_edges = [(visited_sequence[i], visited_sequence[i + 1]) for i in range(len(visited_sequence) - 1)]

        # Check Goal condition
        if curr_node == goal_node:
            frontier_repr = [
                (item[3], item[0], item[4], item[1], item[5][-2] if len(item[5]) > 1 else None, item[5])
                for item in sorted(pq)
            ]
            snapshots.append({
                'step_num': step_counter,
                'action': 'GOAL_REACHED',
                'current_node': curr_node,
                'f_score': f_curr,
                'g_score': g_curr,
                'h_score': 0.0,
                'current_path': curr_path,
                'frontier': frontier_repr,
                'explored': dict(explored),
                'visited_edges': current_edges,
                'description': (
                    f"TARGET GOAL REACHED! '{goal_node}' popped with zero heuristic h=0.00 km. "
                    f"Optimal Path: {' -> '.join(curr_path)} | Total Step Cost: {g_curr:.1f} km."
                ),
                'final_path': curr_path,
                'total_cost': g_curr,
                'newly_added': [],
            })
            goal_found = True
            break

        # Expand Neighbors
        newly_added_list = []
        for neighbor, step_cost, _ in ADJACENCY[curr_node]:
            new_g = g_curr + step_cost
            new_h = get_heuristic(neighbor, goal_node)
            new_f = (new_g + new_h) if algorithm == 'A*' else new_h

            if algorithm == 'A*':
                if neighbor not in explored or new_g < explored[neighbor]:
                    counter += 1
                    heapq.heappush(pq, (new_f, new_h, counter, neighbor, new_g, curr_path + [neighbor]))
                    newly_added_list.append((neighbor, new_f, new_g, new_h, step_cost))
            else:  # Greedy Best-First Search
                if neighbor not in explored:
                    counter += 1
                    heapq.heappush(pq, (new_h, new_h, counter, neighbor, new_g, curr_path + [neighbor]))
                    newly_added_list.append((neighbor, new_h, new_g, new_h, step_cost))

        frontier_repr = [
            (item[3], item[0], item[4], item[1], item[5][-2] if len(item[5]) > 1 else None, item[5])
            for item in sorted(pq)
        ]

        if newly_added_list:
            if algorithm == 'A*':
                nbr_details = ', '.join(
                    f"{n}(f={f:.1f}, g={g:.1f}, h={h:.1f})" for n, f, g, h, c in newly_added_list
                )
            else:
                nbr_details = ', '.join(
                    f"{n}(h={h:.1f}, g={g:.1f})" for n, f, g, h, c in newly_added_list
                )
            msg = f"Expanded '{curr_node}' (g={g_curr:.1f}, h={h_curr:.2f}, f={f_curr:.2f}). Pushed neighbors: {nbr_details}."
        else:
            msg = f"Expanded '{curr_node}' (g={g_curr:.1f}, h={h_curr:.2f}). No unvisited neighbors to add."

        snapshots.append({
            'step_num': step_counter,
            'action': 'EXPAND',
            'current_node': curr_node,
            'f_score': f_curr,
            'g_score': g_curr,
            'h_score': h_curr,
            'current_path': curr_path,
            'frontier': frontier_repr,
            'explored': dict(explored),
            'visited_edges': current_edges,
            'description': msg,
            'final_path': None,
            'total_cost': None,
            'newly_added': [item[0] for item in newly_added_list],
        })

    if not goal_found:
        final_edges = [(visited_sequence[i], visited_sequence[i + 1]) for i in range(len(visited_sequence) - 1)]
        snapshots.append({
            'step_num': step_counter + 1,
            'action': 'EXHAUSTED',
            'current_node': None,
            'f_score': None,
            'g_score': None,
            'h_score': None,
            'current_path': [],
            'frontier': [],
            'explored': dict(explored),
            'visited_edges': final_edges,
            'description': f"Frontier exhausted. No path found from '{start_node}' to '{goal_node}'.",
            'final_path': None,
            'total_cost': None,
            'newly_added': [],
        })

    return snapshots


# ---------------------------------------------------------
# TKINTER GUI APPLICATION
# ---------------------------------------------------------
class InformedSearchApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Informed Search Simulation | Surigao del Norte (A* & Greedy Best-First)")
        self.geometry("1420x860")
        self.minsize(1180, 720)
        self.configure(bg="#181926")

        # Application state
        self.start_city = tk.StringVar(value="Surigao City")
        self.goal_city = tk.StringVar(value="Dapa")
        self.algorithm = tk.StringVar(value="A*")
        self.preset_var = tk.StringVar(value="Surigao City -> Dapa (Default)")
        self.anim_speed_ms = tk.IntVar(value=600)
        self.is_playing = False
        self.show_edge_costs = tk.BooleanVar(value=True)
        self.show_heuristics = tk.BooleanVar(value=True)
        self.show_visited_dashes = tk.BooleanVar(value=True)

        self.snapshots = []
        self.current_step_idx = 0
        self.timer_id = None
        self.map_photo = None
        self.hovered_city = None

        self._init_theme()
        self._load_map_image()
        self._build_layout()
        self._bind_keyboard_shortcuts()
        self._recompute_and_reset()

    def _init_theme(self):
        """Configure modern dark ttk theme styles."""
        self.colors = {
            'bg': '#181926',
            'panel_bg': '#24273a',
            'card_bg': '#1e2030',
            'border': '#363a4f',
            'text_main': '#cad3f5',
            'text_dim': '#8087a2',
            'accent': '#8aadf4',
            'start': '#a6da95',     # Emerald Green
            'goal': '#ed8796',      # Coral Red
            'current': '#eed49f',   # Golden Yellow
            'frontier': '#f5a97f',  # Peach Orange
            'explored': '#b7bdf8',  # Indigo Violet
            'path_glow': '#eed49f',
            'path_line': '#8bd5ca', # Cyan Teal
            'road_line': '#494d64',
        }

        style = ttk.Style(self)
        try:
            style.theme_use('clam')
        except Exception:
            pass

        style.configure('.', background=self.colors['panel_bg'], foreground=self.colors['text_main'], font=('Segoe UI', 9))
        style.configure('TFrame', background=self.colors['panel_bg'])
        style.configure('Card.TFrame', background=self.colors['card_bg'], relief='flat')
        style.configure('TLabel', background=self.colors['panel_bg'], foreground=self.colors['text_main'], font=('Segoe UI', 9))
        style.configure('Card.TLabel', background=self.colors['card_bg'], foreground=self.colors['text_main'], font=('Segoe UI', 9))
        style.configure('Header.TLabel', background=self.colors['panel_bg'], foreground=self.colors['accent'], font=('Segoe UI', 13, 'bold'))
        style.configure('SubHeader.TLabel', background=self.colors['panel_bg'], foreground=self.colors['text_dim'], font=('Segoe UI', 9))
        style.configure('StatVal.TLabel', background=self.colors['card_bg'], foreground=self.colors['current'], font=('Segoe UI', 11, 'bold'))
        style.configure('StatTitle.TLabel', background=self.colors['card_bg'], foreground=self.colors['text_dim'], font=('Segoe UI', 8))

        style.configure('TCombobox', fieldbackground=self.colors['card_bg'], background=self.colors['card_bg'], foreground='#cad3f5', bordercolor=self.colors['border'])
        style.map('TCombobox', fieldbackground=[('readonly', self.colors['card_bg'])], foreground=[('readonly', '#cad3f5')])

        # Treeview styling
        style.configure('Treeview',
                        background=self.colors['card_bg'],
                        foreground=self.colors['text_main'],
                        fieldbackground=self.colors['card_bg'],
                        rowheight=24,
                        bordercolor=self.colors['border'],
                        font=('Segoe UI', 8))
        style.configure('Treeview.Heading',
                        background=self.colors['panel_bg'],
                        foreground=self.colors['accent'],
                        font=('Segoe UI', 8, 'bold'))
        style.map('Treeview', background=[('selected', '#363a4f')], foreground=[('selected', '#f4dbd6')])

        # Notebook styling
        style.configure('TNotebook', background=self.colors['panel_bg'], tabmargins=[2, 5, 2, 0])
        style.configure('TNotebook.Tab', background=self.colors['card_bg'], foreground=self.colors['text_dim'], padding=[10, 4], font=('Segoe UI', 9))
        style.map('TNotebook.Tab', background=[('selected', self.colors['panel_bg'])], foreground=[('selected', self.colors['accent'])])

    def _load_map_image(self):
        """Loads the satellite image from local directory or relative fallback."""
        img_candidates = ['map.png', os.path.join(os.path.dirname(__file__), 'map.png')]
        for path in img_candidates:
            if os.path.exists(path):
                try:
                    self.map_photo = tk.PhotoImage(file=path)
                    return
                except Exception as e:
                    print(f"Warning: could not load image at {path}: {e}")
        self.map_photo = None

    def _bind_keyboard_shortcuts(self):
        """Allows keyboard control for quick demonstration."""
        self.bind('<space>', lambda e: self.toggle_play())
        self.bind('<Right>', lambda e: self.step_forward())
        self.bind('<Left>', lambda e: self.step_back())
        self.bind('<Home>', lambda e: self.step_first())
        self.bind('<End>', lambda e: self.step_last())
        self.bind('<r>', lambda e: self.reset_sim())
        self.bind('<R>', lambda e: self.reset_sim())

    def _build_layout(self):
        """Builds the main window grid layout: header, map pane, controls, and deep-dive info pane."""
        # Top Header Bar
        header_frame = ttk.Frame(self, padding=(12, 8, 12, 8))
        header_frame.pack(side=tk.TOP, fill=tk.X)

        title_box = ttk.Frame(header_frame)
        title_box.pack(side=tk.LEFT, fill=tk.Y)

        ttk.Label(title_box, text="Surigao Informed Search Simulation", style='Header.TLabel').pack(anchor='w')
        ttk.Label(
            title_box,
            text="A* Search (f = g + h) vs Greedy Best-First Search (f = h) on Surigao Del Norte & Siargao Island",
            style='SubHeader.TLabel'
        ).pack(anchor='w')

        # Goal Zero Heuristic Badge
        badge_frame = tk.Frame(header_frame, bg='#363a4f', bd=1, padx=10, pady=4)
        badge_frame.pack(side=tk.RIGHT, padx=5)
        tk.Label(
            badge_frame,
            text="✓ Goal Heuristic h(Goal) = 0.00 km | Admissible & Consistent SLD",
            font=('Segoe UI', 9, 'bold'),
            fg='#a6da95',
            bg='#363a4f'
        ).pack()

        # Main Paned / Split Layout
        main_split = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        main_split.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        # Left Container: Map Canvas + Playback Controls
        left_container = ttk.Frame(main_split)
        main_split.add(left_container, weight=3)

        # Right Container: Step Tracker, Frontier, Heuristic Table, Comparison
        right_container = ttk.Frame(main_split)
        main_split.add(right_container, weight=2)

        self._build_left_pane(left_container)
        self._build_right_pane(right_container)

    def _build_left_pane(self, parent):
        """Constructs the interactive Map Canvas and Search Control Bar."""
        # Canvas Frame with dark border
        canvas_border = tk.Frame(parent, bg=self.colors['border'], bd=2)
        canvas_border.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=4, pady=4)

        canvas_w = self.map_photo.width() if self.map_photo else 1024
        canvas_h = self.map_photo.height() if self.map_photo else 509

        self.canvas = tk.Canvas(
            canvas_border,
            width=canvas_w,
            height=canvas_h,
            bg='#11111b',
            highlightthickness=0,
            cursor='crosshair'
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Canvas Event bindings
        self.canvas.bind('<Motion>', self._on_canvas_motion)
        self.canvas.bind('<Button-1>', self._on_canvas_click_start)
        self.canvas.bind('<Button-3>', self._on_canvas_click_goal)
        self.canvas.bind('<Shift-Button-1>', self._on_canvas_click_goal)

        # Legend Bar under canvas
        legend_frame = tk.Frame(parent, bg=self.colors['panel_bg'], pady=3)
        legend_frame.pack(side=tk.TOP, fill=tk.X, padx=4)

        legends = [
            ("Start Node", self.colors['start']),
            ("Goal (h=0)", self.colors['goal']),
            ("Current Node", self.colors['current']),
            ("Frontier (Open)", self.colors['frontier']),
            ("Visited Nodes (- -)", '#eed49f'),
            ("Final Path (Solid)", self.colors['path_line']),
        ]
        tk.Label(legend_frame, text="Legend:", font=('Segoe UI', 8, 'bold'), fg=self.colors['text_dim'], bg=self.colors['panel_bg']).pack(side=tk.LEFT, padx=(2, 6))
        for text, col in legends:
            tk.Label(legend_frame, text="●", fg=col, bg=self.colors['panel_bg'], font=('Segoe UI', 11)).pack(side=tk.LEFT)
            tk.Label(legend_frame, text=f" {text}  ", fg=self.colors['text_main'], bg=self.colors['panel_bg'], font=('Segoe UI', 8)).pack(side=tk.LEFT)

        tk.Label(legend_frame, text="(Left-Click = Start | Right-Click / Shift-Click = Goal | Space = Play/Pause)",
                 fg=self.colors['text_dim'], bg=self.colors['panel_bg'], font=('Segoe UI', 8, 'italic')).pack(side=tk.RIGHT, padx=4)

        # Control Panel Frame
        control_card = tk.Frame(parent, bg=self.colors['card_bg'], bd=1, relief='solid', padx=10, pady=8)
        control_card.pack(side=tk.BOTTOM, fill=tk.X, padx=4, pady=4)

        # Row 1: Presets + Configurations (Start, Goal, Algorithm)
        row1 = tk.Frame(control_card, bg=self.colors['card_bg'])
        row1.pack(fill=tk.X, pady=(0, 6))

        tk.Label(row1, text="Preset Route:", fg=self.colors['accent'], bg=self.colors['card_bg'], font=('Segoe UI', 8, 'bold')).pack(side=tk.LEFT, padx=(0, 4))
        presets = [
            "Surigao City -> Dapa (Default)",
            "★ Malimono -> Tagana-an (Greedy Suboptimal Demo: 96km vs 55km)",
            "Surigao City -> General Luna (Island Crossing)",
            "Santa Monica -> Alegria (North to South)",
            "Del Carmen -> Burgos (Siargao Local)",
        ]
        self.combo_presets = ttk.Combobox(row1, textvariable=self.preset_var, values=presets, width=28, state='readonly')
        self.combo_presets.pack(side=tk.LEFT, padx=(0, 14))
        self.combo_presets.bind("<<ComboboxSelected>>", self._on_preset_selected)

        tk.Label(row1, text="Start:", fg=self.colors['start'], bg=self.colors['card_bg'], font=('Segoe UI', 9, 'bold')).pack(side=tk.LEFT, padx=(0, 4))
        self.combo_start = ttk.Combobox(row1, textvariable=self.start_city, values=sorted(COORDINATES_KM.keys()), width=13, state='readonly')
        self.combo_start.pack(side=tk.LEFT, padx=(0, 8))
        self.combo_start.bind("<<ComboboxSelected>>", lambda e: self._recompute_and_reset())

        tk.Label(row1, text="Goal:", fg=self.colors['goal'], bg=self.colors['card_bg'], font=('Segoe UI', 9, 'bold')).pack(side=tk.LEFT, padx=(0, 4))
        self.combo_goal = ttk.Combobox(row1, textvariable=self.goal_city, values=sorted(COORDINATES_KM.keys()), width=13, state='readonly')
        self.combo_goal.pack(side=tk.LEFT, padx=(0, 8))
        self.combo_goal.bind("<<ComboboxSelected>>", lambda e: self._recompute_and_reset())

        # Swap button
        btn_swap = tk.Button(row1, text="⇄ Swap", command=self._swap_start_goal, bg='#363a4f', fg='#cad3f5', relief='flat', font=('Segoe UI', 8, 'bold'), padx=6)
        btn_swap.pack(side=tk.LEFT, padx=(0, 12))

        # Algorithm Radio buttons
        tk.Label(row1, text="Algo:", fg=self.colors['accent'], bg=self.colors['card_bg'], font=('Segoe UI', 9, 'bold')).pack(side=tk.LEFT, padx=(0, 4))
        rb1 = tk.Radiobutton(row1, text="A* Search", variable=self.algorithm, value="A*",
                             command=self._recompute_and_reset, bg=self.colors['card_bg'], fg='#cad3f5', selectcolor='#363a4f', font=('Segoe UI', 9, 'bold'))
        rb1.pack(side=tk.LEFT, padx=(0, 4))

        rb2 = tk.Radiobutton(row1, text="Greedy Best-First", variable=self.algorithm, value="Greedy",
                             command=self._recompute_and_reset, bg=self.colors['card_bg'], fg='#cad3f5', selectcolor='#363a4f', font=('Segoe UI', 9, 'bold'))
        rb2.pack(side=tk.LEFT, padx=(0, 4))

        # Row 2: Playback buttons, Speed, and Compare
        row2 = tk.Frame(control_card, bg=self.colors['card_bg'])
        row2.pack(fill=tk.X, pady=(2, 0))

        btn_style = {'relief': 'flat', 'font': ('Segoe UI', 9, 'bold'), 'padx': 7, 'pady': 3}

        self.btn_first = tk.Button(row2, text="⏮ First", command=self.step_first, bg='#363a4f', fg='#cad3f5', **btn_style)
        self.btn_first.pack(side=tk.LEFT, padx=2)

        self.btn_prev = tk.Button(row2, text="◀ Step Back", command=self.step_back, bg='#363a4f', fg='#cad3f5', **btn_style)
        self.btn_prev.pack(side=tk.LEFT, padx=2)

        self.btn_play = tk.Button(row2, text="▶ Play", command=self.toggle_play, bg='#8aadf4', fg='#181926', **btn_style)
        self.btn_play.pack(side=tk.LEFT, padx=4)

        self.btn_next = tk.Button(row2, text="Step Forward ▶", command=self.step_forward, bg='#8aadf4', fg='#181926', **btn_style)
        self.btn_next.pack(side=tk.LEFT, padx=2)

        self.btn_last = tk.Button(row2, text="⏭ Finish", command=self.step_last, bg='#363a4f', fg='#cad3f5', **btn_style)
        self.btn_last.pack(side=tk.LEFT, padx=2)

        self.btn_reset = tk.Button(row2, text="↺ Reset", command=self.reset_sim, bg='#494d64', fg='#cad3f5', **btn_style)
        self.btn_reset.pack(side=tk.LEFT, padx=(6, 12))

        # Speed Slider
        tk.Label(row2, text="Speed:", fg=self.colors['text_dim'], bg=self.colors['card_bg'], font=('Segoe UI', 8)).pack(side=tk.LEFT)
        speed_slider = tk.Scale(row2, from_=1000, to=100, orient=tk.HORIZONTAL, variable=self.anim_speed_ms,
                                showvalue=0, length=70, bg=self.colors['card_bg'], highlightthickness=0, troughcolor='#363a4f')
        speed_slider.pack(side=tk.LEFT, padx=3)
        tk.Label(row2, text="Fast", fg=self.colors['text_dim'], bg=self.colors['card_bg'], font=('Segoe UI', 8)).pack(side=tk.LEFT, padx=(0, 12))

        # Compare Modal Launcher
        btn_compare = tk.Button(row2, text="⚔ Compare A* vs Greedy", command=self.open_comparison_view,
                                bg='#c6a0f6', fg='#181926', font=('Segoe UI', 9, 'bold'), relief='flat', padx=10, pady=3)
        btn_compare.pack(side=tk.RIGHT)

        # Canvas View Options
        chk_cost = tk.Checkbutton(row2, text="Edge Costs", variable=self.show_edge_costs, command=self._render_current_step,
                                  bg=self.colors['card_bg'], fg='#cad3f5', selectcolor='#363a4f', font=('Segoe UI', 8))
        chk_cost.pack(side=tk.RIGHT, padx=3)

        chk_heur = tk.Checkbutton(row2, text="Node Heuristics", variable=self.show_heuristics, command=self._render_current_step,
                                  bg=self.colors['card_bg'], fg='#cad3f5', selectcolor='#363a4f', font=('Segoe UI', 8))
        chk_heur.pack(side=tk.RIGHT, padx=3)

        chk_visited = tk.Checkbutton(row2, text="Visited Dashes", variable=self.show_visited_dashes, command=self._render_current_step,
                                     bg=self.colors['card_bg'], fg='#cad3f5', selectcolor='#363a4f', font=('Segoe UI', 8))
        chk_visited.pack(side=tk.RIGHT, padx=3)

    def _build_right_pane(self, parent):
        """Constructs information cards, step tracker, frontier queue, and heuristic table."""
        # Top Status Card
        status_card = tk.Frame(parent, bg=self.colors['card_bg'], bd=1, relief='solid', padx=10, pady=8)
        status_card.pack(side=tk.TOP, fill=tk.X, padx=4, pady=4)

        # Metrics Row
        stats_grid = tk.Frame(status_card, bg=self.colors['card_bg'])
        stats_grid.pack(fill=tk.X, pady=(0, 6))

        # Col 1: Step
        c1 = ttk.Frame(stats_grid)
        c1.pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Label(c1, text="STEP", style='StatTitle.TLabel').pack(anchor='w')
        self.lbl_step_metric = ttk.Label(c1, text="0 / 0", style='StatVal.TLabel')
        self.lbl_step_metric.pack(anchor='w')

        # Col 2: Current Node
        c2 = ttk.Frame(stats_grid)
        c2.pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Label(c2, text="EVALUATING NODE", style='StatTitle.TLabel').pack(anchor='w')
        self.lbl_curr_node = tk.Label(c2, text="None", font=('Segoe UI', 11, 'bold'), fg=self.colors['current'], bg=self.colors['card_bg'])
        self.lbl_curr_node.pack(anchor='w')

        # Col 3: Cost / Function breakdown
        c3 = ttk.Frame(stats_grid)
        c3.pack(side=tk.LEFT, expand=True, fill=tk.X)
        self.lbl_eval_title = ttk.Label(c3, text="EVALUATION f(n)", style='StatTitle.TLabel')
        self.lbl_eval_title.pack(anchor='w')
        self.lbl_eval_val = ttk.Label(c3, text="--", style='StatVal.TLabel')
        self.lbl_eval_val.pack(anchor='w')

        # Col 4: Total Path Cost so far
        c4 = ttk.Frame(stats_grid)
        c4.pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Label(c4, text="TOTAL COST", style='StatTitle.TLabel').pack(anchor='w')
        self.lbl_cost_val = ttk.Label(c4, text="--", style='StatVal.TLabel')
        self.lbl_cost_val.pack(anchor='w')

        # Explanation box
        self.lbl_step_desc = tk.Label(
            status_card,
            text="Ready to search.",
            bg='#181926',
            fg=self.colors['text_main'],
            font=('Segoe UI', 9),
            wraplength=460,
            justify=tk.LEFT,
            padx=8,
            pady=5,
            bd=1,
            relief='flat'
        )
        self.lbl_step_desc.pack(fill=tk.X)

        # Tabbed Notebook for Deep-Dive Information
        self.notebook = ttk.Notebook(parent)
        self.notebook.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=4, pady=4)

        # Tab 1: Frontier Priority Queue (Open List)
        self.tab_frontier = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_frontier, text=" Frontier (Open List) ")
        self._build_frontier_tab(self.tab_frontier)

        # Tab 2: Heuristic Table (h values to Goal)
        self.tab_heuristics = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_heuristics, text=" Heuristic Table h(n) ")
        self._build_heuristics_tab(self.tab_heuristics)

        # Tab 3: Step-by-Step History Log
        self.tab_history = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_history, text=" Search Trace Log ")
        self._build_history_tab(self.tab_history)

        # Tab 4: Road Network Details (23 Edges)
        self.tab_network = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_network, text=" Map Edges & Costs ")
        self._build_network_tab(self.tab_network)

    def _build_frontier_tab(self, parent):
        """Displays live Priority Queue sorted by evaluation function f(n)."""
        frame = ttk.Frame(parent, padding=4)
        frame.pack(fill=tk.BOTH, expand=True)

        cols = ('rank', 'node', 'f', 'g', 'h', 'parent')
        self.tree_frontier = ttk.Treeview(frame, columns=cols, show='headings', selectmode='browse')
        self.tree_frontier.heading('rank', text='#')
        self.tree_frontier.heading('node', text='Node')
        self.tree_frontier.heading('f', text='f(n)')
        self.tree_frontier.heading('g', text='g(n) [Cost]')
        self.tree_frontier.heading('h', text='h(n) [SLD]')
        self.tree_frontier.heading('parent', text='Parent Node')

        self.tree_frontier.column('rank', width=28, anchor='center')
        self.tree_frontier.column('node', width=115, anchor='w')
        self.tree_frontier.column('f', width=65, anchor='e')
        self.tree_frontier.column('g', width=75, anchor='e')
        self.tree_frontier.column('h', width=75, anchor='e')
        self.tree_frontier.column('parent', width=105, anchor='w')

        sb = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=self.tree_frontier.yview)
        self.tree_frontier.configure(yscrollcommand=sb.set)
        self.tree_frontier.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

    def _build_heuristics_tab(self, parent):
        """Displays Straight Line Distance heuristic to the chosen Goal for every single town."""
        frame = ttk.Frame(parent, padding=4)
        frame.pack(fill=tk.BOTH, expand=True)

        info_lbl = tk.Label(
            frame,
            text="Straight Line Distance to current Goal (Goal h = 0.00 km):",
            bg=self.colors['panel_bg'],
            fg=self.colors['accent'],
            font=('Segoe UI', 9, 'bold')
        )
        info_lbl.pack(anchor='w', pady=(0, 4))

        cols = ('node', 'h_val', 'status')
        self.tree_heur = ttk.Treeview(frame, columns=cols, show='headings', selectmode='browse')
        self.tree_heur.heading('node', text='Town / City')
        self.tree_heur.heading('h_val', text='Heuristic h(n) to Goal')
        self.tree_heur.heading('status', text='Status')

        self.tree_heur.column('node', width=150, anchor='w')
        self.tree_heur.column('h_val', width=140, anchor='e')
        self.tree_heur.column('status', width=130, anchor='center')

        sb = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=self.tree_heur.yview)
        self.tree_heur.configure(yscrollcommand=sb.set)
        self.tree_heur.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

    def _build_history_tab(self, parent):
        """Displays full execution step log with details."""
        frame = ttk.Frame(parent, padding=4)
        frame.pack(fill=tk.BOTH, expand=True)

        self.txt_history = tk.Text(
            frame,
            bg=self.colors['card_bg'],
            fg=self.colors['text_main'],
            font=('Consolas', 9),
            bd=0,
            wrap=tk.WORD
        )
        sb = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=self.txt_history.yview)
        self.txt_history.configure(yscrollcommand=sb.set)
        self.txt_history.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

    def _build_network_tab(self, parent):
        """Displays table of all 23 map edges with step costs and straight line distances."""
        frame = ttk.Frame(parent, padding=4)
        frame.pack(fill=tk.BOTH, expand=True)

        cols = ('edge', 'step_cost', 'sld', 'ratio')
        tree = ttk.Treeview(frame, columns=cols, show='headings', selectmode='browse')
        tree.heading('edge', text='Connection')
        tree.heading('step_cost', text='Step Cost (km)')
        tree.heading('sld', text='Straight Line (km)')
        tree.heading('ratio', text='Road / SLD Ratio')

        tree.column('edge', width=190, anchor='w')
        tree.column('step_cost', width=95, anchor='e')
        tree.column('sld', width=105, anchor='e')
        tree.column('ratio', width=95, anchor='e')

        for u, v, sld, cost in EDGES_RAW:
            ratio = cost / sld if sld > 0 else 1.0
            tree.insert('', tk.END, values=(f"{u} ↔ {v}", f"{cost:.1f} km", f"{sld:.2f} km", f"{ratio:.2f}x"))

        sb = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=tree.yview)
        tree.configure(yscrollcommand=sb.set)
        tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

    # ---------------------------------------------------------
    # SEARCH SIMULATION CONTROLS
    # ---------------------------------------------------------
    def _on_preset_selected(self, event=None):
        choice = self.preset_var.get()
        if "Malimono -> Tagana-an" in choice:
            self.start_city.set("Malimono")
            self.goal_city.set("Tagana-an")
        elif "General Luna" in choice:
            self.start_city.set("Surigao City")
            self.goal_city.set("General Luna")
        elif "Santa Monica -> Alegria" in choice:
            self.start_city.set("Santa Monica")
            self.goal_city.set("Alegria")
        elif "Del Carmen -> Burgos" in choice:
            self.start_city.set("Del Carmen")
            self.goal_city.set("Burgos")
        else:
            self.start_city.set("Surigao City")
            self.goal_city.set("Dapa")
        self._recompute_and_reset()

    def _swap_start_goal(self):
        s, g = self.start_city.get(), self.goal_city.get()
        self.start_city.set(g)
        self.goal_city.set(s)
        self._recompute_and_reset()

    def _recompute_and_reset(self):
        """Precomputes the entire search trace for the active Start, Goal, and Algorithm."""
        self.pause()
        start = self.start_city.get()
        goal = self.goal_city.get()
        algo = self.algorithm.get()

        if start == goal:
            messagebox.showwarning("Same Node", "Start Node and Goal Node cannot be the same.")
            return

        self.snapshots = generate_search_trace(start, goal, algo)
        self.current_step_idx = 0
        self._refresh_heuristic_table()
        self._populate_history_log()
        self._render_current_step()

    def _refresh_heuristic_table(self):
        """Populates the Heuristic Table for all 21 towns relative to the current Goal."""
        self.tree_heur.delete(*self.tree_heur.get_children())
        goal = self.goal_city.get()
        start = self.start_city.get()

        sorted_cities = sorted(COORDINATES_KM.keys(), key=lambda c: get_heuristic(c, goal))
        for city in sorted_cities:
            h = get_heuristic(city, goal)
            status = ""
            if city == goal:
                status = "🎯 GOAL (Zero Heuristic)"
            elif city == start:
                status = "🏁 START"
            item_id = self.tree_heur.insert('', tk.END, values=(city, f"{h:.2f} km", status))
            if city == goal:
                self.tree_heur.item(item_id, tags=('goal_tag',))

        self.tree_heur.tag_configure('goal_tag', background='#363a4f', foreground='#ed8796')

    def _populate_history_log(self):
        """Writes the initial summary and roadmap in the Search Trace log."""
        self.txt_history.delete('1.0', tk.END)
        algo = self.algorithm.get()
        start = self.start_city.get()
        goal = self.goal_city.get()

        header = (
            f"=== {algo} Search Trace ===\n"
            f"Start: {start} | Goal: {goal} | h(Goal) = {get_heuristic(goal, goal):.2f} km\n"
            f"Formula: {'f(n) = g(n) + h(n)' if algo == 'A*' else 'f(n) = h(n)'}\n"
            f"Total Steps Precomputed: {len(self.snapshots) - 1}\n"
            + "-" * 50 + "\n"
        )
        self.txt_history.insert(tk.END, header)

    def toggle_play(self):
        if self.is_playing:
            self.pause()
        else:
            self.play()

    def play(self):
        if self.current_step_idx >= len(self.snapshots) - 1:
            self.current_step_idx = 0
        self.is_playing = True
        self.btn_play.configure(text="⏸ Pause", bg='#eed49f')
        self._schedule_next_step()

    def pause(self):
        self.is_playing = False
        if self.timer_id:
            self.after_cancel(self.timer_id)
            self.timer_id = None
        self.btn_play.configure(text="▶ Play", bg='#8aadf4')

    def _schedule_next_step(self):
        if not self.is_playing:
            return
        if self.current_step_idx < len(self.snapshots) - 1:
            self.current_step_idx += 1
            self._render_current_step()
            self.timer_id = self.after(self.anim_speed_ms.get(), self._schedule_next_step)
        else:
            self.pause()

    def step_forward(self):
        self.pause()
        if self.current_step_idx < len(self.snapshots) - 1:
            self.current_step_idx += 1
            self._render_current_step()

    def step_back(self):
        self.pause()
        if self.current_step_idx > 0:
            self.current_step_idx -= 1
            self._render_current_step()

    def step_first(self):
        self.pause()
        self.current_step_idx = 0
        self._render_current_step()

    def step_last(self):
        self.pause()
        self.current_step_idx = len(self.snapshots) - 1
        self._render_current_step()

    def reset_sim(self):
        self.pause()
        self.current_step_idx = 0
        self._render_current_step()

    # ---------------------------------------------------------
    # CANVAS RENDERING
    # ---------------------------------------------------------
    def _render_current_step(self):
        """Draws map background, edges, node statuses, path, and updates dashboard metrics."""
        if not self.snapshots:
            return

        snap = self.snapshots[self.current_step_idx]
        self.canvas.delete('all')

        # 1. Background satellite map image
        cw = self.map_photo.width() if self.map_photo else 1024
        ch = self.map_photo.height() if self.map_photo else 509

        if self.map_photo:
            self.canvas.create_image(0, 0, image=self.map_photo, anchor='nw')
        else:
            # Fallback high-tech cartographic grid
            self.canvas.create_rectangle(0, 0, cw, ch, fill='#11111b', outline='')
            for x in range(0, cw, 60):
                self.canvas.create_line(x, 0, x, ch, fill='#181926')
            for y in range(0, ch, 60):
                self.canvas.create_line(0, y, cw, y, fill='#181926')

        start = self.start_city.get()
        goal = self.goal_city.get()
        curr_node = snap['current_node']
        frontier_dict = {item[0]: item for item in snap['frontier']}
        explored_dict = snap['explored']
        active_path = snap['final_path'] if snap['final_path'] else snap['current_path']

        # 2. Draw Road Network (Edges)
        drawn_edges = set()
        for u, v, sld, cost in EDGES_RAW:
            edge_key = tuple(sorted([u, v]))
            if edge_key in drawn_edges:
                continue
            drawn_edges.add(edge_key)

            x1, y1 = PIN_PIXELS[u]
            x2, y2 = PIN_PIXELS[v]

            # Road line shadow & main stroke
            self.canvas.create_line(x1, y1, x2, y2, fill='#11111b', width=4)
            self.canvas.create_line(x1, y1, x2, y2, fill=self.colors['road_line'], width=2)

            # Edge cost labels
            if self.show_edge_costs.get():
                mx = (x1 + x2) / 2
                my = (y1 + y2) / 2
                lbl_text = f"{cost:.0f}km"
                self.canvas.create_rectangle(mx - 15, my - 8, mx + 15, my + 8, fill='#181926', outline='#363a4f', width=1)
                self.canvas.create_text(mx, my, text=lbl_text, fill='#eed49f', font=('Segoe UI', 7, 'bold'))

        # 3. Draw Dashed Lines to Every Node Visited (These Stay There!)
        if self.show_visited_dashes.get():
            visited_edges = snap.get('visited_edges', [])
            for u, v in visited_edges:
                p1 = PIN_PIXELS[u]
                p2 = PIN_PIXELS[v]
                self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill='#11111b', width=6, dash=(8, 4), capstyle=tk.ROUND)
                self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill='#eed49f', width=3.5, dash=(8, 4), capstyle=tk.ROUND)

        # 4. Draw Final Optimal Path (Only when Goal is reached!)
        if snap.get('final_path') and len(snap['final_path']) > 1:
            for i in range(len(snap['final_path']) - 1):
                p1 = PIN_PIXELS[snap['final_path'][i]]
                p2 = PIN_PIXELS[snap['final_path'][i + 1]]
                self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill='#a6da95', width=7, capstyle=tk.ROUND)
                self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill='#8bd5ca', width=3, capstyle=tk.ROUND)

        # 5. Draw Nodes (Pins & Badges)
        for city, (px, py) in PIN_PIXELS.items():
            r = 7  # radius
            is_start = (city == start)
            is_goal = (city == goal)
            is_curr = (city == curr_node)
            is_in_frontier = (city in frontier_dict)
            is_explored = (city in explored_dict)
            is_on_path = (city in snap['final_path']) if snap.get('final_path') else False

            # Determine Node Color & Style
            if is_curr:
                node_fill = self.colors['current']
                node_outline = '#ffffff'
                halo_color = self.colors['current']
                r = 10
            elif is_goal:
                node_fill = self.colors['goal']
                node_outline = '#ffffff'
                halo_color = self.colors['goal']
                r = 9
            elif is_start:
                node_fill = self.colors['start']
                node_outline = '#ffffff'
                halo_color = self.colors['start']
                r = 9
            elif is_on_path:
                node_fill = self.colors['path_line']
                node_outline = '#ffffff'
                halo_color = self.colors['path_line']
                r = 8
            elif is_in_frontier:
                node_fill = self.colors['frontier']
                node_outline = '#181926'
                halo_color = self.colors['frontier']
                r = 8
            elif is_explored:
                node_fill = self.colors['explored']
                node_outline = '#181926'
                halo_color = None
                r = 7
            else:
                node_fill = '#6e738d'
                node_outline = '#181926'
                halo_color = None
                r = 6

            # Halo ring for active/key nodes
            if halo_color:
                self.canvas.create_oval(px - r - 4, py - r - 4, px + r + 4, py + r + 4,
                                        outline=halo_color, width=2)
            elif is_explored:
                # Dashed halo ring around visited node
                self.canvas.create_oval(px - r - 4, py - r - 4, px + r + 4, py + r + 4,
                                        outline=self.colors['explored'], width=1.5, dash=(3, 3))

            # Node main circle
            self.canvas.create_oval(px - r, py - r, px + r, py + r, fill=node_fill, outline=node_outline, width=2)

            # Labels and Badges
            h_val = get_heuristic(city, goal)
            if self.show_heuristics.get() or is_goal or is_start or is_curr:
                ty = py - 16 if py > 60 else py + 18
                if is_goal:
                    label_txt = f"{city} [GOAL | h=0]"
                elif is_start:
                    label_txt = f"{city} [START]"
                elif self.show_heuristics.get():
                    label_txt = f"{city} (h={h_val:.1f})"
                else:
                    label_txt = city

                # Text shadow
                self.canvas.create_text(px + 1, ty + 1, text=label_txt, fill='#11111b', font=('Segoe UI', 8, 'bold'))
                self.canvas.create_text(px, ty, text=label_txt, fill=node_fill if (is_start or is_goal or is_curr) else '#cad3f5', font=('Segoe UI', 8, 'bold'))

        # 5. Draw Tooltip if hovering over a node
        if self.hovered_city:
            self._draw_node_tooltip(self.hovered_city, goal, frontier_dict, explored_dict)

        # 6. Update Status Card & Metrics
        total_steps = len(self.snapshots) - 1
        self.lbl_step_metric.configure(text=f"{self.current_step_idx} / {total_steps}")
        self.lbl_curr_node.configure(text=str(curr_node) if curr_node else "Initial State")

        algo = self.algorithm.get()
        if snap['f_score'] is not None:
            if algo == 'A*':
                self.lbl_eval_title.configure(text="EVALUATION f(n) = g + h")
                self.lbl_eval_val.configure(
                    text=f"{snap['f_score']:.2f} (g={snap['g_score']:.1f}, h={snap['h_score']:.2f})"
                )
            else:
                self.lbl_eval_title.configure(text="EVALUATION f(n) = h(n)")
                self.lbl_eval_val.configure(
                    text=f"{snap['h_score']:.2f} km (g so far={snap['g_score']:.1f})"
                )
        else:
            self.lbl_eval_val.configure(text="--")

        if snap['g_score'] is not None:
            self.lbl_cost_val.configure(text=f"{snap['g_score']:.1f} km")
        else:
            self.lbl_cost_val.configure(text="0.0 km")

        self.lbl_step_desc.configure(text=snap['description'])

        # 7. Update Frontier Treeview
        self.tree_frontier.delete(*self.tree_frontier.get_children())
        for idx, item in enumerate(snap['frontier'], 1):
            n_name, f_val, g_val, h_val, p_name, _ = item
            self.tree_frontier.insert(
                '', tk.END,
                values=(idx, n_name, f"{f_val:.2f}", f"{g_val:.1f}", f"{h_val:.2f}", p_name or "None")
            )

        # 8. Append to History Log
        step_log_entry = f"Step {self.current_step_idx}: {snap['description']}\n"
        self.txt_history.insert(tk.END, step_log_entry)
        self.txt_history.see(tk.END)

    def _draw_node_tooltip(self, city, goal, frontier_dict, explored_dict):
        """Renders an informative hovering tooltip box on the canvas."""
        px, py = PIN_PIXELS[city]
        h_val = get_heuristic(city, goal)
        nbrs = [f"{nbr} ({cost:.0f}km)" for nbr, cost, _ in ADJACENCY[city]]
        nbr_str = ", ".join(nbrs[:3])
        if len(nbrs) > 3:
            nbr_str += "..."

        status_str = "Unvisited"
        if city == self.start_city.get():
            status_str = "START Node"
        elif city == self.goal_city.get():
            status_str = "GOAL Node (h=0.0)"
        elif city in frontier_dict:
            item = frontier_dict[city]
            status_str = f"In Frontier: f={item[1]:.1f}, g={item[2]:.1f}, h={item[3]:.1f}"
        elif city in explored_dict:
            status_str = f"Explored (Cost g = {explored_dict[city]:.1f} km)"

        lines = [
            f"📍 {city}",
            f"Heuristic to Goal: {h_val:.2f} km",
            f"State: {status_str}",
            f"Roads to: {nbr_str}",
        ]

        tw, th = 220, 72
        tx = px + 15 if px + 15 + tw < 1020 else px - tw - 15
        ty = py - th // 2 if py - th // 2 > 10 else 10
        ch = self.map_photo.height() if self.map_photo else 509
        if ty + th > ch - 10:
            ty = ch - 10 - th

        self.canvas.create_rectangle(tx, ty, tx + tw, ty + th, fill='#1e2030', outline='#8aadf4', width=1)
        for i, line in enumerate(lines):
            col = '#eed49f' if i == 0 else ('#a6da95' if i == 1 else '#cad3f5')
            fnt = ('Segoe UI', 8, 'bold') if i < 2 else ('Segoe UI', 8)
            self.canvas.create_text(tx + 8, ty + 10 + i * 14, text=line, fill=col, anchor='w', font=fnt)

    # ---------------------------------------------------------
    # CANVAS MOUSE INTERACTIONS
    # ---------------------------------------------------------
    def _find_nearest_city(self, x, y, max_dist=22):
        for city, (px, py) in PIN_PIXELS.items():
            if math.hypot(x - px, y - py) <= max_dist:
                return city
        return None

    def _on_canvas_motion(self, event):
        city = self._find_nearest_city(event.x, event.y)
        if city != self.hovered_city:
            self.hovered_city = city
            if city:
                self.canvas.config(cursor='hand2')
            else:
                self.canvas.config(cursor='crosshair')
            self._render_current_step()

    def _on_canvas_click_start(self, event):
        city = self._find_nearest_city(event.x, event.y)
        if city:
            if city == self.goal_city.get():
                messagebox.showinfo("Start Selection", f"'{city}' is currently the Goal node. Select a different node or swap.")
                return
            self.start_city.set(city)
            self.preset_var.set("Custom Route")
            self._recompute_and_reset()

    def _on_canvas_click_goal(self, event):
        city = self._find_nearest_city(event.x, event.y)
        if city:
            if city == self.start_city.get():
                messagebox.showinfo("Goal Selection", f"'{city}' is currently the Start node. Select a different node or swap.")
                return
            self.goal_city.set(city)
            self.preset_var.set("Custom Route")
            self._recompute_and_reset()

    # ---------------------------------------------------------
    # COMPARISON MODAL (A* vs GREEDY BEST-FIRST)
    # ---------------------------------------------------------
    def open_comparison_view(self):
        """Displays side-by-side performance comparison between A* and Greedy Best-First Search."""
        start = self.start_city.get()
        goal = self.goal_city.get()

        snaps_astar = generate_search_trace(start, goal, 'A*')
        snaps_greedy = generate_search_trace(start, goal, 'Greedy')

        cost_astar = snaps_astar[-1]['total_cost']
        path_astar = snaps_astar[-1]['final_path']
        exp_astar = len(snaps_astar[-1]['explored'])

        cost_greedy = snaps_greedy[-1]['total_cost']
        path_greedy = snaps_greedy[-1]['final_path']
        exp_greedy = len(snaps_greedy[-1]['explored'])

        win = tk.Toplevel(self)
        win.title(f"A* Search vs Greedy Best-First Search Comparison ({start} → {goal})")
        win.geometry("820x520")
        win.minsize(700, 450)
        win.configure(bg=self.colors['panel_bg'])
        win.transient(self)

        ttk.Label(win, text=f"Search Algorithm Comparison: '{start}' to '{goal}'", style='Header.TLabel').pack(pady=(12, 4))
        ttk.Label(win, text="Comparing Optimality, Step Cost, Path Choice, and Node Expansions", style='SubHeader.TLabel').pack(pady=(0, 12))

        cards_frame = tk.Frame(win, bg=self.colors['panel_bg'])
        cards_frame.pack(fill=tk.BOTH, expand=True, padx=16, pady=4)

        # Card A: A* Search
        card_a = tk.Frame(cards_frame, bg=self.colors['card_bg'], bd=2, relief='solid', padx=12, pady=12)
        card_a.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=6)

        tk.Label(card_a, text="⭐ A* Search", font=('Segoe UI', 13, 'bold'), fg='#8aadf4', bg=self.colors['card_bg']).pack(anchor='w')
        tk.Label(card_a, text="Evaluation: f(n) = g(n) + h(n)", font=('Segoe UI', 9, 'italic'), fg=self.colors['text_dim'], bg=self.colors['card_bg']).pack(anchor='w', pady=(0, 8))

        tk.Label(card_a, text=f"Total Path Cost: {cost_astar:.1f} km", font=('Segoe UI', 11, 'bold'), fg='#a6da95', bg=self.colors['card_bg']).pack(anchor='w')
        tk.Label(card_a, text=f"Nodes Explored: {exp_astar}", font=('Segoe UI', 10), fg='#cad3f5', bg=self.colors['card_bg']).pack(anchor='w')
        tk.Label(card_a, text=f"Total Simulation Steps: {len(snaps_astar) - 1}", font=('Segoe UI', 10), fg='#cad3f5', bg=self.colors['card_bg']).pack(anchor='w')
        tk.Label(card_a, text="Optimality: Guaranteed Optimal (Shortest Road Cost)", font=('Segoe UI', 9, 'bold'), fg='#a6da95', bg=self.colors['card_bg']).pack(anchor='w', pady=(4, 8))

        tk.Label(card_a, text="Path Taken:", font=('Segoe UI', 9, 'bold'), fg=self.colors['accent'], bg=self.colors['card_bg']).pack(anchor='w')
        path_txt_a = tk.Text(card_a, height=6, bg='#181926', fg='#cad3f5', font=('Consolas', 9), bd=0, wrap=tk.WORD)
        path_txt_a.pack(fill=tk.BOTH, expand=True, pady=4)
        path_txt_a.insert(tk.END, " -> \n".join(path_astar) if path_astar else "No path found")
        path_txt_a.config(state='disabled')

        # Card B: Greedy Best-First Search
        card_b = tk.Frame(cards_frame, bg=self.colors['card_bg'], bd=2, relief='solid', padx=12, pady=12)
        card_b.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=6)

        tk.Label(card_b, text="⚡ Greedy Best-First Search", font=('Segoe UI', 13, 'bold'), fg='#f5a97f', bg=self.colors['card_bg']).pack(anchor='w')
        tk.Label(card_b, text="Evaluation: f(n) = h(n)", font=('Segoe UI', 9, 'italic'), fg=self.colors['text_dim'], bg=self.colors['card_bg']).pack(anchor='w', pady=(0, 8))

        cost_col = '#eed49f' if cost_greedy == cost_astar else '#ed8796'
        tk.Label(card_b, text=f"Total Path Cost: {cost_greedy:.1f} km", font=('Segoe UI', 11, 'bold'), fg=cost_col, bg=self.colors['card_bg']).pack(anchor='w')
        tk.Label(card_b, text=f"Nodes Explored: {exp_greedy}", font=('Segoe UI', 10), fg='#cad3f5', bg=self.colors['card_bg']).pack(anchor='w')
        tk.Label(card_b, text=f"Total Simulation Steps: {len(snaps_greedy) - 1}", font=('Segoe UI', 10), fg='#cad3f5', bg=self.colors['card_bg']).pack(anchor='w')

        opt_text = "Same as A* on this route" if cost_greedy == cost_astar else f"Suboptimal (+{cost_greedy - cost_astar:.1f} km longer!)"
        tk.Label(card_b, text=f"Verdict: {opt_text}", font=('Segoe UI', 9, 'bold'), fg=cost_col, bg=self.colors['card_bg']).pack(anchor='w', pady=(4, 8))

        tk.Label(card_b, text="Path Taken:", font=('Segoe UI', 9, 'bold'), fg='#f5a97f', bg=self.colors['card_bg']).pack(anchor='w')
        path_txt_b = tk.Text(card_b, height=6, bg='#181926', fg='#cad3f5', font=('Consolas', 9), bd=0, wrap=tk.WORD)
        path_txt_b.pack(fill=tk.BOTH, expand=True, pady=4)
        path_txt_b.insert(tk.END, " -> \n".join(path_greedy) if path_greedy else "No path found")
        path_txt_b.config(state='disabled')

        # Bottom Conclusion Banner
        concl_frame = tk.Frame(win, bg='#1e2030', padx=12, pady=8)
        concl_frame.pack(fill=tk.X, padx=16, pady=8)

        if cost_astar < cost_greedy:
            concl_msg = (
                f"Insight: Greedy Best-First Search chose a route {cost_greedy - cost_astar:.1f} km longer "
                f"because it greedily pursued nodes with smaller straight-line distance, ignoring accumulated road step costs. "
                f"A* balanced g(n) + h(n) to guarantee the truly optimal path."
            )
        else:
            concl_msg = (
                f"Insight: For this pair, Greedy Best-First Search managed to reach the goal with the same optimal road cost "
                f"({cost_astar:.1f} km), expanding {exp_greedy} nodes compared to A*'s {exp_astar} nodes."
            )
        tk.Label(concl_frame, text=concl_msg, font=('Segoe UI', 9), fg='#eed49f', bg='#1e2030', wraplength=760, justify=tk.LEFT).pack()


if __name__ == '__main__':
    app = InformedSearchApp()
    app.mainloop()
