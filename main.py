import pygame
import heapq
import sys

# ============================================================
# WAREHOUSE AGV FLEET SIMULATOR - VERSION 4
# Modern industrial dashboard | Pygame | A* + Time-based A*
# ============================================================

pygame.init()
pygame.display.set_caption("WAREHOUSE AGV FLEET | CONTROL CENTER")

WIDTH, HEIGHT = 1440, 900
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# --------------------------- THEME ---------------------------

BG = (13, 19, 29)
HEADER = (19, 28, 41)
PANEL = (24, 35, 50)
PANEL_ALT = (29, 43, 60)
BORDER = (48, 65, 84)
GRID_BG = (18, 28, 41)
GRID_LINE = (47, 63, 80)
TEXT = (231, 239, 247)
MUTED = (143, 160, 178)

BLUE = (64, 151, 255)
GREEN = (47, 207, 139)
YELLOW = (255, 190, 75)
RED = (245, 92, 92)
ORANGE = (255, 145, 77)
OBSTACLE = (112, 83, 63)
WHITE = (255, 255, 255)

FONT = pygame.font.SysFont("segoeui", 17)
SMALL = pygame.font.SysFont("segoeui", 14)
TINY = pygame.font.SysFont("segoeui", 12)
MEDIUM = pygame.font.SysFont("segoeui", 21, bold=True)
HEADING = pygame.font.SysFont("segoeui", 27, bold=True)
BIG = pygame.font.SysFont("segoeui", 30, bold=True)

# ---------------------- WAREHOUSE GRID -----------------------

ROWS, COLS, CELL = 15, 20, 35
GRID_X, GRID_Y = 55, 235
GRID_W, GRID_H = COLS * CELL, ROWS * CELL

obstacles = set()

robots = [
    {
        "name": "AGV 1",
        "start": (1, 1),
        "goal": (13, 18),
        "pos": (1, 1),
        "color": BLUE,
        "path": [],
        "status": "Ready",
        "completed": False,
        "distance": 0
    },
    {
        "name": "AGV 2",
        "start": (13, 1),
        "goal": (1, 13),
        "pos": (13, 1),
        "color": GREEN,
        "path": [],
        "status": "Ready",
        "completed": False,
        "distance": 0
    }
]

mode = "EDIT"
active_goal_robot = None

simulation_running = False
simulation_tick = 0
last_move_time = 0
MOVE_DELAY = 180

simulation_message = "Add obstacles, set destinations, then start the fleet."

# --------------------------- LAYOUT --------------------------

map_panel = pygame.Rect(30, 165, 770, 650)
sidebar = pygame.Rect(825, 165, 585, 680)

buttons = {
    "edit": pygame.Rect(850, 250, 250, 44),
    "simulate": pygame.Rect(1120, 250, 250, 44),
    "start": pygame.Rect(850, 315, 520, 48),
    "reset": pygame.Rect(850, 378, 250, 42),
    "clear": pygame.Rect(1120, 378, 250, 42),
    "goal1": pygame.Rect(850, 475, 250, 42),
    "goal2": pygame.Rect(1120, 475, 250, 42),
}

# -------------------------- HELPERS --------------------------

def draw_text(text, x, y, color=TEXT, font=FONT):
    screen.blit(font.render(str(text), True, color), (x, y))


def draw_centered(text, rect, color=TEXT, font=FONT):
    surf = font.render(str(text), True, color)

    screen.blit(
        surf,
        (
            rect.centerx - surf.get_width() // 2,
            rect.centery - surf.get_height() // 2
        )
    )


def rounded(rect, color, radius=10, border=None, border_width=1):
    pygame.draw.rect(
        screen,
        color,
        rect,
        border_radius=radius
    )

    if border:
        pygame.draw.rect(
            screen,
            border,
            rect,
            border_width,
            border_radius=radius
        )


def draw_button(key, label, color, enabled=True):
    rect = buttons[key]

    fill = color if enabled else PANEL_ALT

    rounded(
        rect,
        fill,
        9,
        BORDER if not enabled else None
    )

    draw_centered(
        label,
        rect,
        WHITE if enabled else MUTED,
        FONT
    )


def cell_rect(position):
    row, col = position

    return pygame.Rect(
        GRID_X + col * CELL,
        GRID_Y + row * CELL,
        CELL,
        CELL
    )


# ------------------------ PATHFINDING ------------------------

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(position, blocked):

    row, col = position

    result = []

    for dr, dc in [
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1)
    ]:

        nr = row + dr
        nc = col + dc

        if (
            0 <= nr < ROWS
            and 0 <= nc < COLS
            and (nr, nc) not in blocked
        ):
            result.append((nr, nc))

    return result


def astar(start, goal, extra_blocked=None):

    blocked = set(obstacles)

    if extra_blocked:
        blocked.update(extra_blocked)

    if start in blocked or goal in blocked:
        return []

    open_list = [
        (heuristic(start, goal), 0, start)
    ]

    came_from = {}
    g_score = {start: 0}
    closed = set()

    while open_list:

        _, cost, current = heapq.heappop(open_list)

        if current in closed:
            continue

        if current == goal:

            path = [current]

            while current in came_from:
                current = came_from[current]
                path.append(current)

            return list(reversed(path))

        closed.add(current)

        for neighbor in get_neighbors(current, blocked):

            if neighbor in closed:
                continue

            new_cost = cost + 1

            if new_cost < g_score.get(
                neighbor,
                float("inf")
            ):

                g_score[neighbor] = new_cost
                came_from[neighbor] = current

                priority = (
                    new_cost + heuristic(neighbor, goal)
                )

                heapq.heappush(
                    open_list,
                    (priority, new_cost, neighbor)
                )

    return []


# ------------------ TIME-BASED COLLISION A* ------------------

def time_astar(start, goal, reserved_path):

    if start in obstacles or goal in obstacles:
        return []

    if not reserved_path:
        return astar(start, goal)

    if start == reserved_path[0]:
        return []

    start_state = (start[0], start[1], 0)

    open_list = [
        (heuristic(start, goal), 0, start_state)
    ]

    came_from = {}
    visited = set()

    max_time = 600

    while open_list:

        _, _, state = heapq.heappop(open_list)

        if state in visited:
            continue

        visited.add(state)

        row, col, time_step = state

        current = (row, col)

        if current == goal:

            path = [current]

            while state in came_from:

                state = came_from[state]

                path.append(
                    (state[0], state[1])
                )

            return list(reversed(path))

        if time_step >= max_time:
            continue

        next_time = time_step + 1

        reserved_current = reserved_path[
            min(time_step, len(reserved_path) - 1)
        ]

        reserved_next = reserved_path[
            min(next_time, len(reserved_path) - 1)
        ]

        # Movement or waiting

        actions = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (0, 0)
        ]

        for dr, dc in actions:

            nr = row + dr
            nc = col + dc

            candidate = (nr, nc)

            if not (
                0 <= nr < ROWS
                and 0 <= nc < COLS
            ):
                continue

            if candidate in obstacles:
                continue

            # Same-cell collision prevention

            if candidate == reserved_next:
                continue

            # Head-on collision prevention

            if (
                candidate == reserved_current
                and current == reserved_next
            ):
                continue

            next_state = (nr, nc, next_time)

            if next_state in visited:
                continue

            if next_state in came_from:
                continue

            came_from[next_state] = state

            priority = (
                next_time + heuristic(candidate, goal)
            )

            heapq.heappush(
                open_list,
                (
                    priority,
                    next_time,
                    next_state
                )
            )

    return []


# ---------------------- SIMULATION LOGIC --------------------

def reset_simulation():

    global simulation_running
    global simulation_tick
    global simulation_message

    simulation_running = False
    simulation_tick = 0

    for robot in robots:

        robot["pos"] = robot["start"]
        robot["path"] = []
        robot["status"] = "Ready"
        robot["completed"] = False
        robot["distance"] = 0

    simulation_message = "Simulation reset. Fleet is ready."


def start_simulation():

    global simulation_running
    global simulation_message
    global simulation_tick
    global last_move_time

    reset_simulation()

    agv1, agv2 = robots

    endpoints = []

    for robot in robots:

        endpoints.extend([
            robot["start"],
            robot["goal"]
        ])

        if (
            robot["start"] in obstacles
            or robot["goal"] in obstacles
        ):

            simulation_message = (
                f"{robot['name']}: start or destination is blocked."
            )

            return

    # Check endpoint conflicts

    if len(set(endpoints)) != len(endpoints):

        simulation_message = (
            "AGV endpoint conflict: check start and destination cells."
        )

        return

    # AGV 1 shortest path

    protected = {
        agv2["start"],
        agv2["goal"]
    }

    path1 = astar(
        agv1["start"],
        agv1["goal"],
        protected
    )

    if not path1:

        simulation_message = "AGV 1: no valid route found."
        return

    # AGV 2 collision-free path

    path2 = time_astar(
        agv2["start"],
        agv2["goal"],
        path1
    )

    if not path2:

        simulation_message = (
            "AGV 2: no collision-free route found."
        )

        return

    agv1["path"] = path1
    agv2["path"] = path2

    for robot in robots:
        robot["status"] = "Moving"

    simulation_tick = 0
    simulation_running = True

    last_move_time = pygame.time.get_ticks()

    simulation_message = (
        "Fleet dispatched. Collision-aware routes are active."
    )


def update_simulation():

    global simulation_running
    global simulation_tick
    global simulation_message

    if not simulation_running:
        return

    simulation_tick += 1

    for robot in robots:

        path = robot["path"]

        if simulation_tick < len(path):

            next_position = path[simulation_tick]

            if next_position != robot["pos"]:
                robot["distance"] += 1

            robot["pos"] = next_position

            if (
                simulation_tick < len(path) - 1
                and path[simulation_tick] == path[simulation_tick - 1]
            ):

                robot["status"] = "Waiting"

            else:

                robot["status"] = "Moving"

        else:

            robot["pos"] = robot["goal"]
            robot["completed"] = True
            robot["status"] = "Delivered"

    if all(robot["completed"] for robot in robots):

        simulation_running = False

        simulation_message = (
            "All AGVs completed their deliveries successfully."
        )


# ------------------------- DRAWING --------------------------

def draw_header():

    pygame.draw.rect(
        screen,
        HEADER,
        (0, 0, WIDTH, 125)
    )

    pygame.draw.line(
        screen,
        BORDER,
        (0, 124),
        (WIDTH, 124),
        1
    )

    # Brand mark

    rounded(
        pygame.Rect(30, 27, 48, 48),
        BLUE,
        12
    )

    draw_centered(
        "AGV",
        pygame.Rect(30, 27, 48, 48),
        WHITE,
        SMALL
    )

    draw_text(
        "WAREHOUSE FLEET CONTROL",
        95,
        25,
        TEXT,
        HEADING
    )

    draw_text(
        "AUTONOMOUS MOBILE ROBOTICS  /  PATH PLANNING & DISPATCH",
        96,
        62,
        MUTED,
        SMALL
    )

    status_color = (
        GREEN if simulation_running
        else YELLOW if any(r["completed"] for r in robots)
        else BLUE
    )

    rounded(
        pygame.Rect(WIDTH - 245, 35, 205, 38),
        PANEL_ALT,
        18,
        BORDER
    )

    pygame.draw.circle(
        screen,
        status_color,
        (WIDTH - 224, 54),
        6
    )

    draw_text(
        "FLEET ACTIVE" if simulation_running else "SYSTEM STANDBY",
        WIDTH - 208,
        43,
        TEXT,
        SMALL
    )


def draw_metric_card(rect, label, value, accent):

    rounded(rect, PANEL, 10, BORDER)

    pygame.draw.rect(
        screen,
        accent,
        (rect.x, rect.y + 9, 4, rect.height - 18),
        border_radius=2
    )

    draw_text(
        label.upper(),
        rect.x + 18,
        rect.y + 12,
        MUTED,
        TINY
    )

    draw_text(
        value,
        rect.x + 18,
        rect.y + 32,
        TEXT,
        BIG
    )


def draw_map():

    rounded(map_panel, PANEL, 14, BORDER)

    draw_text(
        "WAREHOUSE FLOOR MAP",
        map_panel.x + 22,
        map_panel.y + 18,
        TEXT,
        MEDIUM
    )

    draw_text(
        "20 columns  x  15 rows",
        map_panel.x + 22,
        map_panel.y + 47,
        MUTED,
        SMALL
    )

    # Map legend

    key_y = map_panel.y + 76

    legend = [
        (BLUE, "AGV 1"),
        (GREEN, "AGV 2"),
        (YELLOW, "Destination"),
        (OBSTACLE, "Obstacle")
    ]

    lx = map_panel.x + 22

    for color, label in legend:

        pygame.draw.circle(
            screen,
            color,
            (lx + 6, key_y + 7),
            5
        )

        draw_text(
            label,
            lx + 17,
            key_y,
            MUTED,
            TINY
        )

        lx += 120 if label != "Destination" else 145

    # Grid background

    grid_rect = pygame.Rect(
        GRID_X - 2,
        GRID_Y - 2,
        GRID_W + 4,
        GRID_H + 4
    )

    rounded(
        grid_rect,
        GRID_BG,
        6,
        BORDER
    )

    # Draw cells

    for row in range(ROWS):

        for col in range(COLS):

            pos = (row, col)

            rect = cell_rect(pos)

            if pos in obstacles:

                pygame.draw.rect(
                    screen,
                    OBSTACLE,
                    rect.inflate(-2, -2),
                    border_radius=3
                )

                # Obstacle cross pattern

                pygame.draw.line(
                    screen,
                    (151, 117, 91),
                    (rect.x + 9, rect.y + 9),
                    (rect.right - 9, rect.bottom - 9),
                    1
                )

                pygame.draw.line(
                    screen,
                    (151, 117, 91),
                    (rect.right - 9, rect.y + 9),
                    (rect.x + 9, rect.bottom - 9),
                    1
                )

            pygame.draw.rect(
                screen,
                GRID_LINE,
                rect,
                1
            )

    # Draw robot routes

    for robot in robots:

        path = robot["path"]

        if len(path) > 1:

            points = [
                cell_rect(p).center
                for p in path
            ]

            pygame.draw.lines(
                screen,
                robot["color"],
                False,
                points,
                4
            )

            for point in points[1:-1:3]:

                pygame.draw.circle(
                    screen,
                    robot["color"],
                    point,
                    3
                )

    # Start and destination markers

    for robot in robots:

        start_rect = cell_rect(robot["start"])
        goal_rect = cell_rect(robot["goal"])

        pygame.draw.rect(
            screen,
            robot["color"],
            start_rect.inflate(-8, -8),
            2,
            border_radius=5
        )

        pygame.draw.rect(
            screen,
            YELLOW,
            goal_rect.inflate(-9, -9),
            border_radius=5
        )

        draw_centered(
            "S",
            start_rect,
            WHITE,
            TINY
        )

        draw_centered(
            "G",
            goal_rect,
            (31, 37, 45),
            TINY
        )

    # Draw AGVs

    for robot in robots:

        rect = cell_rect(robot["pos"])

        pygame.draw.circle(
            screen,
            robot["color"],
            rect.center,
            12
        )

        pygame.draw.circle(
            screen,
            WHITE,
            rect.center,
            12,
            2
        )

        pygame.draw.circle(
            screen,
            WHITE,
            (rect.centerx - 4, rect.centery - 3),
            2
        )

        pygame.draw.circle(
            screen,
            WHITE,
            (rect.centerx + 4, rect.centery - 3),
            2
        )

    # Destination selection instruction

    if active_goal_robot is not None:

        hint = (
            f"SELECT DESTINATION FOR "
            f"{robots[active_goal_robot]['name']}  -  click a free cell"
        )

        rounded(
            pygame.Rect(
                map_panel.x + 20,
                map_panel.bottom - 43,
                map_panel.width - 40,
                28
            ),
            (65, 49, 31),
            7
        )

        draw_centered(
            hint,
            pygame.Rect(
                map_panel.x + 20,
                map_panel.bottom - 43,
                map_panel.width - 40,
                28
            ),
            YELLOW,
            SMALL
        )

    elif mode == "EDIT":

        draw_text(
            "EDIT: left-click to add/remove obstacle  |  right-click to remove",
            map_panel.x + 22,
            map_panel.bottom - 30,
            MUTED,
            SMALL
        )

    else:

        draw_text(
            "SIMULATION: routes are locked while the fleet is running",
            map_panel.x + 22,
            map_panel.bottom - 30,
            MUTED,
            SMALL
        )


# ----------------------- AGV STATUS CARDS -------------------

def draw_agv_card(robot, rect):

    rounded(
        rect,
        PANEL_ALT,
        10,
        BORDER
    )

    pygame.draw.circle(
        screen,
        robot["color"],
        (rect.x + 22, rect.y + 25),
        7
    )

    draw_text(
        robot["name"],
        rect.x + 40,
        rect.y + 14,
        TEXT,
        MEDIUM
    )

    status_color = (
        GREEN if robot["status"] == "Delivered"
        else YELLOW if robot["status"] == "Waiting"
        else BLUE if robot["status"] == "Moving"
        else MUTED
    )

    status_surf = SMALL.render(
        robot["status"].upper(),
        True,
        status_color
    )

    screen.blit(
        status_surf,
        (
            rect.right - status_surf.get_width() - 15,
            rect.y + 19
        )
    )

    draw_text(
        f"Position   {robot['pos'][0] + 1}, {robot['pos'][1] + 1}",
        rect.x + 16,
        rect.y + 52,
        MUTED,
        SMALL
    )

    draw_text(
        f"Destination   {robot['goal'][0] + 1}, {robot['goal'][1] + 1}",
        rect.x + 16,
        rect.y + 75,
        MUTED,
        SMALL
    )

    draw_text(
        f"Travelled   {robot['distance']} cells",
        rect.x + 16,
        rect.y + 98,
        TEXT,
        SMALL
    )


# ------------------------- SIDEBAR --------------------------

def draw_sidebar():

    rounded(
        sidebar,
        PANEL,
        14,
        BORDER
    )

    draw_text(
        "FLEET COMMAND",
        sidebar.x + 24,
        sidebar.y + 20,
        TEXT,
        MEDIUM
    )

    draw_text(
        "Configure the floor and dispatch robots",
        sidebar.x + 24,
        sidebar.y + 49,
        MUTED,
        SMALL
    )

    draw_button(
        "edit",
        "EDIT WAREHOUSE",
        GREEN if mode == "EDIT" else PANEL_ALT,
        mode == "EDIT"
    )

    draw_button(
        "simulate",
        "SIMULATION VIEW",
        BLUE if mode == "SIMULATION" else PANEL_ALT,
        mode == "SIMULATION"
    )

    draw_button(
        "start",
        "▶   START FLEET SIMULATION",
        GREEN
    )

    draw_button(
        "reset",
        "RESET FLEET",
        RED
    )

    draw_button(
        "clear",
        "CLEAR OBSTACLES",
        PANEL_ALT,
        mode == "EDIT"
    )

    draw_text(
        "DESTINATION CONFIGURATION",
        sidebar.x + 24,
        445,
        TEXT,
        SMALL
    )

    draw_button(
        "goal1",
        "SET AGV 1 DESTINATION",
        ORANGE if active_goal_robot == 0 else BLUE,
        mode == "EDIT"
    )

    draw_button(
        "goal2",
        "SET AGV 2 DESTINATION",
        ORANGE if active_goal_robot == 1 else GREEN,
        mode == "EDIT"
    )

    draw_text(
        "ROBOT TELEMETRY",
        sidebar.x + 24,
        540,
        TEXT,
        SMALL
    )

    draw_agv_card(
        robots[0],
        pygame.Rect(
            sidebar.x + 20,
            565,
            sidebar.width - 40,
            132
        )
    )

    draw_agv_card(
        robots[1],
        pygame.Rect(
            sidebar.x + 20,
            710,
            sidebar.width - 40,
            132
        )
    )


# ------------------------- FOOTER ---------------------------

def draw_footer():

    rounded(
        pygame.Rect(30, 855, 1380, 35),
        HEADER,
        9,
        BORDER
    )

    dot_color = (
        RED if (
            "No " in simulation_message
            or "conflict" in simulation_message.lower()
            or "blocked" in simulation_message.lower()
        )
        else GREEN if (
            "successfully" in simulation_message.lower()
            or "completed" in simulation_message.lower()
        )
        else YELLOW
    )

    pygame.draw.circle(
        screen,
        dot_color,
        (52, 872),
        5
    )

    draw_text(
        "SYSTEM MESSAGE",
        67,
        861,
        MUTED,
        TINY
    )

    draw_text(
        simulation_message[:105],
        190,
        860,
        TEXT,
        SMALL
    )


# ---------------------- MAIN DASHBOARD ----------------------

def draw_dashboard():

    screen.fill(BG)

    draw_header()

    total_distance = sum(
        r["distance"] for r in robots
    )

    delivered = sum(
        1 for r in robots if r["completed"]
    )

    cards = [
        (
            pygame.Rect(30, 140, 245, 85),
            "AGVs IN FLEET",
            "02",
            BLUE
        ),
        (
            pygame.Rect(290, 140, 245, 85),
            "DELIVERIES",
            f"{delivered}/2",
            GREEN
        ),
        (
            pygame.Rect(550, 140, 245, 85),
            "OBSTACLES",
            f"{len(obstacles):02d}",
            ORANGE
        ),
        (
            pygame.Rect(825, 140, 285, 85),
            "TOTAL DISTANCE",
            f"{total_distance} cells",
            YELLOW
        ),
    ]

    for rect, label, value, accent in cards:

        draw_metric_card(
            rect,
            label,
            value,
            accent
        )

    draw_map()
    draw_sidebar()
    draw_footer()

    pygame.display.flip()


# ---------------------- USER INTERACTION --------------------

def handle_grid_click(position):

    global active_goal_robot
    global simulation_message

    if mode != "EDIT":
        return

    x, y = position

    if not (
        GRID_X <= x < GRID_X + GRID_W
        and GRID_Y <= y < GRID_Y + GRID_H
    ):
        return

    col = (x - GRID_X) // CELL
    row = (y - GRID_Y) // CELL

    cell = (row, col)

    # Destination selection

    if active_goal_robot is not None:

        robot = robots[active_goal_robot]

        protected = set()

        for other in robots:

            protected.add(other["start"])

            if other is not robot:
                protected.add(other["goal"])

        if cell in protected:

            simulation_message = (
                "That cell is already reserved by another AGV."
            )

            return

        if cell in obstacles:

            simulation_message = (
                "Remove the obstacle before assigning this destination."
            )

            return

        if cell == robot["start"]:

            simulation_message = (
                "Choose a destination different from the start cell."
            )

            return

        robot["goal"] = cell

        selected_name = robot["name"]

        active_goal_robot = None

        reset_simulation()

        simulation_message = (
            f"{selected_name} destination updated."
        )

        return

    # Normal obstacle editing

    protected_cells = (
        {r["start"] for r in robots}
        | {r["goal"] for r in robots}
    )

    if cell in protected_cells:

        simulation_message = (
            "This cell is reserved for an AGV start or destination."
        )

        return

    if cell in obstacles:
        obstacles.remove(cell)
    else:
        obstacles.add(cell)

    reset_simulation()

    simulation_message = "Warehouse layout updated."


# ----------------------- BUTTON HANDLING --------------------

def handle_button_click(position):

    global mode
    global active_goal_robot
    global simulation_message

    if buttons["edit"].collidepoint(position):

        reset_simulation()

        mode = "EDIT"
        active_goal_robot = None

        simulation_message = (
            "Edit mode enabled. Modify obstacles and destinations."
        )

    elif buttons["simulate"].collidepoint(position):

        active_goal_robot = None
        mode = "SIMULATION"

        reset_simulation()

        simulation_message = (
            "Simulation view ready. Press Start Fleet Simulation."
        )

    elif buttons["start"].collidepoint(position):

        active_goal_robot = None
        mode = "SIMULATION"

        start_simulation()

    elif buttons["reset"].collidepoint(position):

        active_goal_robot = None

        reset_simulation()

    elif buttons["clear"].collidepoint(position):

        if mode == "EDIT":

            obstacles.clear()

            reset_simulation()

            simulation_message = (
                "All warehouse obstacles removed."
            )

        else:

            simulation_message = (
                "Switch to Edit Warehouse before clearing obstacles."
            )

    elif buttons["goal1"].collidepoint(position):

        if mode == "EDIT":

            reset_simulation()

            active_goal_robot = 0

            simulation_message = (
                "Click a free grid cell to set AGV 1 destination."
            )

        else:

            simulation_message = (
                "Switch to Edit Warehouse to change destinations."
            )

    elif buttons["goal2"].collidepoint(position):

        if mode == "EDIT":

            reset_simulation()

            active_goal_robot = 1

            simulation_message = (
                "Click a free grid cell to set AGV 2 destination."
            )

        else:

            simulation_message = (
                "Switch to Edit Warehouse to change destinations."
            )


# --------------------------- MAIN LOOP -----------------------

running = True

while running:

    clock.tick(60)

    current_time = pygame.time.get_ticks()

    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.MOUSEBUTTONDOWN:

            if event.button == 1:

                if any(
                    rect.collidepoint(event.pos)
                    for rect in buttons.values()
                ):

                    handle_button_click(event.pos)

                else:

                    handle_grid_click(event.pos)

            elif event.button == 3 and mode == "EDIT":

                x, y = event.pos

                if (
                    GRID_X <= x < GRID_X + GRID_W
                    and GRID_Y <= y < GRID_Y + GRID_H
                ):

                    col = (x - GRID_X) // CELL
                    row = (y - GRID_Y) // CELL

                    cell = (row, col)

                    protected = any(
                        cell in (r["start"], r["goal"])
                        for r in robots
                    )

                    if not protected:

                        obstacles.discard(cell)

                        reset_simulation()

                        simulation_message = "Obstacle removed."

    # Timed AGV movement

    if (
        simulation_running
        and current_time - last_move_time >= MOVE_DELAY
    ):

        update_simulation()

        last_move_time = current_time

    draw_dashboard()


pygame.quit()
sys.exit()