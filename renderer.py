"""
Pygame Desktop Interface Renderer
=================================
Provides rendering components, dark-theme styling, custom drawing for grid cells,
markers, path overlays, robot animations, sidebar metrics cards, and UI buttons.
"""

from __future__ import annotations

import math
from typing import Callable, List, Optional, Tuple

import pygame

from astar import AlgorithmMode, Coordinate, SearchResult, SearchStatus
from grid import GridModel

# ============================================================================
# Color Palette - Professional Dark Theme
# ============================================================================
COLOR_BG = (18, 20, 24)               # Main dark background #121418
COLOR_HEADER_BG = (24, 27, 34)        # Header background #181B22
COLOR_PANEL_BG = (26, 31, 40)         # Card/Panel background #1A1F28
COLOR_PANEL_BORDER = (42, 50, 64)     # Border for cards #2A3240
COLOR_PANEL_HEADER = (34, 40, 52)     # Card subheader bar

COLOR_CELL_EMPTY = (22, 26, 34)       # Normal grid cell #161A22
COLOR_CELL_BORDER = (32, 38, 50)      # Grid cell outline #202632
COLOR_CELL_OBSTACLE = (65, 75, 92)    # Dark slate obstacle #414B5C
COLOR_CELL_OBSTACLE_HI = (80, 92, 112)# Obstacle highlight edge

COLOR_EXPLORED = (45, 85, 145)        # Muted blue for closed set #2D5591
COLOR_EXPLORED_BORDER = (60, 110, 180)
COLOR_FRONTIER = (217, 130, 28)       # Warm amber for open set #D9821C
COLOR_FRONTIER_BORDER = (245, 166, 35)

COLOR_PATH = (6, 182, 212)            # Vibrant electric cyan for shortest path #06B6D4
COLOR_PATH_GLOW = (14, 116, 144)      # Path border glow
COLOR_PATH_LINE = (165, 243, 252)     # Path center connection line

COLOR_START = (16, 185, 129)          # Emerald green #10B981
COLOR_START_BORDER = (5, 150, 105)
COLOR_GOAL = (239, 68, 68)            # Vivid Crimson / Coral #EF4444
COLOR_GOAL_BORDER = (220, 38, 38)

COLOR_ROBOT_BODY = (241, 245, 249)    # Metallic off-white robot chassis
COLOR_ROBOT_ACCENT = (6, 182, 212)    # Robot cyan sensor eye
COLOR_ROBOT_TRACKS = (51, 65, 85)     # Dark slate tracks/wheels

COLOR_TEXT_PRIMARY = (248, 250, 252)  # High-contrast white
COLOR_TEXT_MUTED = (148, 163, 184)    # Slate muted text
COLOR_TEXT_ACCENT = (56, 189, 248)    # Cyan highlight
COLOR_TEXT_SUCCESS = (52, 211, 153)   # Green status text
COLOR_TEXT_WARNING = (251, 191, 36)   # Amber status text
COLOR_TEXT_ERROR = (248, 113, 113)    # Red error text

COLOR_BTN_DEFAULT = (38, 45, 58)
COLOR_BTN_HOVER = (51, 62, 80)
COLOR_BTN_ACTIVE = (14, 116, 144)
COLOR_BTN_BORDER = (58, 69, 88)
COLOR_BTN_TEXT = (241, 245, 249)


class UIButton:
    """Interactive button widget for Pygame with hover, click, and active states."""

    def __init__(
        self,
        rect: Tuple[int, int, int, int],
        text: str,
        callback: Callable[[], None],
        tooltip: str = "",
        active: bool = False,
        accent_color: Optional[Tuple[int, int, int]] = None
    ) -> None:
        self.rect = pygame.Rect(rect)
        self.text = text
        self.callback = callback
        self.tooltip = tooltip
        self.active = active
        self.hovered = False
        self.accent_color = accent_color

    def handle_event(self, event: pygame.event.Event) -> bool:
        """Handle mouse motion and click events. Returns True if clicked."""
        if event.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.callback()
                return True
        return False

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        """Draw button with rounded corners and appropriate state styling."""
        if self.active:
            bg_col = self.accent_color if self.accent_color else COLOR_BTN_ACTIVE
            border_col = COLOR_TEXT_ACCENT
        elif self.hovered:
            bg_col = COLOR_BTN_HOVER
            border_col = COLOR_TEXT_ACCENT
        else:
            bg_col = COLOR_BTN_DEFAULT
            border_col = COLOR_BTN_BORDER

        pygame.draw.rect(surface, bg_col, self.rect, border_radius=6)
        pygame.draw.rect(surface, border_col, self.rect, width=1, border_radius=6)

        txt_surf = font.render(self.text, True, COLOR_TEXT_PRIMARY)
        txt_rect = txt_surf.get_rect(center=self.rect.center)
        surface.blit(txt_surf, txt_rect)


class Renderer:
    """Manages all graphical rendering for the robot path planning application."""

    def __init__(self, screen_width: int = 1280, screen_height: int = 780) -> None:
        self.width = screen_width
        self.height = screen_height

        # Initialize Fonts
        pygame.font.init()
        self.font_title = pygame.font.Font(None, 26)
        self.font_subtitle = pygame.font.Font(None, 18)
        self.font_header = pygame.font.Font(None, 20)
        self.font_body = pygame.font.Font(None, 17)
        self.font_bold = pygame.font.Font(None, 17)
        self.font_small = pygame.font.Font(None, 15)
        self.font_badge = pygame.font.Font(None, 14)

        # Layout Areas
        self.header_rect = pygame.Rect(0, 0, self.width, 60)
        self.footer_rect = pygame.Rect(0, self.height - 35, self.width, 35)

        # Main Grid View Area
        self.grid_container_rect = pygame.Rect(20, 75, 870, 580)

        # Control Panel Area below Grid
        self.controls_rect = pygame.Rect(20, 665, 870, 70)

        # Sidebar Area
        self.sidebar_rect = pygame.Rect(905, 75, 355, 660)

    def compute_cell_geometry(
        self, grid: GridModel
    ) -> Tuple[int, int, int]:
        """
        Compute optimal cell size and offsets to center the grid inside its container.
        Returns: (cell_size, offset_x, offset_y)
        """
        max_w = self.grid_container_rect.width - 16
        max_h = self.grid_container_rect.height - 16

        cell_w = max_w // grid.cols
        cell_h = max_h // grid.rows
        cell_size = max(10, min(cell_w, cell_h))

        grid_total_w = cell_size * grid.cols
        grid_total_h = cell_size * grid.rows

        offset_x = self.grid_container_rect.x + (self.grid_container_rect.width - grid_total_w) // 2
        offset_y = self.grid_container_rect.y + (self.grid_container_rect.height - grid_total_h) // 2

        return cell_size, offset_x, offset_y

    def grid_to_screen(
        self, r: int, c: int, cell_size: int, offset_x: int, offset_y: int
    ) -> Tuple[int, int]:
        """Convert grid (row, col) to pixel coordinates (x, y) of cell's top-left."""
        return offset_x + c * cell_size, offset_y + r * cell_size

    def screen_to_grid(
        self, pos: Tuple[int, int], grid: GridModel
    ) -> Optional[Coordinate]:
        """Convert screen pixel coordinates to grid (row, col), or None if outside."""
        cell_size, offset_x, offset_y = self.compute_cell_geometry(grid)
        x, y = pos

        rel_x = x - offset_x
        rel_y = y - offset_y

        if 0 <= rel_x < grid.cols * cell_size and 0 <= rel_y < grid.rows * cell_size:
            c = rel_x // cell_size
            r = rel_y // cell_size
            if grid.in_bounds(r, c):
                return r, c
        return None

    def draw_header(
        self,
        surface: pygame.Surface,
        algorithm_mode: AlgorithmMode,
        status_text: str,
        is_running: bool
    ) -> None:
        """Draw top navigation header with project title and status indicator."""
        pygame.draw.rect(surface, COLOR_HEADER_BG, self.header_rect)
        pygame.draw.line(
            surface, COLOR_PANEL_BORDER, (0, self.header_rect.bottom),
            (self.width, self.header_rect.bottom), 1
        )

        # Title and Subtitle
        title_surf = self.font_title.render(
            "Robot Path Planning Simulator", True, COLOR_TEXT_PRIMARY
        )
        surface.blit(title_surf, (24, 12))

        sub_text = f"Implementation: A* Search Algorithm ({'Manhattan Distance' if algorithm_mode == AlgorithmMode.ASTAR else 'Dijkstra Mode h=0'})"
        sub_surf = self.font_subtitle.render(sub_text, True, COLOR_TEXT_MUTED)
        surface.blit(sub_surf, (24, 38))

        # Status Badge on Header Right
        badge_bg = COLOR_PANEL_BG
        badge_border = COLOR_PANEL_BORDER
        badge_text_col = COLOR_TEXT_MUTED

        if is_running:
            badge_bg = (14, 50, 70)
            badge_border = COLOR_PATH
            badge_text_col = COLOR_TEXT_ACCENT
        elif "Found" in status_text:
            badge_bg = (10, 50, 35)
            badge_border = COLOR_START
            badge_text_col = COLOR_TEXT_SUCCESS
        elif "No Path" in status_text or "Blocked" in status_text:
            badge_bg = (60, 20, 20)
            badge_border = COLOR_GOAL
            badge_text_col = COLOR_TEXT_ERROR

        badge_rect = pygame.Rect(self.width - 320, 14, 295, 32)
        pygame.draw.rect(surface, badge_bg, badge_rect, border_radius=6)
        pygame.draw.rect(surface, badge_border, badge_rect, width=1, border_radius=6)

        # Small pulsating dot
        dot_color = badge_text_col
        pygame.draw.circle(surface, dot_color, (badge_rect.x + 16, badge_rect.centery), 4)

        status_surf = self.font_small.render(status_text[:38], True, badge_text_col)
        surface.blit(status_surf, (badge_rect.x + 28, badge_rect.y + 9))

    def draw_footer(
        self, surface: pygame.Surface, hint_text: str
    ) -> None:
        """Draw bottom footer bar with mouse hints and operational context."""
        pygame.draw.rect(surface, COLOR_HEADER_BG, self.footer_rect)
        pygame.draw.line(
            surface, COLOR_PANEL_BORDER, (0, self.footer_rect.top),
            (self.width, self.footer_rect.top), 1
        )

        hint_surf = self.font_small.render(hint_text, True, COLOR_TEXT_MUTED)
        surface.blit(hint_surf, (24, self.footer_rect.y + 10))

        credit_surf = self.font_badge.render("DAA College Project | Python & Pygame", True, (100, 116, 139))
        surface.blit(credit_surf, (self.width - 240, self.footer_rect.y + 11))

    def draw_grid_canvas(
        self,
        surface: pygame.Surface,
        grid: GridModel,
        robot_interp_pos: Optional[Tuple[float, float]] = None
    ) -> None:
        """Draw the main grid environment, cells, states, and markers."""
        # Container background
        pygame.draw.rect(surface, COLOR_PANEL_BG, self.grid_container_rect, border_radius=8)
        pygame.draw.rect(surface, COLOR_PANEL_BORDER, self.grid_container_rect, width=1, border_radius=8)

        cell_size, offset_x, offset_y = self.compute_cell_geometry(grid)

        path_set = set(grid.path)

        # Draw Cells
        for r in range(grid.rows):
            for c in range(grid.cols):
                cx, cy = self.grid_to_screen(r, c, cell_size, offset_x, offset_y)
                cell_rect = pygame.Rect(cx, cy, cell_size, cell_size)

                # Determine base cell color
                coord = (r, c)
                if coord in grid.obstacles:
                    # Obstacle Cell
                    pygame.draw.rect(surface, COLOR_CELL_OBSTACLE, cell_rect)
                    # Subtle 3D bevel / highlight
                    pygame.draw.rect(surface, COLOR_CELL_OBSTACLE_HI, cell_rect, width=1)
                elif coord in path_set and coord != grid.start and coord != grid.goal:
                    # Shortest Path Cell
                    pygame.draw.rect(surface, COLOR_PATH, cell_rect)
                    pygame.draw.rect(surface, COLOR_PATH_GLOW, cell_rect, width=1)
                elif coord in grid.explored and coord != grid.start and coord != grid.goal:
                    # Explored (Closed Set) Cell
                    pygame.draw.rect(surface, COLOR_EXPLORED, cell_rect)
                    pygame.draw.rect(surface, COLOR_EXPLORED_BORDER, cell_rect, width=1)
                elif coord in grid.frontier and coord != grid.start and coord != grid.goal:
                    # Frontier (Open Set) Cell
                    pygame.draw.rect(surface, COLOR_FRONTIER, cell_rect)
                    pygame.draw.rect(surface, COLOR_FRONTIER_BORDER, cell_rect, width=1)
                else:
                    # Normal Walkable Cell
                    pygame.draw.rect(surface, COLOR_CELL_EMPTY, cell_rect)
                    pygame.draw.rect(surface, COLOR_CELL_BORDER, cell_rect, width=1)

        # Draw Connecting Line through Path for visual elegance
        if len(grid.path) > 1:
            points: List[Tuple[int, int]] = []
            for r, c in grid.path:
                px, py = self.grid_to_screen(r, c, cell_size, offset_x, offset_y)
                points.append((px + cell_size // 2, py + cell_size // 2))
            if len(points) >= 2:
                pygame.draw.lines(surface, COLOR_PATH_LINE, False, points, width=max(2, cell_size // 5))

        # Draw Start Marker (Emerald Green with 'S')
        sx, sy = self.grid_to_screen(grid.start[0], grid.start[1], cell_size, offset_x, offset_y)
        start_rect = pygame.Rect(sx, sy, cell_size, cell_size)
        pygame.draw.rect(surface, COLOR_START, start_rect, border_radius=4)
        pygame.draw.rect(surface, COLOR_START_BORDER, start_rect, width=2, border_radius=4)
        s_txt = self.font_bold.render("S", True, COLOR_TEXT_PRIMARY)
        surface.blit(s_txt, s_txt.get_rect(center=start_rect.center))

        # Draw Goal Marker (Vivid Crimson with 'G' & target ring)
        gx, gy = self.grid_to_screen(grid.goal[0], grid.goal[1], cell_size, offset_x, offset_y)
        goal_rect = pygame.Rect(gx, gy, cell_size, cell_size)
        pygame.draw.rect(surface, COLOR_GOAL, goal_rect, border_radius=4)
        pygame.draw.rect(surface, COLOR_GOAL_BORDER, goal_rect, width=2, border_radius=4)
        g_txt = self.font_bold.render("G", True, COLOR_TEXT_PRIMARY)
        surface.blit(g_txt, g_txt.get_rect(center=goal_rect.center))

        # Draw Animated Robot
        robot_r, robot_c = robot_interp_pos if robot_interp_pos is not None else grid.robot_pos
        self.draw_robot(surface, robot_r, robot_c, cell_size, offset_x, offset_y)

    def draw_robot(
        self,
        surface: pygame.Surface,
        r: float,
        c: float,
        cell_size: int,
        offset_x: int,
        offset_y: int
    ) -> None:
        """
        Draw a custom high-visibility metallic robot sprite with tracks, chassis,
        and glowing cyan sensor eye at position (r, c).
        """
        center_x = int(offset_x + c * cell_size + cell_size / 2.0)
        center_y = int(offset_y + r * cell_size + cell_size / 2.0)
        radius = max(6, int(cell_size * 0.42))

        # Outer track / wheel base
        track_w = int(radius * 1.8)
        track_h = int(radius * 1.5)
        track_rect = pygame.Rect(0, 0, track_w, track_h)
        track_rect.center = (center_x, center_y)
        pygame.draw.rect(surface, COLOR_ROBOT_TRACKS, track_rect, border_radius=3)

        # Main Chassis
        chassis_radius = int(radius * 0.78)
        pygame.draw.circle(surface, COLOR_ROBOT_BODY, (center_x, center_y), chassis_radius)
        pygame.draw.circle(surface, (148, 163, 184), (center_x, center_y), chassis_radius, width=1)

        # Glowing cyan directional sensor eye
        eye_radius = max(2, int(radius * 0.35))
        pygame.draw.circle(surface, COLOR_ROBOT_ACCENT, (center_x, center_y), eye_radius)
        pygame.draw.circle(surface, (255, 255, 255), (center_x - 1, center_y - 1), max(1, eye_radius // 2))

    def draw_sidebar(
        self,
        surface: pygame.Surface,
        grid: GridModel,
        result: SearchResult,
        algorithm_mode: AlgorithmMode,
        comparison_data: Optional[Tuple[SearchResult, SearchResult]] = None
    ) -> None:
        """Render information cards, live metrics, color legend, and comparison tables."""
        # Main sidebar panel
        pygame.draw.rect(surface, COLOR_PANEL_BG, self.sidebar_rect, border_radius=8)
        pygame.draw.rect(surface, COLOR_PANEL_BORDER, self.sidebar_rect, width=1, border_radius=8)

        curr_y = self.sidebar_rect.y + 12
        side_x = self.sidebar_rect.x + 14
        inner_w = self.sidebar_rect.width - 28

        # CARD 1: Simulation Configuration
        curr_y = self._draw_card_header(surface, side_x, curr_y, inner_w, "ENVIRONMENT CONFIG")
        obs_density = (len(grid.obstacles) / (grid.rows * grid.cols)) * 100.0
        curr_y = self._draw_stat_row(surface, side_x, curr_y, inner_w, "Grid Size:", f"{grid.rows} × {grid.cols} ({grid.rows * grid.cols} cells)")
        curr_y = self._draw_stat_row(surface, side_x, curr_y, inner_w, "Start Node (S):", f"Row {grid.start[0]}, Col {grid.start[1]}")
        curr_y = self._draw_stat_row(surface, side_x, curr_y, inner_w, "Goal Node (G):", f"Row {grid.goal[0]}, Col {grid.goal[1]}")
        curr_y = self._draw_stat_row(surface, side_x, curr_y, inner_w, "Obstacles:", f"{len(grid.obstacles)} cells ({obs_density:.1f}% density)")
        curr_y += 10

        # CARD 2: Performance Metrics (Measured Real Values)
        curr_y = self._draw_card_header(surface, side_x, curr_y, inner_w, "ALGORITHM PERFORMANCE")
        curr_y = self._draw_stat_row(surface, side_x, curr_y, inner_w, "Active Algorithm:", f"{algorithm_mode.value}")

        # High resolution computation time
        time_str = f"{result.execution_time_ms:.3f} ms" if result.execution_time_ms > 0 else "0.000 ms"
        curr_y = self._draw_stat_row(surface, side_x, curr_y, inner_w, "Computation Time:", time_str, val_color=COLOR_TEXT_ACCENT)

        path_len_str = f"{result.path_length} moves" if result.found else "N/A"
        curr_y = self._draw_stat_row(surface, side_x, curr_y, inner_w, "Path Length:", path_len_str)

        path_cost_str = f"{result.path_cost} (Uniform 1/step)" if result.found else "N/A"
        curr_y = self._draw_stat_row(surface, side_x, curr_y, inner_w, "Total Path Cost:", path_cost_str)

        explored_pct = (result.explored_count / (grid.rows * grid.cols)) * 100.0 if (grid.rows * grid.cols) > 0 else 0
        curr_y = self._draw_stat_row(surface, side_x, curr_y, inner_w, "Explored Nodes:", f"{result.explored_count} ({explored_pct:.1f}% of grid)")
        curr_y = self._draw_stat_row(surface, side_x, curr_y, inner_w, "Discovered Nodes:", f"{result.discovered_count}")

        status_col = COLOR_TEXT_SUCCESS if result.found else (COLOR_TEXT_ERROR if result.status != SearchStatus.NO_PATH else COLOR_TEXT_WARNING)
        curr_y = self._draw_stat_row(surface, side_x, curr_y, inner_w, "Status:", result.status.name, val_color=status_col)
        curr_y += 10

        # CARD 3: Comparison Mode (A* vs Dijkstra) or Color Legend
        if comparison_data is not None:
            res_astar, res_dijkstra = comparison_data
            curr_y = self._draw_card_header(surface, side_x, curr_y, inner_w, "A* vs DIJKSTRA COMPARISON")

            # Header Row
            self._draw_comp_row(surface, side_x, curr_y, inner_w, "Metric", "A* (Manh.)", "Dijkstra", is_header=True)
            curr_y += 20
            self._draw_comp_row(surface, side_x, curr_y, inner_w, "Status", "Success" if res_astar.found else "Fail", "Success" if res_dijkstra.found else "Fail")
            curr_y += 18
            self._draw_comp_row(surface, side_x, curr_y, inner_w, "Path Cost", str(res_astar.path_cost), str(res_dijkstra.path_cost))
            curr_y += 18
            self._draw_comp_row(surface, side_x, curr_y, inner_w, "Explored", str(res_astar.explored_count), str(res_dijkstra.explored_count), highlight_better=True)
            curr_y += 18
            self._draw_comp_row(surface, side_x, curr_y, inner_w, "Time (ms)", f"{res_astar.execution_time_ms:.2f}", f"{res_dijkstra.execution_time_ms:.2f}")
            curr_y += 22

            # Efficiency note
            if res_dijkstra.explored_count > 0:
                saved = ((res_dijkstra.explored_count - res_astar.explored_count) / res_dijkstra.explored_count) * 100.0
                note_txt = f"A* explored {saved:.1f}% fewer nodes to find optimal path!"
                note_surf = self.font_badge.render(note_txt, True, COLOR_TEXT_SUCCESS)
                surface.blit(note_surf, (side_x, curr_y))
                curr_y += 18
        else:
            # Color Legend
            curr_y = self._draw_card_header(surface, side_x, curr_y, inner_w, "COLOR LEGEND & KEY")
            legend_items = [
                ("Start Node", COLOR_START, "S"),
                ("Goal Node", COLOR_GOAL, "G"),
                ("Obstacle Wall", COLOR_CELL_OBSTACLE, ""),
                ("Explored (Closed)", COLOR_EXPLORED, ""),
                ("Frontier (Open)", COLOR_FRONTIER, ""),
                ("Optimal Path", COLOR_PATH, ""),
            ]

            leg_x = side_x
            for label, color, glyph in legend_items:
                swatch_rect = pygame.Rect(leg_x, curr_y, 14, 14)
                pygame.draw.rect(surface, color, swatch_rect, border_radius=3)
                if glyph:
                    g_surf = self.font_badge.render(glyph, True, COLOR_TEXT_PRIMARY)
                    surface.blit(g_surf, g_surf.get_rect(center=swatch_rect.center))
                lbl_surf = self.font_small.render(label, True, COLOR_TEXT_MUTED)
                surface.blit(lbl_surf, (leg_x + 22, curr_y))
                curr_y += 18
            curr_y += 6

        # CARD 4: Quick Controls Guide
        curr_y = self._draw_card_header(surface, side_x, curr_y, inner_w, "KEYBOARD SHORTCUTS")
        shortcuts = [
            ("SPACE", "Run / Pause Search"),
            ("S", "Step Single Event"),
            ("R", "Reset Simulation"),
            ("C", "Clear Obstacles"),
            ("G", "Random Obstacle Map"),
            ("M", "Compare A* vs Dijkstra"),
            ("1 / 2", "Switch A* / Dijkstra"),
        ]
        for key, desc in shortcuts:
            k_surf = self.font_badge.render(key, True, COLOR_TEXT_ACCENT)
            surface.blit(k_surf, (side_x, curr_y))
            d_surf = self.font_badge.render(desc, True, COLOR_TEXT_MUTED)
            surface.blit(d_surf, (side_x + 60, curr_y))
            curr_y += 16

    def _draw_card_header(
        self, surface: pygame.Surface, x: int, y: int, w: int, title: str
    ) -> int:
        """Draw a distinct section header in sidebar."""
        header_rect = pygame.Rect(x, y, w, 22)
        pygame.draw.rect(surface, COLOR_PANEL_HEADER, header_rect, border_radius=4)
        t_surf = self.font_badge.render(title, True, COLOR_TEXT_ACCENT)
        surface.blit(t_surf, (x + 8, y + 4))
        return y + 26

    def _draw_stat_row(
        self,
        surface: pygame.Surface,
        x: int,
        y: int,
        w: int,
        label: str,
        value: str,
        val_color: Tuple[int, int, int] = COLOR_TEXT_PRIMARY
    ) -> int:
        """Draw a formatted label-value metric row."""
        lbl_surf = self.font_small.render(label, True, COLOR_TEXT_MUTED)
        val_surf = self.font_small.render(value, True, val_color)

        surface.blit(lbl_surf, (x + 4, y))
        val_rect = val_surf.get_rect(right=x + w - 4, top=y)
        surface.blit(val_surf, val_rect)
        return y + 18

    def _draw_comp_row(
        self,
        surface: pygame.Surface,
        x: int,
        y: int,
        w: int,
        col1: str,
        col2: str,
        col3: str,
        is_header: bool = False,
        highlight_better: bool = False
    ) -> None:
        """Draw a 3-column comparison row for A* vs Dijkstra."""
        c1_w = int(w * 0.36)
        c2_w = int(w * 0.32)

        font = self.font_bold if is_header else self.font_small
        col = COLOR_TEXT_ACCENT if is_header else COLOR_TEXT_MUTED
        val_col_a = COLOR_TEXT_SUCCESS if highlight_better else COLOR_TEXT_PRIMARY
        val_col_d = COLOR_TEXT_PRIMARY

        s1 = font.render(col1, True, col)
        s2 = font.render(col2, True, col if is_header else val_col_a)
        s3 = font.render(col3, True, col if is_header else val_col_d)

        surface.blit(s1, (x + 4, y))
        surface.blit(s2, (x + 4 + c1_w, y))
        surface.blit(s3, (x + 4 + c1_w + c2_w, y))
