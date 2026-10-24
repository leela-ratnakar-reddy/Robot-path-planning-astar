"""
Main Application Controller
===========================
Robot Path Planning Simulator using the A* Search Algorithm.
Provides the Pygame interactive event loop, simulation state machine,
interactive controls, animated search replay, robot path traversal,
and real-time metric computation.
"""

from __future__ import annotations

import argparse
import sys
import time
from enum import Enum
from typing import List, Optional, Tuple

import pygame

from astar import AlgorithmMode, Coordinate, SearchResult, SearchStatus, astar_search
from grid import GridModel
from renderer import (
    COLOR_BG,
    COLOR_BTN_ACTIVE,
    COLOR_START,
    COLOR_GOAL,
    COLOR_CELL_OBSTACLE,
    Renderer,
    UIButton,
)


class ToolMode(str, Enum):
    """Mouse interaction mode."""
    DRAW_OBSTACLE = "Draw Obstacles"
    MOVE_START = "Move Start"
    MOVE_GOAL = "Move Goal"


class SimState(str, Enum):
    """Simulation state machine states."""
    IDLE = "IDLE"
    ANIMATING_SEARCH = "ANIMATING_SEARCH"
    PAUSED_SEARCH = "PAUSED_SEARCH"
    ANIMATING_ROBOT = "ANIMATING_ROBOT"
    PAUSED_ROBOT = "PAUSED_ROBOT"
    FINISHED = "FINISHED"


class PathPlanningApp:
    """Master application controller coordinating model, renderer, and events."""

    def __init__(self, headless: bool = False) -> None:
        self.headless = headless
        self.width = 1280
        self.height = 780
        self.fps = 60

        # Initialize Pygame Display
        if self.headless:
            import os
            os.environ["SDL_VIDEODRIVER"] = "dummy"

        pygame.init()
        pygame.display.set_caption("Robot Path Planning Simulator — A* Search Algorithm")

        self.screen = pygame.display.set_mode((self.width, self.height))
        self.clock = pygame.time.Clock()
        self.renderer = Renderer(self.width, self.height)

        # Core Models & State
        self.grid = GridModel(rows=25, cols=35)
        self.algorithm_mode = AlgorithmMode.ASTAR
        self.tool_mode = ToolMode.DRAW_OBSTACLE
        self.sim_state = SimState.IDLE

        # Search Results and Trace
        self.latest_result: SearchResult = SearchResult(
            found=False,
            status=SearchStatus.NO_PATH,
            status_message="Ready. Press Run or Space to start search."
        )
        self.comparison_data: Optional[Tuple[SearchResult, SearchResult]] = None

        # Animation State Variables
        self.trace_index: int = 0
        self.robot_path_index: int = 0
        self.robot_anim_substep: float = 0.0  # Smooth sub-cell animation [0.0, 1.0]
        self.robot_interp_pos: Optional[Tuple[float, float]] = None

        # Speed setting: steps per frame
        # Options: 1 (Slow), 5 (Normal), 20 (Fast), 999999 (Instant)
        self.steps_per_frame = 5
        self.random_seed_counter = 42

        # Mouse Interaction State
        self.mouse_pressed = False
        self.mouse_erase_mode = False

        # Status text for header and footer
        self.header_status = "Ready"
        self.footer_hint = "Left-click to place obstacles | Right-click to erase | SPACE to run A* search"

        # Initialize UI Buttons
        self.buttons: List[UIButton] = []
        self._init_buttons()

    def _init_buttons(self) -> None:
        """Create interactive buttons positioned in the control panel below the grid."""
        bx = self.renderer.controls_rect.x + 10
        by = self.renderer.controls_rect.y + 12
        btn_h = 32

        # Run / Pause Button
        self.btn_run = UIButton(
            (bx, by, 110, btn_h), "Run A*", self.toggle_run_pause, accent_color=COLOR_BTN_ACTIVE
        )
        bx += 118

        # Step Button
        self.btn_step = UIButton(
            (bx, by, 80, btn_h), "Step", self.step_forward
        )
        bx += 88

        # Reset Simulation Button
        self.btn_reset = UIButton(
            (bx, by, 85, btn_h), "Reset", self.reset_simulation
        )
        bx += 93

        # Clear Obstacles Button
        self.btn_clear = UIButton(
            (bx, by, 85, btn_h), "Clear", self.clear_obstacles
        )
        bx += 93

        # Random Map Button
        self.btn_random = UIButton(
            (bx, by, 100, btn_h), "Random", self.generate_random_map
        )
        bx += 108

        # Compare A* vs Dijkstra Button
        self.btn_compare = UIButton(
            (bx, by, 120, btn_h), "Compare", self.run_comparison
        )
        bx += 128

        # Algorithm Mode Toggle (A* vs Dijkstra)
        self.btn_algo = UIButton(
            (bx, by, 110, btn_h), "Algo: A*", self.toggle_algorithm
        )
        bx += 118

        # Speed Cycle Button
        self.btn_speed = UIButton(
            (bx, by, 90, btn_h), "Speed: 5x", self.cycle_speed
        )

        # Row 2 Tool Mode Buttons
        by_row2 = by + 36
        bx_row2 = self.renderer.controls_rect.x + 10

        self.btn_mode_obs = UIButton(
            (bx_row2, by_row2, 130, 24), "Tool: Wall", lambda: self.set_tool_mode(ToolMode.DRAW_OBSTACLE),
            active=True
        )
        bx_row2 += 138

        self.btn_mode_start = UIButton(
            (bx_row2, by_row2, 130, 24), "Tool: Start", lambda: self.set_tool_mode(ToolMode.MOVE_START),
            accent_color=COLOR_START
        )
        bx_row2 += 138

        self.btn_mode_goal = UIButton(
            (bx_row2, by_row2, 130, 24), "Tool: Goal", lambda: self.set_tool_mode(ToolMode.MOVE_GOAL),
            accent_color=COLOR_GOAL
        )

        self.buttons = [
            self.btn_run,
            self.btn_step,
            self.btn_reset,
            self.btn_clear,
            self.btn_random,
            self.btn_compare,
            self.btn_algo,
            self.btn_speed,
            self.btn_mode_obs,
            self.btn_mode_start,
            self.btn_mode_goal,
        ]

    def set_tool_mode(self, mode: ToolMode) -> None:
        """Switch mouse interaction tool mode."""
        self.tool_mode = mode
        self.btn_mode_obs.active = (mode == ToolMode.DRAW_OBSTACLE)
        self.btn_mode_start.active = (mode == ToolMode.MOVE_START)
        self.btn_mode_goal.active = (mode == ToolMode.MOVE_GOAL)

        if mode == ToolMode.DRAW_OBSTACLE:
            self.footer_hint = "Left-click to place walls | Right-click to erase walls"
        elif mode == ToolMode.MOVE_START:
            self.footer_hint = "Click any walkable cell to reposition Start Node (S)"
        elif mode == ToolMode.MOVE_GOAL:
            self.footer_hint = "Click any walkable cell to reposition Goal Node (G)"

    def toggle_algorithm(self) -> None:
        """Toggle between A* (Manhattan heuristic) and Dijkstra (h=0)."""
        if self.algorithm_mode == AlgorithmMode.ASTAR:
            self.algorithm_mode = AlgorithmMode.DIJKSTRA
            self.btn_algo.text = "Algo: Dijkstra"
        else:
            self.algorithm_mode = AlgorithmMode.ASTAR
            self.btn_algo.text = "Algo: A*"

        self.reset_simulation()
        self.header_status = f"Mode changed to {self.algorithm_mode.value}"

    def cycle_speed(self) -> None:
        """Cycle through animation speeds: 1x -> 5x -> 20x -> Instant -> 1x."""
        if self.steps_per_frame == 1:
            self.steps_per_frame = 5
            self.btn_speed.text = "Speed: 5x"
        elif self.steps_per_frame == 5:
            self.steps_per_frame = 20
            self.btn_speed.text = "Speed: 20x"
        elif self.steps_per_frame == 20:
            self.steps_per_frame = 999999
            self.btn_speed.text = "Speed: Max"
        else:
            self.steps_per_frame = 1
            self.btn_speed.text = "Speed: 1x"

    def run_comparison(self) -> None:
        """
        Execute both A* and Dijkstra on the identical current map, record real performance,
        and display a side-by-side comparison.
        """
        self.grid.clear_search_metadata()
        self.sim_state = SimState.IDLE

        # Run A*
        res_astar = astar_search(
            start=self.grid.start,
            goal=self.grid.goal,
            rows=self.grid.rows,
            cols=self.grid.cols,
            obstacles=self.grid.obstacles,
            mode=AlgorithmMode.ASTAR,
            record_trace=False
        )

        # Run Dijkstra
        res_dijkstra = astar_search(
            start=self.grid.start,
            goal=self.grid.goal,
            rows=self.grid.rows,
            cols=self.grid.cols,
            obstacles=self.grid.obstacles,
            mode=AlgorithmMode.DIJKSTRA,
            record_trace=False
        )

        self.comparison_data = (res_astar, res_dijkstra)
        self.latest_result = res_astar if self.algorithm_mode == AlgorithmMode.ASTAR else res_dijkstra

        if res_astar.found:
            self.grid.path = list(res_astar.path)
            self.grid.explored = set(res_astar.explored_nodes)
            self.header_status = f"Comparison complete: A* explored {res_astar.explored_count} vs Dijkstra {res_dijkstra.explored_count} nodes."
        else:
            self.header_status = "Comparison complete: Unreachable destination for both."

    def start_search(self) -> None:
        """Compute the search result and launch the animated trace replay."""
        self.grid.clear_search_metadata()
        self.comparison_data = None
        self.trace_index = 0
        self.robot_path_index = 0
        self.robot_anim_substep = 0.0
        self.robot_interp_pos = (float(self.grid.start[0]), float(self.grid.start[1]))

        # High-resolution monotonic timer measures real execution time
        self.latest_result = astar_search(
            start=self.grid.start,
            goal=self.grid.goal,
            rows=self.grid.rows,
            cols=self.grid.cols,
            obstacles=self.grid.obstacles,
            mode=self.algorithm_mode,
            record_trace=True
        )

        if not self.latest_result.found:
            # Replay exploration to show failure
            self.sim_state = SimState.ANIMATING_SEARCH
            self.btn_run.text = "Pause"
            self.header_status = f"Searching ({self.algorithm_mode.value})..."
            return

        if self.steps_per_frame >= 999999:
            # Instant mode: reveal immediately
            self.grid.explored = set(self.latest_result.explored_nodes)
            self.grid.path = list(self.latest_result.path)
            self.grid.robot_pos = self.grid.goal
            self.robot_interp_pos = (float(self.grid.goal[0]), float(self.grid.goal[1]))
            self.sim_state = SimState.FINISHED
            self.btn_run.text = "Run Search"
            self.header_status = f"Optimal path found! Cost: {self.latest_result.path_cost}"
        else:
            self.sim_state = SimState.ANIMATING_SEARCH
            self.btn_run.text = "Pause"
            self.header_status = f"Exploring grid ({self.algorithm_mode.value})..."

    def toggle_run_pause(self) -> None:
        """Toggle simulation execution between running and paused states."""
        if self.sim_state == SimState.IDLE or self.sim_state == SimState.FINISHED:
            self.start_search()
        elif self.sim_state == SimState.ANIMATING_SEARCH:
            self.sim_state = SimState.PAUSED_SEARCH
            self.btn_run.text = "Resume"
            self.header_status = "Search paused."
        elif self.sim_state == SimState.PAUSED_SEARCH:
            self.sim_state = SimState.ANIMATING_SEARCH
            self.btn_run.text = "Pause"
            self.header_status = "Resuming search..."
        elif self.sim_state == SimState.ANIMATING_ROBOT:
            self.sim_state = SimState.PAUSED_ROBOT
            self.btn_run.text = "Resume"
            self.header_status = "Robot travel paused."
        elif self.sim_state == SimState.PAUSED_ROBOT:
            self.sim_state = SimState.ANIMATING_ROBOT
            self.btn_run.text = "Pause"
            self.header_status = "Robot traveling..."

    def step_forward(self) -> None:
        """Step forward by one event during search or one cell during robot movement."""
        if self.sim_state == SimState.IDLE:
            self.start_search()
            self.sim_state = SimState.PAUSED_SEARCH
            self.btn_run.text = "Resume"
            return

        if self.sim_state in (SimState.ANIMATING_SEARCH, SimState.PAUSED_SEARCH):
            self.sim_state = SimState.PAUSED_SEARCH
            self.btn_run.text = "Resume"
            self._advance_trace_events(1)
        elif self.sim_state in (SimState.ANIMATING_ROBOT, SimState.PAUSED_ROBOT):
            self.sim_state = SimState.PAUSED_ROBOT
            self.btn_run.text = "Resume"
            self._advance_robot_step()

    def reset_simulation(self) -> None:
        """Reset search exploration, paths, and robot position while keeping obstacles."""
        self.sim_state = SimState.IDLE
        self.grid.clear_search_metadata()
        self.comparison_data = None
        self.trace_index = 0
        self.robot_path_index = 0
        self.robot_anim_substep = 0.0
        self.robot_interp_pos = (float(self.grid.start[0]), float(self.grid.start[1]))
        self.btn_run.text = "Run Search"
        self.header_status = "Simulation reset. Ready."

    def clear_obstacles(self) -> None:
        """Clear all obstacles and reset simulation state."""
        self.grid.clear_obstacles()
        self.reset_simulation()
        self.header_status = "Cleared all obstacles."

    def generate_random_map(self) -> None:
        """Generate random reproducible obstacle map."""
        self.random_seed_counter += 1
        count = self.grid.generate_random_obstacles(density=0.25, seed=self.random_seed_counter)
        self.reset_simulation()
        self.header_status = f"Generated random map with {count} obstacle cells (Seed: {self.random_seed_counter})."

    def _advance_trace_events(self, count: int) -> None:
        """Process search trace events for animation playback."""
        trace = self.latest_result.trace
        for _ in range(count):
            if self.trace_index >= len(trace):
                # Search trace finished
                if self.latest_result.found:
                    self.grid.path = list(self.latest_result.path)
                    self.sim_state = SimState.ANIMATING_ROBOT
                    self.header_status = f"Path discovered! Robot moving along {self.latest_result.path_length} moves..."
                else:
                    self.sim_state = SimState.FINISHED
                    self.btn_run.text = "Run Search"
                    self.header_status = "Search completed: No path found!"
                break

            event = trace[self.trace_index]
            self.trace_index += 1

            if event.event_type == "DISCOVER":
                self.grid.frontier.add(event.coord)
            elif event.event_type == "VISIT":
                self.grid.frontier.discard(event.coord)
                self.grid.explored.add(event.coord)
            elif event.event_type == "PATH":
                self.grid.path.append(event.coord)

    def _advance_robot_step(self) -> None:
        """Advance robot one whole cell along the path."""
        path = self.latest_result.path
        if self.robot_path_index < len(path) - 1:
            self.robot_path_index += 1
            node = path[self.robot_path_index]
            self.grid.robot_pos = node
            self.robot_interp_pos = (float(node[0]), float(node[1]))

            if self.robot_path_index == len(path) - 1:
                self.sim_state = SimState.FINISHED
                self.btn_run.text = "Run Search"
                self.header_status = f"Goal Reached! Optimal Path Cost: {self.latest_result.path_cost}"

    def _update_robot_animation(self) -> None:
        """Smoothly interpolate robot between consecutive path cells."""
        path = self.latest_result.path
        if not path or self.robot_path_index >= len(path) - 1:
            self.sim_state = SimState.FINISHED
            self.btn_run.text = "Run Search"
            self.header_status = f"Goal Reached! Optimal Path Cost: {self.latest_result.path_cost}"
            return

        # Advance substep based on speed
        speed_factor = 0.08 if self.steps_per_frame == 1 else (0.18 if self.steps_per_frame == 5 else 0.40)
        self.robot_anim_substep += speed_factor

        curr_node = path[self.robot_path_index]
        next_node = path[self.robot_path_index + 1]

        if self.robot_anim_substep >= 1.0:
            self.robot_anim_substep = 0.0
            self.robot_path_index += 1
            self.grid.robot_pos = next_node
            self.robot_interp_pos = (float(next_node[0]), float(next_node[1]))

            if self.robot_path_index >= len(path) - 1:
                self.sim_state = SimState.FINISHED
                self.btn_run.text = "Run Search"
                self.header_status = f"Goal Reached! Optimal Path Cost: {self.latest_result.path_cost}"
        else:
            # Linear interpolation
            interp_r = curr_node[0] + (next_node[0] - curr_node[0]) * self.robot_anim_substep
            interp_c = curr_node[1] + (next_node[1] - curr_node[1]) * self.robot_anim_substep
            self.robot_interp_pos = (interp_r, interp_c)

    def handle_mouse_grid_action(self, pos: Tuple[int, int], is_drag: bool = False) -> None:
        """Apply current tool action when clicking or dragging on grid."""
        # Prevent grid modifications during active animation
        if self.sim_state in (SimState.ANIMATING_SEARCH, SimState.ANIMATING_ROBOT):
            return

        coord = self.renderer.screen_to_grid(pos, self.grid)
        if coord is None:
            return

        r, c = coord

        if self.tool_mode == ToolMode.DRAW_OBSTACLE:
            if self.mouse_erase_mode:
                if self.grid.is_obstacle(r, c):
                    self.grid.remove_obstacle(r, c)
                    self.reset_simulation()
            else:
                if not self.grid.is_obstacle(r, c) and (r, c) != self.grid.start and (r, c) != self.grid.goal:
                    self.grid.add_obstacle(r, c)
                    self.reset_simulation()
        elif self.tool_mode == ToolMode.MOVE_START and not is_drag:
            if self.grid.set_start(r, c):
                self.reset_simulation()
                self.header_status = f"Start moved to ({r}, {c})."
        elif self.tool_mode == ToolMode.MOVE_GOAL and not is_drag:
            if self.grid.set_goal(r, c):
                self.reset_simulation()
                self.header_status = f"Goal moved to ({r}, {c})."

    def handle_events(self) -> bool:
        """Handle Pygame user input and system events. Returns False on quit."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

            # Buttons event dispatch
            btn_clicked = False
            for btn in self.buttons:
                if btn.handle_event(event):
                    btn_clicked = True
                    break

            if btn_clicked:
                continue

            # Keyboard Shortcuts
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.toggle_run_pause()
                elif event.key == pygame.K_s:
                    self.step_forward()
                elif event.key == pygame.K_r:
                    self.reset_simulation()
                elif event.key == pygame.K_c:
                    self.clear_obstacles()
                elif event.key == pygame.K_g:
                    self.generate_random_map()
                elif event.key == pygame.K_m:
                    self.run_comparison()
                elif event.key == pygame.K_1:
                    if self.algorithm_mode != AlgorithmMode.ASTAR:
                        self.toggle_algorithm()
                elif event.key == pygame.K_2:
                    if self.algorithm_mode != AlgorithmMode.DIJKSTRA:
                        self.toggle_algorithm()
                elif event.key == pygame.K_o:
                    self.set_tool_mode(ToolMode.DRAW_OBSTACLE)
                elif event.key == pygame.K_ESCAPE:
                    return False

            # Mouse Grid Interactions
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left Click
                    self.mouse_pressed = True
                    self.mouse_erase_mode = False
                    self.handle_mouse_grid_action(event.pos, is_drag=False)
                elif event.button == 3:  # Right Click (Erase)
                    self.mouse_pressed = True
                    self.mouse_erase_mode = True
                    self.handle_mouse_grid_action(event.pos, is_drag=False)
            elif event.type == pygame.MOUSEBUTTONUP:
                self.mouse_pressed = False
                self.mouse_erase_mode = False
            elif event.type == pygame.MOUSEMOTION:
                if self.mouse_pressed:
                    self.handle_mouse_grid_action(event.pos, is_drag=True)

        return True

    def update(self) -> None:
        """Update simulation states and animations per frame."""
        if self.sim_state == SimState.ANIMATING_SEARCH:
            self._advance_trace_events(self.steps_per_frame)
        elif self.sim_state == SimState.ANIMATING_ROBOT:
            self._update_robot_animation()

    def render(self) -> None:
        """Render all scene layers, components, and widgets."""
        self.screen.fill(COLOR_BG)

        # 1. Header with title and status
        is_running = self.sim_state in (SimState.ANIMATING_SEARCH, SimState.ANIMATING_ROBOT)
        self.renderer.draw_header(self.screen, self.algorithm_mode, self.header_status, is_running)

        # 2. Main Grid
        self.renderer.draw_grid_canvas(self.screen, self.grid, self.robot_interp_pos)

        # 3. Sidebar with Info, Metrics, Comparison, Legend
        self.renderer.draw_sidebar(
            self.screen, self.grid, self.latest_result, self.algorithm_mode, self.comparison_data
        )

        # 4. Control Panel Buttons
        for btn in self.buttons:
            btn.draw(self.screen, self.renderer.font_small)

        # 5. Footer Bar
        self.renderer.draw_footer(self.screen, self.footer_hint)

        pygame.display.flip()

    def run(self, max_frames: Optional[int] = None) -> None:
        """Main application lifecycle loop."""
        running = True
        frame_count = 0
        while running:
            running = self.handle_events()
            self.update()
            self.render()
            self.clock.tick(self.fps)

            frame_count += 1
            if max_frames is not None and frame_count >= max_frames:
                break

        pygame.quit()


def main() -> None:
    """CLI entry point for running the simulator."""
    parser = argparse.ArgumentParser(description="Robot Path Planning Simulator using A* Search Algorithm")
    parser.add_argument("--headless", action="store_true", help="Run without graphical display for automated tests/CI")
    parser.add_argument("--frames", type=int, default=None, help="Exit after N frames (useful for smoke tests)")
    args = parser.parse_args()

    app = PathPlanningApp(headless=args.headless)
    app.run(max_frames=args.frames)


if __name__ == "__main__":
    main()
