#pragma once
#include "app_state.hpp"

namespace robot_nav::arena {

// Renders the main 2D interactive workspace drawing canvas:
// - Transforms continuous coordinates [0, 10] m into screen pixels.
// - Draws the grid lines, regions R1, R2, R3 (goal), and R4 (obstacle).
// - Draws the animated robot triangle with orientation heading angle theta.
// - Draws the robot's past path trajectory trail.
// - Handles mouse dragging/resizing of regions and dragging the robot start pose.
void render_arena_card(AppState& app, float width, float height);

// Renders the color legend beneath the arena showing what each region represents:
// R1 / R2 (Waypoints), R3 (Goal), R4 (Obstacle), Robot Pose, and Trajectory Trail.
void render_legend_card(AppState& app, float width);

} // namespace robot_nav::arena
