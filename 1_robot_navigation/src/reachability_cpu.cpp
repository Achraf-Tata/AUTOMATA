#include "reachability_cpu.hpp"
#include <omp.h>
#include <fstream>
#include <cstring>
#include <algorithm>

namespace solver {

int solve_reachability_openmp(
    std::vector<float>&              value_function,
    std::vector<uint8_t>&            feedback_controller,
    const grid::RegionConfig&        arena_regions,
    const grid::AbstractionTables&   abstraction_tables,
    ProgressCallback                 progress_callback)
{
    // Precompute 2D observation label grid for fast cell lookups
    uint8_t region_label_grid[grid::GRID_CELLS_X][grid::GRID_CELLS_Y];
    for (int cell_x_index = 0; cell_x_index < grid::GRID_CELLS_X; ++cell_x_index) {
        double continuous_x = grid::COORDINATE_MIN_X + (cell_x_index + 0.5) * grid::CELL_WIDTH_DX;
        for (int cell_y_index = 0; cell_y_index < grid::GRID_CELLS_Y; ++cell_y_index) {
            double continuous_y = grid::COORDINATE_MIN_Y + (cell_y_index + 0.5) * grid::CELL_WIDTH_DY;
            region_label_grid[cell_x_index][cell_y_index] =
                static_cast<uint8_t>(grid::get_region_label(continuous_x, continuous_y, arena_regions));
        }
    }

    int reachability_iteration = 0;
    while (reachability_iteration < MAX_ITERATIONS) {
        ++reachability_iteration;
        std::vector<float> previous_value_function = value_function;
        int controllable_state_updates = 0;

        #pragma omp parallel for reduction(+:controllable_state_updates) schedule(static)
        for (int cell_x_index = 0; cell_x_index < grid::GRID_CELLS_X; ++cell_x_index) {
            for (int cell_y_index = 0; cell_y_index < grid::GRID_CELLS_Y; ++cell_y_index) {
                int observation_symbol = region_label_grid[cell_x_index][cell_y_index];

                for (int cell_theta_index = 0; cell_theta_index < grid::GRID_CELLS_THETA; ++cell_theta_index) {
                    int spatial_flat_index = grid::flat_index(cell_x_index, cell_y_index, cell_theta_index);

                    for (int current_automaton_state = 1; current_automaton_state <= automaton::NDFA; ++current_automaton_state) {
                        int current_product_state_index =
                            automaton::product_index(current_automaton_state, spatial_flat_index);

                        // Skip if already solved in earlier iteration
                        if (value_function[current_product_state_index] < 1e8f) {
                            continue;
                        }

                        int successor_automaton_state =
                            automaton::dfa_transition(current_automaton_state, observation_symbol);
                        int successor_product_state_index =
                            automaton::product_index(successor_automaton_state, spatial_flat_index);

                        // If successor automaton state already has valid control in this cell
                        if (feedback_controller[successor_product_state_index] != 0) {
                            value_function[current_product_state_index] = static_cast<float>(reachability_iteration);
                            ++controllable_state_updates;
                            continue;
                        }

                        uint8_t selected_control_action = 0;

                        for (int control_index = 0; control_index < 15; ++control_index) {
                            if (!abstraction_tables.get_valid1(cell_x_index, cell_theta_index, control_index) ||
                                !abstraction_tables.get_valid2(cell_y_index, cell_theta_index, control_index))
                            {
                                continue;
                            }

                            int successor_min_x = abstraction_tables.get_c1_min(cell_x_index, cell_theta_index, control_index);
                            int successor_max_x = abstraction_tables.get_c1_max(cell_x_index, cell_theta_index, control_index);
                            int successor_min_y = abstraction_tables.get_c2_min(cell_y_index, cell_theta_index, control_index);
                            int successor_max_y = abstraction_tables.get_c2_max(cell_y_index, cell_theta_index, control_index);
                            int successor_min_theta = abstraction_tables.get_c3_min(cell_theta_index, control_index);
                            int successor_max_theta = abstraction_tables.get_c3_max(cell_theta_index, control_index);

                            int successor_automaton_base_offset =
                                (successor_automaton_state - 1) * grid::TOTAL_SPATIAL_CELLS;
                            bool all_successors_controllable = true;

                            if (successor_min_theta <= successor_max_theta) {
                                for (int succ_x = successor_min_x; succ_x <= successor_max_x && all_successors_controllable; ++succ_x) {
                                    int x_stride_offset = successor_automaton_base_offset + succ_x * (grid::GRID_CELLS_Y * grid::GRID_CELLS_THETA);
                                    for (int succ_y = successor_min_y; succ_y <= successor_max_y && all_successors_controllable; ++succ_y) {
                                        int xy_stride_offset = x_stride_offset + succ_y * grid::GRID_CELLS_THETA;
                                        for (int succ_theta = successor_min_theta; succ_theta <= successor_max_theta; ++succ_theta) {
                                            if (previous_value_function[xy_stride_offset + succ_theta] > 1e7f) {
                                                all_successors_controllable = false;
                                                break;
                                            }
                                        }
                                    }
                                }
                            } else {
                                // Angular wrap-around handling
                                for (int succ_x = successor_min_x; succ_x <= successor_max_x && all_successors_controllable; ++succ_x) {
                                    int x_stride_offset = successor_automaton_base_offset + succ_x * (grid::GRID_CELLS_Y * grid::GRID_CELLS_THETA);
                                    for (int succ_y = successor_min_y; succ_y <= successor_max_y && all_successors_controllable; ++succ_y) {
                                        int xy_stride_offset = x_stride_offset + succ_y * grid::GRID_CELLS_THETA;
                                        for (int succ_theta = successor_min_theta; succ_theta < grid::GRID_CELLS_THETA; ++succ_theta) {
                                            if (previous_value_function[xy_stride_offset + succ_theta] > 1e7f) {
                                                all_successors_controllable = false;
                                                break;
                                            }
                                        }
                                        if (!all_successors_controllable) break;
                                        for (int succ_theta = 0; succ_theta <= successor_max_theta; ++succ_theta) {
                                            if (previous_value_function[xy_stride_offset + succ_theta] > 1e7f) {
                                                all_successors_controllable = false;
                                                break;
                                            }
                                        }
                                    }
                                }
                            }

                            if (all_successors_controllable) {
                                selected_control_action = static_cast<uint8_t>(control_index + 1);
                                break;
                            }
                        }

                        if (selected_control_action > 0) {
                            value_function[current_product_state_index] = static_cast<float>(reachability_iteration);
                            feedback_controller[successor_product_state_index] = selected_control_action;
                            ++controllable_state_updates;
                        }
                    }
                }
            }
        }

        if (progress_callback) {
            progress_callback(reachability_iteration, static_cast<float>(controllable_state_updates));
        }

        if (controllable_state_updates == 0) {
            break;
        }
    }

    return reachability_iteration;
}

bool save_controller_binary(const char*                 file_path,
                            const std::vector<float>&   value_function,
                            const std::vector<uint8_t>& feedback_controller)
{
    std::ofstream output_file(file_path, std::ios::binary);
    if (!output_file) return false;
    output_file.write("CTRL1.00", 8);
    uint32_t total_elements = static_cast<uint32_t>(automaton::NTOTAL);
    output_file.write(reinterpret_cast<const char*>(&total_elements), 4);
    output_file.write(reinterpret_cast<const char*>(value_function.data()), total_elements * sizeof(float));
    output_file.write(reinterpret_cast<const char*>(feedback_controller.data()), total_elements * sizeof(uint8_t));
    return output_file.good();
}

bool load_controller_binary(const char*            file_path,
                            std::vector<float>&    value_function,
                            std::vector<uint8_t>&  feedback_controller)
{
    std::ifstream input_file(file_path, std::ios::binary);
    if (!input_file) return false;
    char file_magic_header[8];
    input_file.read(file_magic_header, 8);
    if (std::memcmp(file_magic_header, "CTRL1.00", 8) != 0) return false;
    uint32_t total_elements = 0;
    input_file.read(reinterpret_cast<char*>(&total_elements), 4);
    if (total_elements != static_cast<uint32_t>(automaton::NTOTAL)) return false;
    value_function.resize(total_elements);
    feedback_controller.resize(total_elements);
    input_file.read(reinterpret_cast<char*>(value_function.data()), total_elements * sizeof(float));
    input_file.read(reinterpret_cast<char*>(feedback_controller.data()), total_elements * sizeof(uint8_t));
    return input_file.good();
}

} // namespace solver
