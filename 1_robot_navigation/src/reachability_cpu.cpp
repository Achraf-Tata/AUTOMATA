#include "reachability_cpu.hpp"
#include <omp.h>
#include <fstream>
#include <cstring>
#include <algorithm>

namespace solver {

int solve_reachability_openmp(
    std::vector<float>&              V,
    std::vector<uint8_t>&            ctrl,
    const grid::RegionConfig&        regions,
    const grid::AbstractionTables&   abs_tables,
    ProgressCallback                 callback)
{
    uint8_t lab_2d[grid::NX][grid::NY];
    for (int i1 = 0; i1 < grid::NX; ++i1) {
        double x = grid::X_MIN + (i1 + 0.5) * grid::DX;
        for (int i2 = 0; i2 < grid::NY; ++i2) {
            double y = grid::Y_MIN + (i2 + 0.5) * grid::DY;
            lab_2d[i1][i2] = static_cast<uint8_t>(grid::get_region_label(x, y, regions));
        }
    }

    int it = 0;
    while (it < MAX_ITERATIONS) {
        ++it;
        std::vector<float> Vp = V;
        int changes = 0;

        #pragma omp parallel for reduction(+:changes) schedule(static)
        for (int i1 = 0; i1 < grid::NX; ++i1) {
            for (int i2 = 0; i2 < grid::NY; ++i2) {
                int lab = lab_2d[i1][i2];
                for (int i3 = 0; i3 < grid::NTHETA; ++i3) {
                    int cell_flat = grid::flat_index(i1, i2, i3);

                    for (int psi = 1; psi <= automaton::NDFA; ++psi) {
                        int pidx_curr = automaton::product_index(psi, cell_flat);
                        if (V[pidx_curr] < 1e8f) {
                            continue;
                        }

                        int psi_succ = automaton::dfa_transition(psi, lab);
                        int pidx_succ = automaton::product_index(psi_succ, cell_flat);

                        if (ctrl[pidx_succ] != 0) {
                            V[pidx_curr] = static_cast<float>(it);
                            ++changes;
                            continue;
                        }

                        uint8_t found_sigma = 0;
                        for (int s = 0; s < 15; ++s) {
                            if (!abs_tables.get_valid1(i1, i3, s) || !abs_tables.get_valid2(i2, i3, s)) {
                                continue;
                            }

                            int r1_lo = abs_tables.get_c1_min(i1, i3, s);
                            int r1_hi = abs_tables.get_c1_max(i1, i3, s);
                            int r2_lo = abs_tables.get_c2_min(i2, i3, s);
                            int r2_hi = abs_tables.get_c2_max(i2, i3, s);
                            int r3_lo = abs_tables.get_c3_min(i3, s);
                            int r3_hi = abs_tables.get_c3_max(i3, s);

                            int succ_base = (psi_succ - 1) * grid::NCELLS;
                            bool all_ok = true;

                            if (r3_lo <= r3_hi) {
                                for (int k1 = r1_lo; k1 <= r1_hi && all_ok; ++k1) {
                                    int k1_offset = succ_base + k1 * (grid::NY * grid::NTHETA);
                                    for (int k2 = r2_lo; k2 <= r2_hi && all_ok; ++k2) {
                                        int k12_offset = k1_offset + k2 * grid::NTHETA;
                                        for (int k3 = r3_lo; k3 <= r3_hi; ++k3) {
                                            if (Vp[k12_offset + k3] > 1e7f) {
                                                all_ok = false;
                                                break;
                                            }
                                        }
                                    }
                                }
                            } else {
                                for (int k1 = r1_lo; k1 <= r1_hi && all_ok; ++k1) {
                                    int k1_offset = succ_base + k1 * (grid::NY * grid::NTHETA);
                                    for (int k2 = r2_lo; k2 <= r2_hi && all_ok; ++k2) {
                                        int k12_offset = k1_offset + k2 * grid::NTHETA;
                                        for (int k3 = r3_lo; k3 < grid::NTHETA; ++k3) {
                                            if (Vp[k12_offset + k3] > 1e7f) {
                                                all_ok = false;
                                                break;
                                            }
                                        }
                                        if (!all_ok) break;
                                        for (int k3 = 0; k3 <= r3_hi; ++k3) {
                                            if (Vp[k12_offset + k3] > 1e7f) {
                                                all_ok = false;
                                                break;
                                            }
                                        }
                                    }
                                }
                            }

                            if (all_ok) {
                                found_sigma = static_cast<uint8_t>(s + 1);
                                break;
                            }
                        }

                        if (found_sigma > 0) {
                            V[pidx_curr] = static_cast<float>(it);
                            ctrl[pidx_succ] = found_sigma;
                            ++changes;
                        }
                    }
                }
            }
        }

        if (callback) {
            callback(it, static_cast<float>(changes));
        }

        if (changes == 0) {
            break;
        }
    }

    return it;
}

bool save_controller_binary(const char*                 path,
                            const std::vector<float>&   V,
                            const std::vector<uint8_t>& ctrl)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write("CTRL1.00", 8);
    uint32_t n = static_cast<uint32_t>(automaton::NTOTAL);
    f.write(reinterpret_cast<const char*>(&n), 4);
    f.write(reinterpret_cast<const char*>(V.data()),    n * sizeof(float));
    f.write(reinterpret_cast<const char*>(ctrl.data()), n * sizeof(uint8_t));
    return f.good();
}

bool load_controller_binary(const char*            path,
                            std::vector<float>&    V,
                            std::vector<uint8_t>&  ctrl)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    char magic[8];
    f.read(magic, 8);
    if (std::memcmp(magic, "CTRL1.00", 8) != 0) return false;
    uint32_t n = 0;
    f.read(reinterpret_cast<char*>(&n), 4);
    if (n != static_cast<uint32_t>(automaton::NTOTAL)) return false;
    V.resize(n);
    ctrl.resize(n);
    f.read(reinterpret_cast<char*>(V.data()),    n * sizeof(float));
    f.read(reinterpret_cast<char*>(ctrl.data()), n * sizeof(uint8_t));
    return f.good();
}

} // namespace solver
