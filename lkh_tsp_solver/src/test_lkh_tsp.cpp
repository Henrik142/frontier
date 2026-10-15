#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <lkh_tsp_solver/lkh_interface.h>

#ifndef LKH_DATA_DIR
#define LKH_DATA_DIR "data"
#endif

int main() {
  const std::string data_dir = LKH_DATA_DIR;

  // Example asymmetric cost matrix (zero diagonal). Index 0 is the current state.
  const std::vector<std::vector<double>> cost_mat = {
      {0.0, 2.5, 9.0, 10.0, 7.0},
      {1.5, 0.0, 4.0, 8.5, 3.0},
      {8.0, 3.5, 0.0, 2.0, 6.0},
      {9.5, 7.0, 1.0, 0.0, 5.5},
      {6.0, 2.0, 5.0, 4.5, 0.0}};
  const int dimension = static_cast<int>(cost_mat.size());

  // Parameter file for LKH
  {
    std::ofstream par_file(data_dir + "/single.par");
    par_file << "PROBLEM_FILE = " << data_dir << "/single.tsp\n"
             << "MOVE_TYPE = 5\n"
             << "PATCHING_C = 3\n"
             << "PATCHING_A = 2\n"
             << "RUNS = 10\n"
             << "OUTPUT_TOUR_FILE = " << data_dir << "/single.txt\n";
  }

  // Problem file in TSPLIB format
  {
    std::ofstream prob_file(data_dir + "/single.tsp");
    prob_file << "NAME : single\nTYPE : ATSP\nDIMENSION : " << dimension
              << "\nEDGE_WEIGHT_TYPE : EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\n"
                 "EDGE_WEIGHT_SECTION\n";

    // LKH needs integer costs
    const int scale = 100;
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = static_cast<int>(cost_mat[i][j] * scale);
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
    prob_file << "EOF";
  }

  solveTSPLKH((data_dir + "/single.par").c_str());

  std::ifstream res_file(data_dir + "/single.txt");
  if (!res_file) {
    std::cerr << "Could not open result file " << data_dir << "/single.txt" << std::endl;
    return 1;
  }

  std::string res;
  while (std::getline(res_file, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  std::vector<int> indices;
  while (std::getline(res_file, res)) {
    int id = std::stoi(res);
    if (id == 1)  // Ignore the current state
      continue;
    if (id == -1) break;
    indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
  }

  std::cout << "Optimal visiting order (frontier indices):";
  for (int idx : indices) std::cout << " " << idx;
  std::cout << std::endl;
  return 0;
}
