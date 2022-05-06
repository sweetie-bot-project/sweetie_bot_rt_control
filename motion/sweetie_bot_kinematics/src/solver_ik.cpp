#include "solver_ik.hpp"

namespace sweetie_bot {
namespace motion {

std::vector<SolverIKFactory::SolverIKCreatePtr> SolverIKFactory::solver_list;

bool SolverIKFactory::Register(SolverIKFactory::SolverIKCreatePtr create_method) {
	solver_list.push_back(create_method);
	return true;
}

std::unique_ptr<SolverIK> SolverIKFactory::GetSolver(const KDL::Chain& chain, sweetie_bot::logger::Logger& log) {
	std::unique_ptr<SolverIK> solver;
	for (auto createMethod : solver_list) {
		solver = createMethod(chain, log);
		if (solver != nullptr) break;
	}
	return solver;
}

}
}

