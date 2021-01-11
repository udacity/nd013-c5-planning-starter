/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/


#include "mpc_controller.h"
#include "ortools/linear_solver/linear_solver.h"


namespace operations_research {
    void SimpleLpProgram() {
        // Create the linear solver with the GLOP backend.
        MPSolver* solver = MPSolver::CreateSolver("GLOP");

        const double infinity = solver->infinity();
        // Create the variables x and y.
        MPVariable* const x = solver->MakeNumVar(0.0, infinity, "x");
        MPVariable* const y = solver->MakeNumVar(0.0, infinity, "y");

        LOG(INFO) << "Number of variables = " << solver->NumVariables();

        // x + 7 * y <= 17.5.
        MPConstraint* const c0 = solver->MakeRowConstraint(-infinity, 17.5, "c0");
        c0->SetCoefficient(x, 1);
        c0->SetCoefficient(y, 7);

        // x <= 3.5.
        MPConstraint* const c1 = solver->MakeRowConstraint(-infinity, 3.5, "c1");
        c1->SetCoefficient(x, 1);
        c1->SetCoefficient(y, 0);

        LOG(INFO) << "Number of constraints = " << solver->NumConstraints();

        // Maximize x + 10 * y.
        MPObjective* const objective = solver->MutableObjective();
        objective->SetCoefficient(x, 1);
        objective->SetCoefficient(y, 10);
        objective->SetMaximization();

        const MPSolver::ResultStatus result_status = solver->Solve();
        // Check that the problem has an optimal solution.
        if (result_status != MPSolver::OPTIMAL) {
            LOG(FATAL) << "The problem does not have an optimal solution!";
        }

        LOG(INFO) << "Solution:" << std::endl;
        LOG(INFO) << "Objective value = " << objective->Value();
        LOG(INFO) << "x = " << x->solution_value();
        LOG(INFO) << "y = " << y->solution_value();

        LOG(INFO) << "\nAdvanced usage:";
        LOG(INFO) << "Problem solved in " << solver->wall_time() << " milliseconds";
        LOG(INFO) << "Problem solved in " << solver->iterations() << " iterations";
        LOG(INFO) << "Problem solved in " << solver->nodes()
                  << " branch-and-bound nodes";
    }
}  // namespace operations_research