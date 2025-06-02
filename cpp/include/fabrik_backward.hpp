#ifndef DELTA_FABRIK_BACKWARD_HPP
#define DELTA_FABRIK_BACKWARD_HPP

#include "math_utils.hpp"
#include "constants.hpp"
#include "fabrik_initialization.hpp"

namespace delta {

// Result of backward iteration
struct FabrikBackwardResult {
    FabrikChain updated_chain;             // Chain with updated joint positions
    Vector3 target_position;               // Target position used
    bool target_reachable;                 // Whether target was within reach
    double distance_to_base;               // Final distance from last joint to base
    int iterations_used;                   // Number of iterations performed
    std::vector<Vector3> iteration_history; // History of end-effector positions per iteration
    
    FabrikBackwardResult(const FabrikChain& chain, const Vector3& target, bool reachable,
                        double dist_to_base, int iterations)
        : updated_chain(chain), target_position(target), target_reachable(reachable)
        , distance_to_base(dist_to_base), iterations_used(iterations) {}
};

// Single iteration step result
struct BackwardStepResult {
    Vector3 new_joint_position;           // New position after constraint application
    bool constraint_applied;              // Whether constraint was applied
    double constraint_violation;          // Amount of constraint violation (0 = no violation)
    
    BackwardStepResult(const Vector3& pos, bool applied, double violation = 0.0)
        : new_joint_position(pos), constraint_applied(applied), constraint_violation(violation) {}
};

class FabrikBackward {
public:
    // Main interface: perform backward iteration from target to base
    static FabrikBackwardResult iterate_to_target(const FabrikChain& initial_chain,
                                                 const Vector3& target_position,
                                                 double tolerance = FABRIK_TOLERANCE,
                                                 int max_iterations = FABRIK_MAX_ITERATIONS);
    
    // Single backward iteration step
    static FabrikChain single_backward_iteration(const FabrikChain& chain,
                                                const Vector3& target_position);
    
    // Check if target is reachable
    static bool is_target_reachable(const FabrikChain& chain, const Vector3& target_position);
    
    // Calculate distance from chain end to base
    static double calculate_distance_to_base(const FabrikChain& chain);
    
    // Utility: Get end-effector position from chain
    static Vector3 get_end_effector_position(const FabrikChain& chain);
    
private:
    // Apply constraints to a joint during backward iteration
    static BackwardStepResult apply_joint_constraint(const Vector3& unconstrained_position,
                                                    const Vector3& previous_joint_position,
                                                    const Vector3& next_joint_position,
                                                    const FabrikJoint& joint,
                                                    double segment_length);
    
    // Move joint to maintain segment length while going toward target
    static Vector3 move_joint_toward_target(const Vector3& current_joint,
                                          const Vector3& target_joint,
                                          double required_distance);
    
    // Check convergence criteria
    static bool has_converged(const Vector3& current_end_effector,
                            const Vector3& target_position,
                            double tolerance);
};

} // namespace delta

#endif // DELTA_FABRIK_BACKWARD_HPP