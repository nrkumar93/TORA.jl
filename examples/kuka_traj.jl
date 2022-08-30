using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
Pkg.instantiate()

using Revise
using TORA

using LinearAlgebra; BLAS.set_num_threads(1)

# using KNITRO
using MeshCat
using RigidBodyDynamics


vis = Visualizer()

setprop!(vis["/Cameras/default/rotated/<object>"], "fov", 40)

open(vis)  # Show the viewer in a separate tab

robot = TORA.create_robot_kuka("iiwa14", vis)
problem = TORA.Problem(robot, 301, 1/150)

# Constrain initial and final joint velocities to zero
TORA.fix_joint_velocities!(problem, robot, 1, zeros(robot.n_v))
TORA.fix_joint_velocities!(problem, robot, problem.num_knots, zeros(robot.n_v))

# # Constrain the position of the end-effector
# TORA.constrain_ee_position!(problem,   1, [ 1.0,  0.0,  0.5])
# TORA.constrain_ee_position!(problem, 101, [ 0.0,  1.0,  0.5])
# TORA.constrain_ee_position!(problem, 201, [-1.0,  0.0,  0.5])
# TORA.constrain_ee_position!(problem, 301, [ 0.0, -1.0,  0.5])

let
    CubicTimeScaling(Tf::Number, t::Number) = 3(t / Tf)^2 - 2(t / Tf)^3
    QuinticTimeScaling(Tf::Number, t::Number) = 10(t / Tf)^3 - 15(t / Tf)^4 + 6(t / Tf)^5

    for k = 1:2:problem.num_knots
        θ = CubicTimeScaling(problem.num_knots - 1, k - 1) * 2π
        pos = [0.5, 0.2 * cos(θ), 0.8 + 0.2 * sin(θ)]
        # pos = [0.5, 0.3 * sin(θ) + 0.1 * sin(8 * θ), 0.8 + 0.3 * cos(θ) + 0.1 * cos(8 * θ)]
        TORA.constrain_ee_position!(problem, k, pos)
    end
end

TORA.show_problem_info(problem)

initial_q = [0, 0, 0, -π/2, 0, 0, 0]

zero!(robot.state)
set_configuration!(robot.state, initial_q)
set_configuration!(robot.mvis, configuration(robot.state))

initial_qs = repeat(initial_q, 1, problem.num_knots)
initial_vs = zeros(robot.n_v, problem.num_knots)
initial_τs = zeros(robot.n_τ, problem.num_knots)

initial_guess = [initial_qs; initial_vs; initial_τs]

# Flatten matrix and truncate torques of last knot
initial_guess = vec(initial_guess)[1:end - robot.n_τ];

use_inv_dyn = true
minimise_τ = false

# Choose which solver you want to use:
solve = TORA.solve_with_ipopt  # Uses Ipopt (https://github.com/coin-or/Ipopt)
# solve = TORA.solve_with_knitro  # Uses KNITRO (https://www.artelys.com/solvers/knitro/)

# Calling this will start the optimization.
cpu_time, x, solver_log = solve(problem, robot,
                                initial_guess=initial_guess,
                                use_inv_dyn=use_inv_dyn,
                                minimise_τ=minimise_τ)

TORA.play_trajectory(vis, problem, robot, x)

TORA.plot_results(problem, robot, x)

TORA.plot_log(solver_log)