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

robot = TORA.create_robot_kinova_gen3("gen3", vis)
# problem = TORA.Problem(robot, 681, 1/150)
problem = TORA.Problem(robot, 2041, 1/500)

# q0 = [-0.5, 1.6, 0, -0.56452, 0, 0.492, -1.6]
# qf = [-0.019991, 2.21439, 0.000866731, -0.608677, -0.00021391, -0.0411999, -1.6]

# q0 = [-0.019991, 2.21439, 0.000866731, -0.608677, -0.00021391, -0.0411999, -1.6]
# qf = [0.8, 2.21439, 0.000870275, -0.608678, -0.000186858, -0.0405323, -1.6]

# q0 = [0.8, 2.21439, 0.000870275, -0.608678, -0.000186858, -0.0405323, -1.6]
# qf = [0.542141, 1.4352, 0.0588043, -0.329963, -0.257948, -0.128281, -0.963067]

# TORA.fix_joint_positions!(problem, robot, 1, q0)
# TORA.fix_joint_positions!(problem, robot, problem.num_knots, qf)

using DataFrames
using CSV
disx_file = "/home/gaussian/cmu_ri_phd/phd_misc/cito_ws/logs/bkp/gen3_drag/real_slide/crct_ws/try3/insat_ur5l_x.csv"
df = DataFrame(CSV.File(disx_file,header=false))
disx = Matrix{Float64}(df)

let
    i=1
    for k = 1:Int((problem.num_knots-1)/size(disx)[1]):problem.num_knots-1
        TORA.fix_joint_positions!(problem, robot, k, disx[i,:])
        i=i+1
    end
end

# Constrain initial and final joint velocities to zero
TORA.fix_joint_velocities!(problem, robot, 1, zeros(robot.n_v))
TORA.fix_joint_velocities!(problem, robot, problem.num_knots, zeros(robot.n_v))


TORA.show_problem_info(problem)

zero!(robot.state)
set_configuration!(robot.state, q0)
set_configuration!(robot.mvis, configuration(robot.state))

initial_qs = repeat(q0, 1, problem.num_knots)
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

# TORA.plot_log(solver_log)

traj_file = "/home/gaussian/cmu_ri_phd/phd_misc/cito_ws/logs/bkp/gen3_drag/real_slide/crct_ws/try3/tora.npz";
TORA.export_trajectory(traj_file, problem, robot, x)