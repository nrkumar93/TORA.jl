using Pkg
# Pkg.activate(joinpath(@__DIR__, ".."))
# Pkg.instantiate()


################
### MJ stuff
################
using MuJoCo
using DelimitedFiles

mj_activate("/home/gaussian/cmu_ri_phd/phd_misc/mujoco210/bin/mjkey.txt")
m = jlModel("/home/gaussian/cmu_ri_phd/phd_misc/cito_ws/src/cito/model/gen3_shelf.xml")

function mjFwdSim(q0::AbstractVector, v0::AbstractVector, u::AbstractVecOrMat; wraptopi::Bool=false, wrapto2pi::Bool=false)

  d = jlData(m)

  t = zeros(size(u)[1], 1)
  q = zeros(size(u)[1], size(u)[2])
  v = zeros(size(u)[1], size(u)[2])
  a = zeros(size(u)[1], size(u)[2])
  
  q[1,:] = q0
  v[1,:] = v0

  for i=1:size(u)[1]-1
    d.qpos[:] = q[i,:]
    for j=1:size(u)[2]
      if wrapto2pi
        # wrap to 0,2π
        d.qpos[j] = mod(mod(q[i,j], 2π)+2π, 2π) 
      elseif wraptopi
        # wrap to -π,π
        d.qpos[j] = mod(q[i,j], 2π)
        if d.qpos[j] > π
          d.qpos[j] = d.qpos[j] - 2π
        end
      end
    end
    d.qvel[:] = v[i,:]
    d.ctrl[:] = u[i,:]
  
    mj_step(m, d);
  
    t[i+1] = i*m.opt.timestep
    q[i+1, :] = d.qpos[:]
    v[i+1, :] = d.qvel[:]
    a[i+1, :] = d.qacc[:]
  end
  return t, q, v, a
end

################
### TORA stuff
################

using Revise
using TORA
using LinearAlgebra; BLAS.set_num_threads(1)
# using KNITRO
using MeshCat
using RigidBodyDynamics

function runTORA(robot::TORA.Robot, duration::Float64, hz::Float64, sk::Int, 
                 disx::Matrix{Float64}, v0::AbstractVecOrMat, vf::AbstractVecOrMat)

  dt = 1/hz
  knots = Int(floor(hz*duration+1))
  problem = TORA.Problem(robot, knots, dt)
  
  q0 = disx[1,:]
  let
      i=1
      for k = 1:Int(ceil((problem.num_knots-1)/(size(disx)[1]/sk))):problem.num_knots-1
          TORA.fix_joint_positions!(problem, robot, k, disx[i,:])
          i=i+sk
      end
  end
  
  if !isempty(v0)
    TORA.fix_joint_velocities!(problem, robot, 1, v0)
  end

  if !isempty(vf)
    TORA.fix_joint_velocities!(problem, robot, problem.num_knots, vf)
  end

  zero!(robot.state)
  set_configuration!(robot.state, q0)

  # initual guess
  initial_qs = repeat(q0, 1, problem.num_knots)
  initial_vs = zeros(robot.n_v, problem.num_knots)
  initial_τs = zeros(robot.n_τ, problem.num_knots)
  initial_guess = [initial_qs; initial_vs; initial_τs]

  # Flatten matrix and truncate torques of last knot
  initial_guess = vec(initial_guess)[1:end - robot.n_τ];

  use_inv_dyn = true
  minimise_τ = true

  # Calling this will start the optimization.
  cpu_time, x, solver_log = TORA.solve_with_ipopt(problem, robot,
                                initial_guess=initial_guess, 
                                use_inv_dyn=use_inv_dyn, 
                                minimise_τ=minimise_τ)

  t, q, v, a, u = TORA.get_tqvau(problem, robot, x)

  return x, t, q, v, a, u, problem

end

function normAng(angle::Float64)
  # wrap to -π,π
  out = mod(angle+π, 2π)
  if out <= 0.0
    return out + π
  end
  return out-π
end

function shortestAngDist(from::Float64, to::Float64)
  return normAng(to-from)
end

function angDist(from::AbstractVector, to::AbstractVector)
  tot_ang = 0.0
  for i=1:size(from)[1]
    u = shortestAngDist(from[i], to[i])
    tot_ang = tot_ang + u*u
  end
  return tot_ang
end

function findNN(traj::AbstractVecOrMat, x::AbstractVector)
  dist_vec = zeros(size(traj)[1])
  for i=1:size(traj)[1]
    dist_vec[i] = angDist(traj[i, :], x)
  end

  return findmin(dist_vec)[2]
end

function rhp(robot::TORA.Robot, disx::Matrix{Float64}, T::Float64, hz::Float64, ncycl::Int, sk::Int)

  dof = size(disx)[2]
  traj = []
  first = true

  st = T/ncycl

  while !isempty(disx)
    wp = disx[1:ncycl, :]
    v0 = []
    vf = []
    if isempty(traj)
      v0 = zeros(dof)
    else
      v0 = traj[end, dof+2:2*dof+1]
    end


    # _, t, q, v, a, u, _ = runTORA(robot, st, hz, sk, wp, v0, vf)
    _, _, _, _, _, torau, _ = runTORA(robot, st, hz, sk, wp, v0, vf)

    # dropping
    drop_fac = .4
    drop_idx = floor(Int, size(torau)[1]*drop_fac)

    # t = t[1:drop_idx]
    # q = q[1:drop_idx, :]
    # v = v[1:drop_idx, :]
    # a = a[1:drop_idx, :]
    # u = u[1:drop_idx, :]
    
    torau = torau[1:drop_idx, :]

    t, q, v, a = mjFwdSim(disx[1,:], v0, torau)    

    tqvau = hcat(t, q, v, a, torau)
    if first
      traj=tqvau
      first=false
    else
      traj = vcat(traj, tqvau)
    end

    # println(q)

    dis_idx = findNN(disx, q[end,:])
    disx = disx[dis_idx:end, :]
    disx = vcat(transpose(q[end,:]), disx)

    if size(disx)[1] < ncycl
      break
    end
  end

  return traj
end

using DataFrames
using CSV
disx_file = "/home/gaussian/cmu_ri_phd/phd_misc/cito_ws/logs/bkp/shelf/insat_ur5l_x.csv"
df = DataFrame(CSV.File(disx_file,header=false))
disx = Matrix{Float64}(df)
v0 = zeros(size(disx)[2])
vf = []

vis = Visualizer()
robot = TORA.create_robot_kinova_gen3("gen3_360", vis)
T = 5.0
ncycl = 5
hz = 500.0
sk = 3

traj = rhp(robot, disx, T, hz, ncycl, sk)



# # per iteration stuff
# duration = 5.0
# hz = 500.0
# dt = 1/hz
# sk = 6

# set_configuration!(robot.mvis, configuration(robot.state))

# x, t, q, v, a, u, problem = runTORA(robot, duration, hz, sk, disx, v0, vf)

# TORA.play_trajectory(vis, problem, robot, x)

# TORA.plot_results(problem, robot, x)

# TORA.plot_log(solver_log)

# traj_file = "/home/gaussian/cmu_ri_phd/phd_misc/cito_ws/logs/bkp/shelf/tora.txt";
# TORA.export_trajectory(traj_file, problem, robot, x)
# TORA.export_trajectory_as_txt(traj_file, problem, robot, x)
