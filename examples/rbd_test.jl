using RigidBodyDynamics
using MeshCat
using MeshCatMechanisms
using DelimitedFiles
using Random 

ufile = "/home/gaussian/cmu_ri_phd/phd_misc/cito_ws/logs/bkp/shelf/tora_u.txt"
u = readdlm(ufile, ',')
# u = zeros(size(q)[1], size(q)[2])

package_path = "/home/gaussian/scratch/ros_kortex"
urdfpath = "/home/gaussian/cmu_ri_phd/phd_research/vanderbot/TORA.jl/robots/gen3_360.urdf"


mechanism = parse_urdf(urdfpath, remove_fixed_tree_joints=false);
frame_ee = default_frame(findbody(mechanism, "tool_frame"));
remove_fixed_tree_joints!(mechanism);


state = MechanismState(mechanism)

x0 = [-1.11792, 1.14091, -1.85759, -1.1654, -0.568693, 0.0120205, 1.06915] 
dx0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

set_configuration!(state, x0)
set_velocity!(state, dx0)

function control!(torques::AbstractVector, t, state::MechanismState)
  torques = u[Int(floor(t/2e-3))+1, :]
  # println(torques)
end

# t, q, v = simulate(state, 4.998, Δt = 2e-3);
# t, q, v = simulate(state, 0.998, Δt = 2e-3);
# t, q, v = simulate(state, 0.998, control!; Δt = 2e-3);
t, q, v = simulate(state, 4.998, control!; Δt = 2e-3);

q = mapreduce(permutedims, vcat, q)
v = mapreduce(permutedims, vcat, v)
a = zeros(size(q)[1], size(q)[2])

tqvau = hcat(t, q, v, a, u)

savefile = "/home/gaussian/cmu_ri_phd/phd_misc/cito_ws/logs/bkp/shelf/rbd_tqvau.txt"
writedlm(savefile, tqvau, ',')

