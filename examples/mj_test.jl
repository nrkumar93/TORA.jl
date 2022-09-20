using MuJoCo
using DelimitedFiles

mj_activate("/home/gaussian/cmu_ri_phd/phd_misc/mujoco210/bin/mjkey.txt")
# alternatively: set the environment variable `MUJOCO_KEY_PATH`
#                before `using MuJoCo`.

# m = jlModel("/home/gaussian/cmu_ri_phd/phd_misc/cito_ws/src/cito/model/iiwa14.xml")
m = jlModel("/home/gaussian/cmu_ri_phd/phd_misc/cito_ws/src/cito/model/gen3_shelf.xml")
d = jlData(m)

ufile = "/home/gaussian/cmu_ri_phd/phd_misc/cito_ws/logs/bkp/shelf/tora_u.txt"
u = readdlm(ufile, ',')
# u = zeros(500, size(u)[2])
# u = zeros(size(u)[1], size(u)[2])

x0 = [-1.11792, 1.14091, -1.85759, -1.1654, -0.568693, 0.0120205, 1.06915] 
dx0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

t = zeros(size(u)[1], 1)
q = zeros(size(u)[1], size(u)[2])
v = zeros(size(u)[1], size(u)[2])
a = zeros(size(u)[1], size(u)[2])

q[1,:] = x0
v[1,:] = dx0


for i=1:size(u)[1]-1
  d.qpos[:] = q[i,:]
  # for j=1:size(u)[2]
  #   # wrap to 0,2π
  #   # d.qpos[j] = mod(mod(q[i,j], 2π)+2π, 2π) 
  #   # wrap to -π,π
  #   d.qpos[j] = mod(q[i,j], 2π)
  #   if d.qpos[j] > π
  #     d.qpos[j] = d.qpos[j] - 2π
  #   end
  # end
  d.qvel[:] = v[i,:]
  d.ctrl[:] = u[i,:]

  mj_step(m, d);

  t[i+1] = i*m.opt.timestep
  q[i+1, :] = d.qpos[:]
  v[i+1, :] = d.qvel[:]
  a[i+1, :] = d.qacc[:]
end

tqvau = hcat(t, q, v, a, u)

savex = "/home/gaussian/cmu_ri_phd/phd_misc/cito_ws/logs/bkp/shelf/mjjl_x.txt"
savedx = "/home/gaussian/cmu_ri_phd/phd_misc/cito_ws/logs/bkp/shelf/mjjl_dx.txt"

savefile = "/home/gaussian/cmu_ri_phd/phd_misc/cito_ws/logs/bkp/shelf/mjjl_tqvau.txt"
writedlm(savex, q, ',')
writedlm(savedx, v, ',')
writedlm(savefile, tqvau, ',')

