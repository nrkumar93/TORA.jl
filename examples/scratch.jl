
using Revise

function checksz(v0::AbstractVecOrMat, vf::AbstractVecOrMat)
  if isempty(v0) && isempty(vf)
      # println("empty")
      println("full")
    else
      # println("full")
      println("empty")
  end
end

a = []
b = [1]

checksz(a, b)

