"""
A `Robot` represents a mechanism, its state, and other relevant objects.

# Relevant fields in `Robot`
- The arrays `q_lo`, `q_hi`, `v_lo`, `v_hi`, `τ_lo`, `τ_hi` store the lower and upper bounds of the joint positions, velocities, and torques, respectively.
- The number of generalized coordinates, generalized velocities, and actuated joints are stored in `n_q`, `n_v`, `n_τ`, respectively.
- The end-effector frame is stored in `frame_ee`.
"""
struct Robot{T,T_SC,n_q,n_v,n_τ}
    urdfpath::String
    mechanism::Mechanism{T}
    state::MechanismState{T}
    statecache::T_SC
    dynamicsresultcache::DynamicsResultCache{T}
    segmentedvectorcache::SegmentedVectorCache{JointID,Base.OneTo{JointID}}
    mvis::MechanismVisualizer

    q_lo::Vector{T}
    q_hi::Vector{T}
    v_lo::Vector{T}
    v_hi::Vector{T}
    τ_lo::Vector{T}
    τ_hi::Vector{T}

    n_q::Int64  # Number of generalized coordinates
    n_v::Int64  # Number of generalized velocities
    n_τ::Int64  # Number of actuated joints

    frame_ee::CartesianFrame3D  # End-effector frame

    @doc """
        Robot(urdfpath, mechanism, frame_ee, mvis)

    Create a new `Robot`.
    """
    function Robot(urdfpath::String,
                   mechanism::Mechanism,
                   frame_ee::CartesianFrame3D,
                   mvis::MechanismVisualizer)
        state = MechanismState(mechanism)
        statecache = StateCache(mechanism)
        dynamicsresultcache = DynamicsResultCache(mechanism)
        segmentedvectorcache = SegmentedVectorCache(RigidBodyDynamics.ranges(velocity(state)))

        q_lo = [lim for joint in joints(mechanism) for lim in map(x -> x.lower, position_bounds(joint))]
        q_hi = [lim for joint in joints(mechanism) for lim in map(x -> x.upper, position_bounds(joint))]
        v_lo = [lim for joint in joints(mechanism) for lim in map(x -> x.lower, velocity_bounds(joint))]
        v_hi = [lim for joint in joints(mechanism) for lim in map(x -> x.upper, velocity_bounds(joint))]
        τ_lo = [lim for joint in joints(mechanism) for lim in map(x -> x.lower,   effort_bounds(joint))]
        τ_hi = [lim for joint in joints(mechanism) for lim in map(x -> x.upper,   effort_bounds(joint))]

        n_q = num_positions(mechanism)
        n_v = num_velocities(mechanism)
        n_τ = num_velocities(mechanism)

        T_SC = typeof(statecache)

        new{Float64,T_SC,n_q,n_v,n_τ}(
            urdfpath,
            mechanism,
            state,
            statecache,
            dynamicsresultcache,
            segmentedvectorcache,
            mvis,
            q_lo,
            q_hi,
            v_lo,
            v_hi,
            τ_lo,
            τ_hi,
            n_q,
            n_v,
            n_τ,
            frame_ee
        )
    end
end

"""
    create_robot_franka(model, vis)

Create a new [Franka Emika](https://www.franka.de/research) robot.
"""
function create_robot_franka(model::String, vis::Visualizer)
    choices = ["panda_arm"]
    if model ∉ choices
        msg = """
        Argument `model` provided value \"$(model)\" is not valid.
        Valid options are: $(choices)"""
        error(msg)
    else
        commit_hash = artifact_commit_hash("franka_ros")
        package_path = joinpath(artifact"franka_ros", "franka_ros-$(commit_hash)")
        urdfpath = joinpath(@__DIR__, "..", "robots", "$(model).urdf")

        mechanism = parse_urdf(urdfpath, remove_fixed_tree_joints=false)
        frame_ee = default_frame(findbody(mechanism, "panda_hand_tcp"))
        remove_fixed_tree_joints!(mechanism)

        urdfvisuals = URDFVisuals(urdfpath, package_path=[package_path])
        mvis = MechanismVisualizer(mechanism, urdfvisuals, vis["robot"])
        # setelement!(mvis, frame_ee)  # Visualize a triad at the end-effector

        Robot(urdfpath, mechanism, frame_ee, mvis)
    end
end

"""
    create_robot_kinova_gen2(model, vis)

Create a new [Kinova Gen2](https://www.kinovarobotics.com/en/products/gen2-robot) robot.
"""
function create_robot_kinova_gen2(model::String, vis::Visualizer)
    choices = ["j2s6s200"]
    if model ∉ choices
        msg = """
        Argument `model` provided value \"$(model)\" is not valid.
        Valid options are: $(choices)"""
        error(msg)
    else
        commit_hash = artifact_commit_hash("kinova-ros")
        package_path = joinpath(artifact"kinova-ros", "kinova-ros-$(commit_hash)")
        urdfpath = joinpath(@__DIR__, "..", "robots", "$(model).urdf")

        mechanism = parse_urdf(urdfpath, remove_fixed_tree_joints=false)
        frame_ee = default_frame(findbody(mechanism, "$(model)_end_effector"))
        remove_fixed_tree_joints!(mechanism)

        urdfvisuals = URDFVisuals(urdfpath, package_path=[package_path])
        mvis = MechanismVisualizer(mechanism, urdfvisuals, vis["robot"])
        # setelement!(mvis, frame_ee)  # Visualize a triad at the end-effector

        Robot(urdfpath, mechanism, frame_ee, mvis)
    end
end

"""
    create_robot_kinova_gen3(model, vis)

Create a new [Kinova Gen3 lite](https://www.kinovarobotics.com/en/products/gen3-lite-robot) robot.
"""
function create_robot_kinova_gen3(model::String, vis::Visualizer)
    choices = ["gen3", "gen3_lite_gen3_lite_2f", "gen3_robotiq_2f_85", "gen3_robotiq_2f_140", "gen3_360", "gen3_360_pl"]
    if model ∉ choices
        msg = """
        Argument `model` provided value \"$(model)\" is not valid.
        Valid options are: $(choices)"""
        error(msg)
    else
        commit_hash = artifact_commit_hash("ros_kortex")
        package_path = joinpath(artifact"ros_kortex", "ros_kortex-$(commit_hash)")
        urdfpath = joinpath(@__DIR__, "..", "robots", "$(model).urdf")

        mechanism = parse_urdf(urdfpath, remove_fixed_tree_joints=false)
        frame_ee = default_frame(findbody(mechanism, "tool_frame"))
        remove_fixed_tree_joints!(mechanism)

        urdfvisuals = URDFVisuals(urdfpath, package_path=[package_path])
        mvis = MechanismVisualizer(mechanism, urdfvisuals, vis["robot"])
        # setelement!(mvis, frame_ee)  # Visualize a triad at the end-effector

        Robot(urdfpath, mechanism, frame_ee, mvis)
    end
end

"""
    create_robot_kuka(model, vis)

Create a new [KUKA LBR iiwa 7/14](https://www.kuka.com/en-gb/products/robotics-systems/industrial-robots/lbr-iiwa) robot.
"""
function create_robot_kuka(model::String, vis::Visualizer)
    choices = ["iiwa7", "iiwa14"]
    if model ∉ choices
        msg = """
        Argument `model` provided value \"$(model)\" is not valid.
        Valid options are: $(choices)"""
        error(msg)
    else
        commit_hash = artifact_commit_hash("iiwa_stack")
        package_path = joinpath(artifact"iiwa_stack", "iiwa_stack-$(commit_hash)")
        urdfpath = joinpath(@__DIR__, "..", "robots", "$(model).urdf")

        mechanism = parse_urdf(urdfpath, remove_fixed_tree_joints=false)
        frame_ee = default_frame(findbody(mechanism, "iiwa_link_ee"))
        remove_fixed_tree_joints!(mechanism)

        urdfvisuals = URDFVisuals(urdfpath, package_path=[package_path])
        mvis = MechanismVisualizer(mechanism, urdfvisuals, vis["robot"])
        # setelement!(mvis, frame_ee)  # Visualize a triad at the end-effector

        Robot(urdfpath, mechanism, frame_ee, mvis)
    end
end

"""
    create_robot_ur(ur_type, vis)

Create a new [Universal Robots](https://www.universal-robots.com/products/) robot.
"""
function create_robot_ur(ur_type::String, vis::Visualizer)
    choices = ["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"]
    if ur_type ∉ choices
        msg = """
        Argument `ur_type` provided value \"$(ur_type)\" is not valid.
        Valid options are: $(choices)"""
        error(msg)
    else
        commit_hash = artifact_commit_hash("Universal_Robots_ROS2_Description")
        package_path = joinpath(artifact"Universal_Robots_ROS2_Description", "Universal_Robots_ROS2_Description-$(commit_hash)")
        urdfpath = joinpath(@__DIR__, "..", "robots", "$(ur_type).urdf")

        mechanism = parse_urdf(urdfpath, remove_fixed_tree_joints=false)
        frame_ee = default_frame(findbody(mechanism, "tool0"))
        remove_fixed_tree_joints!(mechanism)

        urdfvisuals = URDFVisuals(urdfpath, package_path=[package_path])
        mvis = MechanismVisualizer(mechanism, urdfvisuals, vis["robot"])
        # setelement!(mvis, frame_ee)  # Visualize a triad at the end-effector

        Robot(urdfpath, mechanism, frame_ee, mvis)
    end
end

export
    Robot,
    create_robot_franka,
    create_robot_kinova_gen2,
    create_robot_kinova_gen3,
    create_robot_kuka,
    create_robot_ur
