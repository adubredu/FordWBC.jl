module SubPose 

using CxxWrap

@wrapmodule(joinpath(@__DIR__, "ros/ws/devel/lib/libSubpub"))

function __init__()
    @initcxx
end

function get_object_poses(s)
    poses = get_poses(s)
    N = length(poses)
    object_poses = []
    for i=1:7:N-6
        pose = poses[i:i+6]
        push!(object_poses, pose)
    end
    return object_poses
end
end

# s = SubPose.Subpub() 
# poses = SubPose.get_poses(s)

# for i=1:1000000
#     poses = SubPose.get_poses(s)
#     sleep(1e-3)
# end