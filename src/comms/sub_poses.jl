module SubPose 

using CxxWrap

@wrapmodule(joinpath(@__DIR__, "ros/ws/devel/lib/libSubpub"))

function __init__()
    @initcxx
end
end

s = SubPose.Subpub() 
poses = SubPose.get_poses(s)

# for i=1:1000000
#     poses = SubPose.get_poses(s)
#     sleep(1e-3)
# end