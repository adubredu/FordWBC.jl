import rospy 
from geometry_msgs.msg import PoseArray, Pose
from subpub.msg import SceneObjectArray, SceneObject

pub = rospy.Publisher("/state_estimation", SceneObjectArray, queue_size=1)
rospy.init_node("pub_node")
r = rospy.Rate(100)
i=0
while not rospy.is_shutdown():
    pose = SceneObject()

    posearray = SceneObjectArray() 
    i += 1
    posearray.scene_objects.append(pose)
    posearray.scene_objects.append(pose)
    pub.publish(posearray)
    print(i)
    r.sleep()
