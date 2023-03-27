import rospy

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

## TODO: rewrite this in cpp

def callback_from_odometry(msg: Odometry):
    global ofset, publiser, last_theta
    outMessage = msg
    outMessage.pose.pose.position.x = outMessage.pose.pose.position.x - ofset.x
    outMessage.pose.pose.position.y = outMessage.pose.pose.position.y - ofset.y
    
    # TODO: Test if this is working
    x = outMessage.pose.pose.orientation.x
    y = outMessage.pose.pose.orientation.y
    z = outMessage.pose.pose.orientation.z
    w = outMessage.pose.pose.orientation.w
    
    th = euler_from_quaternion([x,y,z,w])[2]
    last_theta = th

    [x,y,z,w] = quaternion_from_euler(0,0, th - ofset.z)
    outMessage.pose.pose.orientation.x = x
    outMessage.pose.pose.orientation.y = y
    outMessage.pose.pose.orientation.z = z
    outMessage.pose.pose.orientation.w = w
    publiser.publish(outMessage)

def callback_from_new_ofset(msg: Point):
    global ofset, last_theta
    ofset.x = msg.x
    ofset.y = msg.y

    ofset.z = msg.z - last_theta # Save the last theta ofeset


def setup_listeners():
    rospy.Subscriber("/odometry/filtered", Odometry, callback_from_odometry)
    rospy.Subscriber("/odometry_ofset_point", Point, callback_from_new_ofset)



if __name__ == '__main__':
    rospy.init_node('ofeset_odometry')
    global ofset, publiser, last_theta
    last_theta = 0 # Save the last theta needed to reset the theta ofset
    ofset    = Point()
    publiser = rospy.Publisher('/odometry/ofset', Odometry, queue_size=10) 

    setup_listeners()
    rospy.spin()
