import numpy
import rospy  # module for ROS APIs
import tf  # library for transformations.


def get_current_position(frame2, frame1, transform_listener):
    time = rospy.get_rostime()
    transform_listener.waitForTransform(frame2, frame1, time, rospy.Duration(0.2))
    (trans, rot) = transform_listener.lookupTransform(frame2, frame1, time)
    return trans, rot


def translate_point_between_frames(point, frame1, frame2, transform_listener):
    time = rospy.get_rostime()
    transform_listener.waitForTransform(frame2, frame1, time, rospy.Duration(0.2))
    (trans, rot) = transform_listener.lookupTransform(frame2, frame1, time)
    t = tf.transformations.translation_matrix(trans)
    R = tf.transformations.quaternion_matrix(rot)

    frame2_T_frame1 = t.dot(R)
    new_point = frame2_T_frame1.dot(numpy.array([point["x"], point["y"], 0, 1]))

    return {"x": new_point[0], "y": new_point[1]}
