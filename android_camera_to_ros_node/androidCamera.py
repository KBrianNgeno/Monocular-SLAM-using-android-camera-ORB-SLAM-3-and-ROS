import cv2
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('video_publisher', anonymous=True)
frame_rate = rospy.Rate(30)  # Adjust frame rate as needed
bridge = CvBridge()
publisher = rospy.Publisher('/cam0/image_raw', Image, queue_size=10)

cap = cv2.VideoCapture('/dev/video0')# or ('http://192.168.0.101:4747/video') if using as IP camera

while(True):
    ret, frame = cap.read()
    
    if ret == True:
        # print("Receiving streaam...")
        video.write(frame)
        cv2.imshow('frame',frame)
        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
        publisher.publish(ros_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # print("Exiting.")
            break

cap.release()
cv2.destroyAllWindows()
