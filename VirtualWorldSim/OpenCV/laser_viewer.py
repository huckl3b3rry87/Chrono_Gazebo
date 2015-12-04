import rospy, cv2, cv_bridge, math
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Float32

class Laser:
  def __init__(self):
      self.bridge = cv_bridge.CvBridge()
      cv2.namedWindow("window", 1)
      self.laser_sub = rospy.Subscriber('base_scan', LaserScan, self.laser_callback)
      self.num = 0
      self.height = 320
      self.width = 480


  def laser_callback(self, msg):
      laserscans = msg.ranges
      min_angle = msg.angle_min
      max_angle = msg.angle_max
      increment = msg.angle_increment

      image = np.zeros((self.height, self.width, 3), np.uint8)
      color = tuple(reversed((255, 255, 255)))
      image[:] = color

      for x in range(0, len(laserscans)):
          dist = laserscans[x]
          if dist <= 25.0:
              xPos = int(-1*math.sin(min_angle+x*increment)*dist*12.5) + int(self.width/2.0)
              yPos = self.height - int(math.cos(min_angle+x*increment)*dist*12.5)
              cv2.circle(image, (xPos, yPos), 2, (0,0,0), -1)

      cv2.imshow("window", image)
      string = 'Captures/LaserSensor/2015_10_08/image'
      string += str(self.num)
      string += '.png'
      self.num += 1
      cv2.imwrite(string, image)
      cv2.waitKey(3)



rospy.init_node('laser')
laser = Laser()
rospy.spin()
