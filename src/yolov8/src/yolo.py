import os
import sys

import cv2
from ultralytics import YOLO

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Twist, Vector3
from cv_bridge import CvBridge

class Base:

    def __init__(self, model_name: str):

        rospy.init_node('yolo', anonymous=True)

        self.__rate = rospy.Rate(10)

        self.__pub_coordinate = rospy.Publisher('/vision/coordinate', Point, queue_size=10)
        self.__pub_distance = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/usb_cam/image_raw', Image, self.__callback)

        self.__init_component(model_name)

    def __init_component(self, model_name: str):

        model_path = os.path.join(os.environ['HOME'], 'model', model_name)
        if not os.path.exists(model_path):
            print('Model not found', file=sys.stderr)
            sys.exit(1)

        self.__mat = None
        self.__bridge = CvBridge()
        self.__model = YOLO(model_path)
    
    def __callback(self, img_msg):
        mat = self.__bridge.imgmsg_to_cv2(img_msg, 'passthrough')
        self.__mat = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)

    def __update(self):
        return self.__mat
    
    def run(self):

        while not rospy.is_shutdown():
            
            mat = self.__update()

            if mat is not None:

                results = self.__model(mat, conf=0.2, verbose=False)

                self.post_process(results)

            self.__rate.sleep()

    def post_process(self, input_frame):
        pass

    def publish_coordinate(self, x, y):

        mid = Point(x, y, -1)

        self.__pub_coordinate.publish(mid)

    def publish_distance(self, input):

        if(input > 40):
            linear = Vector3(1, 0, 0)
            angular = Vector3(0, 0, 0)
            data = Twist(linear, angular)

            self.__pub_distance.publish(data)
        else:
            linear = Vector3(0, 0, 0)
            angular = Vector3(0, 0, 0)
            data = Twist(linear, angular)

            self.__pub_distance.publish(data)



class Main(Base):

    def __init__(self, model_name:str):
        super().__init__(model_name)

    def post_process(self, results):
    
        annotated_frame = results[0].plot()

        for result in results:
            for i in result.boxes.xywh:
                x = i.numpy()[0]
                y = i.numpy()[1]
                w = i.numpy()[2]
                h = i.numpy()[3]
                pixelArea = float(w*h)
                distance = 10292 * pow(pixelArea, -0.502)

                self.publish_coordinate(x, y)
                self.publish_distance(distance)

        if len(result.boxes.xywh) == 0:
            self.publish_coordinate(-1, -1)
            
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        cv2.waitKey(30)

if __name__ == '__main__':
    try:
        Main('best.onnx').run()
    except rospy.ROSInterruptException:
        sys.exit(1)