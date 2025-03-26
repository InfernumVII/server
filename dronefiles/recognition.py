# Импорт библиотек


import rospy
from sensor_msgs.msg import Image, CameraInfo, Range
from cv_bridge import CvBridge
from clover import srv

import os

from geometry_msgs.msg import Quaternion, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

import tf
import tf.transformations as t

import numpy as np
import math
import cv2

import pprint
from threading import Thread, Event
import threading
from std_msgs.msg import Int32

def long_callback(fn):
    """
    Decorator fixing a rospy issue for long-running topic callbacks, primarily
    for image processing.

    See: https://github.com/ros/ros_comm/issues/1901.

    Usage example:

    @long_callback
    def image_callback(msg):
        # perform image processing
        # ...

    rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)
    """
    e = Event()

    def thread():
        
        while not rospy.is_shutdown():
            #print(thread.self)
            e.wait()
            e.clear()
            fn(thread.self, thread.current_msg)

    thread.current_msg = None
    thread.self = None
    Thread(target=thread, daemon=True).start()

    def wrapper(self, msg):
        thread.self = self
        thread.current_msg = msg
        e.set()

    return wrapper


class ObjectDetection:
    def __init__(self, objects, transform_method='pnp', init=True, dev='virtual', cnt_method='default'):
        self.STOP_ALL = False
        self.cnt_pub = True
        self.mask_pub = False
        self.rectify_anytime = True
        self.max_cnt = False
        self.mag_radius = 0.5


        self.lastRange = None
        if dev == 'from_virtual_to_clover':
            os.environ['ROS_MASTER_URI'] = 'http://192.168.50.90:11311'
        
        if init:
            rospy.init_node('a', disable_signals=True)

        self.init_max_min_map()
        #self.max_x_map, self.max_y_map, self.min_x_map, self.min_y_map = 2, 2, -2, -2

        self.objects = objects
        self.publishers = {}
        for name in self.objects.keys():
            self.publishers[name] = rospy.Publisher(f'~{name}', Image, queue_size=1)
        
        self.masks = {}
        for name in self.objects.keys():
            self.masks[name] = None


        self.img_from_topic = None
        self.img_for_visualisation = None

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        

        self.blue_pub = rospy.Publisher("~blue_viz", MarkerArray, queue_size=1)
        self.contours_publisher = rospy.Publisher('~cnts', Image, queue_size=1)

        #Словарь с  усредненными найденными точками {"index": [кол-во точек, усредненная точка]
        self.Points = {}
        self.lastIndex = 0

        self.transform_method = transform_method

        self.cnt_method = cnt_method
        if cnt_method == 'neuro':
            from ultralytics import YOLO
            self.model = YOLO("best.pt")
        # Условное получение информации о камере на основе среды разработки
        if dev == 'virtual':
            self.camera_info = rospy.wait_for_message('main_camera/camera_info', CameraInfo)
            self.distortion = np.zeros(5, dtype="float64").flatten()
            #self.camera_matrix = np.array([[ 332.47884746146343, 0., 320.0], [0., 333.1761847948052, 240.0], [0., 0., 1.]], dtype="float64")
            self.camera_matrix = np.float64(self.camera_info.K).reshape(3, 3)
        elif dev == 'clover' or dev == 'from_virtual_to_clover':
            self.camera_info = rospy.wait_for_message('main_camera/camera_info', CameraInfo)
            self.camera_matrix = np.float64(self.camera_info.K).reshape(3, 3)
            self.distortion = np.float64(self.camera_info.D).flatten()
        # Установите смещение для метода PNP
        self.tolerance_of_recognise_inPixels = 100
        if transform_method == 'pnp':
            self.OFFSET = [61, 35]
            self.range = None
            self.range_sub = rospy.Subscriber('rangefinder/range', Range, self.range_callback, queue_size=1)
        elif transform_method == 'rectify_point':
            self.range_sub = rospy.Subscriber('rangefinder/range', Range, self.range_callback, queue_size=1)
            self.tolerance_of_recognise_inPixels = 100
            self.range = None
            self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        elif transform_method == 'privazka':
            self.range_sub = rospy.Subscriber('rangefinder/range', Range, self.range_callback, queue_size=1)
            self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
            self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
            self.range = None
        
        self.pub = rospy.Publisher('~neuro', Int32, queue_size=1)
        self.neuro = 0
        # Подписка на тему изображения камеры с троттлингом
        self.image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, self.image_callback, queue_size=1)
        #self.image_sub.unregister()
        print(self.max_x_map, self.max_y_map, self.min_x_map, self.min_y_map)
        # Запускаем отдельный поток для визуализации изображений был выключен в 4 день, так как код начали запускать на дроне
        #if dev != 'clover':
            #thread = threading.Thread(target=self.show, daemon=True)
            #thread.start()
    def show(self):
        while not rospy.is_shutdown():
            #print(self.__dict__)
            if self.img_from_topic is not None:
                cv2.imshow('Image', self.img_from_topic)
                cv2.imshow('Cnt', self.img_for_visualisation)
                for name, mask in self.masks.items():
                    cv2.imshow(name, mask)
                cv2.waitKey(1)
    # Инициализируем объект GetMinMaxXYMap и получаем данные карты min-max
    def init_max_min_map(self):
        self.max_x_map, self.max_y_map, self.min_x_map, self.min_y_map =  3.6, 5.4, -0.2, -0.2
    
    def default_cnt(self, mask):
        conturs, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #conturs of all objects
        #Если размер контуров больше 20, то мы их оставляем
        conturs = list(filter(lambda x: cv2.contourArea(x) > 50, conturs))
        return conturs
    
    def neuro_cnt(self, conf=0.5):
        frame = self.img_from_topic
        conturs = {}
        # Take each frame
        #_, frame = cap.read()
        #print(frame.shape)
        results = self.model.predict(source=frame, conf=conf, device=0, imgsz=320)
        #name = results[0].names[int(box.cls)]
        for result in results:
            #print(result.obb)
            for box in result.obb:
                name = results[0].names[int(box.cls)]
                cnt = np.array(box.xyxyxyxy.cpu(), dtype=np.int32).squeeze()
                if name not in conturs:
                    conturs[name] = []
                conturs[name].append(cnt)
        return conturs

    def resolve_method(self, cnt, name):
        obj = self.objects[name] # Получаем данные объекта из словаря
        xy = self.get_center_of_mass(cnt) # Вычисляем центр масс контура
        #Отрисовка контуров для визуализации
        cv2.drawContours(self.img_for_visualisation, [cnt], 0, obj[3], 2)
        cv2.putText(self.img_for_visualisation,"Fire", (int(xy[0]),int(xy[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, obj[3], 1)
        # Выполняем преобразование на основе self.transform_method
        if self.transform_method == 'pnp':
            self.pnp(cnt, obj, name)
        elif self.transform_method == 'rectify_point':
            self.rectify(cnt, name)
        elif self.transform_method == 'privazka':
            self.priv(cnt, name)

    #@long_callback
    def image_callback(self, msg):
        self.publish_markers_blue()
        self.img_from_topic = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.pub.publish(self.neuro)
        if self.STOP_ALL == True:
            return
        # Конвертировать сообщение изображения ROS в изображение OpenCV
        self.img_for_visualisation = self.img_from_topic.copy()
        # Обработка контуров на основе self.cnt_method
        if self.cnt_method == 'default':
            for name, obj in self.objects.items():
                if obj[7] == 1:
                    mask = self.image_mask(name=name, hsv_range=obj[0], bitwise=obj[1], hsv_range_list=obj[2])

                    conturs = self.default_cnt(mask)
                    if len(conturs) != 0:
                        # Выбираем самый большой контур, если max_cnt имеет значение True
                        if self.max_cnt == True:
                            conturs = [max(conturs, key=cv2.contourArea)]
                    for cnt in conturs:
                        #print(cnt, type(cnt), cnt[0], type(cnt[0]))
                        self.resolve_method(cnt, name)
        # Обнаружение контуров с помощью нейронной сети
        elif self.cnt_method == 'neuro':
            # Обнаружение контуров с помощью нейронной сети
            conturs = self.neuro_cnt(0.5)
            # Обработка контуров с выхода нейронной сети
            for name, cnt in conturs.items():
                cnt = np.array(cnt)
                self.resolve_method(cnt, name)
        if self.cnt_pub:    
            self.contours_publisher.publish(self.bridge.cv2_to_imgmsg(self.img_for_visualisation, 'bgr8'))
        
        

    # Привязка к цветовому маркеру  
    def priv(self, cnt, name):
        xy = self.get_center_of_mass(cnt)

        if xy is None:
            return
        
        xc_mark, yc_mark = xy[0], xy[1]

        use_rectify = True
        if use_rectify:
            poseOfMark = self.rectify(cnt, name)
        else:
            rang = self.range
            #center of image
            xc_frame = 160
            yc_frame = 120

            xDist, yDist = xc_mark - xc_frame, yc_frame - yc_mark
            mxDist, myDist = xDist * 0.007 * rang, yDist * 0.007 * rang
            mDist = (mxDist, -myDist)
            #print(self.mDist)
            poseOfMark = self.transform_xyz_yaw(mxDist, -myDist, rang, (0,0,0), "main_camera_optical", "aruco_map", self.listener)
        k1 = round(poseOfMark[0], 2)
        k2 = round(poseOfMark[1], 2)

        if poseOfMark[0] > self.max_x_map or poseOfMark[0] < self.min_x_map or poseOfMark[0] > self.max_y_map or poseOfMark[0] < self.min_y_map:
            return
        #print(k1, k2)
        #print(k1, k2, name)
        fl = True
        
        for n,v in self.Points.items():
            #print(v[1][0], v[1][1], abs(v[1][0] - k1), abs(v[1][1] - k2), name)
            if abs(v[1][0] - poseOfMark[0]) < 0.5 and abs(v[1][1] - poseOfMark[1]) < 0.5:
                fl = False
        
        #print(fl)
        
        
        if fl == True:
            x_ar, y_ar, z_ar, _yaw = self.transform_xyz_yaw(0, 0, 0, (0,0,0), "main_camera_optical", "aruco_map", self.listener)
            if abs(x_ar - k1) < 0.05 and abs(y_ar - k2) < 0.05:
                self.magic((x_ar, y_ar, 0), (0,0,0), name)
                #sprint(1)
                return
            self.navigate(x=k1, y=k2, z=1, speed=1, yaw=float('nan'), frame_id='aruco_map')
            #rospy.sleep(0.1)


    
    def img_xy_to_point(self, xy):
        xy = cv2.undistortPoints(xy, self.camera_matrix, self.distortion, P=self.camera_matrix)[0][0]

        # Shift points to center
        xy -= self.camera_info.width // 2, self.camera_info.height // 2

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]

        return xy[0] * self.range / fx, xy[1] * self.range / fy, self.range
    
    def rectify(self, cnt, name):
        xy = self.get_center_of_mass(cnt)
        if xy is None:
            return
        if self.transform_method == 'rectify_point':
            if abs(xy[0] - 160) > self.tolerance_of_recognise_inPixels or abs(120 - xy[1]) > self.tolerance_of_recognise_inPixels:
                return
        
        rect = cv2.minAreaRect(cnt)  # Получаем минимальный ограничивающий прямоугольник
        box = cv2.boxPoints(rect)    # Получаем 4 точки прямоугольника
        box = np.int0(box)           # Округляем координаты точек

        # Вычисляем площадь прямоугольника (не контура!)
        area_px = cv2.contourArea(box)

        # calculate and publish the position of the circle in 3D space
        if self.range is not None: 
            if self.rectify_anytime == False:
                if self.lastRange is None:
                    self.lastRange = self.range
                else:
                    if abs(self.lastRange - self.range) < 0.2:
                        self.lastRange = self.range
                        return
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            pixel_size_x = self.range / fx
            pixel_size_y = self.range / fy
            area_m2 = area_px * pixel_size_x * pixel_size_y

            #print(f"Area of {name}: {area_m2:.6f} m²")

            cx_cam1, cy_cam1, cz_cam1 = self.img_xy_to_point(xy)
            #print(xy3d)
            cx_map, cy_map, cz_map, rvec = self.transform_xyz_yaw(cx_cam1, cy_cam1, cz_cam1, (0,0,0), "main_camera_optical", "aruco_map", self.listener)
            #print(cx_map, cy_map, cz_map)

            if self.transform_method == 'privazka':
                return cx_map, cy_map
            #telemetry = self.get_telemetry(frame_id='aruco_map')
            #math.sqrt( (telemetry.x - cx_map) ** 2 + (telemetry.y - cy_map) ** 2) < 0.6

            #x_ar, y_ar, z_ar, _yaw = self.transform_xyz_yaw(0, 0, 0, (0,0,0), "main_camera_optical", "aruco_map", self.listener)
            #print(x_ar, y_ar)

            #print(cx_map, cy_map, x_ar, y_ar, name)
            #if math.sqrt( (x_ar - cx_map) ** 2 + (y_ar - cy_map) ** 2) < 0.2:
            self.magic((cx_map, cy_map, 0), (0,0,0), name, area_m2)

    def image_mask(self, name, hsv_range, bitwise=False, hsv_range_list=None):
        img_hsv = cv2.cvtColor(self.img_from_topic, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img_hsv, hsv_range[0], hsv_range[1])

        if bitwise:
            for hsv in hsv_range_list:
                #print(hsv_range_list)
                mask2 = cv2.inRange(img_hsv, hsv[0], hsv[1])
                mask = cv2.bitwise_or(mask, mask2)
        
        self.masks[name] = mask
        '''
        if self.publishers[name].get_num_connections() > 0:
            if self.mask_pub:
                self.publishers[name].publish(self.bridge.cv2_to_imgmsg(mask, 'mono8'))
            else:
                pass
        '''
        
            

        return mask
    
    def pnp(self, cnt, obj, name):
        approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
        MARKER_SIDE1_SIZE = obj[4] # in m
        MARKER_SIDE2_SIZE = obj[4] # in m

        #Реальные размеры цветной метки
        self.objectPoint = np.array([(-MARKER_SIDE1_SIZE / 2, -MARKER_SIDE2_SIZE / 2, 0), (MARKER_SIDE1_SIZE / 2, -MARKER_SIDE2_SIZE / 2, 0), 
                                                        (MARKER_SIDE1_SIZE / 2, MARKER_SIDE2_SIZE / 2, 0), (-MARKER_SIDE1_SIZE / 2, MARKER_SIDE2_SIZE / 2, 0)])
        self.objectPoint1 = np.array([(-MARKER_SIDE1_SIZE / 2, 0, -MARKER_SIDE2_SIZE / 2), (MARKER_SIDE1_SIZE / 2, 0, -MARKER_SIDE2_SIZE / 2), 
                                                        (MARKER_SIDE1_SIZE / 2, 0, MARKER_SIDE2_SIZE / 2), (-MARKER_SIDE1_SIZE / 2, 0, MARKER_SIDE2_SIZE / 2)])
        

        #Поиск минимального ограничивающего прямоугольника и его площади
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        #approx = cv2.approxPolyDP(box, 0.04 * cv2.arcLength(box, True), True)
        box_area = cv2.contourArea(box)  + 1e-7
        c_area = cv2.contourArea(cnt)  + 1e-7
        #print(abs(1 - rect[1][0] / (rect[1][1] + 1e-7)))
        #print(abs(c_area / box_area))
        #Проверка, является ли аппроксимированная фигура квазиквадратным четырехугольником 0.15
        #print(approx)
        #and abs(1 - rect[1][0] / (rect[1][1] + 1e-7)) < 0.25 and abs(c_area / box_area) > 0.85)
        if (len(approx) == 4 and abs(1 - rect[1][0] / (rect[1][1] + 1e-7)) < 0.25 and abs(c_area / box_area) > 0.85):
            points_img = np.array([np.array(p[0]) for p in approx]) #Извлечение угловых точек

            '''
            if name == 'blue':
                
                MARKER_SIDE1_SIZE = 0.29 # in m
                MARKER_SIDE2_SIZE = 0.29 # in m

                #Реальные размеры цветной метки
                self.objectPoint = np.array([(-MARKER_SIDE1_SIZE / 2, -MARKER_SIDE2_SIZE / 2, 0), (MARKER_SIDE1_SIZE / 2, -MARKER_SIDE2_SIZE / 2, 0), 
                                                                (MARKER_SIDE1_SIZE / 2, MARKER_SIDE2_SIZE / 2, 0), (-MARKER_SIDE1_SIZE / 2, MARKER_SIDE2_SIZE / 2, 0)])
                self.objectPoint1 = np.array([(-MARKER_SIDE1_SIZE / 2, 0, -MARKER_SIDE2_SIZE / 2), (MARKER_SIDE1_SIZE / 2, 0, -MARKER_SIDE2_SIZE / 2), 
                                                                (MARKER_SIDE1_SIZE / 2, 0, MARKER_SIDE2_SIZE / 2), (-MARKER_SIDE1_SIZE / 2, 0, MARKER_SIDE2_SIZE / 2)])
                '''


            '''
            leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
            rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
            topmost = tuple(cnt[cnt[:,:,1].argmin()][0])
            bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])
            '''
            

            #cv2.drawContours(self.img_for_visualisation, [np.array(approx)], 0, (0, 255, 255), 2)
            #cv2.drawContours(self.img_for_visualisation, [approx], 0, (255, 0, 255), 6)
            #points_img = np.array([np.array(p[0]) for p in [box]]) #Извлечение угловых точек
            #print(1)
        else:
            #print(name)
            return
            points_img = box


            #Проверка положения углов в пределах границ изображения
        image_shape=(240, 320, 3)
        minx1 = points_img[:, 0].min() > self.OFFSET[0]
        miny1 = points_img[:, 1].min() > self.OFFSET[1]
        minx2 = image_shape[1] - points_img[:, 0].max() > self.OFFSET[0]
        miny2 = image_shape[0] - points_img[:, 1].max() > self.OFFSET[1]


        if minx1 and minx2 and miny1 and miny2:
            #(извлечение координат объекта в системе координат камеры)
            retval, rvec, tvec = cv2.solvePnP(np.array(self.objectPoint, dtype="float32"), np.array(points_img, dtype="float32"), self.camera_matrix, self.distortion)

            cx_cam1=tvec[0][0]
            cy_cam1=tvec[1][0]
            cz_cam1=tvec[2][0]

            #Преобразование координат объекта в системе координат камеры в систему координат аруко карты
            cx_map, cy_map, cz_map, _ = self.transform_xyz_yaw(cx_cam1, cy_cam1, cz_cam1, rvec, "main_camera_optical", "aruco_map", self.listener)
            x_ar, y_ar, z_ar, _yaw = self.transform_xyz_yaw(0, 0, 0, (0,0,0), "main_camera_optical", "aruco_map", self.listener)
            #print(x_ar, y_ar)
            #print(z_ar)
            #print(cx_map, cy_map)
            #print(self.Points)

            if math.sqrt( (x_ar - cx_map) ** 2 + (y_ar - cy_map) ** 2) < 1.5:
                self.magic((cx_map, cy_map, cz_map), (0,0,0), name)

     #Функция усредняет координаты в словаре с определенным расстоянием
    def magic(self, point, rot, name, aream2=None, radius=0.5):
        radius = self.mag_radius
        if point[0] > self.max_x_map or point[0] < self.min_x_map or point[1] > self.max_y_map or point[1] < self.min_y_map:
            return
        
        # Если площадь не передана, устанавливаем значение по умолчанию (например, 0)
        if aream2 is None:
            aream2 = 0.0
        
        if len(self.Points) == 0:
            # Добавляем новую точку с площадью
            self.Points[self.lastIndex] = [1, point, rot, name, aream2]
        else:
            distances1 = []
            for index, obj in self.Points.items():
                pnts = obj[1]
                kol = obj[0]

                # Находим дистанцию между всеми точками
                distance = math.sqrt((pnts[0] - point[0]) ** 2 + (pnts[1] - point[1]) ** 2)
                distances1.append((distance, index))
            
            # Берём минимальную дистанцию
            miDST = min(distances1, key=lambda x: x[0])

            if miDST[0] <= radius:
                # Текущий список с точкой, который ближе всего к point
                check = self.Points[miDST[1]]
                pnts = check[1]
                kol = check[0]
                rot1 = check[2]
                area1 = check[4] if len(check) > 4 else 0.0  # Проверяем, есть ли площадь в данных

                rotx = rot1[0]
                roty = rot1[1]
                rotz = rot1[2]
                x = pnts[0]
                y = pnts[1]
                z = pnts[2]

                # Усредняем координаты, поворот и площадь
                self.Points[miDST[1]][1] = (
                    (x * kol + point[0]) / (kol + 1),
                    (y * kol + point[1]) / (kol + 1),
                    (z * kol + point[2]) / (kol + 1)
                )
                self.Points[miDST[1]][2] = (
                    (rotx * kol + rot[0]) / (kol + 1),
                    (roty * kol + rot[1]) / (kol + 1),
                    (rotz * kol + rot[2]) / (kol + 1)
                )
                # Усредняем площадь, если она передана
                if aream2 is not None:
                    self.Points[miDST[1]][4] = (area1 * kol + aream2) / (kol + 1)
                self.Points[miDST[1]][0] += 1
            else:
                # Если point находится на расстоянии больше radius от всех точек, добавляем новую точку с площадью
                self.Points[self.lastIndex + 1] = [1, point, rot, name, aream2]
                self.lastIndex += 1

    def get_center_of_mass(self, mask) -> tuple:
        M = cv2.moments(mask)
        if M['m00'] == 0:
            return None
        return M['m10'] // M['m00'], M['m01'] // M['m00']
    
    
    def transform_xyz_yaw(self, x, y, z, yaw, framefrom, frameto, listener):
        p = PoseStamped()
        p.header.frame_id = framefrom
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation = self.orientation_from_euler(yaw[0], yaw[1], yaw[2])
        #print(p.pose.orientation)
        pose_local = listener.transformPose(frameto, p)
        target_x = pose_local.pose.position.x
        target_y = pose_local.pose.position.y
        target_z = pose_local.pose.position.z
        target_yaw = self.euler_from_orientation(pose_local.pose.orientation)
        return target_x, target_y, target_z, target_yaw
    
    def orientation_from_quaternion(self, q):
        return Quaternion(*q)


    def orientation_from_euler(self, roll, pitch, yaw):
        q = t.quaternion_from_euler(roll, pitch, yaw)
        return self.orientation_from_quaternion(q)


    def quaternion_from_orientation(self, o):
        return o.x, o.y, o.z, o.w


    def euler_from_orientation(self, o):
        q = self.quaternion_from_orientation(o)
        return t.euler_from_quaternion(q)
    
    def range_callback(self, msg):
        self.range = msg.range
    
    def publish_markers_blue(self):
        result = []
        iddd = 0

        for fs in self.Points.values():
            #print('fs', fs)
            m = fs[1]
            #z = fs[3]
            rot = fs[2]

            marker = Marker()
            marker.header.frame_id = "aruco_map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "color_markers"
            marker.id = iddd
            marker.type =  Marker.CUBE
            marker.action = Marker.ADD

            # Позиция и ориентация
            marker.pose.position.x = m[0]
            marker.pose.position.y = m[1]
            #marker.pose.position.z = m[2]
            marker.pose.position.z = 0
            marker.pose.orientation = self.orientation_from_euler(rot[0], rot[1], rot[2])

            #marker.pose.orientation.x = 0.0
            #marker.pose.orientation.y = 0.0
            #marker.pose.orientation.z = 0.0
            #marker.pose.orientation.w = 1.0

            # Масштаб
            #print(self.objects[fs[3]][6])
            if self.objects[fs[3]][6] == 'korob':
                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = m[2]
                marker.pose.position.z = m[2]/2
            elif self.objects[fs[3]][6] == 'rab':
                #print(1)
                marker.type =  Marker.CYLINDER
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.01
            else:
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.001


            # Цвет
            marker.color.a = 0.8

            marker.color.r = self.objects[fs[3]][5][0]
            marker.color.g = self.objects[fs[3]][5][1]
            marker.color.b = self.objects[fs[3]][5][2]
            #print(marker.color)

            result.append(marker)
            iddd += 1

        # Публикуем маркеры
        self.blue_pub.publish(MarkerArray(markers=result))
        return None

'''
objects = {
    'name1': ( diap1=( min, max ), bitwise, others_diap (list), color_in_topic(bgr), size, color(rgb) ),
    'name2': ( diap1, bitwise, diap2, color_in_topic, size )
}
transform_method='rectify_point', dev='from_virtual_to_clover'/'clover'/'virtual'
'''
'''

objects = {
    'blue': ( ((89, 120, 70), (125, 255, 255)), False, None, (0, 0, 255), 0.26, (0,0,255) ),
    'red': ( ((0, 120, 70), (15, 255, 255)), False, None, (255, 0, 0), 0.125*2, (255,0,0) )

}
print(11)


mark = ObjectDetection(objects, transform_method='rectify_point', dev='virtual', cnt_method='default')
rospy.spin()
'''
'''
objects = {
    'blue': ( [(102, 94, 40), (116, 255, 150)], False, None, (0, 0, 255), 0.16, (0,0,255), 'korob' ),
    'red': ( ((0, 120, 70), (15, 255, 255)), False, None, (255, 0, 0), 0.4, (255,0,0), 'korob' ),
    'green': ( [(59, 68, 115), (61, 255, 255)], False, None, (255, 0, 255), 0.4, (0,255,0), 'korob' ),
    'yellow': ( [(29, 83, 131), (31, 255, 255)], False, None, (255, 255, 0), 0.4, (255,255,0), 'korob' )

}
mark = ObjectDetection(objects, transform_method='pnp', dev='from_virtual_to_clover', cnt_method='default')
rospy.spin()
objects = {
    'blue': ( [(93, 125, 124), (120, 255, 253)], False, None, (0, 0, 255), 0.4, (0,0,255), 'korob' ),
    'red': ( [(170, 111, 86), (180, 255, 255)], False, None, (255, 0, 0), 0.4, (255,0,0), 'korob' ),
    'green': ( [(53, 68, 115), (74, 255, 255)], False, None, (255, 0, 255), 0.4, (0,255,0), 'korob' ),
    'yellow': ( [(21, 83, 131), (34, 255, 255)], False, None, (255, 255, 0), 0.4, (255,255,0), 'korob' )

}

mark = ObjectDetection(objects, transform_method='rectify_point', dev='from_virtual_to_clover', cnt_method='default')
mark.STOP_ALL = False
mark.max_cnt = True
mark.rectify_anytime = False
rospy.spin()
'''