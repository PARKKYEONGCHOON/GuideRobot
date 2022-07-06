#!/usr/bin/python3
# -*- coding: utf-8 -*-


#from tkinter import Frame
import cv2
import pyzbar.pyzbar as pyzbar
import numpy as np
import cvlib as cv
import os
from time import sleep
import argparse

from scipy.misc import face
from cls_wide_resnet import WideResNet
from keras.utils.data_utils import get_file
import tensorflow as tf
import tensorflow.keras
import glob, dlib


# gpus = tf.config.experimental.list_physical_devices('GPU')
# if gpus:
#     try:
#         # Currently, memory growth needs to be the same across GPUs
#         for gpu in gpus:
#             tf.config.experimental.set_memory_growth(gpu, True)
#         logical_gpus = tf.config.experimental.list_logical_devices('GPU')
#         print(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPUs")
#     except RuntimeError as e:
#         # Memory growth must be set before GPUs have been initialized
#         print(e)
        
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
  # Restrict TensorFlow to only allocate 1GB of memory on the first GPU
  try:
    tf.config.experimental.set_virtual_device_configuration(
        gpus[0],
        [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=1024)])
    logical_gpus = tf.config.experimental.list_logical_devices('GPU')
    print(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPUs")
  except RuntimeError as e:
    # Virtual devices must be set before GPUs have been initialized
    print(e)
    
diretory = os.path.dirname(os.path.abspath(__file__))
os.chdir(diretory)

font = cv2.FONT_HERSHEY_SIMPLEX

age_list = ['(0, 2)','(4, 6)','(8, 12)','(15, 20)','(25, 32)','(38, 43)','(48, 53)','(60, 100)']
gender_list = ['Male', 'Female']

detector = dlib.get_frontal_face_detector()

age_net = cv2.dnn.readNetFromCaffe(
          'models/deploy_age.prototxt', 
          'models/age_net.caffemodel')
gender_net = cv2.dnn.readNetFromCaffe(
          'models/deploy_gender.prototxt', 
          'models/gender_net.caffemodel')


#teacherble

labels=[]
f=open("./models/test4.txt", "r")
for x in f:
     labels.append(x.rstrip('\n'))
label_count = len(labels)
f.close()

# 모델 위치
model_filename ='./models/test4.h5'

# 케라스 모델 가져오기
model = tensorflow.keras.models.load_model(model_filename,compile=False)

site = ['A Site','B Site','C Site','D Site']

data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)
#np.set_printoptions(suppress=True)

class imageprocessing:
    
    def __init__(self):
        
        
        self.args = get_args()
        self.depth = self.args.depth
        self.width = self.args.width

        self.face = FaceCV(depth=self.depth, width=self.width)
        self.result = False
        self.CASE_PATH = "./pretrained_models/haarcascade_frontalface_alt.xml"
        self.WRN_WEIGHTS_PATH = "https://github.com/Tony607/Keras_age_gender/releases/download/V1.0/weights.18-4.06.hdf5"
        
        self.tmpframe = None
    
    # 바코드 인식 및 테두리 설정
    def read_frame(self, frame):
        
        height, width, layers = frame.shape
        
        try:
            
            # 바코드 정보 decoding
            barcodes = pyzbar.decode(frame)
            # 바코드 정보가 여러개 이기 때문에 하나씩 해석
            
            if barcodes:
                for barcode in barcodes:
                    # 바코드 rect정보
                    x, y, w, h = barcode.rect
                    # 바코드 데이터 디코딩
                    barcode_info = barcode.data.decode('utf-8')
                    #print(barcode_info)
                    # 인식한 바코드 사각형 표시
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    # 인식한 바코드 사각형 위에 글자 삽입
                    cv2.putText(frame, barcode_info, (x , y - 20), font, 0.5, (0, 0, 255), 1)
                    self.result = True
                    
                return frame, barcode.data.decode('utf-8')
            else:
                # self.result = False
                # return frame , None

                rectimage = self.rect_Image(frame,int(width/3)-100,int(height/3)-50,int(width/3)*2-100,int(height/3)*2+50,255,255,0,3,"QR Code")
                return rectimage, None
            
        except Exception as e:
            print(e)
            
    
    
    #cv face detection
    def gender_Detect_frame(self, frame):
        
        face_crop = None
        padding = 20
        face, confidence = cv.detect_face(frame)

        #print(face)
        #print(confidence)

        # loop through detected faces
        for idx, f in enumerate(face):
            
            (startX,startY) = max(0, f[0]-padding), max(0, f[1]-padding)
            (endX,endY) = min(frame.shape[1]-1, f[2]+padding), min(frame.shape[0]-1, f[3]+padding)
        
            # draw rectangle over face
            cv2.rectangle(frame, (startX,startY), (endX,endY), (0,255,0), 2)

            face_crop = np.copy(frame[startY:endY, startX:endX])

            # apply face detection    
            (label, confidence) = cv.detect_gender(face_crop)

            #print(confidence)
            #print(label)

            idx = np.argmax(confidence)
            label = label[idx]

            label = "{}: {:.2f}%".format(label, confidence[idx] * 100)

            Y = startY - 10 if startY - 10 > 10 else startY + 10

            # write detected gender and confidence percentage on top of face rectangle
            #cv2.putText(frame, label, (startX,Y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0), 2)

        
        return frame, face, face_crop
    
    def face_Detection(self, frame):
        
        try:
            faces = detector(frame)
            height, width, layers = frame.shape

            if faces:
                for face in faces:
                    x1, y1, x2, y2 = face.left(), face.top(), face.right(), face.bottom()

                    face_img = frame[y1:y2, x1:x2].copy()

                    blob = cv2.dnn.blobFromImage(face_img, scalefactor=1, size=(227, 227),
                    mean=(78.4263377603, 87.7689143744, 114.895847746),
                    swapRB=False, crop=False)

                    # predict gender
                    gender_net.setInput(blob)
                    gender_preds = gender_net.forward()
                    gender = gender_list[gender_preds[0].argmax()]

                    # predict age
                    age_net.setInput(blob)
                    age_preds = age_net.forward()
                    age = age_list[age_preds[0].argmax()]
                    
                    if age_list.index(age) > 1:
                        ac = "Adult"
                    else:
                        ac = "Children"

                    # visualize
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255,255,255), 2)
                    overlay_text = '%s %s' % (gender, ac)
                    cv2.putText(frame, overlay_text, org=(x1, y1), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1, color=(0,0,0), thickness=10)
                    cv2.putText(frame, overlay_text, org=(x1, y1),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255,255,255), thickness=2)

                return frame
            
            else:
                
                rectimage = self.rect_Image(frame,int(width/3)-100,int(height/3)-50,int(width/3)*2-100,int(height/3)*2+50,255,255,0,3,"Face")
                
                return rectimage
            
        except Exception as e:
            print(str(e))
            
    # 이미지 처리하기 티처블 머신 이미지 사이즈로 변환
    def teacherble_reshaped(self,frame):
        #frame_fliped = cv2.flip(frame, 1)
        
        size = (224, 224)
        frame_resized = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
        
        # 이미지 정규화
        # astype : 속성
        frame_normalized = (frame_resized.astype(np.float32) / 127.0) - 1

        # 이미지 차원 재조정 - 예측을 위해 reshape 해줍니다.
        # keras 모델에 공급할 올바른 모양의 배열 생성
        frame_reshaped = frame_normalized.reshape((1, 224, 224, 3))
        #print(frame_reshaped)
        return frame_reshaped
    
        # 예측용 함수
    def teacherble_predict(self,frame):
        prediction = model.predict(frame)
        print(prediction)

        return prediction
    
    def teacherble_detection(self,frame):
        
        
        
        self.tmpframe = frame
        img, face, face_crop = self.gender_Detect_frame(frame)
        
        h, w, layers = img.shape
        
        
        if face:
        
            crop_image = img[0:h, int((w-h)/2):int(w-((w-h)/2))]

            # 바이큐빅보간법(cv2.INTER_CUBIC, 이미지를 확대할 때 주로 사용)을 이용해 frame변수에 들어온 비디오 프레임의 사이즈를 224, 224로 다운사이징하여 image 변수에 넣음
            #image = cv2.resize(crop_image, dsize=(224, 224), interpolation=cv2.INTER_CUBIC)
            image = cv2.resize(face_crop, dsize=(224, 224), interpolation=cv2.INTER_CUBIC)

            # asarray메소드를 이용해 image에 들어있는 크기가 변형된 이미지를 numpy가 처리할 수 있는 배열로 만들어서 image_array 변수에 넣음
            image_array = np.asarray(image)

            # image_array에 들어있는 image의 변형된 배열을 정규화(normalized)하기 위해 수식을 적용함
            normalized_image_array = (image_array.astype(np.float32) / 127.0) - 1

            # 정규화된 배열을 data[0]에 넣음
            data[0] = normalized_image_array

            # 정규화된 배열값으로 정돈된 data를 Teachable Machine으로 학습시켜서 얻은 모델을 이용해 추론하고, 그 결과를 prediction 변수에 넣음
            prediction = model.predict(data)
            
                # 글씨 넣기 준비
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            fontColor = (0,255,0)
            lineThickness = 1

            # 표기 문구 초기화
            scoreLabel = 0
            score = 0
            result = ''

            for x in range(0, label_count):
                # #예측값 모니터링
                # line=('%s=%0.0f' % (labels[x], int(round(prediction[0][x]*100)))) + "%"
                # cv2.putText(frame, line, (10,(x+1)*35), font, fontScale, fontColor, lineThickness)

                # 가장 높은 예측 찾기
                if score < prediction[0][x]:
                    scoreLabel = labels[x]
                    score = prediction[0][x]
                    result = str(scoreLabel) + " : " + str(score)
                    
                    #print(result)
                    
                    
                    
            if score > 0.9:
                # 최고 결과치 보여
                #frame = cv2.putText(frame, result, (10, int(label_count+1)*35), font, 255, (0, 255, 0), 1, cv2.LINE_AA)
                
                # if result[0] == 'B' or result[0] == 'C':
                    
                #     if result[0] == 'B':
                        
                #         result = result.replace('B','C')
                        
                    
                #     elif result[0] == 'C':
                        
                #         result = result.replace('C','B')
                        
                        
                frame = cv2.putText(frame, result, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(255,255,0), 2)
                site = result[0]
                    
                # else:
                    
                #     frame = cv2.putText(frame, result, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(255,255,0), 2)
                #     site = result[0]
                
                
                #print(site)
                
                return frame, site
            
            else:
                
                rectimage = self.rect_Image(self.tmpframe,int(w/3)-100,int(h/3)-50,int(w/3)*2-100,int(h/3)*2+50,255,255,0,3,"Face")
                return rectimage, None
        
        else:
            
            rectimage = self.rect_Image(frame,int(w/3)-100,int(h/3)-50,int(w/3)*2-100,int(h/3)*2+50,255,255,0,3,"Face")
                
            return rectimage, None
    
    def rect_Image(self, frame, Sx,Sy,Ex,Ey,R,G,B,thick,msg):
        
        label = msg
        
        rect_frame = cv2.rectangle(frame,(Sx,Sy),(Ex,Ey),(R,G,B),thick)
        cv2.putText(rect_frame, label, (Sx+10,Sy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(R,G,B), 2)
        
        return rect_frame
    
    def circle_Image(self, frame, Cx,Cy,Radius,R,G,B,thick):
        
        circle_frame = cv2.circle(frame, (Cx,Cy),Radius,(R,G,B),thick)
        
        return circle_frame
        

class FaceCV(object):
    """
    Singleton class for face recongnition task
    """
    CASE_PATH = "./pretrained_models/haarcascade_frontalface_alt.xml"
    WRN_WEIGHTS_PATH = "https://github.com/Tony607/Keras_age_gender/releases/download/V1.0/weights.18-4.06.hdf5"


    def __new__(cls, weight_file=None, depth=16, width=8, face_size=128):
        if not hasattr(cls, 'instance'):
            cls.instance = super(FaceCV, cls).__new__(cls)
        return cls.instance

    def __init__(self, depth=16, width=8, face_size=64):
        self.face_size = face_size
        self.model = WideResNet(face_size, depth=depth, k=width)()
        model_dir = os.path.join(os.getcwd(), "pretrained_models").replace("//", "\\")
        fpath = get_file('weights.18-4.06.hdf5',
                         self.WRN_WEIGHTS_PATH,
                         cache_subdir=model_dir)
        self.model.load_weights(fpath)

    @classmethod
    def draw_label(cls, image, point, label, font=cv2.FONT_HERSHEY_SIMPLEX,
                   font_scale=1, thickness=2):
        size = cv2.getTextSize(label, font, font_scale, thickness)[0]
        x, y = point
        cv2.rectangle(image, (x, y - size[1]), (x + size[0], y), (255, 0, 0), cv2.FILLED)
        cv2.putText(image, label, point, font, font_scale, (255, 255, 255), thickness)

    def crop_face(self, imgarray, section, margin=40, size=64):
        """
        :param imgarray: full image
        :param section: face detected area (x, y, w, h)
        :param margin: add some margin to the face detected area to include a full head
        :param size: the result image resolution with be (size x size)
        :return: resized image in numpy array with shape (size x size x 3)
        """
        img_h, img_w, _ = imgarray.shape
        if section is None:
            section = [0, 0, img_w, img_h]
        (x, y, w, h) = section
        margin = int(min(w,h) * margin / 100)
        x_a = x - margin
        y_a = y - margin
        x_b = x + w + margin
        y_b = y + h + margin
        if x_a < 0:
            x_b = min(x_b - x_a, img_w-1)
            x_a = 0
        if y_a < 0:
            y_b = min(y_b - y_a, img_h-1)
            y_a = 0
        if x_b > img_w:
            x_a = max(x_a - (x_b - img_w), 0)
            x_b = img_w
        if y_b > img_h:
            y_a = max(y_a - (y_b - img_h), 0)
            y_b = img_h
        cropped = imgarray[y_a: y_b, x_a: x_b]
        resized_img = cv2.resize(cropped, (size, size), interpolation=cv2.INTER_AREA)
        resized_img = np.array(resized_img)
        return resized_img, (x_a, y_a, x_b - x_a, y_b - y_a)

    def detect_face(self, frame):
        face_cascade = cv2.CascadeClassifier(self.CASE_PATH)

        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.2,
            minNeighbors=10,
            minSize=(self.face_size, self.face_size)
        )
        
        # placeholder for cropped faces
        face_imgs = np.empty((len(faces), self.face_size, self.face_size, 3))
        for i, face in enumerate(faces):
            face_img, cropped = self.crop_face(frame, face, margin=40, size=self.face_size)
            (x, y, w, h) = cropped
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 200, 0), 2)
            face_imgs[i,:,:,:] = face_img
        if len(face_imgs) > 0:
            # predict ages and genders of the detected faces
            results = self.model.predict(face_imgs)
            predicted_genders = results[0]
            ages = np.arange(0, 101).reshape(101, 1)
            predicted_ages = results[1].dot(ages).flatten()
            # draw results
        for i, face in enumerate(faces):
            label = "{}, {}".format(int(predicted_ages[i]), "F" if predicted_genders[i][0] > 0.5 else "M")
            self.draw_label(frame, (face[0], face[1]), label)

            #cv2.imshow('Keras Faces', frame)

        return frame
       
        
        
def get_args():
    parser = argparse.ArgumentParser(description="This script detects faces from web cam input, "
                                    "and estimates age and gender for the detected faces.",
                                    formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("--depth", type=int, default=16,help="depth of network")
    parser.add_argument("--width", type=int, default=8, help="width of network")
    args = parser.parse_args()
    return args



    

# def main():
#     args = get_args()
#     depth = args.depth
#     width = args.width

#     face = FaceCV(depth=depth, width=width)

#     face.detect_face()

# if __name__ == "__main__":
#     main()    
    
            
    