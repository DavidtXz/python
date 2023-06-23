import cv2
import time
#print("Librerias importadas")
cap = cv2.VideoCapture(0)
img_counter = 0

def getContours(img):
    #frameContour = frame.copy()
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #imgBlur = cv2.GaussianBlur(imgGray, (7,7),1)
    imgCanny = cv2.Canny(imgGray, 50, 55)
    imgCanny = cv2.dilate(imgCanny, None, iterations=1)
    imgCanny = cv2.erode(imgCanny, None, iterations =1)
    cnts,_ = cv2.findContours(imgCanny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)# OpenCV 4
    for c in cnts:
        area = cv2.contourArea(c)
        #print(area)
        if area > 500:
            epsilon = 0.025*cv2.arcLength(c,True)
            approx = cv2.approxPolyDP(c,epsilon,True)
            #print(len(approx))
            #cv2.drawContours(img, [approx], -1, (0,255,0), 2)
            #cv2.imshow('Contours',img)
            #cv2.waitKey(0)
            x, y, w, h = cv2.boundingRect(approx)
            #cv2.drawContours(img, [approx], 0, (255,0,0),2)
            print(len(approx))
            
            if len(approx)==3:
                objectType = "Triangulo"
            elif len(approx)==4:
                aspecto = w/float(h)
                print("aspecto: " + str(aspecto))
                if aspecto > 0.90 and aspecto < 1.10:
                    objectType = "Cuadrado"
                elif aspecto > 0.625 and aspecto < 1.55:
                    objectType = "Rectangulo"
                else:
                    objectType = "Rombo"
            elif len(approx)>4:
                objectType = "Circulo"
            else:
                objectType = "None"
                
            cv2.drawContours(img, [approx], -1, (0,255,0),3)
            #cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
            cv2.putText(img, objectType,  (x+(w//2)-10, y+(h//2)-10),cv2.FONT_HERSHEY_COMPLEX, 0.5, (255,255,255),2)
    
# while True:
#     _, frame = cap.read()
#     getContours(frame)
#     cv2.imshow("Figuras geometricas", frame)
#     k = cv2.waitKey(1) 
#     if k%256 == 97: #a
#         img_name = "imagen_{}.png".format(img_counter) 
#         cv2.imwrite(img_name, frame)
#         print("{} written!".format(img_name)) 
#         img_counter += 1
#     if cv2.waitKey(1) == 27:
#         break



