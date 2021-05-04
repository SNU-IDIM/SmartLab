import cv2
from matplotlib import pyplot as plt



def bgr2rgb(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

def displayplt(img):
    plt.imshow(bgr2rgb(img))
    plt.show()

if __name__ == "__main__":
    # img = cv2.imread("C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/frame/calibimg_0.png")
    img = cv2.imread("C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/D30_A30_150/pics/calibrate_img0.png")

    displayplt(img)


    x = 759
    y = 486
    x1 = 873 #1045
    y1 = 595 #457
    w = x1-x
    h = y1-y

    crop_img = img[y:y+h, x:x+w]
    # cv2.imshow("cropped", crop_img)
    displayplt(crop_img)
    cv2.waitKey(0)

    cv2.imwrite("C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/template/test.png",crop_img)
