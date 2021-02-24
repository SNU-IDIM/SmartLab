import cv2
from matplotlib import pyplot as plt



def bgr2rgb(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

def displayplt(img):
    plt.imshow(bgr2rgb(img))
    plt.show()

if __name__ == "__main__":
    img = cv2.imread("C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/4444/pics/calibrate_img0.png")

    displayplt(img)

    # plt.imshow(img,cmap = 'gray')

    x = 935
    y = 355
    x1 = 1050 #1045
    y1 = 470 #457
    w = x1-x
    h = y1-y

    crop_img = img[y:y+h, x:x+w]
    # cv2.imshow("cropped", crop_img)
    displayplt(crop_img)
    cv2.waitKey(0)

    cv2.imwrite("C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/template/2.png",crop_img)
