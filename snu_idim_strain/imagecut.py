import cv2
from matplotlib import pyplot as plt



def bgr2rgb(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

def displayplt(img):
    plt.imshow(bgr2rgb(img))
    plt.show()

if __name__ == "__main__":
    img = cv2.imread("C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/frame/calibimg_0.png")

    displayplt(img)

    # plt.imshow(img,cmap = 'gray')

    x = 759
    y = 417
    x1 = 959 #1045
    y1 = 572 #457
    w = x1-x
    h = y1-y

    crop_img = img[y:y+h, x:x+w]
    # cv2.imshow("cropped", crop_img)
    displayplt(crop_img)
    cv2.waitKey(0)

    cv2.imwrite("C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/template/3_4.png",crop_img)
