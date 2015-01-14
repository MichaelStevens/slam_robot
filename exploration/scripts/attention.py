import cv2

def attention_map(image):
    image = cv2.GaussianBlur(image, (9, 9), 3)
    image = cv2.Canny(image,80,70)
    image = cv2.blur(image, (10, 10)) * 12
    image = cv2.GaussianBlur(image, (129, 129), 20)
    return image
