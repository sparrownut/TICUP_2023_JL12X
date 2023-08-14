import sensor, image, time
from pyb import UART

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)           # 打开自动增益, 默认打开；追踪颜色，则需关闭白平衡。
sensor.set_auto_whitebal(False)       # 打开（True）或关闭（False）自动白平衡。默认打开；追踪颜色，则需关闭白平衡
sensor.set_auto_exposure(False)       # 打开（True）或关闭（False）自动曝光。默认打开。
#sensor.set_brightness(-50)
sensor.set_contrast(5)
sensor.set_auto_exposure(False, exposure_us=19000)  # Set exposure to 20000 microseconds.

# HSV color thresholds for red and green
red_threshold = (0, 100, 127, 127, -128, 127)
green_threshold = (60, 100, -128, 127, -128, 127)

def findRedForGPT(img):
    img = img.to_hsv()  # Convert the image to HSV color space

    # Find red and green blobs
    red_blobs = img.find_blobs([red_threshold])
    green_blobs = img.find_blobs([green_threshold])

    # Loop over all red blobs
    for blob in red_blobs:
        # Crop the blob from the image
        blob_img = img.crop(blob.rect())

        # Calculate the mean pixel value of the blob
        mean_val = np.mean(blob_img)

        # If the mean pixel value is close to white, then this is a light spot
        if mean_val > 200:
            #print('Red light spot found at', blob.cx(), blob.cy())
            return blob

    ## Repeat the same process for green blobs
    #for blob in green_blobs:
        #blob_img = img.crop(blob.rect())
        #mean_val = np.mean(blob_img)
        #if mean_val > 200:
            #print('Green light spot found at', blob.cx(), blob.cy())


class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.cumulative_error = 0.0

    def reset(self):
        self.prev_error = 0.0
        self.cumulative_error = 0.0

    def compute(self, error):
        self.cumulative_error += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.cumulative_error + self.Kd * derivative
        self.prev_error = error
        return output



def send_cmd(port=3, baudrate=115200, cmd="", wait_time=0.1):
    # Initialize serial port
    #with serial.Serial(port, baudrate, timeout=1) as ser:
        #ser.write(cmd.encode())  # Send the message
    try:
        uart = UART(port, baudrate)
        uart.write(f'{cmd}\n')
        #while True:
            #if uart.any():  # Check if there are incoming bytes
                #data = uart.read(1)  # Read one byte
                #if data == b'@':  # Check if the byte is '@'
                    #return  # Exit the function
            #time.sleep(wait_time)  # Delay for a short period
    except Exception as e:
        pass
def send_cmd_GO(x, y):
    try:
        print(f"{x} {y}#")
        send_cmd(cmd=f"{x} {y}#")
    except Exception as e:
        print(f'- 发送串口失败')
        #traceback.print_exception(e)

def condition_PixelN(blob):
    return (blob.pixels() < 100 and blob.pixels() > 4)

# 卡尔曼滤波实现
class KalmanFilter:
    def __init__(self, dt, process_var, measurement_var):
        self.A = [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]
        self.H = [[1, 0, 0, 0], [0, 1, 0, 0]]
        self.P = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        self.Q = [[process_var, 0, 0, 0], [0, process_var, 0, 0], [0, 0, process_var, 0], [0, 0, 0, process_var]]
        self.R = [[measurement_var, 0], [0, measurement_var]]
        self.x = [[0], [0], [0], [0]]



    def predict(self):
        self.x = self.matrix_mult(self.A, self.x)
        self.P = self.matrix_add(self.matrix_mult(self.matrix_mult(self.A, self.P), self.transpose(self.A)), self.Q)
        return self.x

    def update(self, measurement):
        # First compute the Kalman Gain K
        S = self.matrix_add(self.matrix_mult(self.matrix_mult(self.H, self.P), self.transpose(self.H)), self.R)
        K = self.matrix_mult(self.matrix_mult(self.P, self.transpose(self.H)), self.inverse(S))

        # Now use K to update the state and covariance
        Z = [[measurement[0]], [measurement[1]]]
        Y = self.matrix_sub(Z, self.matrix_mult(self.H, self.x))
        self.x = self.matrix_add(self.x, self.matrix_mult(K, Y))

        I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        self.P = self.matrix_mult(self.matrix_sub(I, self.matrix_mult(K, self.H)), self.P)


    # Matrix utility functions:
    def matrix_add(self, A, B):
        return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]

    def matrix_sub(self, A, B):
        if len(A) != len(B) or len(A[0]) != len(B[0]):
            raise ValueError("Dimensions of A and B don't match!")
        return [[A[i][j] - B[i][j] for j in range(len(A[0]))] for i in range(len(A))]



    def matrix_mult(self, A, B):
        result = [[0 for j in range(len(B[0]))] for i in range(len(A))]
        for i in range(len(A)):
            for j in range(len(B[0])):
                for k in range(len(B)):
                    result[i][j] += A[i][k] * B[k][j]
        return result

    def transpose(self, A):
        return [[A[j][i] for j in range(len(A))] for i in range(len(A[0]))]

    def inverse(self, A):
        if len(A) == 2 and len(A[0]) == 2:
            det = A[0][0] * A[1][1] - A[0][1] * A[1][0]
            if det == 0:
                raise ValueError("Matrix is not invertible")
            inv_det = 1.0 / det
            return [[A[1][1] * inv_det, -A[0][1] * inv_det],
                    [-A[1][0] * inv_det, A[0][0] * inv_det]]
        else:
            raise NotImplementedError("Inverse function for matrices larger than 2x2 is not implemented")



clock = time.clock()

def is_square(rect):
    x, y, w, h = rect
    aspect_ratio = w / h
    return 0.7 <= aspect_ratio <= 1.3

def distance_from_center(blob,img):

    img_center = (img.width() // 2, img.height() // 2)
    blob_center = blob.cx(), blob.cy()
    return (blob_center[0] - img_center[0]) ** 2 + (blob_center[1] - img_center[1]) ** 2


def getBoardRoi(img,wT = (35, 69, -4, 15, -10, 25)):
    blobs = img.find_blobs([wT])
    if blobs:
        blobs = max(blobs, key=lambda b: (b.pixels()))
        img.draw_rectangle(blobs.rect(),color=(255,0,0))
        return (blobs[0],blobs[1],blobs[2],blobs[3])


def getRedPointInBlackrT1(img,rT1=(0, 100, 20, 127, -17, 51)):
    blobs = img.find_blobs([rT1]) # rT1,
    if blobs:
        # 筛选出像素数小于30且形状近似于正方形的红色区域
        square_blobs = [b for b in blobs if condition_PixelN(b)]

        if square_blobs:
            # 按像素数量对方框排序，并选择密度最大的
            smallest_square = max(square_blobs, key=lambda b: (b.density()))
            # 使用绿色的线条在图像上画出这个方框，方便区分
            #img.draw_rectangle(smallest_square.rect(), color=(0, 255, 0))
            return smallest_square
def clip_rect(img, rect):
    x, y, w, h = rect
    x = max(0, min(x, img.width()))
    y = max(0, min(y, img.height()))
    w = max(0, min(w, img.width() - x))
    h = max(0, min(h, img.height() - y))
    return (x, y, w, h)

def max_blob_density(blobs):
    if blobs:
        return max(blob.density() for blob in blobs)
    else:
        return 0

def max_blob_area(blobs):
    if blobs:
        return max(blob.pixels() for blob in blobs)
    else:
        return 0

def get_max_area_blob(blob_list):
    return max(blob_list, key=lambda b: b.area())


def getRedPoint(img ,red_threshold = (0, 100, 10, 127, -128, 127)):
    roi = (60,10,200,200)
    #img.draw_rectangle(roi,(255,255,255))

    red_blobs = img.find_blobs([red_threshold], pixels_threshold=2, area_threshold=4,roi=roi)
    for red_blob in red_blobs:
        img.draw_rectangle(red_blob.rect(),(255,0,0))
    if red_blobs:
        return get_max_area_blob(red_blobs)

def getGreenPoint(img ,green_threshold = (10, 100, -15, -128, -128, 127)):
    roi = (60,10,200,200)
    #img.draw_rectangle(roi,(255,255,255))

    green_blobs = img.find_blobs([green_threshold], pixels_threshold=2, area_threshold=4,roi=roi)
    for green_blob in green_blobs:
        img.draw_rectangle(green_blob.rect(),(255,0,0))
    if green_blobs:
        return get_max_area_blob(green_blobs)

def draw_cross_full(img,x_edit,y_edit):
    # 获取图像的宽度和高度
    width = img.width()
    height = img.height()

    # 画垂直线
    img.draw_line(((width // 2) + y_edit, 0, (width // 2) + y_edit, height - 1), color = (0, 255, 0))

    # 画水平线
    img.draw_line((0, x_edit + (height // 2), width - 1, x_edit + (height // 2)), color = (0, 255, 0))

def auto_config_redThreshold(img):
    # 自动配置红色阈值，逻辑是调整阈值直到屏幕中出现了一个红色点
    Lmin = 0
    Lmax = 100
    Amin = 20
    Amax = 127
    Bmin = -17
    Bmax = 51
    defaultThreshold = (Lmin, Lmax, Amin, Amax, Bmin, Bmax)
    getRedPoint(img)


# 初始化卡尔曼滤波
kf = KalmanFilter(dt=0.5, process_var=1, measurement_var=10)

predicted_x = 0
predicted_y = 0

# test

# test
#send_cmd_GO(100,0)
#time.sleep(1)
#send_cmd_GO(-100,0)
#time.sleep(1)
#send_cmd_GO(0,100)
#time.sleep(1)
#send_cmd_GO(0,-100)

#pidX = PID(Kp=0.5, Ki=0.003, Kd=0.001)
#pidY = PID(Kp=0.5, Ki=0.003, Kd=0.001) # 1
#pidX = PID(Kp=0.72, Ki=0.022, Kd=0.06)
#pidY = PID(Kp=0.72, Ki=0.022, Kd=0.06) # 2
#pidX = PID(Kp=0.7, Ki=0.01, Kd=0.06)
#pidY = PID(Kp=0.7, Ki=0.01 ,Kd=0.06) # 3
pidX = PID(Kp=1.20, Ki=0.006, Kd=0.09)
pidY = PID(Kp=1.30, Ki=0.006 ,Kd=0.09) # 4


def control(img,currentX,currentY, targetX, targetY):

    xErr = targetX - currentX
    yErr = targetY - currentY

    # Compute the corrections using PID controllers
    xCorrection = pidX.compute(xErr)
    yCorrection = pidY.compute(yErr)

    # Since send_cmd_GO accepts only integers, round the correction values
    send_cmd_GO(int(xCorrection), -int(yCorrection))


#def control(img,targetX,targetY,kxP = 1.0,kyP = 1.0):
    #centerX = (img.width() // 2) + 7
    #centerY = (img.height() // 2) + 10
    #xErr = targetX - centerX
    #yErr = targetY - centerY
    ###if xErr < 2:
        ###kxP = xErr / 5
    ###if yErr < 2:
        ###kyP = yErr / 5
    #kxP = abs(xErr) / 30
    #kyP = abs(yErr) / 30
    #if kxP > 1:
        #kxP = 1
    #if kyP > 1:
        #kyP = 1
    #if kxP < 0.1:
        #kxP = 0.1
    #if kyP < 0.1:
        #kyP = 0.1
    #resX = int(xErr*kxP)
    #resY = int(-(yErr*kyP))
    #if(abs(resX - xErr) < 4):
        #resX = xErr
    #if(abs(resY - yErr) < 4):
        #resY = yErr


    ##if abs(resX) < 50:
        ##kxP = 0.5
    ##if abs(resY) < 50:
        ##kyP = 0.5
    #print(f'Err:{xErr} {yErr} res: {resX} {resY}')
    ##if abs(resX) > 2 or abs(resY) > 2:
    #send_cmd_GO(int(resX),int(resY))

def getBlobCenter(blob):
    return blob[0] + (blob[2] / 2) , blob[1] + (blob[3] / 2)

def autoConfigRed(img,rT2=(95, 100, 8, 127, -1, 127)):
    width = img.width()
    height = img.height()
    #img.draw_rectangle((138,110,60,40),(0,255,0))

    light_min = 75
    haveRedPointBlobsList = []
    for Lmin in range(65,100):
        for Amin in range(-5,20):
            print(f"{Lmin}  {Amin}")
            rT2 = (Lmin,) + rT2[1:]
            blobs = img.find_blobs([rT2])
            for it in blobs:
                #print('进入1')
                img = sensor.snapshot()
                if it[0] > 138 and it[0] < 198 and it[1] > 110 and it[1] < 150 and abs(1 - (it[2] / it[3]) < 0.05) and it.density() > 0.5 and condition_PixelN(it):
                    haveRedPointBlobsList.append(rT2)
                    break
    #for rTit in haveRedPointBlobsList:
        #nums = len(img.find_blobs([rTit]))
    if haveRedPointBlobsList:
        min_rTit = min(haveRedPointBlobsList, key=lambda rTit: len(img.find_blobs([rTit])))
        blobs = img.find_blobs([min_rTit])
        print(f'rT:{min_rTit} 能识别到{len(blobs)}个点')

    else:
        autoConfigRed(img,rT2=(91, 100, -7, 127, 0, 127))


    #roi = getBoardRoi(img)
    #if roi:

        #roi = (roi_x + 20, roi_y + 20, roi_width - 50, roi_height - 20)
        #img.draw_rectangle(roi,color=(255,0,0))

    # 查找blobs
    blobs = img.find_blobs([rT2])
    if blobs:
        # 筛选出像素数小于30且形状近似于正方形的红色区域

        square_blobs = [b for b in blobs if (b[2] < 40 and b[3] < 40)]

        if square_blobs:
            # 按像素数量对方框排序，并选择最小的那个
            smallest_square = min(square_blobs, key=lambda b: (b.pixels()))
            # 使用绿色的线条在图像上画出这个方框，方便区分
            #img.draw_rectangle(smallest_square.rect(), color=(0, 255, 0))
            #return smallest_square
    return min_rTit
img = sensor.snapshot()

#rT2 = autoConfigRed(img) #  自动调参

historygX = 0
historygY = 0

while(True):

    clock.tick()
    img = sensor.snapshot()

    rx = 0
    ry = 0
    gx = 0
    gy = 0

    redPoint = getRedPoint(img)
    if redPoint != None:
        rx,ry = getBlobCenter(redPoint)
        img.draw_cross(int(rx),int(ry),(255,0,0))
        #control(img,redPoint[0],redPoint[1])



    greenPoint = getGreenPoint(img)
    if greenPoint != None:
        gx,gy = getBlobCenter(greenPoint)
        img.draw_cross(int(gx),int(gy),(0,255,0))
    if not (rx == 0 or ry == 0 or gx == 0 or gy == 0):
        control(img,gx,gy,rx,ry)
    #else:
        #control(img,0,0,0,0)
        #send_cmd(0,0)
