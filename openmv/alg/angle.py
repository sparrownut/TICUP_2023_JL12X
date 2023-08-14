import math
import numpy as np
si96utstisp6hshhtsith = int(input("x:"))
su397uif333pisti9sfd7 = int(input("y:"))
def s6p3isis37s966pppfdst3(x, y):
    sh9dsthsf77hifutih7fp = 250.0 + 10.0 
    sd3ss3stpih7iisdp7sif = 250.0 - 105.0 
    siuispipssi6u9hhdp9hd = 980.0
    print(
        f"XANG: {np.degrees(np.arctan((x - sh9dsthsf77hifutih7fp) / siuispipssi6u9hhdp9hd))} YANG: {np.degrees(np.arctan((sd3ss3stpih7iisdp7sif - y) / math.sqrt(siuispipssi6u9hhdp9hd ** 2 + (sh9dsthsf77hifutih7fp - x) ** 2)))}")
getAngel(si96utstisp6hshhtsith, su397uif333pisti9sfd7)
