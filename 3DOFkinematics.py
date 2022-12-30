import serial
import time
import math
import sympy


py_serial = serial.Serial(
    port = 'COM4',
    baudrate = 115200,
)





def inverseKinematics(px,py,pz):
    #단위 mm to cm
    l1 = 88.8/10
    l2 = 72/10
    l3 = 113.85/10
    r =  math.sqrt((px**2) + (py**2))
    s = pz-l1

    # 역기구학 계산 1
    angle_base =  180 + math.degrees(math.atan2(py,px))
    # 역기구학 계산 3
    bunja =  ((r**2) + (s**2))-(((l2**2) + (l3**2)))
    bunmo = 2*l2*l3
    D = bunja/bunmo
    angle_elbow = math.degrees(math.atan2(math.sqrt(1-(D**2)),D))
    # 역기구학 계산2
    angle_shoulder = math.degrees(math.atan2(r,s) + (math.atan2(l2 + l3 *math.cos(math.radians(angle_elbow)),
                                l3*math.sin(math.radians(angle_elbow)))))
    
    print("Angle 1 ik " ,angle_base)
    print("Angle 2 ik" ,angle_shoulder)
    print("Angle 3 ik" ,angle_elbow)

    return angle_base, angle_shoulder, angle_elbow

    #3차 경로 계획
def generate_trajectory(t0, tf, p0, pf):
    tf_t0_3 = (tf - t0)**3
    a0 = pf*(t0**2)*(3*tf-t0) + p0*(tf**2)*(tf-3*t0)
    a0 = a0 / tf_t0_3

    a1 = 6 * t0 * tf * (p0 - pf)
    a1 = a1 / tf_t0_3

    a2 = 3 * (t0 + tf) * (pf - p0)
    a2 = a2 / tf_t0_3

    a3 = 2 * (p0 - pf)
    a3 = a3 / tf_t0_3

    return a0, a1, a2, a3

def DH_T(th, d, a, al):
  #DH 파라미터 받아서 동차변환 행렬 T를 계산하는 함수
  Tzth = sympy.Matrix([[sympy.cos(th), -sympy.sin(th),0,0],
                     [sympy.sin(th),sympy.cos(th),0,0],
                     [0, 0,1,0],
                     [0, 0,0,1]])
  Txa =  sympy.Matrix([[1,0,0,a],
                    [0,1,0,0],
                    [0,0,1,0],
                    [0,0,0,1]])
  Txal =  sympy.Matrix([[1,0,0,0],
                    [0,sympy.cos(al),-sympy.sin(al),0],
                    [0,sympy.sin(al),sympy.cos(al),0],
                    [0,0,0,1]])
  Tzd =  sympy.Matrix([[1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,d],
                    [0,0,0,1]])
  T = Tzth@Tzd@Txa@Txal
  return sympy.simplify(T)

def forward_kinematics(th1, th2, th3):
    l1 = 88.8/10
    l2 = 72/10
    l3 = 113.85/10
    
    t01 = DH_T(sympy.rad(th1),l1,0,sympy.rad(90))
    t12 = DH_T(sympy.rad(th2),0,l2,0)
    t23 = DH_T(sympy.rad(th3),0,l3,0)
    result = t01@t12@t23
    x = result[0,3].evalf()
    y = result[1,3].evalf()
    z = result[2,3].evalf()
    print("x from fk " ,x)
    print("y  from fk " ,y)
    print("z  from fk " ,z)
    return x,y,z

#초기각도
th2p = 0; th3p = 180; th1p = 0
#파이썬에 입력
while 1:
    ix = input('x ='); xn = float(ix)
    iy = input('y ='); yn = float(iy)
    iz = input('z ='); zn = float(iz)
    itimefin = input('time ='); timefin = float(itimefin)
    
    t = 0
    th1, th2, th3 = inverseKinematics(xn,yn, zn)
    forward_kinematics(th1,th2, th3)

    while 1:
        start = time.time()
        a0t,a1t,a2t,a3t=generate_trajectory(0, timefin, th1p, th1)
        a0x,a1x,a2x,a3x=generate_trajectory(0, timefin, th2p, th2)
        a0y,a1y,a2y,a3y=generate_trajectory(0, timefin, th3p, th3)
        theta2 = a0x + a1x*t + a2x*t**2 + a3x*t**3
        theta3 = a0y + a1y*t + a2y*t**2 + a3y*t**3
        theta1 = a0t + a1t*t + a2t*t**2 + a3t*t**3
        motorinput = str(round(theta1))+ ","+ str(round(theta2))+ ","+ str(round(theta3)) +"\n"
        print(motorinput)
        if t>timefin:
            break
        py_serial.write(motorinput.encode())
        time.sleep(0.4)
        t += time.time() -start
    th2p = th2; th3p = th3; th1p = th1


    
