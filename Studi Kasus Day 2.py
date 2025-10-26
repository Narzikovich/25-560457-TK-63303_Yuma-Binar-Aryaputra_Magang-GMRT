import math as mt
import numpy as np

def T(theta, L):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, L*c],
        [s,  c, L*s],
        [0,  0, 1]
    ])

def fk(L1, L2, sudut1, sudut2):
    t1 = T(sudut1, L1)
    t2 = T(sudut2, L2)
    T_total = np.dot(t1, t2)

    return T_total

def ik(L1, L2, x, y):
    r2 = x**2 + y**2
    c2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(c2) > 1:
        raise ValueError('Target out of reach')
    s2_pos = np.sqrt(1 - c2**2)
    s2_neg = -s2_pos
    solutions = []
    for s2 in [s2_pos, s2_neg]:
        theta2 = np.arctan2(s2, c2)
        k1 = L1 + L2 * np.cos(theta2)
        k2 = L2 * np.sin(theta2)
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        solutions.append((np.degrees(theta1), np.degrees(theta2)))
    return solutions


print(''' 
Pilih opsi
    1. Forward Kinematics
    2. Inverse Kinematics
''')

opsi = input('Pilihan: ')

if opsi == '1' :
    l1 = float(input("Lengan 1 : "))
    l2 = float(input("Lengan 2 : ")) 
    t1 = float(input("Sudut 1 : "))
    t2 = float(input("Sudut 2 : "))

    print(f'Koordinat titik akhir adalah: \n{fk(l1, l2, t1, t2)} ')
elif opsi == '2' :
    l1 = float(input("Lengan 1 : "))
    l2 = float(input("Lengan 2 : "))
    x = float(input("Koordinat x : "))
    y = float(input("Koordinat y : "))

    print(f'Sudut lengan (θ1, θ2) adalah: ')
    print(ik(l1, l2, x, y))
else :
    print('ERROR: Pilihan tidak valid')




