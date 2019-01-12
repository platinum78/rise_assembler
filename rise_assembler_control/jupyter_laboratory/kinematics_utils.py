import numpy as np
from numpy import sin, cos
import sympy

def trans_matrix(a, alpha, d, theta): # from book
    matrix = np.array([[np.cos(theta),                  -np.sin(theta),                 0,              a                   ],
                       [np.sin(theta) * np.cos(alpha),  np.cos(theta) * np.cos(alpha),  -np.sin(alpha), -np.sin(alpha) * d  ],
                       [np.sin(theta) * np.sin(alpha),  np.cos(theta) * np.sin(alpha),  np.cos(alpha),  np.cos(alpha) * d   ],
                       [0,                              0,                              0,              1                   ]])
    return matrix

def trans_matrix_sym(a, alpha, d, theta): # from book
    a, alpha, d, theta = sympy.symbols(a + " " + alpha + " " + d + " " + theta)
    matrix = sympy.Matrix([[sympy.cos(theta),                  -sympy.sin(theta),                 0,              a                   ],
                           [sympy.sin(theta) * sympy.cos(alpha),  sympy.cos(theta) * sympy.cos(alpha),  -sympy.sin(alpha), -sympy.sin(alpha) * d  ],
                           [sympy.sin(theta) * sympy.sin(alpha),  sympy.cos(theta) * sympy.sin(alpha),  sympy.cos(alpha),  sympy.cos(alpha) * d   ],
                           [0,                              0,                              0,              1                   ]])
    return matrix

def irb120_forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6):
    # Mine
    a = [0, 0, 270, 70, 0, 0]
    alpha = [0, np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2]
    d = [0, 0, 0, 302, 0, 72]

    T01 = trans_matrix(a[0], alpha[0], d[0], theta1)
    T12 = trans_matrix(a[1], alpha[1], d[1], theta2 + np.pi/2)
    T23 = trans_matrix(a[2], alpha[2], d[2], theta3)
    T34 = trans_matrix(a[3], alpha[3], d[3], theta4)
    T45 = trans_matrix(a[4], alpha[4], d[4], theta5)
    T56 = trans_matrix(a[5], alpha[5], d[5], theta6)
    T06 = np.matmul(T01, np.matmul(T12, np.matmul(T23, np.matmul(T34, np.matmul(T45, T56)))))
    P = np.array([[0, 0, 0, 1]]).T
    P_1 = np.matmul(T06, P)
    P_1[P_1 < 1e-10] = 0
    return P_1

def truncate(matrix, digits):
    for coef in sympy.preorder_traversal(matrix):
        if isinstance(coef, sympy.Float):
            matrix = matrix.subs(coef, round(coef, digits))
    return matrix

def print_in_row(matrix):
    i, j = matrix.shape
    print(i, j)
    for row in range(i):
        for col in range(j):
            print("Element (%d, %d): "% (row, col), matrix[row, col])