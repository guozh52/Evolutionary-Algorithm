"""
Bouncing Cube Plot

Created on October 30, 2022

@author: Zihan Guo
"""

import matplotlib.pyplot as plt
import imageio
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def Plot_Cube(filename):
    cor_x, cor_y, cor_z = [], [], []  # Creating 2 lists to store x-y coordinate separately

    plt.ion()
    plt.style.use('ggplot')
    figure1 = plt.figure()
    ax1 = plt.axes(projection='3d')

    with open(filename, 'r+', encoding='utf-8') as f:
        index = 1
        for line in f:
            cor_x.clear()
            cor_y.clear()
            cor_z.clear()

            filename_out = 'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\Bouncing Cube\\' \
                           'data\\movie\\Breathing Cube\\pictures' + str(index) + '.png'

            plt.cla()
            row = line.split(',')
            for i in range(0, 23, 3):
                cor_x.append(float(row[i]))  # Convert strings to float numbers, and store in the list
                cor_y.append(float(row[i + 1]))
                cor_z.append(float(row[i + 2]))

            a, b, c, d, e, f, g, h = zip(cor_x, cor_y, [i * 0 for i in cor_z])
            cube_pos = zip(a, b, c, d, a,
                           e, h, d, a,
                           b, f, e,
                           h, g, f,
                           b, c, g,
                           c, d)

            ax1.plot3D(*cube_pos, 'darkgrey', marker='o', markersize=3)
            # ax1.set(xlim=(-0.4, 0.4), ylim=(-0.4, 0.4), zlim=(0, 0.6))

            a, b, c, d, e, f, g, h = zip(cor_x, cor_y, cor_z)
            cube_pos_shadow = zip(a, b, c, d, a,
                                  e, h, d, a,
                                  b, f, e,
                                  h, g, f,
                                  b, c, g,
                                  c, d)

            ax1.plot3D(*cube_pos_shadow, marker='o', markersize=4)
            ax1.set(xlim=(-0.4, 0.4), ylim=(-0.4, 0.4), zlim=(0, 0.6),
                    title='Bouncing Cube', xlabel='X', ylabel='Y')

            plt.show()
            plt.pause(0.01)
            plt.savefig(filename_out, dpi=100)
            index += 1
            print(index)


def Creat_GIF(bounce_cube_mov_pos1):
    gif_image = []
    for i in range(1, 200):
        gif_image.append(imageio.imread(
            'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\Bouncing Cube\\' \
            'data\\movie\\Breathing Cube\\pictures' + str(i) + '.png'))
    imageio.mimsave(bounce_cube_mov_pos1, gif_image, fps=20)


def plot_position_curves(filename):
    cor_x, cor_y, cor_z, global_time = [], [], [], []  # Creating 2 lists to store x-y coordinate separately

    with open(filename, 'r+', encoding='utf-8') as f:
        for line in f:
            row = line.split(',')
            cor_x.append(float(row[0]))  # Convert strings to float numbers, and store in the list
            cor_y.append(float(row[1]))
            cor_z.append(float(row[2]))
            global_time.append(float(row[3]))

    plt.style.use('ggplot')
    figure1, ax1 = plt.subplots()
    ax1.plot(global_time, cor_x, linewidth=0.5, label='ax')
    ax1.plot(global_time, cor_y, linewidth=0.5, label='ay')
    ax1.plot(global_time, cor_z, linewidth=0.5, label='az')
    ax1.set_title('Acceleration of one cube vertex')
    ax1.set_xlabel('Global time')
    ax1.set_ylabel('XYZ Acceleration')
    plt.legend(loc='best')
    plt.savefig('acceleration.png', dpi=300)
    plt.show()


def plot_energy_curves(filename):
    cor_1, cor_2, cor_3, cor_4, cor_5, global_time = [], [], [], [], [], []  # Creating 2 lists to store x-y coordinate separately

    with open(filename, 'r+', encoding='utf-8') as f:
        for line in f:
            row = line.split(',')
            cor_1.append(float(row[0]))  # Convert strings to float numbers, and store in the list
            cor_2.append(float(row[1]))
            cor_3.append(float(row[2]))
            cor_4.append(float(row[3]))
            cor_5.append(float(row[4]))
            global_time.append(float(row[5]))

    plt.style.use('ggplot')
    figure1, ax1 = plt.subplots()
    ax1.plot(global_time, cor_1, linewidth=0.5, label='Spring Energy')
    ax1.plot(global_time, cor_2, linewidth=1, label='Ground Energy')
    ax1.plot(global_time, cor_3, linewidth=0.5, label='Masses Energy')
    ax1.plot(global_time, cor_4, linewidth=1, label='Gravity Energy')
    ax1.plot(global_time, cor_5, linewidth=1, label='Total Energy')
    ax1.set_title('Total Energy')
    ax1.set_xlabel('Global time')
    ax1.set_ylabel('Energy')
    plt.legend(loc=1)
    plt.savefig('Energy.png', dpi=300)
    plt.show()


cube_position = "D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\Bouncing Cube\\data\\Cube_Position.txt"
bounce_cube_mov_pos = 'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\Bouncing ' \
                      'Cube\\data\\movie\\Bouncing-Cube.gif '
breathing_cube_mov_pos = 'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\Bouncing ' \
                         'Cube\\data\\movie\\Breathing Cube.gif '
bounce_cube_pos_curves = 'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\Bouncing ' \
                         'Cube\\data\\Cube_Velocity_curves.txt '
bounce_cube_energy_curves = 'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\Bouncing ' \
                            'Cube\\data\\Energy.txt '

#Plot_Cube(cube_position)
Creat_GIF(breathing_cube_mov_pos)
#plot_position_curves(bounce_cube_pos_curves)
# plot_energy_curves(bounce_cube_energy_curves)
