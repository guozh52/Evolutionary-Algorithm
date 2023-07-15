"""
Symbolic Regression Plot

Created on October 15, 2022

@author: Zihan Guo
"""

import numpy as np
import math
import heapq
import matplotlib.pyplot as plt
import time
import imageio



def plot_learning_curves(filename):
    cor_x, cor_y = [], []  # Creating 2 lists to store x-y coordinate separately

    with open(filename, 'r+', encoding='utf-8') as f:
        for line in f:
            row = line.split(',')
            cor_x.append(float(row[0]))  # Convert strings to float numbers, and store in the list
            cor_y.append(float(row[1]))

    plt.style.use('ggplot')
    figure1, ax1 = plt.subplots()
    ax1.plot(cor_x, cor_y, linewidth=1, color='#2878B5')
    ax1.set_title('Learning Curves')
    ax1.set_xlabel('Evaluations')
    ax1.set_ylabel('Mean Square Error')
    ax1.set_xscale('log')
    plt.savefig('GP Learning Curve.png', dpi=300)
    plt.show()


def plot_learning_curves_together(filename1, filename2, filename3):
    cor_x1, cor_y1 = [], []
    cor_x2, cor_y2 = [], []
    cor_x3, cor_y3 = [], []

    with open(filename1, 'r+', encoding='utf-8') as f:
        for line in f:
            row = line.split(',')
            cor_x1.append(float(row[0]))  # Convert strings to float numbers, and store in the list
            cor_y1.append(float(row[1]))
    with open(filename2, 'r+', encoding='utf-8') as f:
        for line in f:
            row = line.split(',')
            cor_x2.append(float(row[0]))  # Convert strings to float numbers, and store in the list
            cor_y2.append(float(row[1]))
    with open(filename3, 'r+', encoding='utf-8') as f:
        for line in f:
            row = line.split(',')
            cor_x3.append(float(row[0]))  # Convert strings to float numbers, and store in the list
            cor_y3.append(float(row[1]))

    plt.style.use('ggplot')
    figure1, ax1 = plt.subplots()
    curves1 = ax1.plot(cor_x3, cor_y3, linewidth=1, label='GA', color='#C82423')
    curves2 = ax1.plot(cor_x2, cor_y2, linewidth=1, label='Hill Climber', color='#2878B5')
    curves3 = ax1.plot(cor_x1, cor_y1, linewidth=1, label='Random Search', color='#C497B2')
    ax1.set_title('Learning Curves')
    ax1.set_xlabel('Evaluations')
    ax1.set_ylabel('Mean Square Error')
    ax1.set_xscale('log')

    GAe_x, Hille_x, Rande_x = [10, 100, 1000, 10000], [10, 100, 1000, 10000], [10, 100, 1000, 10000]
    GAe_y, Hille_y, Rande_y = [0.356086, 0.208383, 0.123989, 0.123501], [0.649848, 0.387478, 0.351266, 0.344226], [
        0.498784, 0.37182, 0.304284, 0.176915]
    GAe_yeer, Hille_yeer, Rande_yeer = [0.035, 0.039, 0.043, 0.039], [0.11, 0.077, 0.048, 0.049], [0.051, 0.043, 0.041,
                                                                                                   0.037]

    plt.errorbar(GAe_x, GAe_y, yerr=GAe_yeer, color='#C82423', fmt='o', capsize=4, markersize=1, elinewidth=0.5)
    plt.errorbar(Hille_x, Hille_y, yerr=Hille_yeer, color='#2878B5', fmt='o', capsize=4, markersize=1, elinewidth=0.5)
    plt.errorbar(Rande_x, Rande_y, yerr=Rande_yeer, color='#C497B2', fmt='o', capsize=4, markersize=1, elinewidth=0.5)

    plt.ylim((0, 1.2))
    plt.legend(loc='best')
    plt.savefig('Learning Curves.png', dpi=300)
    plt.show()


def plot_function(filename1, filename2):
    cor_x, cor_y = [], []  # Creating 2 lists to store x-y coordinate separately
    cor_x_origin, cor_y_origin = [], []  # Origin data points' coordinates

    with open(filename1, 'r+', encoding='utf-8') as f:
        for line in f:
            row = line.split(',')
            cor_x.append(float(row[0]))  # Convert strings to float numbers, and store in the list
            cor_y.append(float(row[1]))

    with open(filename2, 'r+', encoding='utf-8') as f:
        for line in f:
            row = line.split(',')
            cor_x_origin.append(float(row[0]))  # Convert strings to float numbers, and store in the list
            cor_y_origin.append(float(row[1]))

    plt.style.use('ggplot')
    figure2, ax2 = plt.subplots()
    ax2.plot(cor_x, cor_y, linewidth=2, label='Fit Function', color='#C82423', linestyle='--')
    ax2.plot(cor_x_origin, cor_y_origin, linewidth=1, label='Origin Data', color='#2878B5')
    ax2.set_title('Function Plot')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    plt.legend(loc='best')
    plt.savefig('GP_function.png', dpi=300)
    plt.show()


def plot_convergence(filename1, filename2, filename3):
    cor_x1, cor_y1 = [], []
    cor_x2, cor_y2 = [], []
    cor_x3, cor_y3 = [], []

    with open(filename1, 'r+', encoding='utf-8') as f:
        for line in f:
            row = line.split(',')
            cor_x1.append(float(row[0]))  # Convert strings to float numbers, and store in the list
            cor_y1.append(1 - float(row[1]))
    with open(filename2, 'r+', encoding='utf-8') as f:
        for line in f:
            row = line.split(',')
            cor_x2.append(float(row[0]))  # Convert strings to float numbers, and store in the list
            cor_y2.append(1 - float(row[1]))
    with open(filename3, 'r+', encoding='utf-8') as f:
        for line in f:
            row = line.split(',')
            cor_x3.append(float(row[0]))  # Convert strings to float numbers, and store in the list
            cor_y3.append(1 - float(row[1]))

    plt.style.use('ggplot')
    figure1, ax1 = plt.subplots()
    curves1 = ax1.plot(cor_x3, cor_y3, linewidth=1, label='GA', color='#C82423')
    curves2 = ax1.plot(cor_x2, cor_y2, linewidth=1, label='Hill Climber', color='#2878B5')
    curves3 = ax1.plot(cor_x1, cor_y1, linewidth=1, label='Random Search', color='#C497B2')
    ax1.set_title('Convergence Plot')
    ax1.set_xlabel('Evaluations')
    ax1.set_ylabel('percentage of fitness < 1')
    ax1.set_xscale('log')
    plt.ylim((0, 1))
    plt.savefig('Convergence Plot.png', dpi=300)
    plt.show()


def plot_movie(filename1, filename2, filename_out):
    cor_x, cor_y = [], []  # Creating 2 lists to store x-y coordinate separately
    cor_x_origin, cor_y_origin = [], []  # Origin data points' coordinates

    with open(filename1, 'r+', encoding='utf-8') as f:
        for line in f:
            row = line.split(',')
            cor_x.append(float(row[0]))  # Convert strings to float numbers, and store in the list
            cor_y.append(float(row[1]))

    with open(filename2, 'r+', encoding='utf-8') as f:
        for line in f:
            row = line.split(',')
            cor_x_origin.append(float(row[0]))  # Convert strings to float numbers, and store in the list
            cor_y_origin.append(float(row[1]))

    plt.style.use('ggplot')
    figure2, ax2 = plt.subplots()
    ax2.plot(cor_x, cor_y, linewidth=1.5, label='Fit Function', color='#C82423', linestyle='--')
    ax2.plot(cor_x_origin, cor_y_origin, linewidth=1, label='Origin Data', color='#2878B5')
    ax2.set_title('Function Plot')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    plt.legend(loc='best')
    plt.savefig(filename_out, dpi=300)






origin_data = \
    'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\result data\\' \
    'origin_data.txt'
data2022_Bronze = \
    'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\result data\\' \
    'data2022_Bronze.txt'
random_learningcurves = \
    'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\result data\\python data\\' \
    'RandomSearch_LearningCurves.txt'
randomosearch_function = \
    'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\result data\\' \
    'RandomSearch_function.txt'
hillclimber_learningcurves = \
    'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\result data\\python data\\' \
    'HillClimber_LearningCurves.txt'
hillclimber_function = \
    'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\result data\\python data\\' \
    'HillClimber_function_simple.txt'
GP_learningcurves = \
    'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\result data\\' \
    'GP_LearningCurves.txt'
GP_function = \
    'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\result data\\python data\\' \
    'GP_function.txt'

plot_learning_curves(GP_learningcurves)
# plot_function(hillclimber_function, data2022_Bronze)
# plot_learning_curves_together(random_learningcurves, hillclimber_learningcurves, GP_learningcurves)
# plot_convergence(random_learningcurves, hillclimber_learningcurves, GP_learningcurves)


# for i in range(1, 76):
#     filename1 = 'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\movie\\' \
#                 'movie_' + str(i) + '.txt'
#     filename_out = 'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\movie\\image\\' \
#                     + str(i) + '.png'
#
#     plot_movie(filename1, origin_data, filename_out)

# gif_image = []
# for i in range(1, 75):
#     gif_image.append(imageio.imread(
#         'D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\movie\\image\\' + str(i) + '.png'))
# imageio.mimsave('final.gif', gif_image, fps=4)