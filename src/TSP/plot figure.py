import numpy as np
import matplotlib.pyplot as plt
import imageio

class Plot_Figure():
    def __init__(self):
        self.cities_x = np.empty((0,))
        self.cities_y = np.empty((0,))
        
    def Read_Coordinates(self, filename):
        with open(filename, 'r+', encoding='utf-8') as f:

            for line in f:
                row = line.split(',')
                self.cities_x = np.append(self.cities_x, float(row[0]))  
                self.cities_y = np.append(self.cities_y, float(row[1]))  

    def Plot_Route(self, filename):
        plt.style.use('ggplot')
        figure1, ax1 = plt.subplots()

        city_order = []
        plot_x = []
        plot_y = []
        best_fit = 10000
        index = 0

        with open(filename, 'r+', encoding='utf-8') as f:

            for line_ in f:
                plot_x.clear()
                plot_y.clear()
                city_order.clear()

                row = line_.split(',')
                generation_ = int(row[0])
                fitness = float(row[1])
                
                if (fitness < best_fit):
                    filename_out = "image\\temp\\" + str(index) + '.png'
                    index += 1

                    best_fit = fitness
                    for i in range(2, len(row) - 1):
                        city_order.append(int(row[i]))
                        # print(int(row[i]))
                    for i in range(len(city_order)):
                        plot_x.append(self.cities_x[city_order[i]])
                        plot_y.append(self.cities_y[city_order[i]])
                    plot_x.append(self.cities_x[city_order[0]])
                    plot_y.append(self.cities_y[city_order[0]])

                    ax1.plot(plot_x, plot_y, linewidth = 0.5, color = '#2878B5')
                    ax1.plot(plot_x, plot_y, "o", markersize = 1, color = 'black')
                    ax1.set_title(f'Generation:{generation_}  Length:{fitness}')
                    ax1.set_aspect('equal')
                    plt.show(block=False)
                    plt.pause(0.05)
                    plt.savefig(filename_out, dpi=200)
                    ax1.cla()
            
    def Creat_GIF(self):
        gif_image = []
        of_name = "TSP_circle.gif"
        print("Done")
        for index in range(0, 60):
            print(index)
            gif_image.append(imageio.imread(
                'image\\temp\\' + str(index) + '.png'))
        imageio.mimsave(of_name, gif_image, duration=200)


    def Plot_Learning_Curves(self, filename):
        fitness_list = []
        plt.style.use('ggplot')
        figure1, ax2 = plt.subplots()

        with open(filename, 'r+', encoding='utf-8') as f:
            for line_ in f:
                fitness = float(line_)
                fitness_list.append(-fitness)
        ax2.semilogx(fitness_list)
        ax2.set_title("Learning Curves")
        ax2.set_xlabel("Generation")
        ax2.set_ylabel("Fitness")

        plt.show()

cities_filename = "tsp_circle.txt"
best_individual_filename = "best_individual.txt"
learning_curves_name = "learning_curves.txt"
gif_image_name = "/image/temp"

plot_ = Plot_Figure()
#plot_.Read_Coordinates(cities_filename)
#plot_.Plot_Route(best_individual_filename)
#plot_.Plot_Learning_Curves(learning_curves_name)
plot_.Creat_GIF()