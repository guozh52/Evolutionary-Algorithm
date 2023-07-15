/*
@author: Zihan Guo
@contact: zg2450@columbia.edu
*/

#include "config.h"

class TSP_EA
{
private:
    struct City
    {
        double x;
        double y;
    };

public:
    std::vector<City> cities;
    std::vector<std::vector<double>> population = std::vector<std::vector<double>>(population_size);
    std::vector<double> pop_fitness = std::vector<double>(population_size);

    void Read_Coordinates(const std::string &file_name)
    {
        // Read cities coordinates from txt fiel in the format of x,y

        std::string line_;
        std::string x_str, y_str;
        std::ifstream file_(file_name); // open the file using "input file stream"

        if (file_.is_open())
        {
            while (getline(file_, line_))
            {
                City city_;
                std::istringstream iss(line_);

                getline(iss, x_str, ','); // Assign the string before the comma to the variable point_x_string
                city_.x = std::stof(x_str);
                getline(iss, y_str);
                city_.y = std::stof(y_str);

                cities.push_back(city_);
            }
            file_.close();
        }
        else
            std::cout << "file is not open"
                      << "\n";
    }

    std::vector<double> Generate_Random_Individual()
    {
        // Generate random individual, which means generate random sequences of cities
        // using Priority Encoding

        std::uniform_real_distribution<double> distribution(0.0, 1.0);
        std::vector<double> individual(cities.size());

        for (int i = 0; i < cities.size(); i++)
        {
            individual[i] = distribution(generator); // Generate priority value of each city
        }

        return individual;
    }

    void Generate_initial_population()
    {
        for (int i = 0; i < population_size; i++)
        {
            population[i] = Generate_Random_Individual();
        }
        for (int i = 0; i < pop_fitness.size(); i++)
        {
            pop_fitness[i] = Get_Fitness(population[i]);
            // std::cout << "Fitness:" << pop_fitness[i];
        }
        std::cout << "Initial population generted." << std::endl;
    }

    double Get_Fitness(const std::vector<double> &individual)
    {
        // Sort the priority value from large to small
        double total_distance = 0.0;
        std::vector<size_t> indices(individual.size());
        std::iota(indices.begin(), indices.end(), 0);

        // Sort the randomValues vector from large to small using a custom comparator
        std::sort(indices.begin(), indices.end(), [&](size_t i, size_t j)
                  { return individual[i] > individual[j]; });

        // Print the sorted values and corresponding indices
        // for (const auto& index : indices) {
        //     std::cout << "Value: " << individual[index] << " | Index: " << index << std::endl;
        // }

        // Calculate length of route
        for (int i = 0; i < individual.size() - 1; i++)
        {
            int city_index_1 = indices[i];
            int city_index_2 = indices[i + 1];

            double diff_x = cities[city_index_1].x - cities[city_index_2].x;
            double diff_y = cities[city_index_1].y - cities[city_index_2].y;

            total_distance += std::sqrt(diff_x * diff_x + diff_y * diff_y);
            // std::cout << total_distance << std::endl;
        }
        int last_city_index = indices[cities.size() - 1];
        int first_city_index = indices[0];
        double diff_x = cities[last_city_index].x - cities[first_city_index].x;
        double diff_y = cities[last_city_index].y - cities[first_city_index].y;
        total_distance += std::sqrt(diff_x * diff_x + diff_y * diff_y);

        // std::cout << "total_distance:" << total_distance << std::endl;
        return total_distance;
    }

    int Roulette_Wheel_Selection()
    {
        std::uniform_real_distribution<double> distribution(0.0, 1.0);

        // for (int i=0; i<pop_fitness.size(); i++){
        //     pop_fitness[i] = Get_Fitness(population[i]);
        //     //std::cout << "Fitness:" << pop_fitness[i];
        // }
        double total_inverse_fitness = 0.0;
        for (const auto &fitness_ : pop_fitness)
        {
            total_inverse_fitness += 1.0 / fitness_;
        }

        double spin_value = distribution(generator) * total_inverse_fitness;
        // std::cout << "total_inverse_fitness:" << total_inverse_fitness << std::endl;
        // std::cout << "spin_value:" << spin_value << std::endl;

        double cumulative_sum = 0.0;
        int selected_individual_idx = -1;
        for (int i = 0; i < population_size; i++)
        {
            cumulative_sum += 1.0 / pop_fitness[i];
            if (cumulative_sum >= spin_value)
            {
                selected_individual_idx = i;
                break;
            }
        }
        // std::cout << "selected individual:" << selected_individual_idx << std::endl;
        return selected_individual_idx;
    }

    int Tournament_Selection()
    {
        std::uniform_int_distribution<> competitor_distribution(0, population_size - 1);

        int winner_idx;
        double best_fitness = std::numeric_limits<double>::max();

        for (int i = 0; i < tournament_size; i++)
        {
            int individual_idx = competitor_distribution(generator);
            double fitness = pop_fitness[individual_idx];
            if (fitness < best_fitness)
            {
                best_fitness = fitness;
                winner_idx = individual_idx;
            }
        }
        // std::cout << "Winner:" << winner_idx << std::endl;

        return winner_idx;
    }

    std::vector<double> Crossover(std::vector<std::vector<double>> &pop, int parent_1_idx, int generation_)
    {
        //
        std::uniform_real_distribution<double> cross_rate_distribution(0.0, 1.0);
        std::uniform_int_distribution<> parent_2_distribution(0, pop.size() - 1);
        std::uniform_int_distribution<> crossover_locs_distribution(0, cities.size() - 1);

        if (cross_rate_distribution(generator) < crossover_rate)
        {
            std::vector<double> parent_1 = pop[parent_1_idx];
            std::vector<double> parent_2;
            int parent_2_idx;
            if (generation_ % 200 == 0){
                //Every 1000 generation introduce new randomly new individual to do crossover
                parent_2 = Generate_Random_Individual();
            }
            else{
                // Randomly select parent_2 to do crossover
                parent_2_idx = parent_2_distribution(generator);
                
                parent_2 = pop[parent_2_idx];
            }
            std::vector<double> child = parent_1;

            // Randomly generate two crossover locations of genes
            int start_point = crossover_locs_distribution(generator);
            int end_point = crossover_locs_distribution(generator);
            // Ensure start point is before the end point
            if (start_point > end_point)
            {
                std::swap(start_point, end_point);
            }
            // Crossover
            for (int gene_loc = start_point; gene_loc < end_point; gene_loc++)
            {
                child[gene_loc] = parent_2[gene_loc];
            }
            //Only keep the child if child is better than its parent
            if (Get_Fitness(child) < pop_fitness[parent_1_idx]){
                return child;
            }
            else{
                return pop[parent_1_idx];
            }

        }
        // Return parent itself if no crossover was done
        return pop[parent_1_idx];
    }

    std::vector<double> Mutate(std::vector<double> &child)
    {
        std::uniform_real_distribution<double> mutate_rate_distribution(0.0, 1.0);
        std::uniform_int_distribution<> mutate_locs_distribution(0, cities.size() - 1);
        std::uniform_int_distribution<> nunmber_of_locs_distribution(1, 3);
        std::uniform_real_distribution<double> distribution(0.0, 1.0);

        if (mutate_rate_distribution(generator) < mutate_rate){
            if (mutate_rate_distribution(generator) < 0.2){
                for (int num_locs=0; num_locs<nunmber_of_locs_distribution(generator); num_locs++){
                        int mutate_locs = mutate_locs_distribution(generator);
                        child[mutate_locs] = distribution(generator);
                    }
            }
            else{
                if (mutate_rate_distribution(generator) < 0.5){
                    //Randomly swapping two cities
                    int locs_1 = mutate_locs_distribution(generator);
                    int locs_2 = mutate_locs_distribution(generator);
                    double temp_value = child[locs_1];
                    child[locs_1] = child[locs_2];
                    child[locs_2] = temp_value;
                }
                else{
                    //Randomly swapping three cities
                    int locs_1 = mutate_locs_distribution(generator);
                    int locs_2 = mutate_locs_distribution(generator);
                    while (locs_2 == locs_1){
                        locs_2 = mutate_locs_distribution(generator);
                    }
                    int locs_3 = mutate_locs_distribution(generator);
                    while (locs_3 == locs_2){
                        locs_3 = mutate_locs_distribution(generator);
                    }
                    double temp_value = child[locs_1];
                    child[locs_1] = child[locs_2];
                    child[locs_2] = child[locs_3];
                    child[locs_3] = temp_value;
                }

            }
        }

        return child;
    }

    int Evolve(int generation_)
    {
        // std::vector<int> pop_idx_selected = std::vector<int>(population_size);
        std::vector<std::vector<double>> next_generation_pop = std::vector<std::vector<double>>(population_size);
        std::vector<double> child;

        // Select surviving individuals from previous generation population
        for (int i = 0; i < population_size; i++)
        {
            // int pop_idx_selected = Roulette_Wheel_Selection();
            int pop_idx_selected = Tournament_Selection();
            next_generation_pop[i] = population[pop_idx_selected];
        }
        // Crossover and Mutation
        for (int parent_idx = 0; parent_idx < population_size; parent_idx++)
        {
            child = Crossover(next_generation_pop, parent_idx, generation_);
            child = Mutate(child);

            double child_fit = Get_Fitness(child);
            next_generation_pop[parent_idx] = child;
            pop_fitness[parent_idx] = child_fit;
        }
        population = next_generation_pop;

        int best_individual_idx = 0;
        double min_distance = 10000.0;
        for (int i = 0; i < pop_fitness.size(); i++)
        {
            if (pop_fitness[i] < min_distance)
            {
                min_distance = pop_fitness[i];
                best_individual_idx = i;
            }
        }
        return best_individual_idx;
    }
};


class Output_Data
{
    public:
        std::vector<double> learning_curves_fitness = std::vector<double>(generation);

        void Output_best_individual(std::vector<double> &individual, int generation, double fitness)
        {
            std::ofstream o_best_idv_file(best_individual_filename, std::ios::app); // Open file in append mode

            if (!o_best_idv_file)
            {
                std::cerr << "Error opening output file:" << best_individual_filename << std::endl;
            }

            std::vector<size_t> indices(individual.size());
            std::iota(indices.begin(), indices.end(), 0);
            // Sort the randomValues vector from large to small using a custom comparator
            std::sort(indices.begin(), indices.end(), [&](size_t i, size_t j)
                    { return individual[i] > individual[j]; });
            // Output in the format like:
            // generation, city1, city2, city3,..., cityN
            o_best_idv_file << generation << "," << fitness << ",";
            for (const auto &city_order : indices)
            {
                o_best_idv_file << city_order << ",";
            }
            o_best_idv_file << std::endl;

            o_best_idv_file.close();
        }

        void Output_Learning_Curves(){
            std::ofstream o_learn_cur_file(learning_curves_filename, std::ios::app); // Open file in append mode

            if (!o_learn_cur_file)
            {
                std::cerr << "Error opening output file:" << learning_curves_filename << std::endl;
            }
            for (const auto &fitness : learning_curves_fitness){
                o_learn_cur_file << fitness << std::endl;
            }
            o_learn_cur_file.close();
    }
};


int main()
{
    TSP_EA tsp_ea;
    Output_Data ouput_data;

    tsp_ea.Read_Coordinates(cities_file_name);

    tsp_ea.Generate_initial_population();
    
    double best_idx = 0;

    for (int gen_ = 0; gen_ < generation; gen_++)
    {
        best_idx = tsp_ea.Evolve(gen_); 

        if (gen_ % 1 == 0)
        {
            std::cout << "Generation:" << gen_ << "   Shortest path:" << tsp_ea.pop_fitness[best_idx] << std::endl;
            for (int i=0; i<10; i++)
            {
                std::cout << tsp_ea.pop_fitness[i] << ",";
            }
            std::cout << std::endl;
            ouput_data.Output_best_individual(tsp_ea.population[best_idx], gen_, tsp_ea.pop_fitness[best_idx]);
        }

        if (gen_ > 0){
            if (tsp_ea.pop_fitness[best_idx] < ouput_data.learning_curves_fitness[gen_ - 1]){
                ouput_data.learning_curves_fitness[gen_] = tsp_ea.pop_fitness[best_idx];
            }
            else{
                ouput_data.learning_curves_fitness[gen_] = ouput_data.learning_curves_fitness[gen_ - 1];
            }
        }
        else{
            ouput_data.learning_curves_fitness[gen_] = tsp_ea.pop_fitness[best_idx];
        }
    }
    ouput_data.Output_Learning_Curves();

    return 0;
}
