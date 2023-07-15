#include <stdio.h> 
#include <iostream>
#include <random>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <fstream>
#include <vector>

//std::random_device rd; 
//unsigned seed = std::chrono::system_clock::now().time_since_epoch().count(); //Set seed as current time
unsigned seed = 0;
std::mt19937 generator(seed);
//std::default_random_engine generator(rd());


//==============File Name===============//
const std::string cities_file_name = "tsp_circle.txt";
// const std::string cities_file_name = "tsp_test.txt"; //For test
// const std::string cities_file_name = "tsp_circle.txt"; //For test
const std::string best_individual_filename = "best_individual.txt";
const std::string learning_curves_filename = "learning_curves.txt";
//==============EA PARAMETERS===============//
const int population_size = 300;
const double selection_pressure = 0.5;
const double tournament_size = 2; //Binary tournament selection
const double crossover_rate = 0.5;
const double mutate_rate = 0.1;
const int generation = 2001;
