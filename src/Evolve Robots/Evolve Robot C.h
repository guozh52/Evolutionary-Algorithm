#pragma once

#include<iostream>
#include<vector>
#include <string>
#include <fstream>
#include <sstream>
#include<iomanip>
#include<random>
#include<numeric>
#include<algorithm>


double GLOBAL_TIME = 0;

std::random_device rd;
std::default_random_engine eng(rd());

class Robot
{
private:
    class Masses
    {
    public:
        double M_mass;
        double M_position[3];
        double M_velocity[3];
        double M_acceleration[3];
    };

    class Springs
    {
    public:
        double S_K;             //Spring rate
        double S_L0;            //Spring rest length
        double S_inital;        //Spring initial length
        int S_index1, S_index2; //The index of two masses it connects
        int S_type;             //Spring type
        double b = 0;           //Parameters of funcioton of active springs
        double c = 0;           //Parameters of funcioton of active springs
    };

    struct Robot_Parameter
    {
        double K_hard = 20000;                                              //Type 1
        double K_soft = 1000;                                               //Type 2
        double R_side_length;             
        double R_fitness;                                                   //Fitness

        std::vector<double> Foot_pos_err = std::vector<double>(4);                //4 legs length parameter
    };  

public:
    std::vector<Masses> C_Masses = std::vector<Masses>(12);
    std::vector<Springs> C_Springs = std::vector<Springs>(44);
    Robot_Parameter R_Parameters;
    double R_Fitness;
    int R_Good_Flag = 1;     //If R_Good_Flag = 0, then this robot need to be regenerated, because it once fall.

    void Calculate_S_L0_expand(int spring_index)                            //Type 3
    {
        C_Springs[spring_index].S_L0 = C_Springs[spring_index].S_inital + C_Springs[spring_index].b * sin(C_Springs[spring_index].c * GLOBAL_TIME);
    }

    void Calculate_S_L0_contract(int spring_index)                          //Type 4
    {
        C_Springs[spring_index].S_L0 = C_Springs[spring_index].S_inital - C_Springs[spring_index].b * sin(C_Springs[spring_index].c * GLOBAL_TIME);
    }
};

struct Child
{
    Robot child_1;
    Robot child_2;
};


//----------Simulator parameters-------------
const double GRAVITY = -9.81;
const double DAMPING = 0.999;
const double FRICTION_MU_S = 1;     //Static friction
const double FRICTION_MU_K = 0.8;   //Kinetic friction
const double K_GROUND = 100000;
const double dt = 0.0004;           //Time step
//----------Robots parameters-----------------
const double MASS = 0.5;
const double SPRING_RATE = 5000;
const std::vector<double> INITIAL_POSITION = { 0, 0, 0.1 };
//----------GP parameters---------------------
const int POPULATION_SIZE = 10;
const int POPULATION_AGE_LAYER = 10;
const float SELECTION_PRESSURE = 0.3;
const float PM = 0.05;                  //Probability of mutataion
const int GENERATION = 1000;
int EVALUATION_TIMES = 0;
//---------Image------------------------------
std::vector<double> FITNESS_IMAGE = { 0 };
std::vector<int> EVALUATION_TIMES_IMAGE = { 0 };
std::vector<std::vector<double>> DOT_PLOT_FITNESS = std::vector<std::vector<double>>(GENERATION);
std::vector<int> DOT_PLOT_GENERATION;
//----------File name-------------------------
const std::string of_position_filename1 = \
"D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\C\\Evolve Robot C\\Data\\txt\\robot_position1.txt";
const std::string of_position_filename2 = \
"D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\C\\Evolve Robot C\\Data\\txt\\learning_curves.txt";
const std::string of_position_filename3 = \
"D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\C\\Evolve Robot C\\Data\\txt\\dot_plot.txt";
