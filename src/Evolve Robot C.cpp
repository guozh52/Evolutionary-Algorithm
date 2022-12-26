/*
Evolving Robot
Phase C

Created on 11/26/2022

@Author: Zihan Guo
*/


#include "Evolve robot C.h"


void Generate_Robot(Robot& initial_robot)
{
    double initial_x = INITIAL_POSITION[0];
    double initial_y = INITIAL_POSITION[1];
    /*
    Generate 12 mass points in the robot
    All masses' initial velocity and acceleration are zero
    */
    //Central cube
    std::uniform_int_distribution<int> side_length_operator(500, 2000);
    initial_robot.R_Parameters.R_side_length = side_length_operator(eng) / (double)10000;
    double side_length = initial_robot.R_Parameters.R_side_length;
    double initial_z = initial_robot.R_Parameters.R_side_length;
    double mass = pow(side_length / 0.1, 2) * MASS;
        
    initial_robot.C_Masses[0] = { mass, {initial_x, initial_y, initial_z}, {0, 0, 0}, {0, 0, 0} };
    initial_robot.C_Masses[1] = { mass, {initial_x + side_length, initial_y, initial_z}, {0, 0, 0}, {0, 0, 0} };
    initial_robot.C_Masses[2] = { mass, {initial_x + side_length, initial_y + side_length, initial_z}, {0, 0, 0}, {0, 0, 0} };
    initial_robot.C_Masses[3] = { mass, {initial_x, initial_y + side_length, initial_z}, {0, 0, 0}, {0, 0, 0} };
    initial_robot.C_Masses[4] = { mass, {initial_x, initial_y, initial_z + side_length}, {0, 0, 0}, {0, 0, 0} };
    initial_robot.C_Masses[5] = { mass, {initial_x + side_length, initial_y, initial_z + side_length}, {0, 0, 0}, {0, 0, 0} };
    initial_robot.C_Masses[6] = { mass, {initial_x + side_length, initial_y + side_length, initial_z + side_length}, {0, 0, 0}, {0, 0, 0} };
    initial_robot.C_Masses[7] = { mass, {initial_x, initial_y + side_length, initial_z + side_length}, {0, 0, 0}, {0, 0, 0} };
    //Four legs
    std::uniform_int_distribution<int> leg_position_operator(-1500,1500);

    for (int i = 0; i < 4; i++)
    {
        initial_robot.R_Parameters.Foot_pos_err[i] = leg_position_operator(eng) / (double)10000;
    }

    initial_robot.C_Masses[8] = { mass,
                                    {initial_x + side_length / 2,
                                    initial_y - side_length + initial_robot.R_Parameters.Foot_pos_err[0],
                                    initial_z - side_length},
                                    {0, 0, 0}, {0, 0, 0} };
    initial_robot.C_Masses[9] = { mass,
                                    {initial_x + side_length * 2 + initial_robot.R_Parameters.Foot_pos_err[1],
                                    initial_y + side_length / 2,
                                    initial_z - side_length},
                                    {0, 0, 0}, {0, 0, 0} };
    initial_robot.C_Masses[10] = { mass,
                                    {initial_x + side_length / 2,
                                    initial_y + side_length * 2 + initial_robot.R_Parameters.Foot_pos_err[2],
                                    initial_z - side_length},
                                    {0, 0, 0}, {0, 0, 0} };
    initial_robot.C_Masses[11] = { mass,
                                    {initial_x - side_length + initial_robot.R_Parameters.Foot_pos_err[3],
                                    initial_y + side_length / 2,
                                    initial_z - side_length},
                                    {0, 0, 0}, {0, 0, 0} };

    /*
    Generate 42 springs in the robot
    Each spring connects 2 masses
    */
    double spring_length1 = side_length;
    double spring_length2 = sqrt(pow(side_length, 2) + pow(side_length, 2));
    double spring_length3 = sqrt(pow(side_length, 2) + pow(side_length, 2) + pow(side_length, 2));
    double spring_length4 = 0;
    double spring_length5 = 0;

    //bottom
    initial_robot.C_Springs[0] = { SPRING_RATE, spring_length1, spring_length1,0, 1 };
    initial_robot.C_Springs[1] = { SPRING_RATE, spring_length1, spring_length1,1, 2 };
    initial_robot.C_Springs[2] = { SPRING_RATE, spring_length1, spring_length1,2, 3 };
    initial_robot.C_Springs[3] = { SPRING_RATE, spring_length1, spring_length1,3, 0 };
    //top
    initial_robot.C_Springs[4] = { SPRING_RATE, spring_length1, spring_length1,4, 5 };
    initial_robot.C_Springs[5] = { SPRING_RATE, spring_length1, spring_length1,5, 6 };
    initial_robot.C_Springs[6] = { SPRING_RATE, spring_length1, spring_length1,6, 7 };
    initial_robot.C_Springs[7] = { SPRING_RATE, spring_length1, spring_length1,7, 4 };
    //side
    initial_robot.C_Springs[8] = { SPRING_RATE, spring_length1, spring_length1, 0, 4 };
    initial_robot.C_Springs[9] = { SPRING_RATE, spring_length1, spring_length1, 1, 5 };
    initial_robot.C_Springs[10] = { SPRING_RATE, spring_length1, spring_length1,2, 6 };
    initial_robot.C_Springs[11] = { SPRING_RATE, spring_length1, spring_length1,3, 7 };
    //bottom diagonal
    initial_robot.C_Springs[12] = { SPRING_RATE, spring_length2, spring_length2,4, 6 };
    initial_robot.C_Springs[13] = { SPRING_RATE, spring_length2, spring_length2,5, 7 };
    //top digonal
    initial_robot.C_Springs[14] = { SPRING_RATE, spring_length2, spring_length2,0, 2 };
    initial_robot.C_Springs[15] = { SPRING_RATE, spring_length2, spring_length2,1, 3 };
    //front dignal
    initial_robot.C_Springs[16] = { SPRING_RATE, spring_length2, spring_length2,0, 5 };
    initial_robot.C_Springs[17] = { SPRING_RATE, spring_length2, spring_length2,1, 4 };
    //back dignal
    initial_robot.C_Springs[18] = { SPRING_RATE, spring_length2, spring_length2,3, 6 };
    initial_robot.C_Springs[19] = { SPRING_RATE, spring_length2, spring_length2,2, 7 };
    //left dignal
    initial_robot.C_Springs[20] = { SPRING_RATE, spring_length2, spring_length2,0, 7 };
    initial_robot.C_Springs[21] = { SPRING_RATE, spring_length2, spring_length2,3, 4 };
    //right dignal
    initial_robot.C_Springs[22] = { SPRING_RATE, spring_length2, spring_length2,1, 6 };
    initial_robot.C_Springs[23] = { SPRING_RATE, spring_length2, spring_length2,2, 5 };
    //inside dignal
    initial_robot.C_Springs[24] = { SPRING_RATE, spring_length3, spring_length3,0, 6 };
    initial_robot.C_Springs[25] = { SPRING_RATE, spring_length3, spring_length3,1, 7 };
    initial_robot.C_Springs[26] = { SPRING_RATE, spring_length3, spring_length3,2, 4 };
    initial_robot.C_Springs[27] = { SPRING_RATE, spring_length3, spring_length3,3, 5 };
    //front leg
    initial_robot.C_Springs[28] = { SPRING_RATE, spring_length4, spring_length4,0, 8 };
    initial_robot.C_Springs[29] = { SPRING_RATE, spring_length4, spring_length4,1, 8 };
    initial_robot.C_Springs[30] = { SPRING_RATE, spring_length5, spring_length5,4, 8 };
    initial_robot.C_Springs[31] = { SPRING_RATE, spring_length5, spring_length5,5, 8 };
    //right leg
    initial_robot.C_Springs[32] = { SPRING_RATE, spring_length4, spring_length4,1, 9 };
    initial_robot.C_Springs[33] = { SPRING_RATE, spring_length4, spring_length4,2, 9 };
    initial_robot.C_Springs[34] = { SPRING_RATE, spring_length5, spring_length5,5, 9 };
    initial_robot.C_Springs[35] = { SPRING_RATE, spring_length5, spring_length5,6, 9 };
    //back leg
    initial_robot.C_Springs[36] = { SPRING_RATE, spring_length4, spring_length4,2, 10 };
    initial_robot.C_Springs[37] = { SPRING_RATE, spring_length4, spring_length4,3, 10 };
    initial_robot.C_Springs[38] = { SPRING_RATE, spring_length5, spring_length5,6, 10 };
    initial_robot.C_Springs[39] = { SPRING_RATE, spring_length5, spring_length5,7, 10 };
    //left leg
    initial_robot.C_Springs[40] = { SPRING_RATE, spring_length4, spring_length4,0, 11 };
    initial_robot.C_Springs[41] = { SPRING_RATE, spring_length4, spring_length4,3, 11 };
    initial_robot.C_Springs[42] = { SPRING_RATE, spring_length5, spring_length5,4, 11 };
    initial_robot.C_Springs[43] = { SPRING_RATE, spring_length5, spring_length5,7, 11 };

    for (int i = 28; i < 44; i++)
    {
        initial_robot.C_Springs[i].S_L0 = sqrt(
            pow(initial_robot.C_Masses[initial_robot.C_Springs[i].S_index1].M_position[0] - 
                initial_robot.C_Masses[initial_robot.C_Springs[i].S_index2].M_position[0], 2) +
            pow(initial_robot.C_Masses[initial_robot.C_Springs[i].S_index1].M_position[1] -
                initial_robot.C_Masses[initial_robot.C_Springs[i].S_index2].M_position[1], 2) +
            pow(initial_robot.C_Masses[initial_robot.C_Springs[i].S_index1].M_position[2] -
                initial_robot.C_Masses[initial_robot.C_Springs[i].S_index2].M_position[2], 2));

        initial_robot.C_Springs[i].S_inital = initial_robot.C_Springs[i].S_L0;
    }


    std::uniform_int_distribution<int> s_type_operators(1, 4);
    std::uniform_int_distribution<int> b_operators(1, 100000);
    std::uniform_int_distribution<int> c_operators(1, 200000);
    double b = b_operators(eng) / (double)1000000;              //b:0~0.1
    double c = c_operators(eng) / (double)10000;                //c:0~20

    //Generate spring types for each spring
    for (int i = 0; i < initial_robot.C_Springs.size(); i++)
    {
        initial_robot.C_Springs[i].S_type = s_type_operators(eng);

        if (initial_robot.C_Springs[i].S_type == 1)
        {
            initial_robot.C_Springs[i].S_K = initial_robot.R_Parameters.K_hard;
        }
        else if (initial_robot.C_Springs[i].S_type == 2)
        {
            initial_robot.C_Springs[i].S_K = initial_robot.R_Parameters.K_soft;
        }
        else if (initial_robot.C_Springs[i].S_type == 3 || initial_robot.C_Springs[i].S_type == 4)
        {
            initial_robot.C_Springs[i].b = b;
            initial_robot.C_Springs[i].c = c;
        }
    }
    initial_robot.R_Good_Flag = 1;
}


void Robot_Update(Robot& robot_0)
{
    //Calculate the forces of each springs
    std::vector<std::vector<double>> robot_masses_forces(robot_0.C_Masses.size(), std::vector<double>(3));
    double spring_energy = 0.0;     //Spring elastic potential energy
    double ground_energy = 0.0;     //Potential energy
    double gravity_energy = 0.0;    //Potential energy
    double masses_energy = 0.0;     //Kinetic energy

    for (int i = 0; i < robot_0.C_Springs.size(); i++)
    {
        //Calculate active spring rest length
        if (robot_0.C_Springs[i].S_type == 3)
        {
            robot_0.Calculate_S_L0_expand(i);
        }
        else if (robot_0.C_Springs[i].S_type == 4)
        {
            robot_0.Calculate_S_L0_contract(i);
        }

        int index1 = robot_0.C_Springs[i].S_index1;
        int index2 = robot_0.C_Springs[i].S_index2;

        double dist_vector = robot_0.C_Masses[index1].M_position - robot_0.C_Masses[index1].M_position;
        double euclidean_dist = std::sqrt(\
            std::pow(robot_0.C_Masses[index1].M_position[0] - robot_0.C_Masses[index2].M_position[0], 2) + \
            std::pow(robot_0.C_Masses[index1].M_position[1] - robot_0.C_Masses[index2].M_position[1], 2) + \
            std::pow(robot_0.C_Masses[index1].M_position[2] - robot_0.C_Masses[index2].M_position[2], 2));
        double spring_force = robot_0.C_Springs[i].S_K * fabs((robot_0.C_Springs[i].S_L0 - euclidean_dist));
        double direction[] = {
            (robot_0.C_Masses[index1].M_position[0] - robot_0.C_Masses[index2].M_position[0]) / euclidean_dist,
            (robot_0.C_Masses[index1].M_position[1] - robot_0.C_Masses[index2].M_position[1]) / euclidean_dist,
            (robot_0.C_Masses[index1].M_position[2] - robot_0.C_Masses[index2].M_position[2]) / euclidean_dist
        };

        //Contraction
        if (robot_0.C_Springs[i].S_L0 - euclidean_dist >= 0)
        {
            robot_masses_forces[robot_0.C_Springs[i].S_index1][0] = robot_masses_forces[robot_0.C_Springs[i].S_index1][0] + direction[0] * spring_force;
            robot_masses_forces[robot_0.C_Springs[i].S_index1][1] = robot_masses_forces[robot_0.C_Springs[i].S_index1][1] + direction[1] * spring_force;
            robot_masses_forces[robot_0.C_Springs[i].S_index1][2] = robot_masses_forces[robot_0.C_Springs[i].S_index1][2] + direction[2] * spring_force;

            robot_masses_forces[robot_0.C_Springs[i].S_index2][0] = robot_masses_forces[robot_0.C_Springs[i].S_index2][0] - direction[0] * spring_force;
            robot_masses_forces[robot_0.C_Springs[i].S_index2][1] = robot_masses_forces[robot_0.C_Springs[i].S_index2][1] - direction[1] * spring_force;
            robot_masses_forces[robot_0.C_Springs[i].S_index2][2] = robot_masses_forces[robot_0.C_Springs[i].S_index2][2] - direction[2] * spring_force;
        }
        else // < 0
        {
            robot_masses_forces[robot_0.C_Springs[i].S_index1][0] = robot_masses_forces[robot_0.C_Springs[i].S_index1][0] - direction[0] * spring_force;
            robot_masses_forces[robot_0.C_Springs[i].S_index1][1] = robot_masses_forces[robot_0.C_Springs[i].S_index1][1] - direction[1] * spring_force;
            robot_masses_forces[robot_0.C_Springs[i].S_index1][2] = robot_masses_forces[robot_0.C_Springs[i].S_index1][2] - direction[2] * spring_force;

            robot_masses_forces[robot_0.C_Springs[i].S_index2][0] = robot_masses_forces[robot_0.C_Springs[i].S_index2][0] + direction[0] * spring_force;
            robot_masses_forces[robot_0.C_Springs[i].S_index2][1] = robot_masses_forces[robot_0.C_Springs[i].S_index2][1] + direction[1] * spring_force;
            robot_masses_forces[robot_0.C_Springs[i].S_index2][2] = robot_masses_forces[robot_0.C_Springs[i].S_index2][2] + direction[2] * spring_force;
        }
        spring_energy = spring_energy + 0.5 * robot_0.C_Springs[i].S_K * (pow(robot_0.C_Springs[i].S_L0 - euclidean_dist, 2));
    }

    //Calculate position, velocity, acceleration of all masses
    for (int i = 0; i < robot_0.C_Masses.size(); i++)
    {
        if (i <= 7)//If it is a reasonable robot
        {
            if (robot_0.C_Masses[i].M_position[2] < -0.005 || robot_0.C_Masses[i].M_position[2] > 0.3)
            {
                robot_0.R_Good_Flag = 0;//If mass0~7 touch the ground, indicate that the robot has fellen, it cannot be used
            }
        }

        //Add gravity
        robot_masses_forces[i][2] = robot_masses_forces[i][2] + robot_0.C_Masses[i].M_mass * GRAVITY;
        //Ground reboud
        if (robot_0.C_Masses[i].M_position[2] < 0)
        {
            robot_masses_forces[i][2] = robot_masses_forces[i][2] + K_GROUND * fabs(robot_0.C_Masses[i].M_position[2]);
            //Calculate ground potential energy
            ground_energy = ground_energy + 0.5 * K_GROUND * pow((robot_0.C_Masses[i].M_position[2]), 2);

            //Add friction--------------------------------------------------------------------------------------
            double force_horizontal = sqrt(pow(robot_masses_forces[i][0], 2) + pow(robot_masses_forces[i][1], 2));
            double force_vertical = K_GROUND * fabs(robot_0.C_Masses[i].M_position[2]);

            if (force_horizontal <= force_vertical * FRICTION_MU_S)
            {
                //If the external force is smaller than the static friction, then the mass will stop moving
                robot_masses_forces[i][0], robot_masses_forces[i][1] = 0, 0;
                robot_0.C_Masses[i].M_velocity[0], robot_0.C_Masses[i].M_velocity[0] = 0, 0;
            }
            else
            {
                if (robot_0.C_Masses[i].M_velocity[0] < 0)
                {
                    robot_masses_forces[i][0] = robot_masses_forces[i][0] + \
                        fabs(FRICTION_MU_K * robot_masses_forces[i][2] * robot_masses_forces[i][0] / force_horizontal);
                }
                else
                {
                    robot_masses_forces[i][0] = robot_masses_forces[i][0] - \
                        fabs(FRICTION_MU_K * robot_masses_forces[i][2] * robot_masses_forces[i][0] / force_horizontal);
                }
                if (robot_0.C_Masses[i].M_velocity[1] < 0)
                {
                    robot_masses_forces[i][1] = robot_masses_forces[i][1] + \
                        fabs(FRICTION_MU_K * robot_masses_forces[i][2] * robot_masses_forces[i][1] / force_horizontal);
                }
                else
                {
                    robot_masses_forces[i][1] = robot_masses_forces[i][1] - \
                        fabs(FRICTION_MU_K * robot_masses_forces[i][2] * robot_masses_forces[i][1] / force_horizontal);
                }
            }
            //----------------------------------------------------------------------------------------------------
        }

        //Update acceleration
        robot_0.C_Masses[i].M_acceleration[0] = robot_masses_forces[i][0] / robot_0.C_Masses[i].M_mass;
        robot_0.C_Masses[i].M_acceleration[1] = robot_masses_forces[i][1] / robot_0.C_Masses[i].M_mass;
        robot_0.C_Masses[i].M_acceleration[2] = robot_masses_forces[i][2] / robot_0.C_Masses[i].M_mass;
        //Update velocity
        robot_0.C_Masses[i].M_velocity[0] = (robot_0.C_Masses[i].M_velocity[0] + robot_0.C_Masses[i].M_acceleration[0] * dt) * DAMPING;
        robot_0.C_Masses[i].M_velocity[1] = (robot_0.C_Masses[i].M_velocity[1] + robot_0.C_Masses[i].M_acceleration[1] * dt) * DAMPING;
        robot_0.C_Masses[i].M_velocity[2] = (robot_0.C_Masses[i].M_velocity[2] + robot_0.C_Masses[i].M_acceleration[2] * dt) * DAMPING;
        //Update position
        robot_0.C_Masses[i].M_position[0] = robot_0.C_Masses[i].M_position[0] + robot_0.C_Masses[i].M_velocity[0] * dt;
        robot_0.C_Masses[i].M_position[1] = robot_0.C_Masses[i].M_position[1] + robot_0.C_Masses[i].M_velocity[1] * dt;
        robot_0.C_Masses[i].M_position[2] = robot_0.C_Masses[i].M_position[2] + robot_0.C_Masses[i].M_velocity[2] * dt;
        //Calculate masses kinetic energy
        masses_energy = masses_energy + \
            0.5 * robot_0.C_Masses[i].M_mass * (pow(robot_0.C_Masses[i].M_velocity[0], 2) + \
                pow(robot_0.C_Masses[i].M_velocity[1], 2) + \
                pow(robot_0.C_Masses[i].M_velocity[2], 2));
        //Calculate gravity potential energy
        gravity_energy = gravity_energy + robot_0.C_Masses[i].M_mass * (-GRAVITY) * robot_0.C_Masses[i].M_position[2];
    }
    double total_energy = spring_energy + ground_energy + masses_energy + gravity_energy;

    GLOBAL_TIME = GLOBAL_TIME + dt;
}


void Generate_Population(std::vector<Robot>& population_robots)
{
    /*
    This function generate the initial population of robots of size (global int)POPULATION_SIZE
    */
    for (int i = 0; i < POPULATION_SIZE; i++)
    {
        Generate_Robot(population_robots[i]);
    }
}


double Calculate_Fitness(Robot& robot_0)
{
    EVALUATION_TIMES++;
    /*
    This function Calculate the fitness (the distance that robot has moved) of every robot individual
    */
    std::vector<double> p_central_mass(3);
    double central_mass_x, central_mass_y, central_mass_z;
    central_mass_x = central_mass_y = central_mass_z = 0;
    for (int i = 0; i < robot_0.C_Masses.size(); i++)
    {
        central_mass_x = central_mass_x + robot_0.C_Masses[i].M_position[0];
        central_mass_y = central_mass_y + robot_0.C_Masses[i].M_position[1];
        central_mass_z = central_mass_z + robot_0.C_Masses[i].M_position[2];
    }
    central_mass_x = central_mass_x / robot_0.C_Masses.size();
    central_mass_y = central_mass_y / robot_0.C_Masses.size();
    central_mass_z = central_mass_z / robot_0.C_Masses.size();
    p_central_mass = { central_mass_x, central_mass_y, central_mass_z };

    //double distance_central_mass = sqrt(pow(p_central_mass[0] - 0.05, 2) + pow(p_central_mass[1] - 0.05, 2) + pow(p_central_mass[2] - 0.1, 2));

    //Only consider Y-axis
    double distance_central_mass = p_central_mass[1] - 0.05;

    //store fitness------------------------------------------------------
    if (distance_central_mass > FITNESS_IMAGE[FITNESS_IMAGE.size() - 1])
    {
        FITNESS_IMAGE.emplace_back(distance_central_mass);
        EVALUATION_TIMES_IMAGE.emplace_back(EVALUATION_TIMES);
    }

    robot_0.R_Parameters.R_fitness = distance_central_mass;
    return distance_central_mass;
}


void Reset_Robot(Robot& robot_0)
{
    /*
    This function reset all states of robot and replace it back to the starting position
    */
    double initial_x = INITIAL_POSITION[0]; 
    double initial_y = INITIAL_POSITION[1];
    double initial_z = robot_0.R_Parameters.R_side_length;
    double side_length = robot_0.R_Parameters.R_side_length;

    //Central cube
    robot_0.C_Masses[0] = { MASS, {initial_x, initial_y, initial_z}, {0, 0, 0}, {0, 0, 0} };
    robot_0.C_Masses[1] = { MASS, {initial_x + side_length, initial_y, initial_z}, {0, 0, 0}, {0, 0, 0} };
    robot_0.C_Masses[2] = { MASS, {initial_x + side_length, initial_y + side_length, initial_z}, {0, 0, 0}, {0, 0, 0} };
    robot_0.C_Masses[3] = { MASS, {initial_x, initial_y + side_length, initial_z}, {0, 0, 0}, {0, 0, 0} };
    robot_0.C_Masses[4] = { MASS, {initial_x, initial_y, initial_z + side_length}, {0, 0, 0}, {0, 0, 0} };
    robot_0.C_Masses[5] = { MASS, {initial_x + side_length, initial_y, initial_z + side_length}, {0, 0, 0}, {0, 0, 0} };
    robot_0.C_Masses[6] = { MASS, {initial_x + side_length, initial_y + side_length, initial_z + side_length}, {0, 0, 0}, {0, 0, 0} };
    robot_0.C_Masses[7] = { MASS, {initial_x, initial_y + side_length, initial_z + side_length}, {0, 0, 0}, {0, 0, 0} };
    //Four legs
    robot_0.C_Masses[8] = { MASS,
                                    {initial_x + side_length / 2,
                                    initial_y - side_length + robot_0.R_Parameters.Foot_pos_err[0],
                                    initial_z - side_length},
                                    {0, 0, 0}, {0, 0, 0} };
    robot_0.C_Masses[9] = { MASS,
                                    {initial_x + side_length * 2 + robot_0.R_Parameters.Foot_pos_err[1],
                                    initial_y + side_length / 2,
                                    initial_z - side_length},
                                    {0, 0, 0}, {0, 0, 0} };
    robot_0.C_Masses[10] = { MASS,
                                    {initial_x + side_length / 2,
                                    initial_y + side_length * 2 + robot_0.R_Parameters.Foot_pos_err[2],
                                    initial_z - side_length},
                                    {0, 0, 0}, {0, 0, 0} };
    robot_0.C_Masses[11] = { MASS,
                                    {initial_x - side_length + robot_0.R_Parameters.Foot_pos_err[3],
                                    initial_y + side_length / 2,
                                    initial_z - side_length},
                                    {0, 0, 0}, {0, 0, 0} };

    for (int i = 28; i < 44; i++)
    {
        robot_0.C_Springs[i].S_L0 = sqrt(
            pow(robot_0.C_Masses[robot_0.C_Springs[i].S_index1].M_position[0] -
                robot_0.C_Masses[robot_0.C_Springs[i].S_index2].M_position[0], 2) +
            pow(robot_0.C_Masses[robot_0.C_Springs[i].S_index1].M_position[1] -
                robot_0.C_Masses[robot_0.C_Springs[i].S_index2].M_position[1], 2) +
            pow(robot_0.C_Masses[robot_0.C_Springs[i].S_index1].M_position[2] -
                robot_0.C_Masses[robot_0.C_Springs[i].S_index2].M_position[2], 2));

        robot_0.C_Springs[i].S_inital = robot_0.C_Springs[i].S_L0;
    }

    for (int i = 0; i < robot_0.C_Springs.size(); i++)
    {
        if (robot_0.C_Springs[i].S_type == 3 || robot_0.C_Springs[i].S_type == 4)
        {
            robot_0.C_Springs[i].S_L0 = robot_0.C_Springs[i].S_inital;
        }
    }
    GLOBAL_TIME = 0.0;
}


void Get_Parents(std::vector<int>& parents_index, std::vector<double>& fitness)
{
    /*
    This function find the best part of individuals in a population by fitness
    Return the index of individual
    */

    parents_index.clear();
    //Sorting the population by fitness
    std::vector<int> sequence_population(fitness.size());
    std::iota(sequence_population.begin(), sequence_population.end(), 0);
    std::sort(sequence_population.begin(), sequence_population.end(), [&](int i, int j) {return fitness[i] > fitness[j];});
    for (int i = 0; i < fitness.size() * SELECTION_PRESSURE; i++)
        parents_index.push_back(sequence_population[i]);
}


void Crossover(Robot& parents_1, Robot& parents_2)
{
    /*
    Crossover robot's spring type number and spring rate
    */
    std::uniform_int_distribution<int> position_operators(1, parents_1.C_Springs.size() - 1);
    std::uniform_int_distribution<int> which_leg_operators(0,3);

    std::vector<int> p_crossover(2);
    p_crossover[0] = position_operators(eng);
    p_crossover[1] = position_operators(eng);
    int which_leg = which_leg_operators(eng);

    double L0_temp;
    double L_initial_temp;
    double Foot_pos_err_temp;
    Robot robot_temp;

    //Crossover spring
    for (int i = *min_element(p_crossover.begin(), p_crossover.end()); i < *max_element(p_crossover.begin(), p_crossover.end()); i++)
    {
        robot_temp.C_Springs[i] = parents_2.C_Springs[i];
        parents_2.C_Springs[i] = parents_1.C_Springs[i];
        parents_1.C_Springs[i] = robot_temp.C_Springs[i];

        L0_temp = parents_2.C_Springs[i].S_L0;
        L_initial_temp = parents_2.C_Springs[i].S_inital;
        parents_2.C_Springs[i].S_L0 = parents_1.C_Springs[i].S_L0;
        parents_2.C_Springs[i].S_inital = parents_1.C_Springs[i].S_inital;
        parents_1.C_Springs[i].S_L0 = L0_temp;
        parents_1.C_Springs[i].S_inital = L_initial_temp;
    }
    //Crossover leg

    Foot_pos_err_temp = parents_2.R_Parameters.Foot_pos_err[which_leg];
    parents_2.R_Parameters.Foot_pos_err[which_leg] = parents_1.R_Parameters.Foot_pos_err[which_leg];
    parents_1.R_Parameters.Foot_pos_err[which_leg] = Foot_pos_err_temp;
}


void Mutate(Robot& parents_robot)
{
    /*
    Change spring's parameters or spring's type
    */
    std::uniform_int_distribution<int> which_spring_operators(0, parents_robot.C_Springs.size() - 1);
    std::uniform_int_distribution<int> which_type_operators(0, 10000);
    std::uniform_int_distribution<int> type_operators(1, 4);
    std::uniform_int_distribution<int> b_operators(1, 100000);
    std::uniform_int_distribution<int> c_operators(1, 200000);
    std::uniform_int_distribution<int> which_leg_operators(0, 3);
    std::uniform_int_distribution<int> leg_position_operator(-500, 500);
    int type_change = type_operators(eng);
    int which_spring = which_spring_operators(eng);
    int which_leg = which_leg_operators(eng);

    if (parents_robot.C_Springs[which_spring].S_type == 3 || parents_robot.C_Springs[which_spring].S_type == 4)
    {
        if (which_type_operators(eng) <= 8000) //80 chance to change a little bit of b and c
        {
            if (which_type_operators(eng) <= 5000) //b
            {
                parents_robot.C_Springs[which_spring].b = parents_robot.C_Springs[which_spring].b * 1.05;
            }
            else
            {
                parents_robot.C_Springs[which_spring].b = parents_robot.C_Springs[which_spring].b * 0.95;
            }
            if (which_type_operators(eng) <= 5000) //c
            {
                parents_robot.C_Springs[which_spring].c = parents_robot.C_Springs[which_spring].c * 1.05;
            }
            else
            {
                parents_robot.C_Springs[which_spring].c = parents_robot.C_Springs[which_spring].c * 0.95;
            }
        }
    }
    else //Change type
    {
        if (type_change == 1)
        {
            parents_robot.C_Springs[which_spring_operators(eng)].S_type = 1;
            parents_robot.C_Springs[which_spring_operators(eng)].S_K = parents_robot.R_Parameters.K_hard;
        }
        else if (type_change == 2)
        {
            parents_robot.C_Springs[which_spring_operators(eng)].S_type = 2;
            parents_robot.C_Springs[which_spring_operators(eng)].S_K = parents_robot.R_Parameters.K_soft;
        }
        else if (type_change == 3)
        {
            parents_robot.C_Springs[which_spring_operators(eng)].S_type = 3;
            parents_robot.C_Springs[which_spring].b = b_operators(eng) / (double)1000000;
            parents_robot.C_Springs[which_spring].c = c_operators(eng) / (double)10000;
        }
        else if (type_change == 4)
        {
            parents_robot.C_Springs[which_spring_operators(eng)].S_type = 4;
            parents_robot.C_Springs[which_spring].b = b_operators(eng) / (double)1000000;
            parents_robot.C_Springs[which_spring].c = c_operators(eng) / (double)10000;
        }
    }
    //Change length of leg
    if (which_type_operators(eng) < 5000)
    {
        parents_robot.R_Parameters.Foot_pos_err[which_leg] = leg_position_operator(eng) / (double)10000;
    }
    else
    {
        if (which_type_operators(eng) < 5000)
        {
            parents_robot.R_Parameters.Foot_pos_err[which_leg] = parents_robot.R_Parameters.Foot_pos_err[which_leg] * 1.05;
        }
        else
        {
            parents_robot.R_Parameters.Foot_pos_err[which_leg] = parents_robot.R_Parameters.Foot_pos_err[which_leg] * 0.95;
        }
    }
}


void Get_Childs(std::vector<int>& parents_index, std::vector<Robot>& parents_pool, Child& child)
{
    /*
    This function select two parents in the parents pool, then do crossover and mutation
    */
    std::uniform_int_distribution<int> parents_operators(0, parents_pool.size() - 1);
    std::uniform_int_distribution<int> mutate_operators(1, 10000);
    std::uniform_int_distribution<int> number_of_spring_operators(1, 10);

    Robot parents_1, parents_2, child_1, child_2;
    int parents_1_index = parents_operators(eng);
    int parents_2_index = parents_operators(eng);
    parents_1 = parents_pool[parents_1_index];
    parents_2 = parents_pool[parents_2_index];

    Crossover(parents_1, parents_2);
    //Mutate
    if (mutate_operators(eng) <= 10000 * PM)
    {
        for (int i = 0; i < number_of_spring_operators(eng); i++)
        {
            Mutate(parents_1);
        }
            
    }
    if (mutate_operators(eng) <= 10000 * PM)
    {
        for (int i = 0; i < number_of_spring_operators(eng); i++)
        {
            Mutate(parents_2);
        }

    }

    child.child_1 = parents_1;
    child.child_2 = parents_2;
}


void Run_Simulator(Robot& robot)
{
    Reset_Robot(robot);
    GLOBAL_TIME = 0.0;
    for (int times_ = 0; times_ < 30000; times_++)
    {
        Robot_Update(robot);
    }
    if (EVALUATION_TIMES % 100 == 0)
    {
        std::cout << "Evaluation Times:" << EVALUATION_TIMES << std::endl;
    }
}


void Generate_initial_Population(std::vector<std::vector<Robot>>& population_robots)
{
    Child child;
    std::vector<int> parents_index;
    std::vector<double> fitness(POPULATION_SIZE);

    //------Generate first layer of population---------   
    Generate_Population(population_robots[0]);

    for (int layer_count = 0; layer_count < POPULATION_AGE_LAYER - 1; layer_count++)
    {
        //------Update robot
        for (int i = 0; i < POPULATION_SIZE; i++)
        {
            Run_Simulator(population_robots[layer_count][i]);
            //If robot fell down, randomly regenerate a new one
            if (population_robots[layer_count][i].R_Good_Flag == 0)
            {
                Generate_Robot(population_robots[layer_count][i]);
                i--;
                //std::cout << i << std::endl;
            }
        }
        //------Calculate each robot's fitness-------------
        for (int i = 0; i < POPULATION_SIZE; i++)
        {
            fitness[i] = Calculate_Fitness(population_robots[layer_count][i]);
        }
        //------Select parents-----------------------------
        Get_Parents(parents_index, fitness);
        //------Generate new population--------------------
        //keep two best individual
        population_robots[layer_count + 1][0] = population_robots[layer_count][parents_index[0]];
        population_robots[layer_count + 1][1] = population_robots[layer_count][parents_index[1]];
        for (int indi = 1; indi < POPULATION_SIZE / 2; indi++)
        {
            Get_Childs(parents_index, population_robots[layer_count], child);
            population_robots[layer_count + 1][indi * 2] = child.child_1;
            population_robots[layer_count + 1][indi * 2 + 1] = child.child_2;
        }
    }
}


void ALPS(std::vector<std::vector<Robot>>& population_robots, std::vector<Robot>& robot_process)
{
    /*

    */
    Child child;
    std::vector<int> parents_index;
    std::vector<Robot> parents_pool(POPULATION_SIZE * 2);
    std::vector<double> fitness(POPULATION_SIZE * 2);

    for (int generation = 0; generation < GENERATION; generation++)
    {
        std::cout << "generation:" << generation << std::endl;
        for (int layer_count = 1; layer_count <= POPULATION_AGE_LAYER - 1; layer_count++)
        {
            //------Update two layer of robot
            for (int i = 0; i < POPULATION_SIZE; i++)
            {
                Run_Simulator(population_robots[layer_count - 1][i]);
                //If robot fell down, randomly regenerate a new one
                if (population_robots[layer_count - 1][i].R_Good_Flag == 0)
                {
                    Generate_Robot(population_robots[layer_count - 1][i]);
                    i--;
                }
            }
            for (int i = 0; i < POPULATION_SIZE; i++)
            {
                Run_Simulator(population_robots[layer_count][i]);
                //If robot fell down, randomly regenerate a new one
                if (population_robots[layer_count][i].R_Good_Flag == 0)
                {
                    Generate_Robot(population_robots[layer_count][i]);
                    i--;
                }
            }
            //------Generate parents pool----------------------
            for (int i = 0; i < POPULATION_SIZE; i++)
            {
                parents_pool[i] = population_robots[layer_count - 1][i];
            }
            for (int i = POPULATION_SIZE; i < POPULATION_SIZE * 2; i++)
            {
                parents_pool[i] = population_robots[layer_count][i - POPULATION_SIZE];
            }
            //------Calculate each robot's fitness in parents pool-------------
            for (int i = 0; i < POPULATION_SIZE * 2; i++)
            {
                fitness[i] = Calculate_Fitness(parents_pool[i]);
            }
            //------Select parents-----------------------------
            Get_Parents(parents_index, fitness);
            //------Generate new layer population--------------------
            //keep two best individual
            population_robots[layer_count][0] = parents_pool[parents_index[0]];
            population_robots[layer_count][1] = parents_pool[parents_index[1]];
            for (int indi = 1; indi < POPULATION_SIZE / 2; indi++)
            {
                Get_Childs(parents_index, parents_pool, child);
                population_robots[layer_count][indi * 2] = child.child_1;
                population_robots[layer_count][indi * 2 + 1] = child.child_2;
            }
        }
        
        if (generation % 10 == 0)
        {
            //------Regenerate first layer of population---------   
            Generate_Population(population_robots[0]);
        }

        for (int i = 0; i < POPULATION_SIZE; i++)
        {
            std::cout << "fitness:" << fitness[i] << std::endl;
        }
        //Store Dot Plot data
        for (int i = 0; i < POPULATION_AGE_LAYER; i++)
        {
            for (int j = 0; j < POPULATION_SIZE; j++)
            {
                DOT_PLOT_FITNESS[generation].push_back(population_robots[i][j].R_Parameters.R_fitness);
            }
        }
        DOT_PLOT_GENERATION.push_back(generation);
        //Write robot out
        if (generation % 300 == 0 || generation == 5 || generation == 100)
        {
            robot_process.push_back(population_robots[POPULATION_AGE_LAYER - 1][0]);
        }
    }
}


Robot Evolve_Robot(std::vector<Robot>& robot_process)
{
    std::vector<std::vector<Robot>> population_robots(POPULATION_AGE_LAYER, std::vector<Robot>(POPULATION_SIZE));

    Generate_initial_Population(population_robots);

    ALPS(population_robots, robot_process);

    return population_robots[POPULATION_AGE_LAYER - 1][0];
}


void Write_Robot_Process(std::vector<Robot>& robot_process)
{
    std::vector<std::string> out_file_name;
    out_file_name.push_back("D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\C\\Evolve Robot C\\Data\\txt\\robot_0.txt");
    out_file_name.push_back("D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\C\\Evolve Robot C\\Data\\txt\\robot_5.txt");
    out_file_name.push_back("D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\C\\Evolve Robot C\\Data\\txt\\robot_30.txt");
    out_file_name.push_back("D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\C\\Evolve Robot C\\Data\\txt\\robot_60.txt");
    out_file_name.push_back("D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW3\\C\\Evolve Robot C\\Data\\txt\\robot_90.txt");


    for (int robo_idx = 0; robo_idx < robot_process.size    (); robo_idx++)
    {
        std::ofstream file_(out_file_name[robo_idx], std::ofstream::out);

        Reset_Robot(robot_process[robo_idx]);
        for (int i = 0; i < 100000; i++)
        {
            Robot_Update(robot_process[robo_idx]);
            if (i % 100 == 0)
            {
                if (file_.is_open())
                {
                    for (int j = 0; j < robot_process[robo_idx].C_Masses.size(); j++)
                    {
                        for (int k = 0; k < 3; k++)
                        {
                            file_ << std::fixed << std::setprecision(6) << robot_process[robo_idx].C_Masses[j].M_position[k] << ",";
                        }
                    }
                    file_ << "\n";
                }
            }

        }
        file_.close();
    }




}


int main()
{
    std::vector<Robot> robot_process; //0 5 100 300 ... 900
    Robot robot_best = Evolve_Robot(robot_process);
    
    Write_Robot_Process(robot_process);


    Reset_Robot(robot_best);
    std::cout << "Good Flag:" << robot_best.R_Good_Flag << std::endl;


    std::ofstream file_1(of_position_filename1, std::ofstream::out);
    std::ofstream file_2(of_position_filename2, std::ofstream::out);
    std::ofstream file_3(of_position_filename3, std::ofstream::out);

    GLOBAL_TIME = 0.0;
    for (int i = 0; i < 100000; i++)
    {
        Robot_Update(robot_best);
        if (i % 100 == 0)
        {
            if (file_1.is_open())
            {
                for (int j = 0; j < robot_best.C_Masses.size(); j++)
                {
                    for (int k = 0; k < 3; k++)
                    {
                        file_1 << std::fixed << std::setprecision(6) << robot_best.C_Masses[j].M_position[k] << ",";
                    }
                }
                file_1 << "\n";
            }
        }

    }
    //Learning Curves
    if (file_2.is_open())
    {
        for (int i = 0; i < EVALUATION_TIMES_IMAGE.size(); i++)
        {
            file_2 << std::fixed << std::setprecision(6) << EVALUATION_TIMES_IMAGE[i] << ",";
            file_2 << std::fixed << std::setprecision(6) << FITNESS_IMAGE[i] << "\n";
        }
    }
    //Dot Plot
    if (file_3.is_open())
    {
        for (int i = 0; i < DOT_PLOT_FITNESS.size(); i++)
        {
            for (int j = 0; j < DOT_PLOT_FITNESS[i].size(); j++)
            {
                file_3 << std::fixed << std::setprecision(6) << DOT_PLOT_FITNESS[i][j] << ",";
            }
            file_3 << std::fixed << std::setprecision(6) << "\n";
        }
    }
    file_1.close();
    file_2.close();
    file_3.close();



    return 0;
}
