/*
Genetic Programming
Symbolic Regression

Created on 10/06/2022

@Author: Zihan Guo
*/

#include<iostream> 
#include<fstream>  
#include<string>   
#include<sstream>  
#include<vector>
#include<variant>
#include<math.h>
#include<algorithm>
#include<random>
#include<numeric>

using namespace std;

struct Calculate_MSE_return
{
	float MSE;
	vector<float> y_hat;
};

struct Child
{
	vector<variant<float, string>> Child_1;
	vector<variant<float, string>> Child_2;
};

//--------------- Function Declaration------------------------------------------------------------------------------------------------------------
vector<float> point_x, point_y;  //The coordinates of data points																				//
vector<variant<float, string>>  Generate_random_individual(int Max_level, string Operators[], string Constant[]);								//
void Calculate_MSE(vector<variant<float, string>> individual, int Max_level, string Operators[], struct Calculate_MSE_return* CMr);				//
vector<variant<float, string>> Mutation(vector<variant<float, string>> individual, int Max_level, string Operators[]);							//
void Plot_Curves(vector<float> Fitness, vector<int> Evaluations);																				//
void Crossover(vector<variant<float, string>>& parent_1, vector<variant<float, string>>& parent_2, Child& temp_Child);							//
Child Get_child(vector<vector<variant<float, string>>>& population, vector<int>& parents_pool, int Max_level, string Operators[], float pm);	//
//------------------------------------------------------------------------------------------------------------------------------------------------

/*==================================================================================================================================*/
void read_coordinates_from_file(string file_name)
{
	/*
	Read the x-y coordinates of points from the txt file.
	*/
	string point_x_string, point_y_string;
	string line_;


	ifstream file_(file_name);  //open the file using "input file stream"
	if (file_.is_open())
	{
		while (getline(file_, line_))
		{
			stringstream ss(line_);
			getline(ss, point_x_string, ',');  //Assign the string before the comma to the variable point_x_string
			point_x.push_back(stof(point_x_string));
			getline(ss, point_y_string);
			point_y.push_back(stof(point_y_string));
		}
		file_.close();
	}
	else
		std::cout << "file is not open" << "\n";
}


void write_LearningCurves_to_file(vector<float> Cor_Y, vector<float> Cor_X, string filename)
{
	ofstream file_(filename, ofstream::out);
	
	if (file_.is_open())
	{
		for (int i = 0; i < Cor_X.size(); i++)
		{
			file_ << Cor_X[i] << "," << Cor_Y[i] << "\n";
		}
		file_.close();
	}
	else std::cout << "file is not open" << "\n";
}


void Random_Search(int Max_level, string Operators[], string Constant[], int Evaluation_times)
{
	/*
		1. Generate a function randomly
		2. Calculate hwo well the function fits the original function
	*/
	//get the randomly generated function
	vector<variant<float, string>> individual, function_best;
	Calculate_MSE_return MSE_and_Y_hat;
	vector<float> Fitness, Evaluations, y_hat_best; // The ordinate and abscissa of the learning curve
	//srand(time(NULL)); // Generate different random numbers each time
	//
	individual = Generate_random_individual(Max_level, Operators, Constant);
	//
	Calculate_MSE(individual, Max_level, Operators, &MSE_and_Y_hat);
	Fitness.emplace_back(MSE_and_Y_hat.MSE); 
	Evaluations.emplace_back(0);
	y_hat_best = MSE_and_Y_hat.y_hat;

	for (int step = 1; step <= Evaluation_times; step++)
	{
		individual = Generate_random_individual(Max_level, Operators, Constant);

		Calculate_MSE(individual, Max_level, Operators, &MSE_and_Y_hat);

		if (MSE_and_Y_hat.MSE < Fitness[Fitness.size() - 1]) 
		{
			Fitness.emplace_back(MSE_and_Y_hat.MSE);
			Evaluations.emplace_back((float)step);
			y_hat_best = MSE_and_Y_hat.y_hat;
			function_best = individual;
			cout << step << endl;
		}
	}
	for (auto i : function_best)
	{
		cout << get<string>(i) << endl;
	}

	string of_LearningCurves_name = \
		"D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\result data\\RandomSearch_LearningCurves.txt";
	write_LearningCurves_to_file(Fitness, Evaluations, of_LearningCurves_name);
	string of_function_name = \
		"D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\result data\\RandomSearch_function.txt";
	write_LearningCurves_to_file(y_hat_best, point_x, of_function_name);
}


void HillClimber(int Max_level, string Operators[], string Constant[], int Evaluation_times)
{
	/*
		1.Generate a function randomly
		2.Calculate its Mean Square Error
		3.Make a little change to the function
		4.Compare with the last function
		5.If the new one performes better, keep it; otherwise abondon it.
	*/

	vector<variant<float, string>> individual, new_individual;
	Calculate_MSE_return MSE_and_Y_hat;
	vector<float> Fitness, Evaluations, y_hat_best; // The ordinate and abscissa of the learning curve
	int counter = 1;

	individual = Generate_random_individual(Max_level, Operators, Constant);
	Calculate_MSE(individual, Max_level, Operators, &MSE_and_Y_hat);
	Fitness.emplace_back(MSE_and_Y_hat.MSE);
	Evaluations.emplace_back(0);
	y_hat_best = MSE_and_Y_hat.y_hat;

	//
	for (int step = 1; step <= Evaluation_times; step++)
	{
		new_individual = individual;
		new_individual = Mutation(individual, Max_level, Operators);

		Calculate_MSE(new_individual, Max_level, Operators, &MSE_and_Y_hat);

		if (MSE_and_Y_hat.MSE < Fitness[Fitness.size() - 1])
		{
			Fitness.emplace_back(MSE_and_Y_hat.MSE);
			Evaluations.emplace_back((float)step);
			y_hat_best = MSE_and_Y_hat.y_hat;
			individual = new_individual;
			cout << MSE_and_Y_hat.MSE;
			
			string of_LearningCurves_name = \
				"D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\movie file\\movie_" + to_string(counter) + ".txt";
			write_LearningCurves_to_file(y_hat_best, point_x, of_LearningCurves_name);
			counter++;
		}
		cout << step << endl;
	}

	for (auto i : individual)
	{
		cout << get<string>(i) << endl;
	}
	
	string of_LearningCurves_name = \
		"D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\result data\\HillClimber_LearningCurves.txt";
	write_LearningCurves_to_file(Fitness, Evaluations, of_LearningCurves_name);
	string of_function_name = \
		"D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\result data\\HillClimber_function.txt";
	write_LearningCurves_to_file(y_hat_best, point_x, of_function_name);
}


void GP(int Max_level, string Operators[], string Constant[], int Evaluation_times, int population_size, float Selection_pressure, float pm)
{
	/*
		Genetic Programming
	*/

	Calculate_MSE_return MSE_and_Y_hat;
	vector<float> Fitness, Evaluations, y_hat_best;
	vector<vector<variant<float, string>>> population, new_population;
	vector<variant<float, string>> best_function;
	Child child_individual;
	vector<float> population_MSE;
	vector<vector<float>> population_y_hat;
	vector<int> parents_pool;
	vector<int> Sequence_population(population_size); 
	for (int i = 0; i < population_size; i++)
	{
		population.emplace_back(Generate_random_individual(Max_level, Operators, Constant));
	}
	
	for (int i = 0; i < population_size; i++)
	{
		Calculate_MSE(population[i], Max_level, Operators, &MSE_and_Y_hat);
		population_MSE.emplace_back(MSE_and_Y_hat.MSE);
	}
	Fitness.emplace_back(*min_element(population_MSE.begin(), population_MSE.end())); // Find minimum MSE in population
	Evaluations.emplace_back(0);
	/*-----------------------------------------------------------------------------------------------------------------------------*/
	for (int step = 1; step <= Evaluation_times; step++)
	{

		//Sorting the population by scale of error
		std::iota(Sequence_population.begin(), Sequence_population.end(), 0);

		sort(Sequence_population.begin(), Sequence_population.end(), [&](int i, int j) {return population_MSE[i] < population_MSE[j];});

		if (*min_element(population_MSE.begin(), population_MSE.end()) < Fitness[Fitness.size() - 1])
		{
			Fitness.emplace_back(*min_element(population_MSE.begin(), population_MSE.end()));
			Evaluations.emplace_back((float)step);
			y_hat_best = population_y_hat[distance(begin(population_MSE), min_element(population_MSE.begin(), population_MSE.end()))];
			best_function = population[distance(begin(population_MSE), min_element(population_MSE.begin(), population_MSE.end()))];
			cout << "min:" << *min_element(population_MSE.begin(), population_MSE.end()) << endl;
		}

		for (int i = 0; i < (int)population_size * Selection_pressure; i++)//
		{
			parents_pool.emplace_back(Sequence_population[i]);
		}
		//Creat next generation population
		for (int i = 0; i < population_size / 2; i++)
		{
			child_individual = Get_child(population, parents_pool, Max_level, Operators, pm);
			new_population.emplace_back(child_individual.Child_1);
			new_population.emplace_back(child_individual.Child_2);
		}

		population = new_population;
		new_population.clear();
		population_MSE.clear();
		parents_pool.clear();
		population_y_hat.clear();

		for (int i = 0; i < population_size; i++)
		{
			//cout << "number:" << i << endl;
			//for (auto i : population[i])
			//{
			//	cout << get<string>(i) << endl;
			//}

			Calculate_MSE(population[i], Max_level, Operators, &MSE_and_Y_hat);
			population_MSE.emplace_back(MSE_and_Y_hat.MSE);
			population_y_hat.emplace_back(MSE_and_Y_hat.y_hat);
		}
		cout << step << endl;
		for (auto i : population_MSE)
		{
			cout << i;
		}
		cout << "\n";
	}

	int locs = 0;
	for (int i = 0; i <= Max_level - 1; i++)
	{
		for (int k = 1; k <= 4 - i; k++)
		{
			cout << "    ";
		}


		for (int j = 1; j <= pow(2, i); j++)
		{
			cout << get<string>(best_function[locs]) << "   ";
			locs++;
		}

		cout << "\n";

		for (int m = 1; m <= 4 - i; m++)
		{
			cout << "    ";
		}

		for (int n = 1; n <= pow(2, i); n++)
		{
			cout << "/" << " " << "\\" << "   ";
		}
		cout << "\n";


	}

	string of_LearningCurves_name = \
		"D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\result data\\GP_LearningCurves.txt";
	write_LearningCurves_to_file(Fitness, Evaluations, of_LearningCurves_name);
	string of_function_name = \
		"D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\result data\\GP_function.txt";
	write_LearningCurves_to_file(y_hat_best, point_x, of_function_name);
	
	/*-----------------------------------------------------------------------------------------------------------------------------*/

	//for (auto i : population_MSE)
	//{
	//	cout << i << endl;
	//}
	//for (auto i : Sequence_population)
	//{
	//	cout << i << endl;
	//}

}


vector<variant<float, string>>  Generate_random_individual(int Max_level, string Operators[], string Constant[])
{
	/*
		This function is to generate a individual (a binary tree of the funtion) randomly

		Max_level: Deepest level of binary tree
		Operators[6] = {"+", "-", "*", "/", "sin", "cos"};
		string Constant[2] = { "x", "C" };
	*/

	// convert Operators from arr to vec
	vector<string> Operators_vector;
	Operators_vector.insert(Operators_vector.begin(), Operators, Operators + 6);
	vector<variant<float, string>> individual; //store binary tree data

	std::random_device rd;
	std::default_random_engine eng(rd());
	std::uniform_int_distribution<int> locs_Operators(0, Operators_vector.size() - 1);


	for (int i = 1; i < pow(2, Max_level); i++)
	{
		individual.emplace_back("T"); //individual initialize to all sign "T"
	}

	//srand(time(NULL)); // Generate different random numbers each time
	//First generate the first node at the top root, which can only be an operator
	individual[0] = Operators[locs_Operators(eng)];

	for (int leaf = 0; leaf <= pow(2, Max_level - 1) - 2; leaf++)
	{
		string item = get<string>(individual[leaf]);
		//If the node on the previous level is an operator, then grow down; Otherwise stop there because it is a constant
		if (std::find(Operators_vector.begin(), Operators_vector.end(), item) != Operators_vector.end())
		{
			//Before the penultimate level of the binary tree, nodes can be operators, and the last level can only be numbers
			if (leaf < pow(2, Max_level - 2) - 1)
			{
				// Node on the left
				if (rand() % 100 / (float)(100) <= 0.9) // 90% chance to generate an operator
				{
					individual[leaf * 2 + 1] = Operators[locs_Operators(eng)];
				}
				else // 20% chance to generate constants and variable x
				{
					if (rand() % 100 / (float)(100) <= 0.5) // 10% chance to generate variable x
					{
						individual[leaf * 2 + 1] = "x";
					}
					else // 10% chance to generate constants
					{
						individual[leaf * 2 + 1] = to_string(((rand() % 4) - 2) + \
							(((rand() % 200) - 100) / (float)(100))); // Generate constants between -21 ~ 21
					}
				}
				// Node on the right (same as the left)
				if (rand() % 100 / (float)(100) <= 0.8)
				{
					individual[leaf * 2 + 2] = Operators[locs_Operators(eng)];
				}
				else
				{
					if (rand() % 100 / (float)(100) <= 0.5)
					{
						individual[leaf * 2 + 2] = "x";
					}
					else
					{
						individual[leaf * 2 + 2] = to_string(((rand() % 4) - 2) + \
							(((rand() % 200) - 100) / (float)(100)));
					}
				}
			}
			else // Last level can only be constants and variables
			{
				if (rand() % 100 / (float)(100) < 0.5)
				{
					individual[leaf * 2 + 1] = "x";
				}
				else
				{
					individual[leaf * 2 + 1] = to_string(((rand() % 4) - 2) + \
						(((rand() % 200) - 100) / (float)(100)));
				}
				if (rand() % 100 / (float)(100) < 0.5)
				{
					individual[leaf * 2 + 2] = "x";
				}
				else
				{
					individual[leaf * 2 + 2] = to_string(((rand() % 4) - 2) + \
						(((rand() % 200) - 100) / (float)(100)));
				}
			}
		}
		//else //Stop growing because the node is a number
		//{
		//	/* No need to write because initialization has already set all elements to 0 */
		//}
	}
	return individual;
}


void Calculate_MSE(vector<variant<float, string>> individual, int Max_level, string Operators[], struct Calculate_MSE_return* MSE_and_Y_hat_p)	
{
	/*
		Calculate each function's Mean Square Error(MSE)
	*/

	// convert Operators from arr to vec
	vector<string> Operators_vector;
	Operators_vector.insert(Operators_vector.begin(), Operators, Operators + 6);
	int point_size = point_x.size();
	
	static vector<float> temp_value; 
	temp_value.resize(0);

	MSE_and_Y_hat_p->y_hat.clear(); // reset the y^


	for (int i = 0; i < point_x.size(); i++)
	{
		for (int leaf = pow(2, Max_level) - 2; leaf >= 0; leaf--) //从最后一片叶子开始算
		{
			string item = get<string>(individual[leaf]);
			// 如果是运算符
			if (std::find(Operators_vector.begin(), Operators_vector.end(), item) != Operators_vector.end())
			{
				if (item == "+")
				{
					temp_value.emplace_back(temp_value[1] + temp_value[0]);
					temp_value.erase(temp_value.begin(), temp_value.begin() + 2);
				}
				else if (item == "-")
				{
					temp_value.emplace_back(temp_value[1] - temp_value[0]);
					temp_value.erase(temp_value.begin(), temp_value.begin() + 2);
				}
				else if (item == "*")
				{
					temp_value.emplace_back(temp_value[1] * temp_value[0]);
					temp_value.erase(temp_value.begin(), temp_value.begin() + 2);
				}
				else if (item == "/")
				{
					temp_value.emplace_back(temp_value[1] / temp_value[0]);
					temp_value.erase(temp_value.begin(), temp_value.begin() + 2);
				}
				else if (item == "sin")
				{
					temp_value.emplace_back(sin(temp_value[1] + temp_value[0]));
					temp_value.erase(temp_value.begin(), temp_value.begin() + 2);
				}
				else if (item == "cos")
				{
					temp_value.emplace_back(cos(temp_value[1] + temp_value[0]));
					temp_value.erase(temp_value.begin(), temp_value.begin() + 2);
				}
			}
			else // constant or variable
			{
				if (item == "x") //If it is a variable
				{
					temp_value.emplace_back(point_x[i]);
				}
				else if (item == "T")// If it is the end
				{
					//jump it
				}
				else
				{
					temp_value.emplace_back(stof(item));
				}
			}
		}
		// store each y^ in the vector
		MSE_and_Y_hat_p->y_hat.emplace_back(temp_value[0]);
		temp_value.clear(); //reset temp_value
	}
	
	float error_all = 0; // reset errorall
	for (int i = 0; i < point_size; i++)
	{
		error_all += pow(pow((point_y[i] - MSE_and_Y_hat_p->y_hat[i]),2),0.5);
	}
	// remove -inf -nan(ind) and replaced by 1000
	float temp_MSE = error_all / point_size; 
	if (std::isfinite(temp_MSE) == 0)
	{
		temp_MSE = 1000;
	}

	MSE_and_Y_hat_p->MSE = temp_MSE; // Mean square error
}


vector<variant<float, string>> Mutation(vector<variant<float, string>> individual, int Max_level, string Operators[])
{
	/*
		Mutate the individual
	*/

	// convert Operators from arr to vec
	vector<string> Operators_vector;
	Operators_vector.insert(Operators_vector.begin(), Operators, Operators + 6);

	std::random_device rd;
	std::default_random_engine eng(rd());
	std::uniform_int_distribution<int> locs_mutate(0, individual.size() - 1); //Generate the location of mutation randomly
	std::uniform_int_distribution<int> locs_operators(0, Operators_vector.size() - 1); 
	std::uniform_int_distribution<int> probability(1, 100); 
	std::uniform_int_distribution<int> add_(1, 20000);
	std::uniform_int_distribution<int> constant_(1, 4);
	std::uniform_int_distribution<int> Number_of_locs(1, individual.size() - 1);

	float temp_item;

	for (int i = 1; i <= Number_of_locs(eng); i++)
	{
		int locs_mutate_num = locs_mutate(eng);
		string item = get<string>(individual[locs_mutate_num]);
		if (std::find(Operators_vector.begin(), Operators_vector.end(), item) != Operators_vector.end())//If it is an operator
		{
			individual[locs_mutate_num] = Operators_vector[locs_operators(eng)]; // Randomly choose a operator
		}
		else if (item == "x")// If it is a variable
		{
			if (probability(eng) <= 20)
			{
				temp_item = constant_(eng) - 2 + ((add_(eng) - 10000) / (float)10000);
				individual[locs_mutate_num] = to_string(temp_item);
			}
		}
		else if (item == "T")
		{
			// Jump it
		}
		else // If it is a constant
		{
			if (probability(eng) <= 80)
			{
				temp_item = stof(item);
				temp_item = temp_item + ((add_(eng) - 10000) / (float)100000);
				individual[locs_mutate_num] = to_string(temp_item);
			}
			else
			{
				individual[locs_mutate_num] = "x"; // 20% chance mutate to a variable
			}
		}
	}
	return individual;
}


void Crossover(vector<variant<float, string>>& parent_1, vector<variant<float, string>>& parent_2, Child& temp_Child)
{
	/*
		1.Randomly generate the first swap location
		2.Let all binary tree after this point be swaped
	*/

	std::random_device rd;
	std::default_random_engine eng(rd());
	std::uniform_int_distribution<int> locs_Crossover_generator(1, parent_1.size() - 1); //Start from level 2
	
	int locs_Crossover = locs_Crossover_generator(eng);
	int flag_ = 0; // Determine if the top swap point is empty
	if ((get<string>(parent_1[locs_Crossover]) == "T") || (get<string>(parent_2[locs_Crossover]) == "T"))
	{
		flag_ = 1;
	}
	// If the top node is empty, choose location again, until both parents' top swap node have an item
	while (flag_ == 1) 
	{
		locs_Crossover = locs_Crossover_generator(eng);
		if ((get<string>(parent_1[locs_Crossover]) != "T") && (get<string>(parent_2[locs_Crossover]) != "T"))
		{
			flag_ = 0;
			
		}	
	}

	vector<int> All_swap_locations;
	int temp_locs = locs_Crossover;
	string temp_item;
	
	All_swap_locations.push_back(temp_locs);
	//Get the indices of all swap locations
	temp_locs = 2 * temp_locs;
	int i = 1;
	while (temp_locs < parent_1.size() - 1)
	{
		i = pow(2, i);
		for (int j = 1; j <= i; j++)
		{
			All_swap_locations.push_back(temp_locs + j);
		}
		temp_locs = 2 * (temp_locs + 1);
	}
	//Swap the item of 2 parents
	for (i = 0; i < All_swap_locations.size(); i++) 
	{
		temp_item = get<string>(parent_1[All_swap_locations[i]]);
		parent_1[All_swap_locations[i]] = parent_2[All_swap_locations[i]];
		parent_2[All_swap_locations[i]] = temp_item;
	}

	//for (auto i : All_swap_locations)
	//{
	//	cout << i << endl;
	//}
}


Child Get_child(vector<vector<variant<float, string>>>& population_p, vector<int>& parents_pool_p, int Max_level, string Operators[], float pm)
{
	/*
		1.随机在父本池中选择两个父本
		2.随机选择一个点位进行杂交
		3.有一定概率发生突变
		4.返回子代
	*/
	std::random_device rd;
	std::default_random_engine eng(rd());
	std::uniform_int_distribution<int> parent_index_generator(0, parents_pool_p.size() - 1); //Choose a parent in parents pool randomly
	std::uniform_int_distribution<int> pm_generator(1, 10000); //Choose a parent in parents pool randomly

	Child temp_Child;
	vector<variant<float, string>> parent_1, parent_2;

	parent_1 = population_p[parents_pool_p[parent_index_generator(eng)]];// Generate 2 parents
	parent_2 = population_p[parents_pool_p[parent_index_generator(eng)]];

	//Generate 2 offsprings by crossing 2 parents
	Crossover(parent_1, parent_2, temp_Child);

	// A certain probability of mutation
	if (pm_generator(eng) <= 10000 * pm) 
	{
		parent_1 = Mutation(parent_1, Max_level, Operators);
	}
	if (pm_generator(eng) <= 10000 * pm) 
	{
		parent_2 = Mutation(parent_2, Max_level, Operators);
	}

	temp_Child.Child_1 = parent_1;
	temp_Child.Child_2 = parent_2;


	return temp_Child;
}
/*==================================================================================================================================*/

int main()
{

	string file_name = "D:\\guozihan\\Columbia\\2022Fall\\Evol Comp\\HomeWork\\HW2\\Final Submission\\code\\data.txt"; //File name
	int Max_level = 4; // The number of layers of heap data structure
	string Operators[6] = { "+", "-", "*", "/", "sin", "cos" };
	string Constant[2] = { "x", "C" };
	int population_size = 100; // Population size(only for GP)
	int Evaluation_times = 100000;
	float Selection_pressure = 0.5;
	float pm = 0.1; //Probability of mutation;


	read_coordinates_from_file(file_name);

	//Random_Search(Max_level, Operators, Constant, Evaluation_times);
	//HillClimber(Max_level, Operators, Constant, Evaluation_times);
	GP(Max_level, Operators, Constant, Evaluation_times, population_size, Selection_pressure, pm);
	cout << "end" << endl;
	return 0;
}
