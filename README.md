# 2D B*-Tree Floorplanner

This is a C++ floorplanning engine developed as part of **ECE 465** at the University of Illinois at Chicago. This project utilizes **Simulated Annealing** and a **B*-Tree data structure** to optimize chip layouts for area, wirelength, and whitespace.

![Language](https://img.shields.io/badge/language-C%2B%2B-blue)
![Platform](https://img.shields.io/badge/platform-Windows%2011-lightgrey)
![Status](https://img.shields.io/badge/status-Course%20Project-success)

##  Overview

This project performs 2D floorplanning by randomly exploring the solution space using a Simulated Annealing schedule. It evaluates moves based on:
* **Total Area:** Minimizing the bounding box of the chiplets.
* **HPWL (Half-Perimeter Wirelength):** Estimating routing costs between modules.
* **Whitespace:** Minimizing the area of wasted space between the modules.
* **Outline Penalties:** Ensuring the floorplan fits within the specified dataset bounding box.

**Future Plans:** This project will eventually be expanded for **3D Thermal-Aware Floorplanner** to consider TSVs and heat dissipation costs.

## Installation & Setup

### Prerequisites

* **Windows 11** (Tested environment).
* **Python 3.x** (3.9.13 or higher).
* **Matplotlib** (Needed for final visualization, install using `pip install matplotlib`)
* **Visual Studio** (if you wish to modify and rebuild the C++ source).

### Running the Program

1. Download or clone the repository.
2. Open **PowerShell** or **VSCode Terminal**.
3. Navigate to the directory:
   ```powershell
   cd "<Your Folder Path>/Floorplanning/x64/Release"
4. Run the Python Script using VS code or 
   ```poweshell
   python3 Floorplan.py

## Program Configuration

When the program starts, it will ask you to enter a few different parameters
1. **Dataset Selction**
    Choose one of 11 possible datasets
    | Dataset ID | Name | Modules |
    | :--- | :--- | :--- |
    | **1** | ami33 | 33 |
    | **2** | ami49 | 49 |
    | **3** | apte | 9 |
    | **4** | hp | 11 |
    | **5** | xerox | 10 |
    | **6** | n10 | 10 |
    | **7** | n30 | 30 |
    | **8** | n50 | 50 |
    | **9** | n100 | 100 |
    | **10** | n200 | 200 |
    | **11** | n300 | 300 |

2. **Core Fraction Divisor**
    Determines what fraction of the CPU cores will be utilized for the Simulated Annealing algorithm. **Note:** This will not cause the program to run faster. Instead more cores you engage, more runs of simulated annealing will take place in parallel with different random starts for each core, thus increasing the exploration of the solution space.
    * *Example:* On a 12-core CPU, a divisor of 2 will only engage 6 cores

3. **Simulated Annealing Settings**
    * **Starting Temperature:** High values cause the algorithm to easily accept worse moves in the beginning to escape local minima. Higher values allow for more exploration of the the solution space.
    * **Cooling Rate:** Enter a decimal between 0 and 1. **Do not enter 1** since this will make the simulated annealing program run forever. Higher the cooling rate, more time the program will take to converge on a solution.
    * **Final Temperature:** Once the temperature reaches this value, the program will terminate an spit out the best result after all the simulated annealing moves.
    * **Moves per Temperature:** This is an important metric which dictates how many moves will be performed before cooling by 1 step. Having this high allows for good exploration of the solution space however, increasing this will also cause the time taken by the algorithm to skyrocket.

## Technical Details

Once you have entered all the relevant parameters, the program will start. The python script is merely a user interface between the user the and code, the Simulated Annealing is actually written and built from C++ code. The python script is located in the same folder at the `Floorplanning.exe` which has already been built from the C++ code. The python script will pass these parameters to the C++ program which will perfrom the simulated annealing program, this will take somehere between a few seconds and a few minutes depending on the dataset. When the simulated annealing step is finished, the C++ program will output the best found dimentions and HPWL in the terminal and generate a `JSON` file with the final coordinates of all the blocks. Th python script will then output the time taken by the program and read the `JSON` file to generate a visualization for the floorplan.

### Making Changes 

If you want to make changes to the main Simulated Annealing program, I would suggest using Visual Studio. If you want to rebuild it using other tools, the source code is located at `/Floorplanning/Floorplanning`. You should see the files `flp_main.cpp`, `SimAnn.cpp`, `ReadInputs.cpp`, and `planner.h`. If you are using visual studio, simply clone the repository using `https://github.com/ArchismanG/Simulated-Annealing-Floorplan-ECE-465.git`, make changes to the source files and rebuild the code in release mode in Visual Studio. You can then run the python script again and it will use the newly rebuilt `Floorplanning.exe` file.
