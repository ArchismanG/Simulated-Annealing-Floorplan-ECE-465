This floorplanner was made as a part of a course project for ECE 465 at the University of Illinois at Chicago.
This is a simple 2D floorplanner that uses simulated annealing to perform floorplanning. Simulated Annealing is an algorithm that effectively
randomly searches the entire solution space to find a better result than the current result but it will have a non-zero probability to accept
worse moves in hopes of escaping possible local minima. The floorplanner currently only performs floorplanning on the basis of Area and Half
Perimeter Wirelength (HPWL) and also some exceed metric that further penalize moves if the chip exceeds the bounding box (each dataset has its
bounding box specified in the input files). There are plans to expand this into a 3D floorplanner that considers some thermal costs to perform
a thermal-aware placement. The data structure used in this project is based on a relation matrix data structure described in this paper:
D. Al Saleh, Y. Safari, F. R. Amik and B. Vaisband, "P* Admissible Thermal-Aware Matrix Floorplanner for 3D ICs," 2023 IEEE 36th International 
System-on-Chip Conference (SOCC), Santa Clara, CA, USA, 2023, pp. 1-6, doi: 10.1109/SOCC58585.2023.10256905.

This Floorplanner has been tested only on Windows 11 systems. You can use PowerShell or VSCode to run the program. Make sure the latest 
version of Python is installed on your system. Also, ensure matplotlib is installed for visualizing the floorplan at the end.
Download the whole repository.
Navigate to Floorplanning/x64/Release in the terminal. You can use cd "<Your Folder Path>/Floorplanning/x64/Release" in PowerShell.
Once in the right directory in PowerShell, enter the command: python3 Floorplan.py
This will start the floorplanning algorithm. Before the algorithm begins, it will ask you to enter a few parameters.

Enter Dataset Choice (1-11): Enter just a number between 1 to 11 based on the dataset you want to evaluate.
1: ami33    Contains 33 modules for floorplanning
2: ami49    Contains 49 modules for floorplanning
3: apte     Contains 9 modules for floorplanning
4: hp       Contains 11 modules for floorplanning
5: xerox    Contains 10 modules for floorplanning
6: n10      Contains 10 modules for floorplanning
7: n30      Contains 30 modules for floorplanning
8: n50      Contains 50 modules for floorplanning
9: n100     Contains 100 modules for floorplanning
10: n200    Contains 200 modules for floorplanning
11: n300    Contains 300 modules for floorplanning

Enter core fraction divisor: What fraction of your CPU cores do you want to engage for this algorithm?
If you have a 12-core system, entering a 1 will engage all 12 cores, entering a 2 will engage only 6 cores, entering a 3 will engage only 4 cores, etc.
The more cores engaged, the better the simulated annealing solution will be since more of the solution space will be explored.

Next, it will ask you for the various simulated annealing parameters.
Starting temperature: Enter whatever number you want, higher starting temperature will mean more probability to accept bad costs initially,
and therefore a better chance to escape local minimia but too high of a temperature will mean too many bad moves are accepted and may hinder
the algorithm from converging to a good result.

Cooling rate: Enter a decimal between zero and 1. Do not enter 1, then the simulated annealing algorithm will not converge. This dictates how fast
you want to lower the temperature (fewer chances of bad moves being accepted). The smaller the cooling rate value, the faster the temperature will cool, and the
result may not fully explore the solution space, but if it is too high, the schedule will be very slow, and the algorithm will take a longer time to 
reach a solution.

Final Temperature: The temperature at which the simulated annealing algorithm should stop. If you stop when the temperature is too high, the 
solution quality will not be very good.

Moves per Temperature: Simulated annealing algorithm performs multiple moves per temperature before cooling down, where it's less likely bad moves will
be accepted. The higher this value is, the better the final result will be, but this also drastically increases the time taken by the algorithm

Once all these parameters are entered, the Python program will pass these parameters to an exe file; the exe file is the main simulated annealing
algorithm that has been built from the C++ code that can be seen in /Floorplanning/Floorplanning folder. You can also open this whole repository
as a Visual Studio project by opening Visual Studio and clicking on "Open Existing" and navigating to /Floorplanning/Floorplanning.sln
You can edit the code to add modifications, but make sure to use the Release mode of Visual Studio when you are rebuilding the C++ code.

Depending on the parameters entered, the code will take some time to run the whole algorithm. The bigger the dataset, the more time it will consume.
Please note that engaging more cores will not speed up the algorithm; it uses the extra cores to explore more solutions using multiple random starts.
Once the best floorplan has been found by the algorithm, the terminal will spit out the final dimensions, area, and HPWL. It will also create a JSON file output.
The program will automatically jump back to the Python code, which will then report the time taken by the algorithm and also read the JSON file to 
draw the layout of the chip, which you can use to visualize the final floorplan.
