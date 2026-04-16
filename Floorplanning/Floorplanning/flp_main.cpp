#include "planner.h"

int x_outline, y_outline;

// --- UI DASHBOARD GLOBALS ---
std::mutex UI_MUTEX;
std::vector<double> LIVE_TEMPS;
std::vector<bool> THREADS_DONE;

// int num_chiplets;

// Function to display the parsed information
void displayBlocks(const std::vector<Chiplet>& blocks) {
    std::cout << "--- Parsed Block Data ---\n";
    for (const auto& b : blocks) {
        std::cout << "Block ID: " << b.id
            << " Block Name: " << b.name
            << " Width (x): " << b.x_dim
            << " Height (y): " << b.y_dim
            << " Pwr Density (W/m^2): " << b.powerDensity << "\n";
    }
    std::cout << "Total blocks parsed: " << blocks.size() << "\n";
}

void displayNets(const std::vector<Net>& nets) {
    std::cout << "\n--- Parsed Net Data ---\n";
    for (size_t i = 0; i < nets.size(); ++i) {
        std::cout << "Net " << i + 1 << " (Expected Size: " << nets[i].size
            << ", Found IDs: " << nets[i].chipletIDs.size() << ") -> IDs: ";
        for (int id : nets[i].chipletIDs) {
            std::cout << id << " ";
        }
        std::cout << "\n";
    }
    std::cout << "Total nets parsed: " << nets.size() << "\n";
}

void displayRelationMatrix(const std::vector<std::vector<int>>& relationMatrix, const std::vector<int>& id_relation)
{
    int n = id_relation.size();
    std::cout << "The current relation Matrix is \n";
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            std::cout << relationMatrix[i][j] << " ";
        }
        std::cout << "\n";
    }
    std::cout << "The IDs corresponding to the rows/columns are: \n";
    for (int i = 0; i < n; i++)
    {
        std::cout << id_relation[i] << " ";
    }
    std::cout << "\n";
}

void displayFloorplan(const std::vector<Chiplet>& chiplets, const std::vector<int>& id_relation) 
{
    std::cout << "\n--- Packed Floorplan Coordinates (In Sequence Order) ---\n";
    for (int idx : id_relation) {
        const auto& c = chiplets[idx];
        std::cout << "Block " << c.name
            << " \t| Bottom-Left: (" << c.x_left<< ", " << c.y_left << ")"
            << " \t| Centroid: (" << c.x_centroid << ", " << c.y_centroid << ")\n";
    }
}

std::pair<double, double> computeFinalDimensions(const std::vector<Chiplet>& chiplets) 
{
    double totalWidth = 0.0;
    double totalHeight = 0.0;

    for (const auto& c : chiplets) 
    {
        double blockRightEdge = c.x_left + c.x_dim;
        double blockTopEdge = c.y_left + c.y_dim;

        if (blockRightEdge > totalWidth)
            totalWidth = blockRightEdge;

        if (blockTopEdge > totalHeight)
            totalHeight = blockTopEdge;
    }

    return { totalWidth, totalHeight }; // Returns the pair
}

int main(int argc, char* argv[])
{
    // Make sure your file is named "ami33.block" and is in the same directory
    int dataset = 6; // Dataset Selection choices go from 1 to 11.
    int core_factor = 2; // Decide what fraction of total cores you want to engage
    double start_temp = 1000.0;
    double cooling_rate = 0.95;
    double final_temp = 0.1;
    int moves_per_temp = 1000; // Simulated Annealing Parameters defaults

    // --- PYTHON / STANDALONE FALLBACK LOGIC ---
    if (argc == 1) 
    {
        std::cout << "[Mode: Visual Studio Standalone]\n";
        // It will just use the defaults of 6 and 2
    }
    else if (argc >= 3) 
    {
        std::cout << "[Mode: Python Controlled]\n";
        dataset = std::stoi(argv[1]);      // Python passed dataset choice
        core_factor = std::stoi(argv[2]);  // Python passed core factor
        start_temp = std::stod(argv[3]);   // Python passed start Temp
        cooling_rate = std::stod(argv[4]); // Python passed cooling rate
        final_temp = std::stod(argv[5]);   // Python passed final Temp
        moves_per_temp = std::stoi(argv[6]);//Python passed moves per temp
    }
    else 
    {
        std::cerr << "Error: Expected 2 arguments (dataset_id core_factor).\n";
        return 1;
    }

    std::string blockfile = "";
    std::string netfile = "";
    std::string pwrfile = "";
    if (!getDatasetLocs(blockfile, netfile, pwrfile, dataset))
    {
        std::cout << "Invalid Dataset choice!\n";
        return 0;
    }

    // Parse the data from the files
    std::vector<Chiplet> myChiplets = parseChiplets(blockfile); // Modules Parsed
    sortChiplets(myChiplets, 0, myChiplets.size() - 1); // Sort Chiplets in descending order height
    for (int i = 0; i < myChiplets.size(); i++)
        myChiplets[i].id = i; // Reassign IDs after sorting

    // Parse the remaining files
    std::vector<Net> myNets = parseNets(netfile, myChiplets); // Nets Parsed
    parsePowerFile(pwrfile, myChiplets); // Power Information Parsed
    FloorplanState IntialFloorplan; // To Store the intial floorplan

    if (!myChiplets.empty()) {
        displayBlocks(myChiplets);
    }

    if (!myNets.empty()) {
        displayNets(myNets);
    }
    std::cout << "The parsed outline for the dataset is " << x_outline << " X " << y_outline << "\n"; // Display the Bounding Box value
    std::cout << "The number of block in this dataset is " << myChiplets.size() << "\n"; // Display the number of blocks in this chip

    ////////////////////Initialization//////////////////////////////

    // Declare a relation matrix for 2D just for now
    size_t n = myChiplets.size();
    std::vector<std::vector<int>> relationMatrix(n, std::vector<int>(n, -1)); 

    // Initialize an array to keep track of which element is related to which row/column of the relation matrix
    std::vector<int> id_relation(n, -1);
    initializeFloorplan(myChiplets, relationMatrix, id_relation); // Creates an initial floorplan to perform simulated annealing on it
    // displayRelationMatrix(relationMatrix, id_relation); // Sorting height and performing a typewriter packing
    compileFloorplan(myChiplets, relationMatrix, id_relation); // Compiles the floorplan to obtain coordinates that can be used for calculating other metrics.
    IntialFloorplan.chiplets = myChiplets;
    IntialFloorplan.relationMatrix = relationMatrix;
    IntialFloorplan.id_relation = id_relation; // Save the initialized floorplan into the structure for use in the simulated annealing engine
    // FloorplanState BestFloorplan = IntialFloorplan;

    //////////////////Perform Simulated Annealing on Multiple Threads/////////////////////////////////////////
    unsigned int numCores = std::thread::hardware_concurrency() / core_factor;
    if (numCores == 0) 
        numCores = 1; // Fall back incase of errors
    std::vector<std::thread> threads;
    std::vector<FloorplanState> threadResults(numCores);
    LIVE_TEMPS.assign(numCores, 1000.0);    // Fills it with 1000.0
    THREADS_DONE.assign(numCores, false);   // Fills it with false

    std::cout << "Performing Simulated Annealing with parameters: \n";
    std::cout << "Starting Temperature: " << start_temp << ", Final Temperature: " << final_temp << "\n";
    std::cout << "Cooling Rate: " << cooling_rate << ", Moves Per Temperature: " << moves_per_temp << "\n";

    for (unsigned int i = 0; i < numCores; i++)
    {
        threads.push_back(std::thread(simulatedAnnealing, i, IntialFloorplan, myNets, std::ref(threadResults[i]), 
            start_temp, cooling_rate, final_temp, moves_per_temp));
    }

    // 2. The UI Watchdog Loop
    bool allDone = false;
    while (!allDone) {
        allDone = true;

        // Read the whiteboard and print the N lines
        {
            std::lock_guard<std::mutex> lock(UI_MUTEX);
            for (unsigned int i = 0; i < numCores; ++i) {
                // We add extra spaces at the end to overwrite any old lingering text
                std::cout << ">> Core " << i << " | Current Temp: " << LIVE_TEMPS[i] << "         \n";
                if (!THREADS_DONE[i]) {
                    allDone = false;
                }
            }
        }

        // If the threads are still running, prepare the console for the next frame
        if (!allDone) {
            // Sleep for 50 milliseconds so we aren't spamming the screen
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // Magic Trick: Move the console cursor UP by 'numCores' lines
            for (unsigned int i = 0; i < numCores; ++i) {
                std::cout << "\033[F";
            }
        }
    }

    for (auto& t : threads) 
    {
        t.join();
    } // Wait for all threads to finish

    // simulatedAnnealing(0, IntialFloorplan, myNets, BestFloorplan); // Perform Simulated Annealing
    // displayFloorplan(BestFloorplan.chiplets, BestFloorplan.id_relation);

    // Display all thread results
    std::pair<double, double> finalDims;
    int BestResult = -1;
    double BestCost, CurrCost;
    calculateAreaMetrics(IntialFloorplan.chiplets, IntialFloorplan.area, IntialFloorplan.xExceed, IntialFloorplan.yExceed);
    calculateHPWL(IntialFloorplan.chiplets, myNets, IntialFloorplan.hpwl);
    BestCost = calculateCost(IntialFloorplan, IntialFloorplan);

    for (unsigned int i = 0; i < numCores; i++)
    {
        finalDims = computeFinalDimensions(threadResults[i].chiplets);

        // Display the results
        std::cout << "--- Final Floorplan Dimensions from Core" << i << "---- \n";
        std::cout << "Total Width (X-Span):  " << finalDims.first << " um\n";
        std::cout << "Total Height (Y-Span): " << finalDims.second << " um\n";
        std::cout << "Total Area: " << threadResults[i].area << " um^2\n";
        std::cout << "Total HPWL: " << threadResults[i].hpwl << " \n";

        CurrCost = calculateCost(threadResults[i], IntialFloorplan);
        if (CurrCost < BestCost)
            BestResult = i;
    }
    finalDims = computeFinalDimensions(threadResults[BestResult].chiplets);
    std::cout << "---Best Computed Floorplan Results from Core" << BestResult << "---- \n";
    std::cout << "Best Found Dimentions: " << finalDims.first << " X " << finalDims.second << " um, with area of " << threadResults[BestResult].area << " um^2\n";
    std::cout << "Best Found HPWL: " << threadResults[BestResult].hpwl << " \n";
    // Compute the final dimensions
    
    std::cout << "The parsed outline for the dataset is " << x_outline << " X " << y_outline << "\n"; // Display the Bounding Box value

    exportFloorplan(threadResults[BestResult], "floorplan_output.json", finalDims.first, finalDims.second);

    // Save another copy as <datasetname>_output.json
    size_t last_slash = blockfile.find_last_of("/\\");
    size_t last_dot = blockfile.find_last_of(".");

    std::string base_name = "backup";

    // Extract just the name in the middle (e.g., "ami33")
    if (last_slash != std::string::npos && last_dot != std::string::npos && last_dot > last_slash) {
        base_name = blockfile.substr(last_slash + 1, last_dot - last_slash - 1);
    }

    // Save a copy
    std::string permanent_filename = base_name + "_out.json";
    exportFloorplan(threadResults[BestResult], permanent_filename, finalDims.first, finalDims.second);

    std::cout << "Simulated Annealing Done.....\n";
    // std::cin.get();
    return 0;
}