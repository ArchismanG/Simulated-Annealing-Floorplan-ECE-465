#include "planner.h"

int x_outline, y_outline;
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

int main() {
    // Make sure your file is named "ami33.block" and is in the same directory
    int dataset = 1; // Dataset Selection choices go from 1 to 11.
    std::string blockfile = "";
    std::string netfile = "";
    std::string pwrfile = "";
    if (!getDatasetLocs(blockfile, netfile, pwrfile, dataset))
    {
        std::cout << "Invalid Dataset choice!\n";
        return 0;
    }

    // Parse the data from the files
    std::vector<Chiplet> myChiplets = parseChiplets(blockfile); // Blocks
    std::vector<Net> myNets = parseNets(netfile, myChiplets); // Nets
    parsePowerFile(pwrfile, myChiplets); // Power
    FloorplanState IntialFloorplan; // To Store the intial floorplan

    if (!myChiplets.empty()) {
        displayBlocks(myChiplets);
    }

    if (!myNets.empty()) {
        displayNets(myNets);
    }
    std::cout << "The parsed outline for the dataset is " << x_outline << " X " << y_outline << "\n"; // Display the Bounding Box value
    std::cout << "The number of block in this dataset is " << myChiplets.size() << "\n"; // Display the number of blocks in this chip

    ////////////////////Initialization Done//////////////////////////////

    // Declare a relation matrix for 2D just for now
    size_t n = myChiplets.size();
    std::vector<std::vector<int>> relationMatrix(n, std::vector<int>(n, -1)); 

    // Initialize an array to keep track of which element is related to which row/column of the relation matrix
    std::vector<int> id_relation(n, -1);
    initializeFloorplan(myChiplets, relationMatrix, id_relation); // Creates an initial floorplan to perform simulated annealing on it
    // displayRelationMatrix(relationMatrix, id_relation); 
    compileFloorplan(myChiplets, relationMatrix, id_relation); // Compiles the floorplan to obtain coordiates that can be used for calculating other metrics.
    IntialFloorplan.chiplets = myChiplets;
    IntialFloorplan.relationMatrix = relationMatrix;
    IntialFloorplan.id_relation = id_relation; // Save the intitalized floorplan into the structure for use in the simulated annealing engine

    simulatedAnnealing(IntialFloorplan, myNets); // Perform Simulated Annealing
    displayFloorplan(IntialFloorplan.chiplets, IntialFloorplan.id_relation);

    // Compute the final dimensions
    std::pair<double, double> finalDims = computeFinalDimensions(IntialFloorplan.chiplets);

    // Display the results
    std::cout << "--- Final Floorplan Dimensions ---\n";
    std::cout << "Total Width (X-Span):  " << finalDims.first << " um\n";
    std::cout << "Total Height (Y-Span): " << finalDims.second << " um\n";
    std::cout << "The parsed outline for the dataset is " << x_outline << " X " << y_outline << "\n"; // Display the Bounding Box value

    return 0;
}