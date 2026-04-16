#include "planner.h"

// Function to parse the blocks from the file
std::vector<Chiplet> parseChiplets(const std::string& filename) 
{
    std::vector<Chiplet> chiplets;
    std::ifstream file(filename);
    std::string templ;
    int tempx, tempy;

    if (!file.is_open()) 
    {
        std::cerr << "Failed to open the file: " << filename << "\n";
        return chiplets;
    }

    std::string line;
    int blankLineCount = 0;
    int id_counter = 0;

    while (std::getline(file, line)) {
        // Check if the line is empty (or only contains whitespace)
        if (line.empty() || line.find_first_not_of(" \r\n\t") == std::string::npos) 
        {
            blankLineCount++;
            // Terminate when we see the 2nd blank line
            if (blankLineCount == 2) {
                break; // We are ignoring the pin locations for now
            }
            continue;
        }

        // Get the Outline bounding box dimentions from file
        if (blankLineCount < 1)
        {
            std::istringstream iss(line);
            if (iss >> templ >> tempx >> tempy)
            {
                if (templ == "Outline:")
                {
                    x_outline = tempx;
                    y_outline = tempy;
                }
                continue;
            }
            continue;
        }

        // We are in the block section, parse the data
        std::istringstream iss(line);
        Chiplet currentChiplet;

        // Extract the name, width (x-direction), and height (y-direction)
        if (iss >> currentChiplet.name >> currentChiplet.x_dim >> currentChiplet.y_dim) 
        {
            currentChiplet.id = id_counter;
            chiplets.push_back(currentChiplet);
            id_counter++;
        }
    }

    file.close();
    return chiplets;
}

int findChipletId(const std::vector<Chiplet>& blocks, const std::string& targetName) 
{
    for (const auto& b : blocks) {
        if (b.name == targetName) {
            return b.id;
        }
    }
    return -1; // Return -1 if the name isn't found.
}

std::vector<Net> parseNets(const std::string& filename, const std::vector<Chiplet>& chiplets) 
{
    std::vector<Net> nets;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open the file: " << filename << "\n";
        return nets;
    }

    std::string line;

    // Ignore the first line
    std::getline(file, line);

    Net currentNet;
    bool isParsingNet = false;

    // Read the file line by line
    while (std::getline(file, line)) {
        // Skip empty lines just in case
        if (line.empty() || line.find_first_not_of(" \r\n\t") == std::string::npos) {
            continue;
        }

        std::istringstream iss(line);
        std::string token;
        iss >> token;

        if (token == "NetDegree:") {
            // If we were already parsing a net, save it before starting a new one
            if (isParsingNet) {
                nets.push_back(currentNet);
            }

            // Start a new net
            isParsingNet = true;
            currentNet = Net(); // Reset the struct
            iss >> currentNet.size; // Store the number next to NetDegree
        }
        else if (isParsingNet) {
            // It's a block name, so look up its ID
            int blockId = findChipletId(chiplets, token);

            // If found, add it to our dynamic array
            if (blockId != -1) {
                currentNet.chipletIDs.push_back(blockId);
            }
            else
            {
                cout << "Associated name is either non existant or a pin: " << token << "\n";
            }
        }
    }

    // Don't forget to push the very last net after the loop ends!
    if (isParsingNet) {
        nets.push_back(currentNet);
    }

    file.close();
    return nets;
}

/* Temporary function to generate some power values
 Function to generate random power values and save them to a file
void generatePowerFile(const std::vector<Chiplet>& chiplets, const std::string& outFilename) {
    std::ofstream outFile(outFilename);

    if (!outFile.is_open()) {
        std::cerr << "Failed to create power file: " << outFilename << "\n";
        return;
    }

    // Set up a random number generator for power density [10^5, 10^7]
    // Using hardware entropy source if available, otherwise a standard seed
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(100000.0, 10000000.0);

    // Optional: Write a header line starting with '#' so your parser can easily ignore it
    outFile << "# BlockName    TotalPower(W)\n";

    // Set fixed floating point formatting so the output file is clean
    outFile << std::fixed << std::setprecision(4);

    for (const auto& c : chiplets) {
        // 1. Generate random power density
        double powerDensity = dist(gen);

        // 2. Calculate Area in square meters (dimensions are in um)
        // 1 um^2 = 10^-12 m^2
        double area_m2 = static_cast<double>(c.x_dim) * static_cast<double>(c.y_dim) * 1e-12;

        // 3. Calculate Total Power in Watts
        double totalPower = powerDensity * area_m2;

        // 4. Write to file separated by spaces/tabs for easy parsing
        outFile << c.name << "\t" << totalPower << "\n";
    }

    outFile.close();
    std::cout << "Successfully generated power file: " << outFilename << "\n";
}*/

void parsePowerFile(const std::string& filename, std::vector<Chiplet>& chiplets) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open the file: " << filename << "\n";
        return;
    }

    std::string line;

    // Read the file line by line
    while (std::getline(file, line)) {
        // Skip empty lines or commented header lines (like "# BlockName TotalPower(W)")
        if (line.empty() || line[0] == '#' || line.find_first_not_of(" \r\n\t") == std::string::npos) {
            continue;
        }

        std::istringstream iss(line);
        std::string blockName;
        double totalPower;

        // Extract the name and the total power
        if (iss >> blockName >> totalPower) {

            // Search for the matching block in our structure
            for (auto& c : chiplets) {
                if (c.name == blockName) {
                    // Calculate Area: width (um) * height (um)
                    double area_m2 = static_cast<double>(c.x_dim) * static_cast<double>(c.y_dim) * 1e-12;
                    // Calculate and store Power Density (W/m^2)
                    c.powerDensity = totalPower / area_m2;
                    break; // Stop searching once we've found and updated the block
                }
            }
        }
    }

    file.close();
}

bool getDatasetLocs(std::string& blkf, std::string& ntf, std::string& pwrf, int dataset_no)
{
    bool flag = false;
    switch (dataset_no)
    {
    case 1:
        blkf = "Inputs//ami33.block";
        ntf = "Inputs//ami33.nets";
        pwrf = "Inputs//ami33.pwr";
        break;
    case 2:
        blkf = "Inputs//ami49.block";
        ntf = "Inputs//ami49.nets";
        pwrf = "Inputs//ami49.pwr";
        break;
    case 3:
        blkf = "Inputs//apte.block";
        ntf = "Inputs//apte.nets";
        pwrf = "Inputs//apte.pwr";
        break;
    case 4:
        blkf = "Inputs//hp.block";
        ntf = "Inputs//hp.nets";
        pwrf = "Inputs//hp.pwr";
        break;
    case 5:
        blkf = "Inputs//xerox.block";
        ntf = "Inputs//xerox.nets";
        pwrf = "Inputs//xerox.pwr";
        break;
    case 6:
        blkf = "Inputs//n10.block";
        ntf = "Inputs//n10.nets";
        pwrf = "Inputs//n10.pwr";
        break;
    case 7:
        blkf = "Inputs//n30.block";
        ntf = "Inputs//n30.nets";
        pwrf = "Inputs//n30.pwr";
        break;
    case 8:
        blkf = "Inputs//n50.block";
        ntf = "Inputs//n50.nets";
        pwrf = "Inputs//n50.pwr";
        break;
    case 9:
        blkf = "Inputs//n100.block";
        ntf = "Inputs//n100.nets";
        pwrf = "Inputs//n100.pwr";
        break;
    case 10:
        blkf = "Inputs//n200.block";
        ntf = "Inputs//n200.nets";
        pwrf = "Inputs//n200.pwr";
        break;
    case 11:
        blkf = "Inputs//n300.block";
        ntf = "Inputs//n300.nets";
        pwrf = "Inputs//n300.pwr";
        break;
    default:
        flag = true;
    }
    if (flag)
        return false; // Invalid input
    
    return true;
}

int partition(std::vector<Chiplet>& chiplets,  int low, int high)
{
    int pivot = high; // Choose the last element as the pivot
    int i = low - 1; // Keeps track of the element with the largest index that is bigger than our Pivot

    for (int j = low; j < high ; j++)
    {
        if (chiplets[j].y_dim >= chiplets[pivot].y_dim)
        {
            i++; // Perfrom swap with the next index where the larger than element will be
            std::swap(chiplets[i], chiplets[j]);
        }
    }
    std::swap(chiplets[i + 1], chiplets[pivot]);
    return i + 1;
}

void sortChiplets(std::vector<Chiplet>& chiplets, int low, int high)
{
    if (low < high)
    {
        int pivot_index = partition(chiplets, low, high);
        sortChiplets(chiplets, low, pivot_index - 1);
        sortChiplets(chiplets, pivot_index + 1, high);
    }
}

void exportFloorplan(const FloorplanState& final_state, const std::string& outfile, const double tot_x, const double tot_y)
{
    std::ofstream outFile(outfile);
    if (!outFile.is_open()) {
        std::cerr << "Failed to open " << outfile << " for exporting.\n";
        return;
    }

    outFile << "{\n";
    outFile << "  \"outline_x\": " << x_outline << ",\n";
    outFile << "  \"outline_y\": " << y_outline << ",\n";
    outFile << "  \"final_width\": " << tot_x << ",\n";
    outFile << "  \"final_height\": " << tot_y << ",\n";
    outFile << "  \"area\": " << final_state.area << ",\n";
    outFile << "  \"hpwl\": " << final_state.hpwl << ",\n";
    outFile << "  \"chiplets\": [\n";

    // Loop through and write every chiplet's coordinates
    for (size_t i = 0; i < final_state.chiplets.size(); ++i) 
    {
        const auto& c = final_state.chiplets[i];
        outFile << "    {\"name\": \"" << c.name << "\", "
            << "\"x\": " << c.x_left << ", "
            << "\"y\": " << c.y_left << ", "
            << "\"w\": " << c.x_dim << ", "
            << "\"h\": " << c.y_dim << "}";

        // Add a comma after every chiplet EXCEPT the very last one
        if (i < final_state.chiplets.size() - 1) 
        {
            outFile << ",";
        }
        outFile << "\n";
    }

    outFile << "  ]\n";
    outFile << "}\n";

    outFile.close();
    std::cout << "\n>>> Successfully exported best floorplan to " << outfile << " <<<\n";
}