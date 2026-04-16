#include "planner.h"

void compileFloorplan(std::vector<Chiplet>& chiplets, const std::vector<std::vector<int>>& relationMatrix, const std::vector<int>& id_relation)
{
    int n = chiplets.size();

    for (int j = 0; j < n; ++j)
    {
        double maxX = 0.0;
        double maxY = 0.0;
        // First element of the relation matrix will always be in the bottom left of the entire chip
        // Check against all blocks placed BEFORE the current one in the sequence
        for (int i = 0; i < j; ++i) {
            // Relation '1' = Horizontal fan-out (curr is to the right of prev)
            if (relationMatrix[i][j] == 1) {
                if (chiplets[id_relation[i]].x_left + chiplets[id_relation[i]].x_dim > maxX)
                {
                    maxX = chiplets[id_relation[i]].x_left + chiplets[id_relation[i]].x_dim;
                }
            }
            // Relation '0' = Vertical fan-out (curr is above prev)
            else if (relationMatrix[i][j] == 0)
            {
                if (chiplets[id_relation[i]].y_left + chiplets[id_relation[i]].y_dim > maxY) {
                    maxY = chiplets[id_relation[i]].y_left + chiplets[id_relation[i]].y_dim;
                }
            }
        }

        // Apply calculated coordinates to the actual block
        chiplets[id_relation[j]].x_left = maxX;
        chiplets[id_relation[j]].y_left = maxY;

        // Calculate Centroids
        chiplets[id_relation[j]].x_centroid = maxX + (chiplets[id_relation[j]].x_dim / 2.0);
        chiplets[id_relation[j]].y_centroid = maxY + (chiplets[id_relation[j]].y_dim / 2.0);
    }
}

void initializeFloorplan(const std::vector<Chiplet>& chiplets, std::vector<std::vector<int>>& relationMatrix, std::vector<int>& id_relation)
{
    int m = chiplets.size();

    double current_row_width = 0.0;
    int current_row_start_index = 0; // Tracks where the current row began in the sequence

    for (int j = 0; j < m; j++)
    {
        // For the initial state, we just pack them in the order they appear in the vector
        id_relation[j] = j;

        // 2. Set the relations with all previously placed blocks (i < j)
        for (int i = 0; i < j; ++i) {
            if (i < current_row_start_index) {
                // Block i is in an older, lower row. 
                // Therefore, Block j is ABOVE Block i -> Vertical relation (0)
                relationMatrix[i][j] = 0;
            }
            else {
                // Block i is in the SAME row.
                // Therefore, Block j is TO THE RIGHT of Block i -> Horizontal relation (1)
                relationMatrix[i][j] = 1;
            }
        }

        // 3. Update the running width of the current row
        current_row_width += chiplets[j].x_dim;

        // 4. Check if we've exceeded the outline boundary
        // If we do, we let this block finish the row, but the NEXT block starts a new row
        if (current_row_width >= x_outline)
        {
            current_row_width = 0.0;             // Reset width counter
            current_row_start_index = j + 1;     // Next block will be the start of a new row
        }
    }
}

void calculateAreaMetrics(const std::vector<Chiplet>& chiplets, double& totalArea, double& xExceed, double& yExceed)
{

    double maxX = 0.0;
    double maxY = 0.0;

    // Iterate through all blocks to find the global bounding box boundaries
    for (const auto& c : chiplets) {
        double blockRightEdge = c.x_left + c.x_dim;
        double blockTopEdge = c.y_left + c.y_dim;

        if (blockRightEdge > maxX) {
            maxX = blockRightEdge;
        }
        if (blockTopEdge > maxY) {
            maxY = blockTopEdge;
        }
    }

    // Assign the calculated values directly to the referenced variables
    totalArea = maxX * maxY;

    // std::max ensures penalties are exactly 0.0 if we are within the outline
    xExceed = std::max(0.0, maxX - x_outline);
    yExceed = std::max(0.0, maxY - y_outline);
}

void calculateHPWL(const std::vector<Chiplet>& chiplets, const std::vector<Net>& nets, double& totalHPWL)
{

    totalHPWL = 0.0; // Reset just in case it's reused in a loop

    for (const auto& net : nets)
    {
        // Skip empty nets or nets with only one block (no wire needed)
        if (net.chipletIDs.size() <= 1)
        {
            continue;
        }

        // Initialize min and max to extreme values
        double min_cx = std::numeric_limits<double>::infinity();
        double max_cx = -std::numeric_limits<double>::infinity();
        double min_cy = std::numeric_limits<double>::infinity();
        double max_cy = -std::numeric_limits<double>::infinity();

        // Find the bounding box of all block centroids in this net
        for (int id : net.chipletIDs) 
        {
            double current_cx = chiplets[id].x_centroid;
            double current_cy = chiplets[id].y_centroid;

            if (current_cx < min_cx) min_cx = current_cx;
            if (current_cx > max_cx) max_cx = current_cx;

            if (current_cy < min_cy) min_cy = current_cy;
            if (current_cy > max_cy) max_cy = current_cy;
        }

        // Calculate Manhattan distance (HPWL) for this net and add to total
        double netHPWL = (max_cx - min_cx) + (max_cy - min_cy);
        totalHPWL += netHPWL;
    }
}

double calculateCost(const FloorplanState& state, const FloorplanState& bestState) 
{
    double normArea = (bestState.area > 0) ? (state.area / bestState.area) : 1.0;
    double normHPWL = (bestState.hpwl > 0) ? (state.hpwl / bestState.hpwl) : 1.0;

    // Penalize heavily for exceeding the outline
    double penalty = (state.xExceed + state.yExceed) * 10.0;

    return (2 * normArea) + (5 * normHPWL) + penalty;
}

// Function to perform a random perturbation on the state
void perturbState(FloorplanState& state, std::mt19937& gen)
{
    int m = state.chiplets.size();
    std::uniform_int_distribution<> distBlock(0, m - 1);
    std::uniform_int_distribution<> distPerturbation(1, 4);

    int pType = distPerturbation(gen);

    if (pType == 1) 
    {
        // 1. Rotation: Pick a random block and swap width/height
        int cId = distBlock(gen);
        std::swap(state.chiplets[cId].x_dim, state.chiplets[cId].y_dim);
    }
    else if (pType == 2) 
    {
        // 2. Module Swap: Swap two elements in the id_relation array
        int id1 = distBlock(gen);
        int id2 = distBlock(gen);
        while (id1 == id2)
        {
            id2 = distBlock(gen);
        } // Make it so that 2 different chiplets are selected
        std::swap(state.id_relation[id1], state.id_relation[id2]);
    }
    else if (pType == 3 || pType == 4) 
    {
        // 3 & 4 require picking valid matrix coordinates (upper triangle where i < j)
        int i = distBlock(gen);
        int j = distBlock(gen);
        while (i == j) 
        { 
            j = distBlock(gen); 
        } // Make it so that 2 different chiplets are selected

        if (i > j) 
            std::swap(i, j); // Force i < j for upper triangle in the relation matrix

        // Flip the relation (1 -> 0 or 0 -> 1)
        state.relationMatrix[i][j] = 1 - state.relationMatrix[i][j];

        if (pType == 4) 
        {
            // 4. Flip and Swap: Also swap their sequence IDs
            std::swap(state.id_relation[i], state.id_relation[j]);
        }
    }
}

// The core Simulated Annealing engine
void simulatedAnnealing(int threadID, FloorplanState initialState, const std::vector<Net>& nets, FloorplanState& BestPlan, 
    double startTemp, double coolRate, double finTemp, int mpt)
{

    std::random_device rd;
    std::mt19937 gen(rd() + (threadID * 1000));
    std::uniform_real_distribution<> distProb(0.0, 1.0);

    // Thread based random starts
    // 1. Completely shuffle the placement sequence
    std::shuffle(initialState.id_relation.begin(), initialState.id_relation.end(), gen);

    // 2. Randomly rotate ~50% of the blocks for more geometric diversity
    std::uniform_int_distribution<> distRot(0, 1);
    for (auto& c : initialState.chiplets) {
        if (distRot(gen) == 1) {
            std::swap(c.x_dim, c.y_dim);
        }
    }

    // 3. Generate new starting floorplans
    initializeFloorplan(initialState.chiplets, initialState.relationMatrix, initialState.id_relation);

    // 4. Recompile floorplan
    compileFloorplan(initialState.chiplets, initialState.relationMatrix, initialState.id_relation);

    // Calcualte Initial Metrics
    calculateAreaMetrics(initialState.chiplets, initialState.area, initialState.xExceed, initialState.yExceed);
    calculateHPWL(initialState.chiplets, nets, initialState.hpwl);

    FloorplanState currentState = initialState;
    FloorplanState bestState = initialState;
    FloorplanState newState = initialState;
    FloorplanState originalState = initialState;
    double currentCost = calculateCost(currentState, originalState);
    double currentBestCost = calculateCost(bestState, originalState);
    double newCost, deltaCost;

    // SA Parameters (These usually require tuning based on the dataset size)
    double temperature = startTemp;
    double coolingRate = coolRate;
    double finalTemperature = finTemp;
    int movesPerTemp = mpt;

    // std::cout << "Starting SA. Initial Area: " << bestState.area << " | HPWL: " << bestState.hpwl << "\n";

    while (temperature > finalTemperature)
    {
        for (int step = 0; step < movesPerTemp; step++)
        {
            // Reset to the current state and continue from there
            newState = currentState;
            
            // Perturb, pack, and evaluate
            perturbState(newState, gen);
            compileFloorplan(newState.chiplets, newState.relationMatrix, newState.id_relation);
            calculateAreaMetrics(newState.chiplets, newState.area, newState.xExceed, newState.yExceed);
            calculateHPWL(newState.chiplets, nets, newState.hpwl);

            // Calculate costs relative to the best known state
            newCost = calculateCost(newState, originalState);
            deltaCost = newCost - currentCost;

            // Accept if better or at a certain probability if worse
            if (deltaCost < 0 || distProb(gen) < std::exp(-deltaCost / temperature)) 
            {
                currentState = newState; // Accept the new state
                currentCost = newCost; // Update the current cost

                // Is it the global best?
                // 
                if (newCost < currentBestCost)
                {
                    bestState = currentState;
                    currentBestCost = calculateCost(bestState, originalState);
                }
            }
        }
        // Cool down
        temperature *= coolingRate;

        // --- UPDATE UI DASHBOARD ---
        // Lock the whiteboard for 1 microsecond to update the temp safely
        std::lock_guard<std::mutex> lock(UI_MUTEX);
        LIVE_TEMPS[threadID] = temperature;
        // std::cout << "Cooling thread " << threadID << " to: " << temperature << "\r";
    }

    // When the thread breaks out of the while loop, mark it as done!
    std::lock_guard<std::mutex> lock(UI_MUTEX);
    THREADS_DONE[threadID] = true;
    // Apply the best state back to the original references if needed
    BestPlan = bestState;
    // std::cout << "Finished SA. Best Area: " << bestState.area << " | HPWL: " << bestState.hpwl << "\n";
}