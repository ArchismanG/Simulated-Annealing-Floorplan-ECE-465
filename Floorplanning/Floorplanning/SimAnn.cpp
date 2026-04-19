#include "planner.h"

void compileFloorplan(std::vector<Chiplet>& chiplets, std::vector<Skyline>& currSky, std::vector<Skyline>& newSky, int child_id, double currentX)
{
    if (child_id == -1)
        return; // Base Case when we reach the end of a branch

    chiplets[child_id].x_left = currentX; // Set the X coordinate of the module
    double endOfBlock = currentX + chiplets[child_id].x_dim; // To check collision with current skyline
    double maxY = 0.0; // Used to Determine final y coordinate of a block

    for (const auto& seg : currSky)
        if (std::max(seg.x_start, currentX) < std::min(seg.x_end, endOfBlock))
            maxY = std::max(maxY, seg.height); // Figure out max height of placement, skyline is not ordered

    chiplets[child_id].y_left = maxY; // Legalize the Y coordinate

    // Update the Skyline, Cannot just dump since it will cause buildup of the skyline
    newSky.clear();
    for (const auto& seg : currSky)
    {
        if (seg.x_end < currentX)
            newSky.push_back(seg);// To the left
        else if (seg.x_start > endOfBlock)
            newSky.push_back(seg);
        else
        {
            // Push partial segments into the skyline, if fully under our new skyline, no need to add
            if (seg.x_start < currentX)
                newSky.push_back(Skyline{ seg.x_start, currentX, seg.height });
            if (seg.x_end > endOfBlock)
                newSky.push_back(Skyline{ endOfBlock, seg.x_end, seg.height });
        }
    }
    // Finally Add out newly placed block to the skyline
    newSky.push_back(Skyline{ currentX, endOfBlock, (chiplets[child_id].y_left + chiplets[child_id].y_dim) });
    std::swap(newSky, currSky); // Update the Skyline

    //Update the Centroids
    chiplets[child_id].x_centroid = chiplets[child_id].x_left + (chiplets[child_id].x_dim / 2.0);
    chiplets[child_id].y_centroid = chiplets[child_id].y_left + (chiplets[child_id].y_dim / 2.0);

    // Perform Recursion
    compileFloorplan(chiplets, currSky, newSky, chiplets[child_id].left_child, endOfBlock); // Left Child
    compileFloorplan(chiplets, currSky, newSky, chiplets[child_id].right_child, currentX); // Right Child
}

void initializeFloorplan(std::vector<Chiplet>& chiplets, std::vector<int>& Indexmap, std::mt19937& gen)
{
    // Shuffle the chips before constructing the tree
    std::shuffle(chiplets.begin(), chiplets.end(), gen);

    // Extract the index map after the shuffle
    for (int i = 0;i < chiplets.size();i++)
    {
        Indexmap[chiplets[i].id] = i;
        chiplets[i].left_child = -1;
        chiplets[i].right_child = -1;
        chiplets[i].parent = -1; // Clean up incase there as some already stored values
    }
    
    // Set the 1st element in the array as the root node
    std::vector<FreeSlots> free_slots; // Vector of available slots
    free_slots.push_back(FreeSlots{ &(chiplets[0].left_child), 0 });
    free_slots.push_back(FreeSlots{ &(chiplets[0].right_child), 0 });

    for (int i = 1; i < chiplets.size(); i++)
    {
        std::uniform_int_distribution<> slot_choose(0, free_slots.size() - 1);
        int chosen_index = slot_choose(gen); // Randomly choose one of the free positions

        *(free_slots[chosen_index].child_ptr) = i;
        chiplets[i].parent = free_slots[chosen_index].parentID; // Update the parent index in the child node

        // Remove the node from the free slot vector
        free_slots[chosen_index] = free_slots.back(); // Copy the last position to the the chosen position
        free_slots.pop_back();

        // Append the new free slots
        free_slots.push_back(FreeSlots{ &(chiplets[i].left_child), i });
        free_slots.push_back(FreeSlots{ &(chiplets[i].right_child), i });
    }
}

int deleteNode(int targetId, int root, std::vector<Chiplet>& chiplets, std::mt19937& gen)
{
    int p_idx = chiplets[targetId].parent;
    int replacement_idx = -1; // Who takes target's place?

    // Case 1 & 2: 0 or 1 child
    if (chiplets[targetId].left_child == -1)
    {
        replacement_idx = chiplets[targetId].right_child;
    }
    else if (chiplets[targetId].right_child == -1)
    {
        replacement_idx = chiplets[targetId].left_child;
    }
    // Case 3: 2 children
    else {
        std::uniform_int_distribution<> coin(0, 1);
        if (coin(gen) == 0)
        {
            // Promote Left!
            replacement_idx = chiplets[targetId].left_child;

            // The Right child must be appended to the right-most extremity of the Left child
            // So that we dont introduce even more unknown changes like a subtree to the right ending up top
            int drop_idx = chiplets[targetId].left_child;
            while (chiplets[drop_idx].right_child != -1) 
            {
                drop_idx = chiplets[drop_idx].right_child;
            }
            chiplets[drop_idx].right_child = chiplets[targetId].right_child;
            chiplets[chiplets[targetId].right_child].parent = drop_idx;

        }
        else
        {
            // Promote Right!
            replacement_idx = chiplets[targetId].right_child;

            // The Left child must be appended to the left-most extremity of the Right child
            int drop_idx = chiplets[targetId].right_child;
            while (chiplets[drop_idx].left_child != -1) 
            {
                drop_idx = chiplets[drop_idx].left_child;
            }
            chiplets[drop_idx].left_child = chiplets[targetId].left_child;
            chiplets[chiplets[targetId].left_child].parent = drop_idx;
        }
    }

    // Connect the replacement to the parent
    if (p_idx != -1) // If target wasn't the root
    { 
        if (chiplets[p_idx].left_child == targetId) 
        {
            chiplets[p_idx].left_child = replacement_idx;
        }
        else 
        {
            chiplets[p_idx].right_child = replacement_idx;
        }
    }
    if (replacement_idx != -1)
    {
        chiplets[replacement_idx].parent = p_idx; // Update parent index in the promoted node
    }

    // Scrub the extracted node clean so it's ready to be inserted elsewhere
    chiplets[targetId].parent = -1;
    chiplets[targetId].left_child = -1;
    chiplets[targetId].right_child = -1;

    // Return the root node incase the root node changes
    if (p_idx == -1)
        return replacement_idx;

    return root;
}

int insertNode(int targetId, int root, std::vector<Chiplet>& chiplets, std::mt19937& gen)
{
    std::uniform_int_distribution<> loc_ch(0, chiplets.size() - 1); // Choose any of the nodes
    int chosen_node = loc_ch(gen);

    std::uniform_int_distribution<> coin(0, 1);
    bool go_left = (coin(gen) == 0); // Choose left or right child

    if (targetId == chosen_node) // Make the node the root
    {
        chiplets[targetId].parent = -1;
        chiplets[root].parent = targetId;
        if (go_left)
        {
            chiplets[targetId].left_child = root;
            chiplets[targetId].right_child = -1;
            return targetId;
        }
        else
        {
            chiplets[targetId].left_child = -1;
            chiplets[targetId].right_child = root;
            return targetId;
        }
    }

    // Otherwise the target node is inserted as a child to a selected node
    if (go_left)
    {
        chiplets[targetId].left_child = chiplets[chosen_node].left_child; // Keep the left child in the left
        // chiplets[targetId].left_child = chiplets[chosen_node].left_child; Keep the right child in the right
        chiplets[targetId].right_child = -1;
        chiplets[targetId].parent = chosen_node;
        chiplets[chosen_node].left_child = targetId;

        if(chiplets[targetId].left_child != -1)
            chiplets[chiplets[targetId].left_child].parent = targetId;
    }
    else
    {
        // chiplets[targetId].left_child = chiplets[chosen_node].left_child; Keep the left child in the left
        chiplets[targetId].right_child = chiplets[chosen_node].right_child; // Keep the right child in the right
        chiplets[targetId].left_child = -1;
        chiplets[targetId].parent = chosen_node;
        chiplets[chosen_node].right_child = targetId;

        if (chiplets[targetId].right_child != -1)
            chiplets[chiplets[targetId].right_child].parent = targetId;
    }
    return root;
}

void calculateAreaMetrics(const std::vector<Chiplet>& chiplets, double& totalArea, double& xExceed, double& yExceed)
{

    double maxX = 0.0;
    double maxY = 0.0;

    // Iterate through all blocks to find the global bounding box boundaries
    for (const auto& c : chiplets) 
    {
        double blockRightEdge = c.x_left + c.x_dim;
        double blockTopEdge = c.y_left + c.y_dim;

        if (blockRightEdge > maxX)
            maxX = blockRightEdge;
        if (blockTopEdge > maxY)
            maxY = blockTopEdge;
    }

    // Assign the calculated values directly to the referenced variables
    totalArea = maxX * maxY;

    // std::max ensures penalties are exactly 0.0 if we are within the outline
    xExceed = std::max(0.0, maxX - x_outline);
    yExceed = std::max(0.0, maxY - y_outline);
}

void calculateHPWL(const std::vector<Chiplet>& chiplets, const std::vector<Net>& nets, const std::vector<int>IndexMap, double& totalHPWL)
{

    totalHPWL = 0.0; // Reset just in case it's reused in a loop
    double netHPWL = 0.0;

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
            double current_cx = chiplets[IndexMap[id]].x_centroid;
            double current_cy = chiplets[IndexMap[id]].y_centroid;

            if (current_cx < min_cx) min_cx = current_cx;
            if (current_cx > max_cx) max_cx = current_cx;

            if (current_cy < min_cy) min_cy = current_cy;
            if (current_cy > max_cy) max_cy = current_cy;
        }

        // Calculate Manhattan distance (HPWL) for this net and add to total
        netHPWL = (max_cx - min_cx) + (max_cy - min_cy);
        totalHPWL += netHPWL;
    }
}

double calculateCost(const FloorplanState& state, const FloorplanState& bestState) 
{
    double normArea = (bestState.area > 0) ? (state.area / bestState.area) : 1.0;
    double normHPWL = (bestState.hpwl > 0) ? (state.hpwl / bestState.hpwl) : 1.0;

    // Penalize heavily for exceeding the outline
    double exceed_penalty = (state.xExceed + state.yExceed) * 20.0;
    double whitespace_penalty = state.area / block_area; // Calculate the white space ratio

    return (1 * normArea) + (2 * normHPWL) + (2 * whitespace_penalty) + exceed_penalty;
}

// Function to perform a random perturbation on the state
int perturbState(FloorplanState& state, std::mt19937& gen, int root_index)
{
    int m = state.chiplets.size();
    std::uniform_int_distribution<> distBlock(0, m - 1);
    std::uniform_int_distribution<> distPerturbation(0, 2);

    int pType = distPerturbation(gen);
    if (pType == 0)
    {
        int id = distBlock(gen); // Choose a block to rotate
        std::swap(state.chiplets[id].x_dim, state.chiplets[id].y_dim);
        return root_index;
    }
    else if (pType == 1)
    {
        int id1 = distBlock(gen);
        int id2;
        do
        {
            id2 = distBlock(gen);
        } while (id1 == id2); // Pick 2 differnt blocks to swap.
        
        // Swapping only the values
        std::swap(state.chiplets[id1].id, state.chiplets[id2].id);
        std::swap(state.chiplets[id1].name, state.chiplets[id2].name);
        std::swap(state.chiplets[id1].x_dim, state.chiplets[id2].x_dim);
        std::swap(state.chiplets[id1].y_dim, state.chiplets[id2].y_dim);

        // Update the Index Map
        state.IndexMap[state.chiplets[id1].id] = id1;
        state.IndexMap[state.chiplets[id2].id] = id2;
        return root_index;
    }
    else
    {
        // Move a node from one location to another location
        int target = distBlock(gen);
        root_index = deleteNode(target, root_index, state.chiplets, gen);
        root_index = insertNode(target, root_index, state.chiplets, gen);
        return root_index;
    }
    return root_index;
}

// The core Simulated Annealing engine
void simulatedAnnealing(int threadID, FloorplanState initialState, const std::vector<Net>& nets, FloorplanState& BestPlan, 
    double startTemp, double coolRate, double finTemp, int mpt)
{

    std::random_device rd;
    std::mt19937 gen(rd() + (threadID * 1000));
    std::uniform_real_distribution<> distProb(0.0, 1.0);
    int temproot = 0;

    // Thread based random starts
    // Randomly rotate ~50% of the blocks for more random start variability
    std::uniform_int_distribution<> distRot(0, 1);
    for (auto& c : initialState.chiplets) {
        if (distRot(gen) == 1) {
            std::swap(c.x_dim, c.y_dim);
        }
    }

    // Create a random floorplan to start simulated annealing from
    initializeFloorplan(initialState.chiplets, initialState.IndexMap, gen);

    // Compile the initial floorplan
    std::vector<Skyline> currentSky;
    std::vector<Skyline> newSkyline;
    currentSky.reserve(500);
    newSkyline.reserve(500);
    currentSky.push_back({ 0.0, 1e9, 0.0 }); // Create a floor
    compileFloorplan(initialState.chiplets, currentSky, newSkyline, initialState.root_node, 0.0);
    // displayFloorplan(initialState.chiplets, initialState.IndexMap);
    // Calcualte Initial Metrics
    calculateAreaMetrics(initialState.chiplets, initialState.area, initialState.xExceed, initialState.yExceed);
    calculateHPWL(initialState.chiplets, nets, initialState.IndexMap, initialState.hpwl);

    FloorplanState currentState = initialState;
    FloorplanState bestState = initialState;
    FloorplanState newState;
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
            currentSky.clear();
            newSkyline.clear();
            currentSky.push_back({ 0.0, 1e9, 0.0 }); // Create a floor
            // Perturb, pack, and evaluate
            temproot = perturbState(newState, gen, newState.root_node);
            compileFloorplan(newState.chiplets, currentSky, newSkyline, temproot, 0.0);
            calculateAreaMetrics(newState.chiplets, newState.area, newState.xExceed, newState.yExceed);
            calculateHPWL(newState.chiplets, nets, newState.IndexMap, newState.hpwl);
            // Calculate costs relative to the best known state
            newCost = calculateCost(newState, originalState);
            deltaCost = newCost - currentCost;

            // Accept if better or at a certain probability if worse
            if (deltaCost < 0 || distProb(gen) < std::exp(-deltaCost / temperature)) 
            {
                currentState = newState; // Accept the new state
                currentCost = newCost; // Update the current cost
                currentState.root_node = temproot; // Accept the temporary root

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