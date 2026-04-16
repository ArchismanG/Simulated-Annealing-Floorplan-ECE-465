import subprocess
import os
import sys
import json
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def main():
    print("=== PyCharm Controller: Floorplanner ===")
    # Update these if your actual path is slightly different
    
    # 1. The exact path to the compiled C++ engine
    current_dir = os.path.dirname(os.path.abspath(__file__))
    exe_path = os.path.join(current_dir, "Floorplanning.exe")
    project_dir = current_dir

    if not os.path.exists(exe_path):
        print(f"\n[ERROR] Cannot find the engine at: {exe_path}")
        # print("Did you compile it in Release mode in Visual Studio?")
        sys.exit(1)

    # --- GET USER INPUT ---
    dataset_input = input("Enter dataset choice (1-11) [Press Enter for default: 1]: ").strip()
    dataset_id = dataset_input if dataset_input else "1"

    core_input = input("Enter core fraction divisor [Press Enter for default: 2]: ").strip()
    core_factor = core_input if core_input else "2"

    # --- GET SIMULATED ANNEALING PARAMETERS --- 
    print("\n--- Simulated Annealing Parameters ---")
    start_temp = input("Start Temp [Default 1000.0]: ").strip() or "1000.0"
    cool_rate = input("Cooling Rate [Default 0.95]: ").strip() or "0.95"
    final_temp = input("Final Temp [Default 0.001]: ").strip() or "0.001"
    moves = input("Moves per Temp [Default 10000]: ").strip() or "10000"

    print(f"Simulated Annealing with Dataset {dataset_id}...\n")
    print("=" * 50)
    # --- START THE CLOCK ---
    start_time = time.perf_counter()

    # --- RUN THE C++ ENGINE ---
    try:
        subprocess.run([exe_path, dataset_id, core_factor, start_temp, cool_rate, final_temp, moves], cwd=project_dir, check=True)
        
    except subprocess.CalledProcessError as e:
        print(f"\n[ERROR] The C++ engine crashed. Exit code: {e.returncode}")
    except KeyboardInterrupt:
        print("\nRun forcibly stopped by user.")
    
    # --- STOP THE CLOCK ---
    end_time = time.perf_counter()
    # Calculate the net time taken
    net_time = end_time - start_time

    print("=" * 50)
    print("\nSimulated Annealing Engine finished successfully!")
    print(f"[System] Net SA Execution Time: {net_time:.4f} seconds")
    
    # Open the JSON file for visualization purposes
    json_output_path = os.path.join(project_dir, "floorplan_output.json")
    # print(f"[System] Your data is waiting at: {json_output_path}")

    # 1. Open and load the JSON data
    try:
        with open(json_output_path, 'r') as file:
            data = json.load(file)
    except FileNotFoundError:
        print("[ERROR] Could not find the JSON file to draw!")
        return

    # Extract the outline dimensions
    out_x = data["outline_x"]
    out_y = data["outline_y"]

    fin_x = data["final_width"]
    fin_y = data["final_height"]

    # print()

    # 2. Set up the Matplotlib canvas
    # Create a figure and an axis
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_title(f"Simulated Annealing Floorplan (Area: {data['area']})", fontsize=14)

    # 3. Draw the Bounding Box (The Outline)
    # patches.Rectangle takes (bottom_left_x, bottom_left_y), width, height
    outline_box = patches.Rectangle((0, 0), out_x, out_y, 
                                    linewidth=2, edgecolor='red', facecolor='none', linestyle='--')
    ax.add_patch(outline_box)
    # Draw a box around the final dimentions
    outline_box = patches.Rectangle((0, 0), fin_x, fin_y, 
                                    linewidth=2, edgecolor='green', facecolor='none', linestyle='--')
    ax.add_patch(outline_box)

    # 4. Draw every Chiplet inside the box
    for chip in data["chiplets"]:
        # Create a rectangle for each chiplet
        rect = patches.Rectangle((chip["x"], chip["y"]), chip["w"], chip["h"],
                                 linewidth=1, edgecolor='black', facecolor='skyblue', alpha=0.8)
        ax.add_patch(rect)

    # 5. Scale the camera to fit everything perfectly
    buffer_x = out_x * 0.1
    buffer_y = out_y * 0.1
    
    # We also check the final computed width/height just in case the blocks exceeded the outline!
    max_x_view = max(out_x, data["final_width"]) + buffer_x
    max_y_view = max(out_y, data["final_height"]) + buffer_y

    ax.set_xlim(-buffer_x, max_x_view)
    ax.set_ylim(-buffer_y, max_y_view)

    # Force the X and Y axes to scale equally (so a 10x10 square actually looks like a square)
    ax.set_aspect('equal', adjustable='box')

    # Turn on a light grid for easier reading
    ax.grid(True, linestyle=':', alpha=0.6)

    print("[System] Rendering complete! Close the window to end the script.")
    
    # 6. Show the window!
    plt.show()

if __name__ == "__main__":
    main()
