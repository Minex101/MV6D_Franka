import numpy as np
import os

# --- CONFIG ---
DOPE_FILE       = os.path.expanduser("~/Downloads/dope_results_hope.npy")
CENTERPOSE_FILE = os.path.expanduser("~/Downloads/centerpose_results_hope.npy")

def print_comparison():
    # Load files
    if not os.path.exists(DOPE_FILE) or not os.path.exists(CENTERPOSE_FILE):
        print("Error: One or both .npy files are missing.")
        return

    dope_results = np.load(DOPE_FILE, allow_pickle=True).item()
    cp_results   = np.load(CENTERPOSE_FILE, allow_pickle=True).item()

    print(f"{'Frame / Scene':<35} | {'Source':<12} | {'X':<8} {'Y':<8} {'Z (Depth)':<8} | {'Error (cm)':<10}")
    print("-" * 95)

    for key in dope_results.keys():
        # Get Ground Truth (Always in CM in HOPE)
        gt_pose = np.array(dope_results[key]['gt_pose'])
        gt_xyz  = gt_pose[:3, 3]
        
        scene_name = "/".join(key.split("/")[-2:])
        print(f"{scene_name:<35} | {'GROUND TRUTH':<12} | {gt_xyz[0]:>7.2f} {gt_xyz[1]:>7.2f} {gt_xyz[2]:>7.2f} | ---")

        # --- DOPE ---
        d_pred = dope_results.get(key)
        if d_pred:
            d_xyz = np.array(d_pred['position']) * 100.0 # Convert m to cm
            d_err = np.linalg.norm(d_xyz - gt_xyz)
            print(f"{'':<35} | {'DOPE':<12} | {d_xyz[0]:>7.2f} {d_xyz[1]:>7.2f} {d_xyz[2]:>7.2f} | {d_err:>8.2f} cm")

        # --- CenterPose ---
        c_pred = cp_results.get(key)
        if c_pred:
            c_xyz_raw = np.array(c_pred['position'])
            
            # Logic to check if CP is in meters or cm
            # If Z is > 500, it's likely a scaled cm error. If Z is ~18, it's meters.
            c_xyz_cm = c_xyz_raw * 100.0 if np.max(np.abs(c_xyz_raw)) < 50.0 else c_xyz_raw
            
            c_err = np.linalg.norm(c_xyz_cm - gt_xyz)
            
            # Labeling the error clearly
            error_str = f"{c_err:>8.2f} cm"
            if c_err > 1000:
                error_str += " (!!) " # Highlight the catastrophic scale error

            print(f"{'':<35} | {'CenterPose':<12} | {c_xyz_cm[0]:>7.2f} {c_xyz_cm[1]:>7.2f} {c_xyz_cm[2]:>7.2f} | {error_str}")

        print("-" * 95)

if __name__ == "__main__":
    print_comparison()