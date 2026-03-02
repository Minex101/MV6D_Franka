import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

# --- CONFIG ---
DOPE_FILE       = os.path.expanduser("~/Downloads/dope_results_hope.npy")
CENTERPOSE_FILE = os.path.expanduser("~/Downloads/centerpose_results_hope.npy")
OUT_DIR         = os.path.expanduser("~/Downloads/")

# Thresholds for AUC calculation
TRANS_THRESHOLDS = np.linspace(0, 20.0, 1000)  # 0 to 20cm
ROT_THRESHOLDS   = np.linspace(0, 30.0, 1000)   # 0 to 30 degrees

# --- ROTATION FIXES ---
# Aligns DOPE to HOPE GT
DOPE_ROT_FIX = Rotation.from_euler('xyz', [0, 270, 270], degrees=True)

# Aligns CenterPose to HOPE GT (Found via search script)
CP_ROT_FIX   = Rotation.from_euler('xyz', [270, 0, 0], degrees=True)

def extract_errors(pred_file, rot_fix=None):
    if not os.path.exists(pred_file):
        return None
    
    preds    = np.load(pred_file, allow_pickle=True).item()
    total    = len(preds)
    
    trans_errors, rot_errors = [], []
    detected_count = 0

    for key, pred in preds.items():
        if pred is None:
            continue

        detected_count += 1
        
        # 1. Get Ground Truth (cm and Matrix)
        gt_pose  = np.array(pred['gt_pose'])
        gt_pos   = gt_pose[:3, 3]  
        gt_rot   = Rotation.from_matrix(gt_pose[:3, :3])

        # 2. Get Prediction (Convert m to cm)
        pred_pos = np.array(pred['position']) * 100.0  
        pred_rot = Rotation.from_quat(pred['orientation'])

        # 3. Apply specific rotation fix
        if rot_fix is not None:
            pred_rot = pred_rot * rot_fix

        # 4. Calculate Distance and Angular Errors
        trans_err = np.linalg.norm(pred_pos - gt_pos)
        rot_err   = np.degrees((pred_rot.inv() * gt_rot).magnitude())
        
        trans_errors.append(trans_err)
        rot_errors.append(rot_err)

    return np.array(trans_errors), np.array(rot_errors), detected_count, total

def compute_auc(errors, thresholds):
    if errors is None or len(errors) == 0:
        return 0.0, np.zeros_like(thresholds)
    recalls = np.array([np.mean(errors < t) for t in thresholds])
    auc     = np.trapz(recalls, thresholds) / (thresholds[-1] - thresholds[0])
    return auc, recalls

def main():
    results = {}
    colors  = {'DOPE': '#FF8C00', 'CenterPose': '#4682B4'}

    # Config for the two models
    model_configs = [
        ('DOPE',       DOPE_FILE,       DOPE_ROT_FIX),
        ('CenterPose', CENTERPOSE_FILE, CP_ROT_FIX),
    ]

    for label, path, rot_fix in model_configs:
        data = extract_errors(path, rot_fix)
        if data is None:
            print(f"Skipping {label}: File not found.")
            continue

        t_err, r_err, detected, total = data
        t_auc, t_recalls = compute_auc(t_err, TRANS_THRESHOLDS)
        r_auc, r_recalls = compute_auc(r_err, ROT_THRESHOLDS)

        print(f"\n── {label} Results ────────────────────────")
        print(f"  Recall (Detection Rate): {detected}/{total} ({100*detected/total:.1f}%)")
        print(f"  Translation Error: Mean={t_err.mean():.2f}cm, Median={np.median(t_err):.2f}cm")
        print(f"  Rotation Error:    Mean={r_err.mean():.1f}°, Median={np.median(r_err):.1f}°")
        print(f"  Translation AUC:   {t_auc:.4f}")
        print(f"  Rotation AUC:      {r_auc:.4f}")

        results[label] = {
            't_auc': t_auc, 'r_auc': r_auc,
            't_recalls': t_recalls, 'r_recalls': r_recalls,
            'detected': detected, 'total': total
        }

    # --- PLOTTING ---
    if not results:
        return

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Model Benchmark: DOPE vs CenterPose (HOPE Ketchup)', fontsize=14, fontweight='bold')

    # Translation Plot
    ax = axes[0]
    for name, r in results.items():
        ax.plot(TRANS_THRESHOLDS, r['t_recalls'] * 100, color=colors[name], 
                linewidth=2.5, label=f"{name} (AUC={r['t_auc']:.3f})")
        ax.fill_between(TRANS_THRESHOLDS, r['t_recalls'] * 100, alpha=0.1, color=colors[name])
    ax.set_title('Translation Precision (ADD)')
    ax.set_xlabel('Error Threshold (cm)')
    ax.set_ylabel('Success Rate (%)')
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 105)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='lower right')

    # Rotation Plot
    ax = axes[1]
    for name, r in results.items():
        ax.plot(ROT_THRESHOLDS, r['r_recalls'] * 100, color=colors[name], 
                linewidth=2.5, label=f"{name} (AUC={r['r_auc']:.3f})")
        ax.fill_between(ROT_THRESHOLDS, r['r_recalls'] * 100, alpha=0.1, color=colors[name])
    ax.set_title('Rotation Precision')
    ax.set_xlabel('Error Threshold (Degrees)')
    ax.set_ylabel('Success Rate (%)')
    ax.set_xlim(0, 30)
    ax.set_ylim(0, 105)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='lower right')

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_path = os.path.join(OUT_DIR, 'benchmark_comparison.png')
    plt.savefig(save_path, dpi=300)
    plt.show()
    print(f"\nBenchmark visualization saved to: {save_path}")

if __name__ == '__main__':
    main()