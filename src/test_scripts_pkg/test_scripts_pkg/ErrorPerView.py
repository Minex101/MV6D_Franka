import matplotlib.pyplot as plt
import numpy as np

views = ['Side 1', 'Side 2', 'Side 3', 'Side 4']
trans_errors = [0.38, 0.41, 0.71, 0.47]  # in cm
rot_errors = [4.17, 6.22, 6.29, 43.51]   # in degrees

T_THRESHOLD = 5.0  # 2cm
R_THRESHOLD = 5.0 # 15 degrees

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5,5))

# --- Translation Error per View ---
bars1 = ax1.bar(views, trans_errors, color='#D55E00', edgecolor='black')
ax1.axhline(y=T_THRESHOLD, color='#009E73', linestyle='--', label='Translation Tolerance (5cm)')
ax1.set_ylabel('Translation Error (cm)', fontsize=10)
ax1.set_title('Translation Accuracy by Perspective', fontsize=10)
ax1.set_ylim(0, 3.0)
ax1.legend()

# --- Rotation Error per View ---
bars2 = ax2.bar(views, rot_errors, color='#D55E00', edgecolor='black')
ax2.axhline(y=R_THRESHOLD, color='#009E73', linestyle='--', label='Rotation Tolerance (5°)')
ax2.set_ylabel('Rotation Error (Degrees)', fontsize=10)
ax2.set_title('Rotation Accuracy by Perspective', fontsize=10)
ax2.set_ylim(0, 50)
ax2.legend()

ax1.grid(axis='both', linestyle='--', alpha=0.7)

ax2.grid(axis='both', linestyle='--', alpha=0.7)

for bar in bars1:
    yval = bar.get_height()
    ax1.text(bar.get_x() + bar.get_width()/2, yval + 0.1, f'{yval}cm', ha='center', va='bottom')

for bar in bars2:
    yval = bar.get_height()
    ax2.text(bar.get_x() + bar.get_width()/2, yval + 1, f'{yval}°', ha='center', va='bottom')

plt.tight_layout()
plt.savefig("view_perspective_analysis.pdf")
plt.show()