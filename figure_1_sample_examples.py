import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

from acrolib.quaternion import Quaternion
from acrobotics.util import get_default_axes3d, plot_reference_frame
from acrobotics.util import rot_z

from acrobotics.path.sampling import SampleMethod
from acrobotics.path.tolerance import (
    Tolerance,
    NoTolerance,
    SymmetricTolerance,
    QuaternionTolerance,
)
from acrobotics.path.path_pt import TolPositionPt, TolEulerPt, TolQuatPt

# ==================================================================================
# Position samples
# ==================================================================================
R_pt = rot_z(np.deg2rad(30))
pos_pt = np.array([0.5, 0.5, 0.0])

pos_tol = [Tolerance(-0.1, 0.1, 3), Tolerance(-0.3, 0.3, 5), NoTolerance()]

pt_pos = TolPositionPt(pos_pt, Quaternion(matrix=R_pt), pos_tol)

# ==================================================================================
# Euler constraints samples
# ==================================================================================
R_pt = np.eye(3)
pos_tol = 3 * [NoTolerance()]
rot_tol = [
    SymmetricTolerance(np.deg2rad(15), 5),
    NoTolerance(),
    Tolerance(0, np.pi / 2, 10),
]

pt_eul = TolEulerPt(np.zeros(3), Quaternion(matrix=R_pt), pos_tol, rot_tol)

# ==================================================================================
# Quaternion constraints samples
# ==================================================================================
R_pt = np.eye(4)
pos_tol = 3 * [NoTolerance()]
quat_tol = QuaternionTolerance(0.1)

pt_quat = TolQuatPt(np.zeros(3), Quaternion(matrix=R_pt), pos_tol, quat_tol)

# ==================================================================================
# Create plot and save it
# ==================================================================================
fig = plt.figure(figsize=plt.figaspect(1 / 3))
axes = [fig.add_subplot(1, 3, i, projection="3d") for i in [1, 2, 3]]

# all axis show the path point reference frame and have no coordinate axes
for pt, ax in zip([pt_pos, pt_eul, pt_quat], axes):
    ax.set_axis_off()
    plot_reference_frame(ax, tf=pt.transformation_matrix, arrow_length=0.2)


# here we plot the samples for each specific point
ax1, ax2, ax3 = axes[0], axes[1], axes[2]

plot_reference_frame(ax1, arrow_length=0.3)
for tf in pt_pos.sample_grid():
    ax1.scatter(tf[0, 3], tf[1, 3], tf[2, 3], "o", c="black")

for tf in pt_eul.sample_grid():
    plot_reference_frame(ax2, tf, arrow_length=0.1)


for tf in pt_quat.sample_incremental(50, SampleMethod.random_uniform):
    plot_reference_frame(ax3, tf, arrow_length=0.1)


# here we tweak the view a bit to make it look nice
ax1.view_init(azim=20, elev=38)
ax1.set_xlim3d([-0.5, 0.5])
ax1.set_ylim3d([-0.5, 0.5])
ax1.set_zlim3d([-0.5, 0.5])
ax2.view_init(azim=35, elev=38)
ax3.view_init(azim=35, elev=38)
plt.tight_layout()

plt.savefig("figures/sample_examples.png", dpi=200)
