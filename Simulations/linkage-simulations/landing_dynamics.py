import numpy as np
from scipy.constants import g
import math
from scipy.optimize import root

import matplotlib.pyplot as plt


# STATIC ANALYSIS - What is the maximum thrust of the drone during landing/picking up the rover?
# ASSUMPTIONS: 
# - Everything is non-moving (ignore rotation and moments)
# - 
# - 



# Constants
mu_static = 0.6 # Coefficient of static friction
theta = np.radians(25) # 25 degrees

# Function describing the relation
func = lambda mu, beta: (mu*np.cos(theta)-np.sin(theta))/(mu*np.cos(theta-beta)-np.sin(theta-beta))


# Drone angles (beta)
betas = np.linspace(0,np.radians(25),100)
# Static friction coefficients (mu)
mus = np.linspace(0.47,1,100)



mesh_mu, mesh_beta = np.meshgrid(mus, betas)
print(mesh_mu.shape, mesh_beta.shape)
twrs = func(mesh_mu, mesh_beta)
print(twrs.shape)



fig, axs = plt.subplots(1, 2)

fig.suptitle("Static Analysis")
ax1 = axs[0]
ax2 = axs[1]

CS = ax1.contour(np.rad2deg(mesh_beta), twrs, mesh_mu, 10)

ax1.set_title("TWR vs Drone Angle")
ax1.set_xlim(0,25)
ax1.set_ylim(0,1)
ax1.set_xlabel("Drone angle (deg)")
ax1.set_ylabel("TWR max")

CL1 = ax1.clabel(CS, CS.levels, inline=True, fmt=lambda x: rf"μ = {x:.2f}", fontsize=8, inline_spacing=-12)

# for ax in axs.flat:
CS2 = ax2.contour(np.rad2deg(mesh_beta), mesh_mu, twrs, 10)
ax2.set_title("Mu vs Drone Angle")
ax2.set_xlim(0,25)
ax2.set_ylim(0.4,1)
ax2.set_xlabel("Drone angle (deg)")
ax2.set_ylabel("Mu min")

ax2.clabel(CS2, CS2.levels, inline=True, fmt=lambda x: rf"TWR = {x:.2f}", fontsize=8, inline_spacing=-12)



fig.set_size_inches(18,10)

plt.show()