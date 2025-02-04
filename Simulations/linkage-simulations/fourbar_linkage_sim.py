import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt

RES = 20
l1 = np.linspace(20,50,RES) # driver arm of the rotating member
l2 = np.linspace(20,50,RES) # coupling arm attached to the thing
l3 = np.linspace(20,50,RES) # Hinge X position
l4 = 32.5 #Approximate height of the hinge

def quadsolve_gen(l1, l2, l3, l4, theta):
    """
    Given an output rod angle theta that L3 makes with L4, solves for the angles alpha, beta, and gamma
    
    alpha: l4 l1
    
    beta: l1 l2
    
    gamma: l2 l3
    
    theta: l3 l4 (given)
    """
    d1 = np.sqrt(l3**2 + l4**2 - 2*l3*l4*np.cos(theta))
    beta = np.arccos( ( d1**2 - (l1**2 + l2**2)) / (-2 * l1 * l2 ) )
    alpha = np.arcsin( (l3/d1) * np.sin(theta)) + np.arcsin( (l2/d1) * np.sin(beta))
    gamma = np.arcsin( (l4/d1) * np.sin(theta)) + np.arcsin( (l1/d1) * np.sin(beta))
    return (beta, alpha, gamma)
    
        
    

    
    
def quadsolve_phi(l1, l2, l3, l4, gamma):
    # Cosine law
    d1_squared = l3**2 + l4**2 - 2*l3*l4*np.cos(gamma)
    # if squared < 0, past limit range of zero angle
    if d1_squared < 0 :
        return float("nan")

    cos_phi = (d1_squared - l1**2 - l2**2) / (-1*l1*l2)    
    # if cos_phi not between (-1,1), past 180deg limit angle
    if abs(cos_phi) > 1:
        return float("nan")
    phi = np.arccos(cos_phi)
    return phi
    
l1_mesh,l2_mesh, l3_mesh = np.meshgrid(l1, l2, l3)

plate_limit_angle = np.array([-25,25])
plate_limit_theta = 90 - plate_limit_angle


theta = np.linspace(np.deg2rad(plate_limit_theta[0]), np.deg2rad(plate_limit_theta[1]), RES, endpoint=True)

advantage = np.zeros_like(l1_mesh)
for i in range(len(l1)):
    for j in range(len(l2)):
        for k in range(len(l3)):
            adv = np.inf #Best case scenario is infinite advantage
            for t in theta:
                alpha, beta, gamma = quadsolve_gen(l1[i], l2[j], l3[k], l4, t)
                adv_new = (l3[k]*np.sin(gamma))/(l1[i]*np.sin(beta))
                if adv > adv_new:
                    adv = adv_new
                if np.isnan(adv_new) or np.isnan(adv):
                    adv = np.nan


            advantage[i][j][k] = adv


print(advantage)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
advantage_filtered = advantage
advantage_filtered[advantage_filtered<1] = np.nan
scatter = ax.scatter(l1_mesh, l2_mesh, l3_mesh, c=advantage_filtered, cmap='coolwarm')
ax.set_title("Torque ratio variation vs. L1, L2, L3")
ax.set_xlabel("Driver (L1) (mm)")
ax.set_ylabel("Coupler (L2) (mm)")
ax.set_zlabel("Follower (L3) (mm)")
cbar = fig.colorbar(scatter, ax=ax)
cbar.set_label("Worst Case Torque Ratio")


advantage_filtered[np.isnan(advantage_filtered)] = 0
max_idx = np.argmax(advantage_filtered)
max_idx = np.unravel_index(max_idx, advantage_filtered.shape)

torque_ratio = advantage_filtered[max_idx[0]][max_idx[1]][max_idx[2]]
driver_length = l1[max_idx[0]]
coupler_length = l2[max_idx[1]]
follower_length = l3[max_idx[2]]
frame_length = l4

print(f"OPTIMAL CONDITIONS")
print(f"Best torque ratio:      {torque_ratio:.3f} ")
print(f"Best driver length:     {driver_length:.2f} (mm)")
print(f"Best coupler length:    {coupler_length:.2f} (mm)")
print(f"Best follower length:   {follower_length:.2f} (mm)")
print(f"Frame length:           {frame_length:.2f} (mm)")
print("\n\n\n")
plt.show()
