# Walking Simulations in OpenSim Moco

This repository contains data and code needed to generate walking simulations for subject S4. All example runs are for a single gait cycle with a single weight on the energy expenditure term used. The code can be expanded to run additional gait cycles or energy expenditure weights. 

*All simulations were completed in OpenSim 4.5.1*
 
## Initial Guess Generation

We used the following procedure to identify:
* GRF weight ( `w1` )  
* Kinematics weight ( `w2` )  
and to generate a muscle‑driven initial guess for each subject.

The same `w1` and `w2` values were applied to all subjects, and steps 1 and 2 were repeated for every subject to keep the pipeline consistent.

1. **Identify GRF weight**: run a torque‑driven simulation with only a GRF
   tracking term (`w1`).  
2. **Identify kinematics weight**: use the solution from step 1 as an initial
   guess, add the kinematic tracking term, and run again ( `w2` ).  
3. **Generate muscle‑driven initial guess**: use the solution from step 2
   as the initial guess for the muscle driven problem across all cost function
   formulations, producing a set of muscle‑driven initial guesses per subject.



## S4 Information

For S4, the following GCs/GC times were used: 



**Initial guess:** GC 9 - 10.6s to 11.85s
*(the initial guess GC was used to create the muscle-driven initial guess. This was not included as one of the 10 gait cycles used in the paper's analysis)*

| Gait Cycle # | Start Time (s) | Stop Time (s) |
| ------------ | ---------- | --------- |
| 2 | 1.71 | 2.99 |
| 3 | 2.99 | 4.26 |
| 10 | 11.85 | 13.14 |
| 14 | 16.91 | 18.21 |
| 15 | 18.21 | 19.5 |
| 18 | 22.02 | 23.29 |
| 23 | 28.35 | 29.62 |
| 25 | 30.87 | 32.13 |
| 30 | 37.2 | 38.46 |
| 32 | 39.71 | 40.97 |

## Cost Function Weights
Eleven different cost‑function formulations were used:

* **Effort simulations**: nine constant weights.
* **CoT simulations**: weights were chosen dynamically
  until a low or high CoT solution was found. 
  To streamline this, a grid search over `w4 = (0.001, 0.0055, 0.01, 0.055, 0.1, 0.55, 1.0, 1.55)` was performed. Some gait cycles required a different CoT weight to achieve the target
  CoT, so the initial CoT guess remained as “Grid 1, ..., Grid 8”. In the results, each grid is labeled as HighCoT or LowCoT. 

### Effort
| Grid # | Weight |
| ------------ | ---------- |
| 1 | 0.00001 |
| 2 | 0.000055 | 
| 3 | 0.0001 |
| 4 | 0.00055 |
| 5 | 0.001 |
| 6 | 0.0055 |
| 7 | 0.01 |
| 8 | 0.055 | 
| 9 | 0.1 |

### CoT Weights 
*(These are the specific weights that were used for S4)*

| Gait Cycle # | High CoT Weight | Low CoT Weight |
| ------------ | ---------- | --------- |
| 2 | 0.0055 | 0.55 |
| 3 | 0.0055 | 0.1 |
| 10 | 0.01 | 0.1 |
| 14 | 0.0055 | 0.1 |
| 15 | 0.001 | 0.1 |
| 18 | 0.0055 | 0.1 |
| 23 | 0.0055 | 0.55 |
| 25 | 0.0055 | 0.1 |
| 30 | 0.0055 | 0.55 |
| 32 | 0.01 | 0.1 |


## Code

The following code is included to run tracking simulations. 

* `MocoTrack_w1.py` – runs a torque‑driven simulation using the fixed GRF weight (`w1`).  
* `MocoTrack_w2.py` – uses the result from `w1` as an initial guess, adds the kinematic tracking term (`w2`), and creates the final torque‑driven initial guess.  
* `MocoTrack_Effort.py` – muscle driven simulations using effort as the energy expenditure term in the cost function.  
* `MocoTrack_CoT.py` – muscle driven simulations using metabolic CoT as the energy expenditure term in the cost function.


Currently, the `w2` and main scripts are set up to use the initial guesses from the results.  
If you want to run your own simulation, be sure to first run `w1`, then update the file paths in the subsequent scripts.  

If you want to follow the entire pipeline used in the paper (step 1 with `w1`, step 2 with `w2`, creating a muscle driven initial guess, then running muscle driven simulations), you must run `MocoTrack_Effort` and `MocoTrack_CoT` initially with `w2` as the initial guess. From there, you will have a muscle driven initial guess for all subsequent gait cycles. Each initial guess belongs to a specific weight grid.
Use the same grid for both the weight and the initial guess (e.g., grid 1 weight with the grid 1 guess).
