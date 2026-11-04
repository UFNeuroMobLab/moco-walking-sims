"""
File: MocoTrack_w2.py
Author: Kaitlyn Downer
Date Created: 5/29/25

Inputs: 
init_guess: initial guess file (.sto)
start_time: simulation start time (s)
stop_time: simulation stop time (s)
subject_ID: subject file identifier
motion_file: experimental kinematics to be tracked (.mot or .sto)
model_name: name of scaled model to be used
control_weight: weight on control effort term (low weight for actuator control)
grf_file: experimental GRFs to be tracked (.xml)
solution_name: desired kinematic solution name (.sto)
GRF_solution_name: desired GRF solution name (.sto)
coordBounds: coordinate bounds from the experimental data
"""

import pandas as pd
import opensim as osim
from pathlib import Path

# --------- DEFINE INPUTS TO THE TRACKING SIMULATION ---------

base_dir = Path(__file__).resolve().parent

init_guess = str(base_dir / 'Results' / 'Initial Guess' / 'Torque Driven' /'GC9_w1.sto')
start_time = 10.6
stop_time = 11.95
subject_ID = 'S4'
motion_file = str(base_dir / 'Input Files' / 'S4_GC9_Trial01_ik_tx_rad.sto')
model_name = str(base_dir / 'Input Files' / 'S4_scaled_fittedPaths.osim')
control_weight = 0.001
grf_file = str(base_dir / 'Input Files' / 'S4_GC9_Trial01_GRF.xml')
solution_name = 'S4_GC9_w1.sto'
GRF_solution_name = 'S4_GC9_w1_GRF.sto'
coordBounds = str(base_dir / 'Input Files' / 'CoordBounds_SD.xlsx')

# ---------------------- MAIN SIMULATION ----------------------

def tracking_sim_effort(init_guess, start_time, stop_time, subject_ID, motion_file, model_name, control_weight ,grf_file, solution_name, GRF_solution_name):
    
    # --------- SET UP INITIAL TRACKING SIMULATION ---------

    # Define OpenSim Moco tracking problem
    track = osim.MocoTrack()
    track.setName("gaitTracking")

    # Load experimental data to be tracked and apply a 6 Hz low pass filter
    tableProcessor = osim.TableProcessor(motion_file)
    tableProcessor.append(osim.TabOpLowPassFilter(6))

    # Load and set subject-specific model, set tendon compliance, and change to torque-driven only
    modelProcessor = osim.ModelProcessor(model_name)
    modelProcessor.append(osim.ModOpTendonComplianceDynamicsModeDGF('implicit'))
    modelProcessor.append(osim.ModOpRemoveMuscles())
    modelProcessor.append(osim.ModOpAddReserves(500,1))

    # Weld locked joints for complatibility in OpenSim Moco
    jointNames = osim.StdVectorString()
    jointNames.append('mtp_l')
    jointNames.append('mtp_r')
    jointNames.append('subtalar_l')
    jointNames.append('subtalar_r')
    modelProcessor.append(osim.ModOpReplaceJointsWithWelds(jointNames))

    # Define settings for the tracking problem
    track.setModel(modelProcessor)
    track.setStatesReference(tableProcessor)
    track.set_allow_unused_references(True)
    track.set_track_reference_position_derivatives(True)
    track.set_apply_tracked_states_to_guess(True)
    track.set_initial_time(start_time)
    track.set_final_time(stop_time)
    track.set_control_effort_weight(control_weight)

    # Load the standard deviation and range data to use
    file_path = coordBounds
    sheet_name = str(subject_ID)
    columns = 'A:H'  
    num_rows = 19  
    df = pd.read_excel(file_path, sheet_name=sheet_name, usecols=columns, nrows=num_rows, index_col=0)
    coordinate_state_weights = df['SD']
    total_bounds = df[['ProblemLower','ProblemUpper']]
    coordinate_bounds_initial = df[['InitLower', 'InitUpper']]
    coordinate_bounds_final = df[['FinalLower','FinalUpper']]

    # Set tracking weights, customized to the coordinate dependent on the SD found in the experimental data
    pq = 0.1 / 19 # 19 DOF
    stateWeights = osim.MocoWeightSet()
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ground_pelvis/pelvis_tx/value', pq/(0.01**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ground_pelvis/pelvis_ty/value', 0))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ground_pelvis/pelvis_tz/value', pq / ((coordinate_state_weights.loc['pelvis_tz']**2))))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ground_pelvis/pelvis_tilt/value', pq / (coordinate_state_weights.loc['pelvis_tilt']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ground_pelvis/pelvis_list/value', pq / (coordinate_state_weights.loc['pelvis_list']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ground_pelvis/pelvis_rotation/value', pq / (coordinate_state_weights.loc['pelvis_rotation']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/back/lumbar_extension/value', pq / (coordinate_state_weights.loc['lumbar_extension']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/back/lumbar_bending/value', pq / (coordinate_state_weights.loc['lumbar_bending']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/back/lumbar_rotation/value', pq / (coordinate_state_weights.loc['lumbar_rotation']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/hip_r/hip_flexion_r/value', pq / (coordinate_state_weights.loc['hip_flexion_r']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/hip_r/hip_adduction_r/value', pq / (coordinate_state_weights.loc['hip_adduction_r']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/hip_r/hip_rotation_r/value', pq / ((coordinate_state_weights.loc['hip_rotation_r']**2))))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/walker_knee_r/knee_angle_r/value', pq / (coordinate_state_weights.loc['knee_angle_r']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ankle_r/ankle_angle_r/value', pq / (coordinate_state_weights.loc['ankle_angle_r']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/hip_l/hip_flexion_l/value', pq / (coordinate_state_weights.loc['hip_flexion_l']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/hip_l/hip_adduction_l/value', pq / (coordinate_state_weights.loc['hip_adduction_l']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/hip_l/hip_rotation_l/value', pq / ((coordinate_state_weights.loc['hip_rotation_l']**2))))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/walker_knee_l/knee_angle_l/value', pq / (coordinate_state_weights.loc['knee_angle_l']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ankle_l/ankle_angle_l/value', pq / (coordinate_state_weights.loc['ankle_angle_l']**2)))
    
    pw = pq * 0.0001  # Scale the generalized speed tracking errors by this constant
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ground_pelvis/pelvis_tx/speed', pw/((0.01**2))))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ground_pelvis/pelvis_ty/speed', 0))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ground_pelvis/pelvis_tz/speed', pw / ((coordinate_state_weights.loc['pelvis_tz']**2))))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ground_pelvis/pelvis_tilt/speed', pw / (coordinate_state_weights.loc['pelvis_tilt']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ground_pelvis/pelvis_list/speed', pw / (coordinate_state_weights.loc['pelvis_list']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ground_pelvis/pelvis_rotation/speed', pw / (coordinate_state_weights.loc['pelvis_rotation']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/back/lumbar_extension/speed', pw / (coordinate_state_weights.loc['lumbar_extension']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/back/lumbar_bending/speed', pw / (coordinate_state_weights.loc['lumbar_bending']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/back/lumbar_rotation/speed', pw / (coordinate_state_weights.loc['lumbar_rotation']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/hip_r/hip_flexion_r/speed', pw / (coordinate_state_weights.loc['hip_flexion_r']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/hip_r/hip_adduction_r/speed', pw / (coordinate_state_weights.loc['hip_adduction_r']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/hip_r/hip_rotation_r/speed', pw / ((coordinate_state_weights.loc['hip_rotation_r']**2))))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/walker_knee_r/knee_angle_r/speed', pw / (coordinate_state_weights.loc['knee_angle_r']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ankle_r/ankle_angle_r/speed', pw / (coordinate_state_weights.loc['ankle_angle_r']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/hip_l/hip_flexion_l/speed', pw / (coordinate_state_weights.loc['hip_flexion_l'])**2))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/hip_l/hip_adduction_l/speed', pw / (coordinate_state_weights.loc['hip_adduction_l'])**2))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/hip_l/hip_rotation_l/speed', pw / ((coordinate_state_weights.loc['hip_rotation_l'])**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/walker_knee_l/knee_angle_l/speed', pw / (coordinate_state_weights.loc['knee_angle_l']**2)))
    stateWeights.cloneAndAppend(osim.MocoWeight('/jointset/ankle_l/ankle_angle_l/speed', pw / (coordinate_state_weights.loc['ankle_angle_l']**2)))
    track.set_states_weight_set(stateWeights)

    # Initialize the Moco study, add the problem, and add model
    study = track.initialize()
    problem = study.updProblem()
    model = modelProcessor.process()
    model.initSystem()

    # --------- ADD GOALS TO PROBLEM ---------
    
    # Add "effort" goal, or sum of squared muscle activations
    controlEffort = osim.MocoControlGoal().safeDownCast(
                    problem.updGoal('control_effort'))
    controlEffort.setExponent(2)

    # Put a large weight on the pelvis CoordinateActuators, which act as the
    # residual, or 'hand-of-god', forces which we would like to keep as small
    # as possible.
    model = modelProcessor.process()
    model.initSystem()
    forceSet = model.getForceSet()
    for i in range(forceSet.getSize()):
        forcePath = forceSet.get(i).getAbsolutePathString()
        if 'pelvis' in str(forcePath):
            controlEffort.setWeightForControl(forcePath, 10)

    # Add pelvis periodicity goal, which helps to minimize GRF oscillations
    periodicityGoal = osim.MocoPeriodicityGoal('periodicity')
    periodicityGoal.addStatePair(osim.MocoPeriodicityGoalPair("/jointset/ground_pelvis/pelvis_ty/value"))
    problem.addGoal(periodicityGoal)
    
    # Add GRF tracking goal
    contactTracking = osim.MocoContactTrackingGoal('contact', 0.1)
    contactTracking.setExternalLoadsFile(grf_file)
    forceNamesRightFoot = ['forceset/contactHeel_r',
                           'forceset/contactLatheel_r',
                           'forceset/contactMedheel_r',
                           'forceset/contactMid_r',
                           'forceset/contactLatmid_r',
                           'forceset/contactMedmid_r', 
                           'forceset/contactMh1_r',
                           'forceset/contactMh3_r',
                           'forceset/contactMh5_r',
                           'forceset/contactHallux_r',
                           'forceset/contactOthertoes_r']
    
    forceNamesLeftFoot = ['forceset/contactHeel_l',
                           'forceset/contactLatheel_l',
                           'forceset/contactMedheel_l',
                           'forceset/contactMid_l',
                           'forceset/contactLatmid_l',
                           'forceset/contactMedmid_l', 
                           'forceset/contactMh1_l',
                           'forceset/contactMh3_l',
                           'forceset/contactMh5_l',
                           'forceset/contactHallux_l',
                           'forceset/contactOthertoes_l']
    rightContactGroup = osim.MocoContactTrackingGoalGroup(
                    forceNamesRightFoot, 'right', ['/bodyset/toes_r'])
    leftContactGroup = osim.MocoContactTrackingGoalGroup(
                    forceNamesLeftFoot, 'left', ['/bodyset/toes_l'])
    contactTracking.addContactGroup(rightContactGroup)
    contactTracking.addContactGroup(leftContactGroup)
    problem.addGoal(contactTracking)    

    # --------- SET PROBLEM BOUNDS ---------

    # Set initial bounds, final bounds, and overall coordinate bounds
    problem.setStateInfo('/jointset/ground_pelvis/pelvis_tilt/value', 
                         total_bounds.loc['pelvis_tilt'].tolist(), 
                         coordinate_bounds_initial.loc['pelvis_tilt'].tolist(), 
                         coordinate_bounds_final.loc['pelvis_tilt'].tolist())

    problem.setStateInfo('/jointset/ground_pelvis/pelvis_list/value', 
                         total_bounds.loc['pelvis_list'].tolist(), 
                         coordinate_bounds_initial.loc['pelvis_list'].tolist(), 
                         coordinate_bounds_final.loc['pelvis_list'].tolist())
    
    problem.setStateInfo('/jointset/ground_pelvis/pelvis_rotation/value', 
                         total_bounds.loc['pelvis_rotation'].tolist(), 
                         coordinate_bounds_initial.loc['pelvis_rotation'].tolist(), 
                         coordinate_bounds_final.loc['pelvis_rotation'].tolist())
    
    problem.setStateInfo('/jointset/ground_pelvis/pelvis_tx/value', 
                         total_bounds.loc['pelvis_tx'].tolist())
    
    ty_bounds = total_bounds.loc['pelvis_ty'].tolist()
    print("Original ty bounds:", ty_bounds)
    modified_bounds = [ty_bounds[0] - 0.5, ty_bounds[1] + 0.5]
    print("Modified ty bounds:", modified_bounds)

    problem.setStateInfo('/jointset/ground_pelvis/pelvis_ty/value', modified_bounds)
        
    problem.setStateInfo('/jointset/ground_pelvis/pelvis_tz/value', 
                         total_bounds.loc['pelvis_tz'].tolist(), 
                         coordinate_bounds_initial.loc['pelvis_tz'].tolist(), 
                         coordinate_bounds_final.loc['pelvis_tz'].tolist())
    
    problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value', 
                         total_bounds.loc['hip_flexion_l'].tolist(), 
                         coordinate_bounds_initial.loc['hip_flexion_l'].tolist(), 
                         coordinate_bounds_final.loc['hip_flexion_l'].tolist())
    
    problem.setStateInfo('/jointset/hip_l/hip_adduction_l/value', 
                         total_bounds.loc['hip_adduction_l'].tolist(), 
                         coordinate_bounds_initial.loc['hip_adduction_l'].tolist(), 
                         coordinate_bounds_final.loc['hip_adduction_l'].tolist())
    
    problem.setStateInfo('/jointset/hip_l/hip_rotation_l/value', 
                         total_bounds.loc['hip_rotation_l'].tolist(), 
                         coordinate_bounds_initial.loc['hip_rotation_l'].tolist(), 
                         coordinate_bounds_final.loc['hip_rotation_l'].tolist())
    
    problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', 
                         total_bounds.loc['hip_flexion_r'].tolist(), 
                         coordinate_bounds_initial.loc['hip_flexion_r'].tolist(), 
                         coordinate_bounds_final.loc['hip_flexion_r'].tolist())
    
    problem.setStateInfo('/jointset/hip_r/hip_adduction_r/value', 
                         total_bounds.loc['hip_adduction_r'].tolist(), 
                         coordinate_bounds_initial.loc['hip_adduction_r'].tolist(), 
                         coordinate_bounds_final.loc['hip_adduction_r'].tolist())
    
    problem.setStateInfo('/jointset/hip_r/hip_rotation_r/value', 
                         total_bounds.loc['hip_rotation_r'].tolist(), 
                         coordinate_bounds_initial.loc['hip_rotation_r'].tolist(), 
                         coordinate_bounds_final.loc['hip_rotation_r'].tolist())
    
    problem.setStateInfo('/jointset/walker_knee_l/knee_angle_l/value', 
                         total_bounds.loc['knee_angle_l'].tolist(), 
                         coordinate_bounds_initial.loc['knee_angle_l'].tolist(), 
                         coordinate_bounds_final.loc['knee_angle_l'].tolist())
    
    problem.setStateInfo('/jointset/walker_knee_r/knee_angle_r/value', 
                         total_bounds.loc['knee_angle_r'].tolist(), 
                         coordinate_bounds_initial.loc['knee_angle_r'].tolist(), 
                         coordinate_bounds_final.loc['knee_angle_r'].tolist())
    
    problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value', 
                         total_bounds.loc['ankle_angle_l'].tolist(), 
                         coordinate_bounds_initial.loc['ankle_angle_l'].tolist(), 
                         coordinate_bounds_final.loc['ankle_angle_l'].tolist())
    
    problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', 
                         total_bounds.loc['ankle_angle_r'].tolist(), 
                         coordinate_bounds_initial.loc['ankle_angle_r'].tolist(), 
                         coordinate_bounds_final.loc['ankle_angle_r'].tolist())
    
    problem.setStateInfo('/jointset/back/lumbar_extension/value', 
                         total_bounds.loc['lumbar_extension'].tolist(), 
                         coordinate_bounds_initial.loc['lumbar_extension'].tolist(), 
                         coordinate_bounds_final.loc['lumbar_extension'].tolist())
    
    problem.setStateInfo('/jointset/back/lumbar_bending/value', 
                         total_bounds.loc['lumbar_bending'].tolist(), 
                         coordinate_bounds_initial.loc['lumbar_bending'].tolist(), 
                         coordinate_bounds_final.loc['lumbar_bending'].tolist())
    
    problem.setStateInfo('/jointset/back/lumbar_rotation/value', 
                         total_bounds.loc['lumbar_rotation'].tolist(), 
                         coordinate_bounds_initial.loc['lumbar_rotation'].tolist(), 
                         coordinate_bounds_final.loc['lumbar_rotation'].tolist())

    # Set torque bounds
    torqueBounds = [-1.0, 1.0]
    problem.setControlInfoPattern('/forceset/torque.*', torqueBounds, [], [])
    problem.setControlInfoPattern('/forceset/reserve.*', torqueBounds, [], [])
    problem.setStateInfoPattern('.*torque.*/activation', torqueBounds, [], [])
    problem.setStateInfoPattern('.*reserve.*/activation', torqueBounds, [], [])
    
    # --------- CONFIGURE SOLVER SETTINGS ---------

    solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
    solver.set_parallel(5)
    solver.set_optim_max_iterations(15000)
    solver.set_optim_constraint_tolerance(1e-04)
    solver.set_optim_convergence_tolerance(1e-04)
    solver.set_multibody_dynamics_mode('explicit')
    solver.set_num_mesh_intervals(75)
    solver.set_enforce_constraint_derivatives(True)
    solver.resetProblem(problem)

    # --------- DEFINE INITIAL GUESS ---------

    solver.setGuessFile(init_guess)
    init_guess = solver.getGuess()
    #init_guess.write('initialGuess_w2.sto') # Optional print for sanity check

    # --------- SOLVE & SAVE RESULTS ---------

    solution = study.solve()
    solution.unseal()
    solution.write(solution_name)
    
    # --------- SAVE GRF FILE ---------

    model = modelProcessor.process();
    contact_r = osim.StdVectorString();
    contact_l = osim.StdVectorString();
    contact_r.append('/forceset/contactHeel_r')
    contact_r.append('/forceset/contactLatheel_r')
    contact_r.append('/forceset/contactMedheel_r')
    contact_r.append('/forceset/contactMid_r')
    contact_r.append('/forceset/contactLatmid_r')
    contact_r.append('/forceset/contactMedmid_r')
    contact_r.append('/forceset/contactMh1_r')
    contact_r.append('/forceset/contactMh3_r')
    contact_r.append('/forceset/contactMh5_r')
    contact_r.append('/forceset/contactHallux_r')
    contact_r.append('/forceset/contactOthertoes_r')
    contact_l.append('/forceset/contactHeel_l')
    contact_l.append('/forceset/contactLatheel_l')
    contact_l.append('/forceset/contactMedheel_l')
    contact_l.append('/forceset/contactMid_l')
    contact_l.append('/forceset/contactLatmid_l')
    contact_l.append('/forceset/contactMedmid_l')
    contact_l.append('/forceset/contactMh1_l')
    contact_l.append('/forceset/contactMh3_l')
    contact_l.append('/forceset/contactMh5_l')
    contact_l.append('/forceset/contactHallux_l')
    contact_l.append('/forceset/contactOthertoes_l')
    externalLoads = osim.createExternalLoadsTableForGait(model, solution,
                contact_r, contact_l)
    osim.STOFileAdapter.write(externalLoads, GRF_solution_name)

# --------- RUN SIMULATION ---------

tracking_sim_effort(init_guess, start_time, stop_time, subject_ID, motion_file, model_name, control_weight ,grf_file, solution_name, GRF_solution_name)