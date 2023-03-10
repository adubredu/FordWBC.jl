function compute_mm_desired_motor_positions(q_all_des, q_all_dot_des, problem::FabricProblem) 
    params = problem.task_data[:walk]
    min_yaw = deg2rad(-40)
    max_yaw = deg2rad(40)
    st_heading_bos = params[:swing_foot] == :right ? 
    wrap_to_pi(params[:q][di.qbase_yaw] - params[:q][di.qleftHipYaw]) : 
    wrap_to_pi(params[:q][di.qbase_yaw] - params[:q][di.qrightHipYaw])
    heading_error = wrap_to_pi(params[:target_heading] - st_heading_bos) 
    q_st_yaw_goal = heading_error

    yaw_error = wrap_to_pi(params[:q][di.qbase_yaw] - params[:target_heading]) 
    q_sw_yaw_goal = yaw_error 

    q_st_yaw_goal = clamp(q_st_yaw_goal, min_yaw, max_yaw)
    q_sw_yaw_goal = clamp(q_sw_yaw_goal, min_yaw, max_yaw)

    q_motors_des = zero(params[:qmotors])
    qdot_motors_des = zero(q_motors_des)
    if params[:swing_foot] == :right 
        q_motors_des[di.LeftKnee] = q_all_des[di.qleftKnee]
        q_motors_des[di.RightHipRoll] = q_all_des[di.qrightHipRoll]
        q_motors_des[di.RightHipPitch] = q_all_des[di.qrightHipPitch]
        q_motors_des[di.RightKnee] = q_all_des[di.qrightKnee]

        qdot_motors_des[di.LeftKnee] = q_all_dot_des[di.qleftKnee]
        qdot_motors_des[di.RightHipPitch] = q_all_dot_des[di.qrightHipPitch]
        qdot_motors_des[di.RightHipRoll] = q_all_dot_des[di.qrightHipRoll]
        qdot_motors_des[di.RightKnee] = q_all_dot_des[di.qrightKnee]

        q_motors_des[di.LeftHipYaw] = 0.5*q_st_yaw_goal
        q_motors_des[di.RightHipYaw] = 0.5*q_sw_yaw_goal
        qdot_motors_des[di.LeftHipYaw] = 0.0
        qdot_motors_des[di.LeftHipYaw] = 0.0

        q_motors_des[di.LeftToeA] = 0.0
        q_motors_des[di.LeftToeB] = 0.0
        toe_pitch, toe_roll = 0.0, 0.0
        toeA = kin.qm_toe(toe_pitch, toe_roll, "a", "right")
        toeB = kin.qm_toe(toe_pitch, toe_roll, "b", "right")
        q_motors_des[di.RightToeA] = toeA
        q_motors_des[di.RightToeB] = toeB
    else 
        q_motors_des[di.LeftHipRoll] = q_all_des[di.qleftHipRoll]
        q_motors_des[di.LeftHipPitch] = q_all_des[di.qleftHipPitch]
        q_motors_des[di.LeftKnee] = q_all_des[di.qleftKnee]
        q_motors_des[di.RightKnee] = q_all_des[di.qrightKnee]

        qdot_motors_des[di.LeftHipRoll] = q_all_dot_des[di.qleftHipRoll]
        qdot_motors_des[di.LeftHipPitch] = q_all_dot_des[di.qleftHipPitch]
        qdot_motors_des[di.LeftKnee] = q_all_dot_des[di.qleftKnee]
        qdot_motors_des[di.RightKnee] = q_all_dot_des[di.qrightKnee]

        q_motors_des[di.LeftHipYaw] = 0.5*q_sw_yaw_goal
        q_motors_des[di.RightHipYaw] = 0.5*q_st_yaw_goal
        qdot_motors_des[di.LeftHipYaw] = 0.0
        qdot_motors_des[di.RightHipYaw] = 0.0

        q_motors_des[di.RightToeA] = 0.0
        q_motors_des[di.RightToeB] = 0.0
        toe_pitch, toe_roll = 0.0, 0.0
        toeA = kin.qm_toe(toe_pitch, toe_roll, "a", "left")
        toeB = kin.qm_toe(toe_pitch, toe_roll, "b", "left")
        q_motors_des[di.LeftToeA] = toeA
        q_motors_des[di.LeftToeB] = toeB
    end
    q_motors_des[di.LeftShoulderRoll] = q_all_des[di.qleftShoulderRoll]
    q_motors_des[di.LeftShoulderPitch] = q_all_des[di.qleftShoulderPitch]
    q_motors_des[di.LeftShoulderYaw] = q_all_des[di.qleftShoulderYaw]
    q_motors_des[di.LeftElbow] = q_all_des[di.qleftElbow]

    q_motors_des[di.RightShoulderRoll] = q_all_des[di.qrightShoulderRoll]
    q_motors_des[di.RightShoulderPitch] = q_all_des[di.qrightShoulderPitch]
    q_motors_des[di.RightShoulderYaw] = q_all_des[di.qrightShoulderYaw]
    q_motors_des[di.RightElbow] = q_all_des[di.qrightElbow]

    return q_motors_des, qdot_motors_des
end

function compute_walking_command_torques(q_motors_des, qdot_motors_des, problem::FabricProblem)
    params = problem.task_data[:walk]
    ?? = zero(q_motors_des)
    v = zero(qdot_motors_des) 
    q_motors_error = params[:qmotors] - q_motors_des

    kp_hiproll_swing = 800.0 
    kp_hipyaw_swing = 400.0    
    kp_hippitch_swing = 800.0 
    kp_knee_swing = 1000.0
    kp_toe_swing = 200.0

    kp_hiproll_stance = 800.0  
    kp_hipyaw_stance = 400.0   
    kp_hippitch_stance = 800.0
    kp_knee_stance = 1000.0
    kp_toe_stance = 0.0

    kp_shoulderroll_stand = 100.0
    kp_shoulderpitch_stand = 100.0
    kp_shoulderyaw_stand = 100.0
    kp_elbow_stand = 100.0

    v = qdot_motors_des 

    if params[:swing_foot] == :right
        ??[di.LeftHipRoll] = -kp_hiproll_stance * params[:q][di.qbase_roll] 
        ??[di.LeftHipYaw] = -kp_hipyaw_stance * q_motors_error[di.LeftHipYaw] 

        ??[di.LeftHipPitch] = -kp_hippitch_stance * -params[:q][di.qbase_pitch] 
        ??[di.LeftKnee] = -kp_knee_stance * q_motors_error[di.LeftKnee] 

        ??[di.RightHipRoll] = -kp_hiproll_swing * q_motors_error[di.RightHipRoll] 
        ??[di.RightHipYaw] = -kp_hipyaw_swing * q_motors_error[di.RightHipYaw]
        ??[di.RightHipPitch] = -kp_hippitch_swing * q_motors_error[di.RightHipPitch] 
        ??[di.RightKnee] = -kp_knee_swing * q_motors_error[di.RightKnee] 

        ??[di.LeftToeA] = -kp_toe_stance * q_motors_error[di.LeftToeA] 
        ??[di.LeftToeB] = -kp_toe_stance * q_motors_error[di.LeftToeB] 
        ??[di.RightToeA] = -kp_toe_swing * q_motors_error[di.RightToeA] 
        ??[di.RightToeB] = -kp_toe_swing * q_motors_error[di.RightToeB] 
    else
        ??[di.LeftHipRoll] = -kp_hiproll_swing * q_motors_error[di.LeftHipRoll] 
        ??[di.LeftHipYaw] = -kp_hipyaw_swing * q_motors_error[di.LeftHipYaw] 
        ??[di.LeftHipPitch] = -kp_hippitch_swing * q_motors_error[di.LeftHipPitch] 
        ??[di.LeftKnee] = -kp_knee_swing * q_motors_error[di.LeftKnee] 

        ??[di.RightHipRoll] = -kp_hiproll_stance * params[:q][di.qbase_roll] 
        ??[di.RightHipYaw] = -kp_hipyaw_stance * q_motors_error[di.RightHipYaw]
        ??[di.RightHipPitch] = -kp_hippitch_stance * params[:q][di.qbase_pitch] 
        ??[di.RightKnee] = -kp_knee_stance * q_motors_error[di.RightKnee] 

        ??[di.LeftToeA] = -kp_toe_swing * q_motors_error[di.LeftToeA] 
        ??[di.LeftToeB] = -kp_toe_swing * q_motors_error[di.LeftToeB] 
        ??[di.RightToeA] = -kp_toe_stance * q_motors_error[di.RightToeA] 
        ??[di.RightToeB] = -kp_toe_stance * q_motors_error[di.RightToeB] 
    end

    ??[di.LeftShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[di.LeftShoulderRoll]
    ??[di.LeftShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[di.LeftShoulderPitch] 
    ??[di.LeftShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[di.LeftShoulderYaw]
    ??[di.LeftElbow] = -kp_elbow_stand * q_motors_error[di.LeftElbow]
    ??[di.RightShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[di.RightShoulderRoll]
    ??[di.RightShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[di.RightShoulderPitch] 
    ??[di.RightShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[di.RightShoulderYaw]
    ??[di.RightElbow] = -kp_elbow_stand * q_motors_error[di.RightElbow]

    u_knee_comp_ = 150.0; u_hiproll_comp_=30
    if (params[:swing_foot] == :right) 
        ??[di.LeftKnee] += u_knee_comp_
        ??[di.LeftHipRoll] -= u_hiproll_comp_ 
    else 
        ??[di.RightKnee] -= u_knee_comp_
        ??[di.RightHipRoll] += u_hiproll_comp_ 
    end   
    return ??, v
end

function compute_standing_command_torques(q, qdot, qdes, qdotdes, qmotors, problem::FabricProblem)
    q_motors = collect(qmotors)
    heading = q[di.qbase_yaw] 
    Rz = RotZ(heading)  

    # Aligned toes
    p_left_toe_world = kin.p_toe_pitch_joint_left(q)
    p_right_toe_world = kin.p_toe_pitch_joint_right(q)
    p_left_toe_aligned = Rz' * p_left_toe_world 
    p_right_toe_aligned = Rz' * p_right_toe_world

    # Aligned com 
    p_com_world = kin.p_COM(q)
    v_com_world = kin.v_COM(q, qdot)
    p_com_aligned =  Rz' * p_com_world
    v_com_aligned =  Rz' * v_com_world  

    # Main Fabric
    q_motors_des = copy(q_motors)

    q_motors_des[di.LeftHipRoll] = qdes[di.qleftHipRoll]
    q_motors_des[di.LeftHipPitch] = qdes[di.qleftHipPitch]
    q_motors_des[di.LeftKnee] = qdes[di.qleftKnee]
    q_motors_des[di.RightHipRoll] = qdes[di.qrightHipRoll]
    q_motors_des[di.RightHipPitch] = qdes[di.qrightHipPitch]
    q_motors_des[di.RightKnee] = qdes[di.qrightKnee]

    q_motors_des[di.LeftHipYaw] = 0
    q_motors_des[di.RightHipYaw] = 0

    q_motors_des[di.LeftShoulderRoll] = qdes[di.qleftShoulderRoll]
    q_motors_des[di.LeftShoulderPitch] = qdes[di.qleftShoulderPitch]
    q_motors_des[di.LeftShoulderYaw] = qdes[di.qleftShoulderYaw]
    q_motors_des[di.LeftElbow] = qdes[di.qleftElbow]
    q_motors_des[di.RightShoulderRoll] = qdes[di.qrightShoulderRoll]
    q_motors_des[di.RightShoulderPitch] = qdes[di.qrightShoulderPitch]
    q_motors_des[di.RightShoulderYaw] = qdes[di.qrightShoulderYaw]
    q_motors_des[di.RightElbow] = qdes[di.qrightElbow]     
    
    
    # toe_pitch, toe_roll = 0.0, 0.0
    # rtoeA = kin.qm_toe(toe_pitch, toe_roll, "a", "right")
    # rtoeB = kin.qm_toe(toe_pitch, toe_roll, "b", "right")
    # ltoeA = kin.qm_toe(toe_pitch, toe_roll, "a", "left")
    # ltoeB = kin.qm_toe(toe_pitch, toe_roll, "b", "left")
    # q_motors_des[di.RightToeA] = rtoeA
    # q_motors_des[di.RightToeB] = rtoeB
    # q_motors_des[di.LeftToeA] = ltoeA
    # q_motors_des[di.LeftToeB] = ltoeB

    com_midpoint_error = p_com_aligned - 0.5 * (p_left_toe_aligned + p_right_toe_aligned)
    toe_pitch_error = com_midpoint_error[1] 
    Kerr = 1.3 
    q_motors_des[di.LeftToeA] =  q_motors_des[di.LeftToeA]  + Kerr*toe_pitch_error
    q_motors_des[di.LeftToeB] = q_motors_des[di.LeftToeB] - Kerr*toe_pitch_error
    q_motors_des[di.RightToeA] = q_motors_des[di.RightToeA] - Kerr*toe_pitch_error
    q_motors_des[di.RightToeB] = q_motors_des[di.RightToeB] + Kerr*toe_pitch_error 

    ?? = zeros(di.NUM_MOTORS) 
    q_motors_error = q_motors - q_motors_des    
    
    kp_hiproll_stand = 800   
    kp_hipyaw_stand = 500.0
    kp_hippitch_stand = 500.0
    kp_knee_stand = 800.0
    kp_toe_stand = 400.0  
    kp_knee_comp_stand = 2700
    kd_knee_comp_stand = 300

    kp_shoulderroll_stand = 100.0
    kp_shoulderpitch_stand = 100.0
    kp_shoulderyaw_stand = 100.0
    kp_elbow_stand = 100.0

    ??[di.LeftHipRoll] = -kp_hiproll_stand * q_motors_error[di.LeftHipRoll]
    ??[di.LeftHipYaw] = -kp_hipyaw_stand * q_motors_error[di.LeftHipYaw]
    ??[di.LeftHipPitch] = -kp_hippitch_stand * q_motors_error[di.LeftHipPitch]
    ??[di.LeftKnee] = -kp_knee_stand * q_motors_error[di.LeftKnee]
    ??[di.RightHipRoll] = -kp_hiproll_stand * q_motors_error[di.RightHipRoll]
    ??[di.RightHipYaw] = -kp_hipyaw_stand * q_motors_error[di.RightHipYaw]
    ??[di.RightHipPitch] = -kp_hippitch_stand * q_motors_error[di.RightHipPitch]
    ??[di.RightKnee] = -kp_knee_stand * q_motors_error[di.RightKnee]

    ??[di.LeftShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[di.LeftShoulderRoll]
    ??[di.LeftShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[di.LeftShoulderPitch]
    ??[di.LeftShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[di.LeftShoulderYaw]
    ??[di.LeftElbow] = -kp_elbow_stand * q_motors_error[di.LeftElbow]
    ??[di.RightShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[di.RightShoulderRoll]
    ??[di.RightShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[di.RightShoulderPitch]
    ??[di.RightShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[di.RightShoulderYaw]
    ??[di.RightElbow] = -kp_elbow_stand * q_motors_error[di.RightElbow]

    kd_toe_stand = 50.0
    ??[di.LeftToeA] = -kp_toe_stand * q_motors_error[di.LeftToeA] + kd_toe_stand * v_com_aligned[1]
    ??[di.LeftToeB] = -kp_toe_stand * q_motors_error[di.LeftToeB] - kd_toe_stand * v_com_aligned[1]
    ??[di.RightToeA] = -kp_toe_stand * q_motors_error[di.RightToeA] - kd_toe_stand * v_com_aligned[1]
    ??[di.RightToeB] = -kp_toe_stand * q_motors_error[di.RightToeB] + kd_toe_stand * v_com_aligned[1]

    # knee compensation 
    knee_comp = -kp_knee_comp_stand * -com_midpoint_error[2] - kd_knee_comp_stand * -v_com_aligned[2] 
    ??[di.LeftKnee] += knee_comp
    ??[di.RightKnee] += knee_comp 

    v = zeros(di.NUM_MOTORS)   
    v[di.LeftHipRoll] = qdotdes[di.qleftHipRoll]
    v[di.LeftHipPitch] = qdotdes[di.qleftHipPitch]
    v[di.LeftKnee] = qdotdes[di.qleftKnee]
    v[di.RightHipRoll] = qdotdes[di.qrightHipRoll]
    v[di.RightHipPitch] = qdotdes[di.qrightHipPitch]
    v[di.RightKnee] = qdotdes[di.qrightKnee] 

    # fallback_opmode = 3
    # apply_command = true
    # ?? = 0.5*digit.damping

    # send_command(??, v, ??, fallback_opmode, apply_command)
    return ??, v
end

function compute_stand_inverse_kinematics(q, com_goal, digit; 
    max_iter=200, tolerance=1e-3, max??=0.1)
    ?? = copy(q)
    leg_indices = [di.qleftHipRoll, di.qleftHipPitch, di.qleftKnee, di.qrightHipRoll, di.qrightHipPitch, di.qrightKnee]  
    ??[di.qbase_pitch] = 0.0
    ??[di.qbase_roll] = 0.0
    ??[di.qleftHipYaw] = 0.0 
    ??[di.qrightHipYaw] = 0.0   
    iter = 1
    for _ = 1:max_iter
        ??[di.qleftShinToTarsus] = -??[di.qleftKnee]
        ??[di.qrightShinToTarsus] = -??[di.qrightKnee]
        com_pos = kin.p_com_wrt_feet(??)
        com_error = com_pos - com_goal 
        error = norm(com_error, Inf) 
        if error < tolerance 
            # printstyled("converged at iter $iter\n";color=:blue)
            break
        end
        Jall = kin.Jp_com_wrt_feet(??) 
        J = Jall[1:end, leg_indices]
        J[1:end, 3] -= Jall[1:end, di.qleftShinToTarsus]
        J[1:end, 6] -= Jall[1:end, di.qrightShinToTarsus]
        ???? = J \ com_error
        maxofD = max(????...)
        if maxofD > max?? ???? = max?? * (???? /maxofD) end 
        ??[leg_indices] -= ????   
        ??[leg_indices] = clamp.(??[leg_indices], digit.??_min[leg_indices], digit.??_max[leg_indices])
        iter+=1
    end
    if iter > max_iter printstyled("Did not converge"; bold=true, color=:red) end 
    return ??
end

function send_command_torques(torques, velocities, problem::FabricProblem)
    params = problem.task_data[:walk]
    fallback_opmode = 3
    apply_command = true
    ?? = 0.8*problem.digit.damping

    if !problem.task_data[:mm][:standing]
        if params[:swing_foot] == :right 
            torques[di.LeftToeA] = 0
            torques[di.LeftToeB] = 0
            ??[di.LeftToeA] = 0
            ??[di.LeftToeB] = 0
        else
            torques[di.RightToeA] = 0
            torques[di.RightToeB] = 0
            ??[di.RightToeA] = 0
            ??[di.RightToeB] = 0
        end
    end
    send_command(torques, velocities, ??, fallback_opmode, apply_command)
end
 
function mm_fabric_controller(q, qdot, qmotors, observation, digit, problem)
    qdes, qdotdes, torqdes = mm_fabric_compute(q, qdot, qmotors, observation, problem)
    q_motors_des, qdot_motors_des = compute_mm_desired_motor_positions(qdes, qdotdes, problem)
    ??, v = compute_standing_command_torques(q, qdot, qdes, qdotdes, qmotors, problem)          
    ??, v = compute_walking_command_torques(q_motors_des, qdot_motors_des, problem)
     if problem.task_data[:mm][:standing]
       ??, v = compute_standing_command_torques(q, qdot, qdes, qdotdes, qmotors, problem)         
    end 
    send_command_torques(??, v, problem)  
end 

function stand_control(observation, params; com_goal=[0.0,0.0,0.95])
    ??, ???? , q_motors = get_generalized_coordinates(observation)
    q_motors = collect(q_motors)
    step_width = 0.27 
    heading = ??[di.qbase_yaw]   
    Rz = RotZ(heading)  
    
    p_left_toe_world = kin.p_toe_pitch_joint_left(??)
    p_right_toe_world = kin.p_toe_pitch_joint_right(??)
    p_left_toe_aligned = Rz' * p_left_toe_world 
    p_right_toe_aligned = Rz' * p_right_toe_world

    p_com_world = kin.p_COM(??)
    v_com_world = kin.v_COM(??, ???? )
    p_com_aligned =  Rz' * p_com_world
    v_com_aligned =  Rz' * v_com_world  
 
    com_wrt_left_aligned_des = [0.0, -0.5*step_width, com_goal[3]]
    com_wrt_right_aligned_des = [0.0, 0.5*step_width, com_goal[3]]
    goal =  [(Rz * com_wrt_left_aligned_des)..., (Rz * com_wrt_right_aligned_des)...] 

    ??d = compute_stand_inverse_kinematics(??, goal, params[:digit])   
    q_motors_des = copy(q_motors) 

    q_motors_des[di.LeftHipRoll] = ??d[di.qleftHipRoll]
    q_motors_des[di.LeftHipPitch] = ??d[di.qleftHipPitch]
    q_motors_des[di.LeftKnee] = ??d[di.qleftKnee]
    q_motors_des[di.RightHipRoll] = ??d[di.qrightHipRoll]
    q_motors_des[di.RightHipPitch] = ??d[di.qrightHipPitch]
    q_motors_des[di.RightKnee] = ??d[di.qrightKnee]

    q_motors_des[di.LeftHipYaw] = 0.0
    q_motors_des[di.RightHipYaw] = 0.0

    q_motors_des[di.LeftShoulderRoll] = -0.15
    q_motors_des[di.LeftShoulderPitch] = 1.1
    q_motors_des[di.LeftShoulderYaw] = 0
    q_motors_des[di.LeftElbow] = -0.145
    q_motors_des[di.RightShoulderRoll] = 0.15
    q_motors_des[di.RightShoulderPitch] = -1.1
    q_motors_des[di.RightShoulderYaw] = 0
    q_motors_des[di.RightElbow] = 0.145

    com_midpoint_error = p_com_aligned - 0.5 * 
                (p_left_toe_aligned + p_right_toe_aligned)
    toe_pitch_error = com_midpoint_error[1] 

    q_motors_des[di.LeftToeA] =   q_motors[di.LeftToeA]  + toe_pitch_error
    q_motors_des[di.LeftToeB] = q_motors[di.LeftToeB] - toe_pitch_error
    q_motors_des[di.RightToeA] =  q_motors[di.RightToeA] - toe_pitch_error
    q_motors_des[di.RightToeB] =  q_motors[di.RightToeB]+ toe_pitch_error  

    ?? = zeros(20) 
    q_motors_error = q_motors - q_motors_des   

    kp_hiproll_stand = 800   
    kp_hipyaw_stand = 500.0
    kp_hippitch_stand = 500.0
    kp_knee_stand = 800.0
    kp_toe_stand = 300.0  
    kp_knee_comp_stand = 2700
    kd_knee_comp_stand = 300

    kp_shoulderroll_stand = 100.0
    kp_shoulderpitch_stand = 100.0
    kp_shoulderyaw_stand = 100.0
    kp_elbow_stand = 100.0 

    ??[di.LeftHipRoll] = -kp_hiproll_stand * q_motors_error[di.LeftHipRoll]
    ??[di.LeftHipYaw] = -kp_hipyaw_stand * q_motors_error[di.LeftHipYaw]
    ??[di.LeftHipPitch] = -kp_hippitch_stand * q_motors_error[di.LeftHipPitch]
    ??[di.LeftKnee] = -kp_knee_stand * q_motors_error[di.LeftKnee]
    ??[di.RightHipRoll] = -kp_hiproll_stand * q_motors_error[di.RightHipRoll]
    ??[di.RightHipYaw] = -kp_hipyaw_stand * q_motors_error[di.RightHipYaw]
    ??[di.RightHipPitch] = -kp_hippitch_stand * q_motors_error[di.RightHipPitch]
    ??[di.RightKnee] = -kp_knee_stand * q_motors_error[di.RightKnee]

    ??[di.LeftShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[di.LeftShoulderRoll]
    ??[di.LeftShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[di.LeftShoulderPitch]
    ??[di.LeftShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[di.LeftShoulderYaw]
    ??[di.LeftElbow] = -kp_elbow_stand * q_motors_error[di.LeftElbow]
    ??[di.RightShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[di.RightShoulderRoll]
    ??[di.RightShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[di.RightShoulderPitch]
    ??[di.RightShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[di.RightShoulderYaw]
    ??[di.RightElbow] = -kp_elbow_stand * q_motors_error[di.RightElbow]

    kd_toe_stand = 50;
    ??[di.LeftToeA] = -kp_toe_stand * q_motors_error[di.LeftToeA] + kd_toe_stand * v_com_aligned[1]
    ??[di.LeftToeB] = -kp_toe_stand * q_motors_error[di.LeftToeB] - kd_toe_stand * v_com_aligned[1]
    ??[di.RightToeA] = -kp_toe_stand * q_motors_error[di.RightToeA] - kd_toe_stand * v_com_aligned[1]
    ??[di.RightToeB] = -kp_toe_stand * q_motors_error[di.RightToeB] + kd_toe_stand * v_com_aligned[1]

    # knee_comp
    knee_comp = -kp_knee_comp_stand * -com_midpoint_error[2] - kd_knee_comp_stand * -v_com_aligned[2] 
    ??[di.LeftKnee] += knee_comp
    ??[di.RightKnee] += knee_comp
    v = zeros(20) 
    ?? = 0.8*params[:digit].damping
    fallback_opmode = 3 
    apply_command = true   
    send_command(??, v, ??, fallback_opmode, apply_command)
end

function set_locomotion_mode(;host=:sim)
    hostip = host == :sim ? "ws://localhost:8080" : "ws://10.10.2.1:8080"
    WebSockets.open(hostip, subprotocol="json-v1-agility") do ws
        msg = ["request-privilege", Dict("privilege" =>"change-action-command", 
        "priority" => 0)]
        jmsg = json(msg)
        writeguarded(ws, jmsg)
        sleep(5e-6)
        msg = [ "action-set-operation-mode",
            Dict(
                "mode"=> "locomotion"
            )
        ]
        jmsg = json(msg)
        writeguarded(ws, jmsg)
        sleep(5e-5)
    end
end

function set_lowlevel_mode(;host=:sim)
    hostip = host == :sim ? "ws://localhost:8080" : "ws://10.10.1.1:8080"
    WebSockets.open(hostip, subprotocol="json-v1-agility") do ws
        msg = ["request-privilege", Dict("privilege" =>"change-action-command", 
        "priority" => 0)]
        jmsg = json(msg)
        writeguarded(ws, jmsg)
        sleep(5e-3)
        msg = [ "action-set-operation-mode",
            Dict(
                "mode"=> "low-level-api"
            )
        ]
        jmsg = json(msg)
        writeguarded(ws, jmsg)
        sleep(5e-3)
    end
end

function behavior_switcher(problem; host=:sim)
    params = problem.task_data[:mm]
    wparams = problem.task_data[:walk]
    fig = Figure(resolution=(800,100))
    ax = Axis(fig[1,1])
    hidedecorations!(ax); hidespines!(ax)
    fig[1,1] = buttongrid = GridLayout(tellwidth = false)
    buttonlabels = ["Low-Level Mode", "Locomotion Mode", 
                        "Standing Mode", "Fabric Mode"]
    buttons = buttongrid[1, 1:length(buttonlabels)] = [Button(fig, label=l) 
                        for l in buttonlabels]
    llobs = Observable(false)
    locobs = Observable(false)
    standingobs = Observable(false)
    walkingobs = Observable(false)

    announce(x)=printstyled("Activated $x mode\n";color=:blue)
    on(buttons[1].clicks) do i; llobs[] = true; locobs[]=false; announce("lowlevel"); set_lowlevel_mode(;host); end
    on(buttons[2].clicks) do i; locobs[] = true; llobs[]=false; announce("locomotion"); set_locomotion_mode(;host); end
    on(buttons[3].clicks) do i; standingobs[] = true;wparams[:inited]=false;  walkingobs[]=false; announce("standing"); end
    on(buttons[4].clicks) do i; walkingobs[] = true; wparams[:t0]=problem.t; standingobs[]=false; announce("fabric"); end

    obs_dict = Dict(:lowlevel_mode => llobs,
                    :locomotion_mode => locobs,
                    :standing_mode => standingobs,
                    :walking_mode => walkingobs)
    params[:observables] = obs_dict

    display(fig)
    return fig, ax
end
