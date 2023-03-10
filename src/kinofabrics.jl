# helpers
function update_walking_observation(q, qdot, qmotors, observation, params, problem)
    params[:t] = problem.t
    params[:q] = q
    params[:qdot] = qdot
    params[:qmotors] = collect(qmotors)
    params[:leftHeelSpring] = observation.joint.position[5]
    params[:rightHeelSpring] = observation.joint.position[10]
    params[:s] = (params[:t] - params[:t0])/params[:swing_time]
end

function init_params(params)
    if !params[:inited] 
        params[:target_heading] = params[:q][di.qbase_yaw]
        params[:inited] = true
    end
end

function made_impact(params)
    deflection = params[:swing_foot] == :left ? params[:leftHeelSpring] : params[:rightHeelSpring]
    if params[:s] > 1.0 || (params[:s] > 0.5 && abs(deflection) > 0.015)
        return true
    end
    return false
end

function transition_to_stand(params)
    if params[:s] > 0.9 #&& params[:swing_foot] == :right
        return true
    end
    return false
end

function check_stance_switch(params)
    if made_impact(params)
        params[:swing_foot] = params[:swing_foot] == :left ? :right : :left 
        params[:t0] = params[:t]
        left_toe = kin.p_toe_pitch_joint_left(params[:q])
        right_toe = kin.p_toe_pitch_joint_right(params[:q])
        params[:target_heading] = wrap_to_pi(params[:target_heading] + params[:vel_des_target][3]*params[:swing_time])
        if params[:swing_foot] == :right
            st_heading_bos = wrap_to_pi(params[:q][di.qbase_yaw] - params[:q][di.qleftHipYaw])
            params[:Rz_st] = RotZ(st_heading_bos)
            params[:p_sw_wrt_st_toe_bos] = params[:Rz_st]' * (right_toe - left_toe)
            p_com_w = kin.p_COM(params[:q])
            params[:p_com_wrt_st_bos] = params[:Rz_st]' * (p_com_w - left_toe)
            
            params[:indices].idx_q_st_hiproll_ = di.qleftHipRoll
            params[:indices].idx_q_st_hipyaw_ = di.qleftHipYaw
            params[:indices].idx_q_st_hippitch_  = di.qleftHipPitch
            params[:indices].idx_q_st_knee_  = di.qleftKnee
            params[:indices].idx_q_st_KneeToShin_  = di.qleftKneeToShin
            params[:indices].idx_q_st_ShinToTarsus_  = di.qleftShinToTarsus

            params[:indices].idx_q_sw_hiproll_  = di.qrightHipRoll
            params[:indices].idx_q_sw_hipyaw_  = di.qrightHipYaw
            params[:indices].idx_q_sw_hippitch_  = di.qrightHipPitch
            params[:indices].idx_q_sw_knee_  = di.qrightKnee
            params[:indices].idx_q_sw_KneeToShin_  = di.qrightKneeToShin
            params[:indices].idx_q_sw_ShinToTarsus_  = di.qrightShinToTarsus
        
            params[:indices].idx_m_st_hiproll_  = di.LeftHipRoll
            params[:indices].idx_m_st_hipyaw_ = di.LeftHipYaw
            params[:indices].idx_m_st_hippitch_  = di.LeftHipPitch
            params[:indices].idx_m_st_knee_  = di.LeftKnee

            params[:indices].idx_m_sw_hiproll_  = di.RightHipRoll
            params[:indices].idx_m_sw_hipyaw_  = di.RightHipYaw
            params[:indices].idx_m_sw_hippitch_  = di.RightHipPitch
            params[:indices].idx_m_sw_knee_  = di.RightKnee
        else
            st_heading_bos = wrap_to_pi(params[:q][di.qbase_yaw] - params[:q][di.qrightHipYaw])
            params[:Rz_st] = RotZ(st_heading_bos)
            params[:p_sw_wrt_st_toe_bos] = params[:Rz_st]' * (left_toe - right_toe)
            p_com_w = kin.p_COM(params[:q])
            params[:p_com_wrt_st_bos] = params[:Rz_st]' * (p_com_w - right_toe)

            params[:indices].idx_q_st_hiproll_ = di.qrightHipRoll
            params[:indices].idx_q_st_hipyaw_ = di.qrightHipYaw
            params[:indices].idx_q_st_hippitch_  = di.qrightHipPitch
            params[:indices].idx_q_st_knee_  = di.qrightKnee
            params[:indices].idx_q_st_KneeToShin_  = di.qrightKneeToShin
            params[:indices].idx_q_st_ShinToTarsus_  = di.qrightShinToTarsus

            params[:indices].idx_q_sw_hiproll_  = di.qleftHipRoll
            params[:indices].idx_q_sw_hipyaw_  = di.qleftHipYaw
            params[:indices].idx_q_sw_hippitch_  = di.qleftHipPitch
            params[:indices].idx_q_sw_knee_  = di.qleftKnee
            params[:indices].idx_q_sw_KneeToShin_  = di.qleftKneeToShin
            params[:indices].idx_q_sw_ShinToTarsus_  = di.qleftShinToTarsus
        
            params[:indices].idx_m_st_hiproll_  = di.RightHipRoll
            params[:indices].idx_m_st_hipyaw_ = di.RightHipYaw
            params[:indices].idx_m_st_hippitch_  = di.RightHipPitch
            params[:indices].idx_m_st_knee_  = di.RightKnee

            params[:indices].idx_m_sw_hiproll_  = di.LeftHipRoll
            params[:indices].idx_m_sw_hipyaw_  = di.LeftHipYaw
            params[:indices].idx_m_sw_hippitch_  = di.LeftHipPitch
            params[:indices].idx_m_sw_knee_  = di.LeftKnee
        end
        
    end
end

function compute_alip_foot_placement(params)
    com_wrt_feet = kin.p_com_wrt_feet(params[:q])
    id = params[:swing_foot] == :left ? 6 : 3
    zH = com_wrt_feet[id]
    params[:walk_height] = zH
    lip_constant = sqrt(9.806/zH)
    Ts = params[:swing_time]
    T_r = (1-params[:s])*Ts
    id = params[:swing_foot] == :left ? 4 : 1
    vel_des_st = params[:vel_des_target]
    mass = 46.2104
    step_width = params[:step_width] 

    p_com_w = kin.p_COM(params[:q])
    v_com_w = kin.v_COM(params[:q], params[:qdot])
    if params[:swing_foot] == :right
        p_left_toe_w = kin.p_toe_pitch_joint_left(params[:q])
        p_com_wrt_st_aligned = params[:Rz_st]' * (p_com_w - p_left_toe_w)
        v_com_wrt_st_aligned = params[:Rz_st]' * v_com_w  

        L = kin.angular_momentum_about_point(params[:q], params[:qdot], p_left_toe_w)
        L_aligned = params[:Rz_st]' * L 
        Lx_st = L_aligned[1]
        Ly_st = L_aligned[2]
    else
        p_right_toe_w = kin.p_toe_pitch_joint_right(params[:q])
        p_com_wrt_st_aligned = params[:Rz_st]' * (p_com_w - p_right_toe_w)
        v_com_wrt_st_aligned = params[:Rz_st]' * v_com_w  

        L = kin.angular_momentum_about_point(params[:q], params[:qdot], p_right_toe_w)
        L_aligned = params[:Rz_st]' * L
        Lx_st = L_aligned[1]
        Ly_st = L_aligned[2]
    end
    xc = p_com_wrt_st_aligned[1] 
    yc = p_com_wrt_st_aligned[2]
    vz = v_com_wrt_st_aligned[3]

    # @show vz


    if params[:swing_foot] == :right  
        Lx_des = (-mass*zH*vel_des_st[2]) - 0.5*mass*zH*step_width*
            ((lip_constant*sinh(lip_constant*Ts)) / (1+cosh(lip_constant*Ts))) 
    else 
        Lx_des = (-mass*zH*vel_des_st[2]) + 0.5*mass*zH*step_width*
        ((lip_constant*sinh(lip_constant*Ts)) / (1+cosh(lip_constant*Ts))) 
    end
    Ly_des = mass*zH*vel_des_st[1]

    xc_eos_est = cosh(lip_constant * T_r) * xc + (1 / (mass*zH*lip_constant)) * sinh(lip_constant*T_r) * Ly_st
    yc_eos_est = cosh(lip_constant * T_r) * yc - (1 / (mass*zH*lip_constant)) * sinh(lip_constant*T_r) * Lx_st

    Lx_eos_est = -1*mass*zH*lip_constant*sinh(lip_constant*T_r)*yc + cosh(lip_constant*T_r) * Lx_st
    Ly_eos_est = mass*zH*lip_constant*sinh(lip_constant*T_r)*xc + cosh(lip_constant*T_r)*Ly_st 

    # p_com_wrt_sw_eos_x = (Ly_des - cosh(lip_constant*Ts)*Ly_eos_est) / (mass*zH*lip_constant*sinh(lip_constant*Ts))
    # p_com_wrt_sw_eos_y = -(Lx_des - cosh(lip_constant*Ts)*Lx_eos_est) / (mass*zH*lip_constant*sinh(lip_constant*Ts))

    # p_com_wrt_sw_eos_x = (Ly_des - cosh(lip_constant*Ts)*(Ly_eos_est + mass*vz*xc))/(mass*(zH*lip_constant*sinh(lip_constant*Ts)-vz)*cosh(lip_constant*Ts))
    # p_com_wrt_sw_eos_y = -(Lx_des - cosh(lip_constant*Ts)*(Lx_eos_est - mass*vz*yc))/(mass*(zH*lip_constant*sinh(lip_constant*Ts)-vz)*cosh(lip_constant*Ts))


    p_com_wrt_sw_eos_x = (Ly_des - cosh(lip_constant*Ts)*Ly_eos_est - mass*cosh(lip_constant*Ts) * xc*vz)  /  (mass*zH*lip_constant*sinh(lip_constant*Ts) - mass * cosh(lip_constant*Ts) * vz)
    p_com_wrt_sw_eos_y = (Lx_des - cosh(lip_constant*Ts)*Lx_eos_est + mass*cosh(lip_constant*Ts) * yc*vz)  /  (mass*-zH*lip_constant*sinh(lip_constant*Ts) + mass * cosh(lip_constant*Ts) * vz)




















    p_sw_wrt_st_eos_x = xc_eos_est - p_com_wrt_sw_eos_x
    p_sw_wrt_st_eos_y = yc_eos_est - p_com_wrt_sw_eos_y
    p_sw_wrt_st_eos_z = 0.0

    foot_placement = [p_sw_wrt_st_eos_x, p_sw_wrt_st_eos_y, p_sw_wrt_st_eos_z]
    max_x_step = 0.7
    foot_placement[1] = max(min(foot_placement[1], max_x_step), -max_x_step)

    max_y_step = 0.6
    min_y_step = 0.1
    sn = params[:swing_foot] == :right ? -1 : 1
    foot_placement[2] = sn * max(min(abs(foot_placement[2]), max_y_step), min_y_step)
    
    return foot_placement
end

function compute_virtual_constraint(foot_placement::AbstractArray, params::Dict)
    bos = params[:p_sw_wrt_st_toe_bos]; ??z = 0.1; ??s = 0.5; s?? = 1.0/params[:swing_time]
    vc_x = 0.5*(bos[1] - foot_placement[1]) * cos(??*params[:s]) + 0.5*(bos[1]+foot_placement[1])
    vc_y = 0.5*(bos[2] - foot_placement[2]) * cos(??*params[:s]) + 0.5*(bos[2]+foot_placement[2])
    
    vc_dot_x = -0.5*??*s??*(bos[1]-foot_placement[1])*sin(??*params[:s])
    vc_dot_y = -0.5*??*s??*(bos[2]-foot_placement[2])*sin(??*params[:s])

    ????? = ??z/(??s-1) - (??z-bos[3])/??s 
    ????? = -(??z - bos[3]+bos[3]*??s*??s)/(??s*(??s-1))
    ????? = bos[3]
    vc_z = ?????*params[:s]*params[:s] + ?????*params[:s] + ?????
    vc_dot_z = (2*params[:s]*????? + ?????)*s??

    id = params[:swing_foot] == :left ? 6 : 3
    vc_H = kin.p_com_wrt_feet(params[:q])[id]
    vc = [vc_x, vc_y, vc_z, 0.92]
    vcdot = [vc_dot_x, vc_dot_y, vc_dot_z, 0.0]
    return vc, vcdot
end

function rotate_vc_to_world(vc, vcdot, params)
    p_sw_wrt_st_eos_des_w = params[:Rz_st] * vc[1:3]
    pdot_sw_wrt_st_eos_des_w = params[:Rz_st] * vcdot[1:3]
    vc_w = [p_sw_wrt_st_eos_des_w; vc[4]]
    vcdot_w = [pdot_sw_wrt_st_eos_des_w; vcdot[4]]

    return vc_w, vcdot_w
end

function attractor_task_map(x, params::Dict)
    res = x - params[:goal] 
    return res 
end

function attractor_potential(x, params::Dict)
    K = params[:K] 
    f = -K*x 
    return f 
end

function get_floating_pose(??, params::Dict)
    p = ??[[di.qbase_pos_x, di.qbase_pos_y, di.qbase_yaw]]-params[:init_position]
    return p
end

function filter_coordinates(vs, prob, val=:vel)
    param = prob.task_data[:filter]
    a = param[:filter_parameter]
    if val == :vel
        if param[:first_iter_vel]
            qdot_filtered = vs
            param[:qdot_filtered] = vs
            param[:first_iter_vel] = false
        else
            qdot_filtered = (1-a)*param[:qdot_filtered] + a*vs
            param[:qdot_filtered] = qdot_filtered
        end
        return qdot_filtered
    else
        if param[:first_iter_pos]
            q_filtered = vs
            param[:q_filtered] = vs
            param[:first_iter_pos] = false
        else
            q_filtered = (1-a)*param[:q_filtered] + a*vs
            param[:q_filtered] = q_filtered
        end
        return q_filtered
    end

end
## Task Maps

# Level 4
function mobile_manipulation_task_map(??, ???? , qmotors, observation, prob)
    params = prob.task_data[:mm]
    if !(params[:action_index] > length(params[:plan]))
        current_action = params[:plan][params[:action_index]]
        name = current_action[1]
        println("")
        @show name, params[:action_index]
        if name == :navigate 
            prob.task_data[name][:goal] = current_action[2] 
            prob.task_data[:mm][:standing] = false
            # activate
            activate_fabric!(name, prob, 3)
            activate_fabric!(:walk, prob, 2)
            activate_fabric!(:walk_attractor, prob, 1)
            activate_fabric!(:upper_body_posture, prob, 1)

            # deactivate
            delete_fabric!(:bimanual_pickup, prob, 3) 
            delete_fabric!(:com_target, prob, 1)
            if !prob.task_data[name][:init_start_time]
                # prob.task_data[name][:start_time] = prob.t
                params[:init_start_time] = true 
            end

        elseif name == :walk_in_place  
            prob.task_data[:mm][:standing] = false
            prob.task_data[name][:period] = current_action[2]
            # activate
            activate_fabric!(name, prob, 3)
            activate_fabric!(:walk, prob, 2)
            activate_fabric!(:walk_attractor, prob, 1)
            activate_fabric!(:upper_body_posture, prob, 1)

            # deactivate
            delete_fabric!(:bimanual_pickup, prob, 3) 
            delete_fabric!(:com_target, prob, 1) 
            if !prob.task_data[name][:init_start_time]
                println("initing")
                prob.task_data[name][:start_time] = prob.t
                prob.task_data[name][:init_start_time] = true 
            end

        elseif name == :precise_move
            prob.task_data[name][:direction] = current_action[2]
            prob.task_data[name][:distance] = current_action[3]
            prob.task_data[:mm][:standing] = false
            # activate
            activate_fabric!(name, prob, 3)
            activate_fabric!(:walk, prob, 2)
            activate_fabric!(:walk_attractor, prob, 1)
            activate_fabric!(:upper_body_posture, prob, 1)

            # deactivate
            delete_fabric!(:bimanual_pickup, prob, 3) 
            delete_fabric!(:com_target, prob, 1)
            if !prob.task_data[name][:init_start_time]
                prob.task_data[name][:start_time] = prob.t
                params[:init_start_time] = true 
            end

        elseif name == :bimanual_pickup
            prob.task_data[:mm][:standing] = true
            prob.task_data[name][:final_com] = current_action[2]
            prob.task_data[name][:final_pitch] = current_action[3]
            # activate
            activate_fabric!(name, prob, 3)
            activate_fabric!(:upper_body_posture, prob, 1)
            activate_fabric!(:com_target, prob, 1)

            # deactivate
            delete_fabric!(:navigate, prob, 3)
            delete_fabric!(:precise_move, prob, 3)
            delete_fabric!(:walk_in_place, prob, 3)
            delete_fabric!(:walk, prob, 2)
            delete_fabric!(:walk_attractor, prob, 1)  
            if !prob.task_data[name][:init_start_time]
                prob.task_data[name][:start_time] = prob.t
                prob.task_data[name][:init_start_time] = true 
            end

        elseif name == :bimanual_place
            prob.task_data[:mm][:standing] = true
            prob.task_data[name][:final_com] = current_action[2]
            prob.task_data[name][:final_pitch] = current_action[3]
            # activate
            activate_fabric!(name, prob, 3)
            activate_fabric!(:upper_body_posture, prob, 1)
            activate_fabric!(:com_target, prob, 1)

            # deactivate
            delete_fabric!(:navigate, prob, 3)
            delete_fabric!(:precise_move, prob, 3)
            delete_fabric!(:walk_in_place, prob, 3)
            delete_fabric!(:walk, prob, 2)
            delete_fabric!(:walk_attractor, prob, 1) 
            if !prob.task_data[name][:init_start_time]
                prob.task_data[name][:start_time] = prob.t
                prob.task_data[name][:init_start_time] = true 
            end
        elseif name == :stand
            prob.task_data[:mm][:standing] = true
            prob.task_data[name][:com_height] = current_action[2]
            prob.task_data[name][:torso_pitch] = current_action[3]
            prob.task_data[name][:period] = current_action[4]
            prob.task_data[name][:torso_roll] = current_action[5]
            
            #activate
            activate_fabric!(name, prob, 2)
            activate_fabric!(:com_target, prob, 1)
            activate_fabric!(:upper_body_posture, prob, 1)

            #deactivate
            delete_fabric!(:navigate, prob, 3)
            delete_fabric!(:precise_move, prob, 3)
            delete_fabric!(:walk_in_place, prob, 3)
            delete_fabric!(:walk, prob, 2)
            delete_fabric!(:walk_attractor, prob, 1) 
        
        elseif name == :cornhole
            prob.task_data[:mm][:standing] = true
            #activate
            activate_fabric!(name, prob, 2)
            activate_fabric!(:com_target, prob, 1)
            activate_fabric!(:upper_body_posture, prob, 1)

            #deactivate
            delete_fabric!(:navigate, prob, 3)
            delete_fabric!(:walk, prob, 2)
            delete_fabric!(:precise_move, prob, 3)
            delete_fabric!(:walk_in_place, prob, 3)
            delete_fabric!(:walk_attractor, prob, 1) 
        end
    end
end

# Level 3
function navigate_task_map(??, ???? , qmotors, observation, prob)
    params = prob.task_data[:navigate]
    state = params[:state]
    # @show state
    u = [0,0,0.]
    if state == :translate        
        ?? = attractor_task_map 
        p = get_floating_pose(??, params)
        # @show p
        x = ??(p, params)
        J = FiniteDiff.finite_difference_jacobian(??->??(??, params), p)
        f = attractor_potential(x, params)
        u = J'*f  
        delta = norm(p-params[:goal])
        # @show delta 
        if delta < params[:tolerance]
            params[:state] = :stand
            params[:stand_start_time] = prob.t
        end
    elseif state == :stand 
        if prob.task_data[:mm][:action_index] >= length(prob.task_data[:mm][:plan])  switch = true else
            switch = prob.task_data[:mm][:plan][prob.task_data[:mm][:action_index]+1][1] != :navigate 
        end
        if  switch
            s = (prob.t - params[:stand_start_time])/params[:stand_period]
            prob.task_data[:walk][:vel_des_target] = [0,0,0.] 
            if s >= 1.0  && transition_to_stand(prob.task_data[:walk])  
                prob.x???[:com_target][[3,6]] .= prob.task_data[:walk][:walk_height]
                activate_fabric!(:com_target, prob, 1)
                delete_fabric!(:walk_attractor, prob, 1)
                prob.task_data[:mm][:standing] = true
                params[:state] = :translate 
                prob.task_data[:mm][:action_index] += 1  
                prob.task_data[:bimanual_pickup][:action_start_time] = prob.t
            end 
        else 
            params[:state] = :translate 
            prob.task_data[:mm][:action_index] += 1
        end
    end
    prob.task_data[:walk][:vel_des_target] = u
end

function walk_in_place_task_map(??, ???? , qmotors, observation, prob)
    params = prob.task_data[:walk_in_place]
    state = params[:state]
    u = [0., 0, 0]
    # @show state
    
    # if state == :init 
    #     params[:state] = :walk

    if state == :walk
        s = (prob.t - params[:start_time])/params[:period]
        # @show s
        if s >= 1.0
            params[:state] = :finish
        end

    elseif state == :finish
        prob.task_data[:mm][:action_index] += 1 
        # params[:state] = :walk
    end
    prob.task_data[:walk][:vel_des_target] = u
end


function precise_move_task_map(??, ???? , qmotors, observation, prob)
    params = prob.task_data[:precise_move]
    state = params[:state]
    @show state
    u = [-0.1, 0, 0]
    if state == :init
        params[:init_position] = ??[[di.qbase_pos_x, di.qbase_pos_y]]
        params[:state] = :walk_in_place
        params[:w_start_time] = prob.t

    elseif state == :walk_in_place
        period = params[:direction] == :forward ? params[:wx_period] : params[:wy_period]
        s = (prob.t - params[:w_start_time])/period
        prob.task_data[:walk][:vel_des_target] = [0,0,0.] 
        if s >= 1.0
            params[:state] = :move
        end
    
    elseif state == :move
        current_pose = ??[[di.qbase_pos_x, di.qbase_pos_y]]
        d = norm(current_pose-params[:init_position])
        sign = params[:distance] < 0 ? -1 : 1
        e = sign*(d - abs(params[:distance]))
        # @show e
        dedt = e/params[:dt]
        v = -params[:Kp]*e - params[:Kd]*dedt
        lim = params[:direction] == :forward ? params[:limx] : params[:limy]
        v = clamp(v, -lim, lim)
        if params[:direction] == :forward u[1] = v else u[2] = v  end
        if abs(e) < params[:tolerance] 
            params[:state] = :purgatory
            params[:stand_start_time] = prob.t
        end

    elseif state == :purgatory
        s = (prob.t - params[:stand_start_time])/params[:stand_period]
        prob.task_data[:walk][:vel_des_target] = [0,0,0.] 
        if s >= 1.0
            params[:state] = :stand
        end


    elseif state == :stand 
        if prob.task_data[:mm][:action_index] >= length(prob.task_data[:mm][:plan])  switch = true else
            switch = !(prob.task_data[:mm][:plan][prob.task_data[:mm][:action_index]+1][1] in (:precise_move, :navigate))
        end
        if  switch
            s = (prob.t - params[:stand_start_time])/params[:stand_period]
            prob.task_data[:walk][:vel_des_target] = [0,0,0.] 
            if s >= 1.0  && transition_to_stand(prob.task_data[:walk])  
                # prob.x???[:com_target][[3,6]] .= prob.task_data[:walk][:walk_height]
                activate_fabric!(:com_target, prob, 1)
                delete_fabric!(:walk_attractor, prob, 1)
                prob.task_data[:mm][:standing] = true
                params[:state] = :init 
                prob.task_data[:mm][:action_index] += 1  
                prob.task_data[:bimanual_pickup][:action_start_time] = prob.t
            end 
        else 
            params[:state] = :init 
            prob.task_data[:mm][:action_index] += 1
        end
    end
    # @show u
    prob.task_data[:walk][:vel_des_target] = u     
end

function bimanual_pickup_task_map(q, qdot, qmotors, observation, prob)  
    params = prob.task_data[:bimanual_pickup]
    # println("state: $(params[:state])")

    if params[:state] == :descend_init
        t0 = prob.t
        T = params[:flight_time]
        p0 = 0.0; z0 = 0.95#kin.p_com_wrt_feet(q)[3]
        pf = params[:final_pitch]; zf = params[:final_com]        
        pitch(t) = (1 - ((t-t0)/T))*p0 + ((t-t0)/T)*pf
        com_height(t) = (1 - ((t-t0)/T))*z0 + ((t-t0)/T)*zf
        params[:pitch_trajectory] = pitch
        params[:com_height_trajectory] = com_height
        s = (prob.t - params[:start_time])/params[:start_buffer_time]
        if s >= 1.0
            params[:state] = :descend
            params[:start_time] = prob.t
        end
    
    elseif params[:state] == :descend
        pitch_traj = params[:pitch_trajectory]
        height_traj = params[:com_height_trajectory] 
        # prob.x???[:com_target][[3, 6]] .= height_traj(prob.t)
        prob.x???[:com_target][7] = pitch_traj(prob.t) 
        prob.x???[:upper_body_posture] = prob.x???[:open_arms_posture]  
        if abs(params[:flight_time] - (prob.t - params[:start_time])) < 1e-2
            params[:state] = :clasp
            params[:clasp_start_time] = prob.t
        end 

    elseif params[:state] == :clasp
        prob.x???[:upper_body_posture] = prob.x???[:close_arms_posture] 
        s = (prob.t - params[:clasp_start_time])/params[:clasp_period]
        if s >= 1.0
            params[:state] = :ascend_init
        end
        
    elseif params[:state] == :ascend_init
        t0 = prob.t
        params[:start_time] = t0
        T = params[:flight_time]
        p0 = params[:final_pitch]; z0 = params[:final_com]
        pf = 0.0; zf = 0.95
        ascend_pitch(t) = (1 - ((t-t0)/T))*p0 + ((t-t0)/T)*pf
        ascend_com_height(t) = (1 - ((t-t0)/T))*z0 + ((t-t0)/T)*zf
        params[:pitch_trajectory] = ascend_pitch
        params[:com_height_trajectory] = ascend_com_height
        params[:state] = :ascend 

    elseif params[:state] == :ascend
        pitch_traj = params[:pitch_trajectory]
        height_traj = params[:com_height_trajectory] 
        # prob.x???[:com_target][[3, 6]] .= height_traj(prob.t)
        prob.x???[:com_target][7] = pitch_traj(prob.t) 
        prob.x???[:upper_body_posture] = prob.x???[:clutch_arms_posture] 
        if abs(params[:flight_time] - (prob.t - params[:start_time])) < 1e-2
            params[:state] = :stand
        end

    elseif params[:state] == :stand 
        params[:state] = :finish
        prob.task_data[:mm][:action_index] += 1
        delete_fabric!(:bimanual_pickup, prob, 3)

    elseif params[:state] == :finish   
        params[:state] = :descend_init
    end

end

function bimanual_place_task_map(q, qdot, qmotors, observation, prob) 
    params = prob.task_data[:bimanual_place]
    # println("state: $(params[:state])")

    if params[:state] == :descend_init
        t0 = prob.t 
        T = params[:flight_time]
        p0 = 0.0; z0 = 0.92#kin.p_com_wrt_feet(q)[3]
        pf = params[:final_pitch]; zf = params[:final_com]        
        pitch(t) = (1 - ((t-t0)/T))*p0 + ((t-t0)/T)*pf
        com_height(t) = (1 - ((t-t0)/T))*z0 + ((t-t0)/T)*zf
        params[:pitch_trajectory] = pitch
        params[:com_height_trajectory] = com_height
        prob.x???[:upper_body_posture] = prob.x???[:close_arms_posture]
        s = (prob.t - params[:start_time])/params[:start_buffer_time] 
        if s >= 1.0
            params[:state] = :descend
            params[:start_time] = prob.t
        end
    
    elseif params[:state] == :descend
        pitch_traj = params[:pitch_trajectory]
        height_traj = params[:com_height_trajectory] 
        # prob.x???[:com_target][[3, 6]] .= height_traj(prob.t)
        prob.x???[:com_target][7] = pitch_traj(prob.t)   
        if abs(params[:flight_time] - (prob.t - params[:start_time])) < 1e-2
            params[:state] = :open
            params[:open_start_time] = prob.t
        end 

    elseif params[:state] == :open
        prob.x???[:upper_body_posture] = prob.x???[:open_arms_posture] 
        s = (prob.t - params[:open_start_time])/params[:open_period]
        if s >= 1.0
            params[:state] = :ascend_init
        end
        
    elseif params[:state] == :ascend_init
        t0 = prob.t
        params[:start_time] = t0
        T = params[:flight_time]
        p0 = params[:final_pitch]; z0 = params[:final_com]
        pf = 0.0; zf = 0.95
        ascend_pitch(t) = (1 - ((t-t0)/T))*p0 + ((t-t0)/T)*pf
        ascend_com_height(t) = (1 - ((t-t0)/T))*z0 + ((t-t0)/T)*zf
        params[:pitch_trajectory] = ascend_pitch
        params[:com_height_trajectory] = ascend_com_height
        params[:state] = :ascend 

    elseif params[:state] == :ascend
        pitch_traj = params[:pitch_trajectory]
        height_traj = params[:com_height_trajectory] 
        # prob.x???[:com_target][[3, 6]] .= height_traj(prob.t)
        prob.x???[:com_target][7] = pitch_traj(prob.t) 
        if abs(params[:flight_time] - (prob.t - params[:start_time])) < 1e-2
            params[:state] = :stand
        end

    elseif params[:state] == :stand 
        params[:state] = :finish
        prob.task_data[:mm][:action_index] += 1
        prob.x???[:upper_body_posture] = prob.x???[:normal_posture]
        delete_fabric!(:bimanual_place, prob, 3)

    elseif params[:state] == :finish  
        params[:state] = :descend_init
    end

end

# Level 2
function walk_task_map(q, qdot, qmotors, observation, problem)
    params = problem.task_data[:walk]
    update_walking_observation(q, qdot, qmotors, observation, params, problem)
    init_params(params)
    check_stance_switch(params)  
    fp = compute_alip_foot_placement(params)
    vc, vcdot = compute_virtual_constraint(fp, params)
    vc, vcdot = rotate_vc_to_world(vc, vcdot, params) 
    s = zeros(length(q))
    leg_indices = [params[:indices].idx_q_sw_hiproll_, params[:indices].idx_q_sw_hippitch_, params[:indices].idx_q_sw_knee_, params[:indices].idx_q_st_knee_]
    s[leg_indices] .= 1.0
    problem.S[:walk_attractor] = diagm(s)
    problem.x???[:walk_attractor] = [vc..., vcdot...] 
end

function stand_task_map(q, qdot, qmotors, observation, prob)
    params = prob.task_data[:stand]
    state = params[:state]
    if state == :init
        params[:start_time] = prob.t
        params[:state] = :run
        p0 = q[di.qbase_pitch]; r0 = q[di.qbase_roll]
        params[:pitch_trajectory] = s->(1-s)*p0 + s*params[:torso_pitch]
        params[:roll_trajectory] = s->(1-s)*p0 + s*params[:torso_roll]
    
    elseif state == :run
        s = (prob.t - params[:start_time])/params[:period]
        prob.x???[:com_target][[3,6]] .= params[:com_height]
        prob.x???[:com_target][7] = params[:pitch_trajectory](s)
        prob.x???[:com_target][8] = params[:roll_trajectory](s)
        if s >= 1.0
            params[:state] = :init
            prob.task_data[:mm][:action_index] += 1
            delete_fabric!(:stand, prob, 2)
            println("done standing")
        end
    end       

end

function cornhole_task_map(q, qdot, qmotors, observation, prob)
    params = prob.task_data[:cornhole]
    state = params[:state]
    @show state

    if state == :init
        open_gripper!(params[:gripper])
        println("open gripper")
        params[:start_time] = prob.t
        params[:state] = :descend_init
    
    elseif params[:state] == :descend_init
        t0 = prob.t
        T = params[:descend_period]
        p0 = 0.0; z0 = 0.95
        pf = params[:pick_torso_pitch]; zf = params[:pick_height]    
        params[:pitch_trajectory] = t -> (1 - ((t-t0)/T))*p0 + ((t-t0)/T)*pf
        params[:com_height_trajectory] = t -> (1 - ((t-t0)/T))*z0 + ((t-t0)/T)*zf  
        params[:state] = :descend
        params[:start_time] = prob.t 
    
    elseif params[:state] == :descend
        pitch_traj = params[:pitch_trajectory]
        height_traj = params[:com_height_trajectory] 
        prob.x???[:com_target][[3, 6]] .= height_traj(prob.t)
        prob.x???[:com_target][7] = pitch_traj(prob.t)  
        if abs(params[:descend_period] - (prob.t - params[:start_time])) < 1e-2
            params[:state] = :pick
            params[:start_time] = prob.t
        end
    
    elseif state == :pick
        s = (prob.t - params[:start_time])/params[:pick_period]
        prob.x???[:upper_body_posture] = prob.x???[:pick_posture]
        if s >= 1.0
            params[:state] = :clasp
            params[:start_time] = prob.t
        end
    
    elseif state == :clasp
        s = (prob.t - params[:start_time])/params[:clasp_period]
        close_gripper!(params[:gripper])
        println("close gripper")
        if s >= 1.0
            params[:state] = :ascend_init
            params[:start_time] = prob.t
        end

    elseif params[:state] == :ascend_init
        t0 = prob.t
        T = params[:ascend_period]
        p0 = 0.0; z0 = params[:pick_height]
        pf = params[:throw_torso_pitch]; zf = params[:throw_height]    
        params[:pitch_trajectory] = t -> (1 - ((t-t0)/T))*p0 + ((t-t0)/T)*pf
        params[:com_height_trajectory] = t -> (1 - ((t-t0)/T))*z0 + ((t-t0)/T)*zf 
        params[:state] = :ascend
        params[:start_time] = prob.t 
    
    elseif params[:state] == :ascend
        pitch_traj = params[:pitch_trajectory]
        height_traj = params[:com_height_trajectory] 
        prob.x???[:com_target][[3, 6]] .= height_traj(prob.t)
        prob.x???[:com_target][7] = pitch_traj(prob.t)  
        if abs(params[:ascend_period] - (prob.t - params[:start_time])) < 1e-2
            params[:state] = :load
            params[:start_time] = prob.t
        end

    elseif state == :load
        s = (prob.t - params[:start_time])/params[:load_period]
        prob.x???[:upper_body_posture] = prob.x???[:load_posture]
        if s >= 1.0
            params[:state] = :throw
            params[:start_time] = prob.t
        end
    
    elseif state == :throw 
        s = (prob.t - params[:start_time])/params[:throw_period]
        prob.x???[:upper_body_posture] = prob.x???[:throw_posture]
        params[:fling] = true
        if 0.2<s<0.4 
            open_gripper!(params[:gripper])
            println("open gripper")
        end

        if s >= 1.0
            params[:state] = :finish
            params[:fling] = false
            params[:start_time] = prob.t
        end

    elseif state == :finish 
        prob.x???[:upper_body_posture] = prob.x???[:normal_posture]
    end

end


# Level 1
function walk_attractor_task_map(q, qdot, problem)
    params = problem.task_data[:walk]
    q[params[:indices].idx_q_st_KneeToShin_] = 0.0
    q[params[:indices].idx_q_sw_KneeToShin_] = 0.0
    q[params[:indices].idx_q_st_ShinToTarsus_] = -q[params[:indices].idx_q_st_knee_]
    q[params[:indices].idx_q_sw_ShinToTarsus_] = -q[params[:indices].idx_q_sw_knee_]

    qdot[params[:indices].idx_q_st_KneeToShin_] = 0.0
    qdot[params[:indices].idx_q_sw_KneeToShin_] = 0.0
    qdot[params[:indices].idx_q_st_ShinToTarsus_] = -qdot[params[:indices].idx_q_st_knee_]
    qdot[params[:indices].idx_q_sw_ShinToTarsus_] = -qdot[params[:indices].idx_q_sw_knee_]

    if params[:swing_foot] == :right 
        pos = kin.vc_walk_LS(q)
    else
        pos = kin.vc_walk_RS(q)
    end 
    return pos
end

function upper_body_posture_task_map(??, ???? , prob::FabricProblem)
    return ??[prob.digit.arm_joint_indices]
end

function com_target_task_map(??, ???? , prob::FabricProblem)
    S = prob.S[:com_target]
    ?? = S*?? 
    ??[di.qleftShinToTarsus] = -??[di.qleftKnee]
    ??[di.qrightShinToTarsus] = -??[di.qrightKnee]
    ??[di.qbase_pitch] = prob.x???[:com_target][7] 
    ??[di.qbase_roll] = prob.x???[:com_target][8] 
    com_pos =  kin.p_com_wrt_feet(??) 
    Rz = RotZ(??[di.qbase_yaw]) 
    com = [(Rz * com_pos[1:3])..., (Rz * com_pos[4:6])...]
    res = com
    return res
end


## Fabric Components
function walk_attractor_fabric(x, x??, problem)
    k = 1.0; ??=0.5; ?? = 1.0
    N = length(x)
    W = problem.W[:walk_attractor]
    k = W*k  
    vc_goal = problem.x???[:walk_attractor][1:4]
    ??x = x - vc_goal
    ??(??) = 0.5*??'*k*??
    ??x = FiniteDiff.finite_difference_gradient(??, ??x)
    x?? = -k*??x  
    M = ?? * I(N)
    return (M, x??)
end

function upper_body_posture_fabric(x, x??, prob::FabricProblem) 
    ????? = 0.25; k = 4.0;  ??=0.75
    M = ????? * I(length(x))
    W = prob.W[:upper_body_posture] 
    k = W*k 
    ??x = x - prob.x???[:upper_body_posture]
    ??(??) = 0.5*??'*k*??
    ??x = FiniteDiff.finite_difference_gradient(??, ??x)
    x?? = -k*??x - ??*x??
    return (M, x??) 
end

function com_target_fabric(x, x??, prob::FabricProblem; ??=40.0, ??=40.0, ??=0.01)
    k = 5.0; ????? = 10.0; ??=0.5; ?? = 0.25 
    N = length(x)
    W = prob.W[:com_target]
    k = W*k  
    ??x = x - prob.x???[:com_target][1:6]
    ??(??) = 0.5*??'*k*??
    ??x = FiniteDiff.finite_difference_gradient(??, ??x)
    x?? = -k*??x -??*x?? 
    M = ?? * I(N)
    return (M, x??)
end


## Solve
function mm_fabric_eval(x, x??, name::Symbol, prob::FabricProblem)
    M = nothing; x?? = nothing 
    fabricname = Symbol(name, :_fabric)
    ?? = eval(fabricname)
    M, x?? = ??(x, x??, prob)
    return (M, x??)
end

function compute_task_map_jacobian(t, ??, ??, ???? , prob::FabricProblem)  
    if t == :walk_attractor
        params = prob.task_data[:walk]
        J = params[:swing_foot] == :right ? kin.Jvc_walk_LS(??) : kin.Jvc_walk_RS(??)
        J[:, params[:indices].idx_q_sw_knee_] = J[:, params[:indices].idx_q_sw_knee_] - J[:, params[:indices].idx_q_sw_ShinToTarsus_]
        J[:, params[:indices].idx_q_st_knee_] = J[:, params[:indices].idx_q_st_knee_] - J[:, params[:indices].idx_q_st_ShinToTarsus_]
        J[:, params[:indices].idx_q_st_knee_] = J[:, params[:indices].idx_q_st_knee_] + 0.5*J[:, params[:indices].idx_q_st_hippitch_]
    else
        J = FiniteDiff.finite_difference_jacobian(??->??(??, ???? , prob), ??) 
    end 
    return J
end

function mm_fabric_solve(??, ???? , qmotors, observation, prob::FabricProblem)
    x??? = []; x????? = []; c??? = []; qvel = zero(??)
    M??? = []; x????? = []; J??? =  [] 
    for t in prob.??[:level4]
        ?? = eval(Symbol(t, :_task_map))
        ??(??, ???? , qmotors, observation, prob)
    end
    for t in prob.??[:level3]
        ?? = eval(Symbol(t, :_task_map))
        ??(??, ???? , qmotors, observation, prob)
    end
    for t in prob.??[:level2]
        ?? = eval(Symbol(t, :_task_map))
        ??(??, ???? , qmotors, observation, prob)
    end 
    for t in prob.task_data[:mm][:task_maps]
        ?? = eval(Symbol(t, :_task_map))
        S = prob.S[t] 
        J = compute_task_map_jacobian(t, ??, ??, ???? , prob)  
        J = J*S 
        x = ??(??, ???? , prob)  
        x?? = J*???? 
        c = zero(x) 
        M, x?? = mm_fabric_eval(x, x??, t, prob)  
        if t == :walk_attractor qvel = J\prob.x???[:walk_attractor][5:end] end
        if t in prob.??[:level1]
            push!(x???, x); push!(x?????, x??); push!(c???, c)
            push!(M???, M); push!(x?????, x??); push!(J???, J)
        end
    end   
    M??? = sum([J' * M * J for (J, M) in zip(J???, M???)])
    f??? = sum([J' * M * (x?? - c) for (J, M, x??, c) in zip(J???, M???, x?????, c???)])
    M??? = convert(Matrix{Float64}, M???)  
    q?? = pinv(M???) * f???
    return  q??, qvel  
end

function mm_fabric_compute(q, qdot, qmotors, observation, problem)
    ????d, qvel = mm_fabric_solve(copy(q), copy(qdot), qmotors, observation, problem) 
    ????d = ????d
    ??d = q + ????d

    q_out = copy(q)
    qdot_out = copy(qdot)

    if problem.task_data[:mm][:standing]
        ??d = q + ????d*1e-1
        q_out[problem.digit.leg_joint_indices] = ??d[problem.digit.leg_joint_indices]
        q_out[problem.digit.arm_joint_indices] = ??d[problem.digit.arm_joint_indices] 
    else
        params = problem.task_data[:walk]
        indices = [params[:indices].idx_q_sw_hiproll_, params[:indices].idx_q_sw_hippitch_, params[:indices].idx_q_sw_knee_, params[:indices].idx_q_st_knee_]
        q_out[indices] = ??d[indices]
        qdot_out[indices] = qvel[indices]+ ????d[indices]
        q_out[problem.digit.arm_joint_indices] = q[problem.digit.arm_joint_indices] + ????d[problem.digit.arm_joint_indices]*1e-1
    end
    qdot_out[problem.digit.arm_joint_indices] = ????d[problem.digit.arm_joint_indices] 

    q_out = clamp.(q_out, problem.digit.??_min, problem.digit.??_max)  
    qdot_out = clamp.(qdot_out, problem.digit.????_min, problem.digit.????_max)

    # q_out = filter_coordinates(q_out, problem, :pos)
    # qdot_out = filter_coordinates(qdot_out, problem, :vel)
    # qdot_out = zero(qdot_out)
    # push!(problem.task_data[:diagnostics][:q], q_out[problem.digit.leg_joint_indices])
    # push!(problem.task_data[:diagnostics][:qdot], qdot_out[problem.digit.leg_joint_indices])
    # push!(problem.task_data[:diagnostics][:t], problem.t)
    

    ?? = zero(q_out) 
    return  q_out, qdot_out, ??
end