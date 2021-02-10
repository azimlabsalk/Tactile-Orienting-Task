% Code to send trial parameters to a connected rotary table device
% Initially for use with the rotary-table experiment at Salk, Azim lab
% Created by Mark Stambaugh 2020-02-27

function send_parameters(port_handle, params)
    delay = 0.05;
    
    if isfield(params, "zero_position_angle_deg")
        fprintf(port_handle, sprintf("sz %.1f", params.zero_position_angle_deg));
        pause(delay);
    end
    
    
    if isfield(params, "target_angle_deg")
        fprintf(port_handle, sprintf("st %.1f", params.target_angle_deg));
        pause(delay);
    end
    
    if isfield(params, "window_play_deg")
        fprintf(port_handle, sprintf("sw %.1f", params.window_play_deg));
        pause(delay);
    end
    
    if isfield(params, "start_angle_deg")
        fprintf(port_handle, sprintf("ss %.1f", params.start_angle_deg));
        pause(delay);
    end
    
    if isfield(params, "hold_time_ms")
        fprintf(port_handle, sprintf("sh %d", params.hold_time_ms));
        pause(delay);
    end
    
    if isfield(params, "trial_duration_ms")
        fprintf(port_handle, sprintf("sd %d", params.trial_duration_ms));
        pause(delay);
    end
    
    if isfield(params, "reward_anyway")
        fprintf(port_handle, sprintf("sa %d", params.reward_anyway));
        pause(delay);
    end
    
    if isfield(params, "reward_duration")
        fprintf(port_handle, sprintf("sr %d", params.reward_duration));
        pause(delay);
    end
    
    if isfield(params, "preamble_ms")
        fprintf(port_handle, sprintf("sp %d", params.preamble_ms));
        pause(delay);
    end
    
    if isfield(params, "postamble_ms")
        fprintf(port_handle, sprintf("so %d", params.postamble_ms));
        pause(delay);
    end
    
    if isfield(params, "data_res_ms")
        fprintf(port_handle, sprintf("sm %d", params.data_res_ms));
        pause(delay);
    end
    
    if isfield(params, "reward_duration_ms")
        fprintf(port_handle, sprintf("sr %d", params.reward_duration_ms));
        pause(delay);
    end
    
    if isfield(params, "opto_enabled")
        fprintf(port_handle, sprintf("oe %d", params.opto_enabled));
        pause(delay);
    end
    
    if isfield(params, "opto_pre_trial_ms")
        fprintf(port_handle, sprintf("op %d", params.opto_pre_trial_ms));
        pause(delay);
    end
    
    if isfield(params, "opto_pre_trial_ms")
        fprintf(port_handle, sprintf("oo %d", params.opto_post_trial_ms));
        pause(delay);
    end
    
    
    
    % read all messages back from the arduino
    while port_handle.BytesAvailable
        fprintf(fscanf(port_handle));
    end
end