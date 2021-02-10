% Code for matlab to automatically identify a connected serial device
% identified by a key phrase
% Initially for use with the rotary-table experiment at Salk, Azim lab
% Created by Mark Stambaugh 2020-02-27

function port_names = search_for_devices(key_text, baud)
    candidates = seriallist();
    Ncans = length(candidates);
    Ngood = 0;

    port_names = "";
    
    if Ncans == 0
        error_msg = sprintf("No ports found. Check connected devices.\n");
        msgbox(error_msg);
        fprintf(error_msg);
        port_names = [];
        return;
    else
        fprintf("%d devices found. Checking IDs:\n", Ncans);
        disp(candidates);
        instrreset();
        for i = 1:Ncans
            fprintf("testing port %s\n",candidates(i));
            s = serial(candidates(i), 'BaudRate', baud);
            fopen(s);
            pause(2);
            if s.BytesAvailable
                msg = fscanf(s);
                if contains(msg, key_text) %"rotary table"
                    Ngood = Ngood + 1;
                    port_names(Ngood) = candidates(i);
                    fprintf("%s found on %s\n", key_text, candidates(i));
                end
            end
            fclose(s);
        end
        if Ngood == 0
            error_msg =  sprintf("No %s found. Check connected devices.\n", key_text);
            msgbox(error_msg);
            fprintf(error_msg);
            port_names = [];
            return;
        end
    end
end
