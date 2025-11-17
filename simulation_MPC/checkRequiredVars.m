function checkRequiredVars(required_vars)
    % Check if the required variables are present in the MATLAB workspace.
    % Input:
    %   required_vars - A cell array of variable names (or structure fields) to check.

    for i = 1:length(required_vars)
        var_name = required_vars{i};    
        % Check if it's a top-level variable
        if exist(var_name, 'var')
            % If top-level variable exists but is empty
            if isempty(eval(var_name))
                warning('The variable "%s" has not been successfully assigned.', var_name);
            end
        else
            % Check if it's a field in a structure
            parts = strsplit(var_name, '.');
            if numel(parts) == 2 && exist(parts{1}, 'var') && isfield(eval(parts{1}), parts{2})
                % If it is a structure field but is empty
                if isempty(eval([parts{1} '.' parts{2}]))
                    warning('The field "%s" has not been successfully assigned.', var_name);
                end
            else
                % If neither a top-level variable nor a structure field
                warning('The variable or field "%s" does not exist or has not been successfully assigned.', var_name);
            end
        end
    end
end
