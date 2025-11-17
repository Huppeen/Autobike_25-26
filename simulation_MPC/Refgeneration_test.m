function [psiref, Vref] = Refgeneration_test(Xref, Yref, t_ref)
    % Initialize psiref and Vref
    psiref = zeros(length(Xref), 1);
    Vref = zeros(length(Xref), 1);

    % Calculate heading angle and speed for each segment
    for inn = 1:length(Xref) - 1
        dx = Xref(inn + 1) - Xref(inn);
        dy = Yref(inn + 1) - Yref(inn);
        dt = t_ref(inn + 1) - t_ref(inn);

        % Compute heading angle psi
        psiref(inn) = atan2(dy, dx);

        % Compute speed vv
        Vref(inn) = sqrt(dx^2 + dy^2) / dt;
    end

    % Set the last point’s psi and vv to match the previous point
    psiref(end) = psiref(end - 1);
    Vref(end) = Vref(end - 1);
end