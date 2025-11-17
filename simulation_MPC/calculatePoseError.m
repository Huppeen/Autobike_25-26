function pose_error = calculatePoseError(Xref_t, Yref_t, X_projection, Y_projection, closest2closestplus1)
    % Function to calculate pose_error based on input coordinates and
    % vector
    
    % Step 1: Create the vector from projection point to reference point
    % This vector points from (X_projection, Y_projection) to (Xref_t, Yref_t)
    projectionpoint2referencepoint = [Xref_t - X_projection, Yref_t - Y_projection];    
    
    % Step 2: Perform the dot product of the two vectors
    % If dot_product >= 0: Bicycle is behind the reference point
    % If dot_product < 0: Bicycle is ahead of the reference point
    dot_product = dot(projectionpoint2referencepoint, closest2closestplus1);    
    
    % Step 3: Calculate the length of the projectionpoint2referencepoint vector
    % The length of the vector is its Euclidean norm
    vector_length = norm(projectionpoint2referencepoint);    
    
    % Step 4: Determine pose_error based on the dot product sign
    if dot_product < 0
        pose_error = -vector_length; % Negative length if dot product is negative (ahead of reference point)
    else
        pose_error = vector_length;  % Positive length otherwise (behind reference point)
    end
end
