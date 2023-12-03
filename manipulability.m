function mu = manipulability(J, measure)
    % Check if the input measure is valid
    valid_measures = {'sigmamin', 'detjac', 'invcond'};
    if ~ismember(measure, valid_measures)
        error('Invalid measure. Choose from ''sigmamin'', ''detjac'', or ''invcond''.');
    end

    % Calculate manipulability based on the selected measure
    switch measure
        case 'sigmamin'
            % Calculate the minimum singular value of J
            mu = min(svd(J));

        case 'detjac'
            % Calculate the Determinant of J
            mu = det(J);

        case 'invcond'
            % Calculate the inverse of the condition number of J
            mu = 1/cond(J);
    end
end