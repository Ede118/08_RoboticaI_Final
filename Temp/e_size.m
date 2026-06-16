function e_size(a, b, show_true)
    if isvector(a) && isvector(b)
        if ~isequal(size(a), size(b))
            disp('[Error]: Diferencia de dimensiones');
            disp(['Dim a: ', num2str(size(a))]);
            disp(['Dim b: ', num2str(size(b))]);
        elseif show_true == 1
            disp('[Success]: Dimensiones correctas');
            disp(['Dim a: ', num2str(size(a))]);
            disp(['Dim b: ', num2str(size(b))]);
        end
    end
    
end