function cl_r = t(cl_vect,x)
%crea la funzione raccordo tra estrapolante e cl alpha
cl_r = zeros(size(x));

cl_r(:,:) = min(cl_vect);

ii_p = find(x>0);

cl_r(ii_p) = max(cl_vect);