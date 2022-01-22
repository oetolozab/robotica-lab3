function T = simsyms(MTH)
% MTH matriz de transformación homogénea de forma simbólica
% T matriz de transformación homogénea, donde los valores cuyos
% coeficientes menores a 0.00001se pasan a cero
    T = MTH;
%     for i=1:6
%         for j=1:6
            [C,E]=coeffs(T);
            for k =1:size(C,2)
                if abs(C(k))<0.00001
                    C(k) = 0;
                end
            end
            C=round(C,2);
            T=dot(C,E);
            
%         end
%     end
end