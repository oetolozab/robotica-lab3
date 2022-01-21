function [q] = ikinemIRB(robot,T)
    [Ref,pef] = tr2rt(T);

    %% Parámetros del manipulador - dm
    l1=6.15;
    l2=1.00;
    l3=7.05;
    l4=sqrt(1.35^2+7.55^2);
    l5=0.85;
    
    %% Posición del efector final
    %posición de la muñeca
    pm = pef-l5*Ref(1:3,3); 
    
    %% q1
    q1 = atan2(pm(2),pm(1));
    
    %% q2
    pw = sqrt((pm(1)-l2*cos(q1))^2+(pm(2)-l2*sin(q1))^2+(pm(3)-l1)^2);

    %alpha
    ca = (l3^2+pw^2-l4^2)/(2*l3*pw);
    sa1 = sqrt(1-ca^2);
    sa2 = -sa1;
    a = atan2(sa1,ca);
    % a=atan2(sa2,ca);
    
    %beta
    sb = (pm(3)-l1)/pw;
    cb1 = sqrt(1-sb^2);
    cb2 = -cb1;
    b = atan2(sb,cb1);
    % b=atan2(sb,cb2);
    
    %q2
    q2 = -pi/2+a+b;

    %% q3
    %gamma
    cg = (l3^2+l4^2-pw^2)/(2*l3*l4);
    sg1 = sqrt(1-cg^2);
    sg2 = -sg1;
    g = atan2(sg1,cg);
    %g=atan2(sg2,cg);
    
    %theta
    theta = atan(1.35/7.55);
    
    q3 = g-pi/2-theta;

    %% Orientación del efector final
    R06 = Ref;
    [R03,~] = tr2rt(robot.A([1 2 3],[q1 q2 q3]));
    R36 = R03\R06;
    
    %q4
    q4 = atan2(R36(3,3),R36(1,3));

    %q5
    q5 = atan2(cos(q4)*R36(1,3)+sin(q4)*R36(3,3),-R36(2,3));

    %q6
    q6 = atan2(-sin(q4)*R36(1,1)+cos(q4)*R36(3,1),-sin(q4)*R36(1,2)+cos(q4)*R36(3,2));

    %% Vector de variables de articulación
    q = [q1 q2 q3 q4 q5 q6];
end