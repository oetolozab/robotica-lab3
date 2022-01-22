%Parámetros del manipulador - dm
l1=6.15;
l2=1.00;
l3=7.05;
l4=sqrt(1.35^2+7.55^2);
l5=0.85;

% % DH modificado
%Creación del robot
L(1)=Link([0 l1 0 0],'modified');
L(2)=Link([0 0 l2 pi/2],'modified');
L(2).offset=pi/2;
L(3)=Link([0 0 l3 0],'modified');
L(4)=Link([0 7.55 1.35 pi/2],'modified');
L(5)=Link([0 0 0 -pi/2],'modified');
L(6)=Link([0 0 0 pi/2],'modified');

irb2400=SerialLink(L,'name','ABB IRB2400');
irb2400.tool = [eye(3) [0 0 l5]';0 0 0 1];
% irb2400

syms q1 q2 q3 q4 q5 q6 
MTH_01=MTH(q1, l1, 0, 0);
MTH_12=MTH(q2, 0, l2, pi/2);
MTH_23=MTH(q3, 0, l3, 0);
MTH_34=MTH(q4, 7.55, 1.35, pi/2);
MTH_45=MTH(q5, 0, 0, -pi/2);
MTH_56=MTH(q6, 0, 0, pi/2);
MTH_6T=[eye(3) [0 0 l5]';0 0 0 1];

MTH_02=simplify(MTH_01*MTH_12);
MTH_03=simplify(MTH_01*MTH_12*MTH_23);
MTH_04=simplify(MTH_01*MTH_12*MTH_23*MTH_34);
MTH_05=simplify(MTH_01*MTH_12*MTH_23*MTH_34*MTH_45);
MTH_06=simplify(MTH_01*MTH_12*MTH_23*MTH_34*MTH_45*MTH_56);
MTH_0T=simplify(MTH_01*MTH_12*MTH_23*MTH_34*MTH_45*MTH_56*MTH_6T);


%%
Jacobiano=sym('a',[6 6]);
J_p=sym('a',[3 6]);
J_w=sym('a',[3 6]);

pos_x=MTH_0T(1,4);
pos_y=MTH_0T(2,4);
pos_z=MTH_0T(3,4);
pos=[pos_x,pos_y,pos_z];
qs=[q1,q2,q3,q4,q5,q6];

for i=1:3
    for j=1:6
        J_p(i,j)=vpa(simsyms(vpa(diff(pos(i),qs(j)))),2);
    end
end

J_w(1:3,1)=vpa(VelAngIndi(MTH_01),2);
J_w(1:3,2)=vpa(VelAngIndi(MTH_02),2);
J_w(1:3,3)=vpa(VelAngIndi(MTH_03),2);
J_w(1:3,4)=vpa(VelAngIndi(MTH_04),2);
J_w(1:3,5)=vpa(VelAngIndi(MTH_05),2);
J_w(1:3,6)=vpa(VelAngIndi(MTH_06),2);

for i=1:3
    for j=1:6
        J_w(i,j)=vpa(simsyms(J_w(i,j)),2);
    end
end

jacobiano=[J_p;J_w];

q_nuevos=deg2rad([100,80,35,100,50,200]);
q1=q_nuevos(1);
q2=q_nuevos(2);
q3=q_nuevos(3);
q4=q_nuevos(4);
q5=q_nuevos(5);
q6=q_nuevos(6);

jacobiano_num=vpa(subs(jacobiano),2);
q_articulares=simplify(rad2deg(inv(jacobiano_num)*[10;20;5;5;10;-5])) %cm/s y rad/s

%%

function [matriz]=MTH(theta,d_traslacion,a_traslacion,alpha)
    T_theta1=trotz(theta);
    T_d1=transl(0,0,d_traslacion);
    T_a1=transl(a_traslacion,0,0);
    T_alpha1=trotx(alpha);
    matriz=T_alpha1*T_a1*T_theta1*T_d1;
end

