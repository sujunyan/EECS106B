function dynamics2 = dynamics2_gen(q,dq,C1,C2,D,Tau)
%DYNAMICS2_GEN
%    DYNAMICS2 = DYNAMICS2_GEN(Q,DQ,C1,C2,D,TAU)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    19-Apr-2019 02:40:33

t2 = q+1.0e-4;
t3 = q.*1.0e4;
t4 = t3+1.0;
t6 = q./2.0;
t7 = t6+5.0e-5;
t8 = sin(t2);
t9 = 1.0./t4;
t10 = 1.0./t4.^2;
t11 = cos(t2);
t13 = sin(t7);
t14 = t13.*(2.1e1./1.0e3);
t15 = t9.*t11.*4.2e2;
t16 = t8.*t10.*4.2e6;
t5 = t14-t15+t16;
t19 = cos(t7);
t20 = t19.*(2.1e1./1.0e3);
t21 = t8.*t9.*4.2e2;
t22 = t11./2.0;
t23 = t22-1.0./2.0;
t24 = t10.*t23.*8.4e6;
t12 = t20+t21+t24;
t17 = t5.^2;
t18 = t17./2.0e1;
t25 = t12.^2;
t26 = t25./2.0e1;
t27 = t18+t26;
t28 = 1.0./t27;
t29 = sin(q);
t30 = t29.^2;
t31 = q./t29-t29./q;
t32 = q.^2;
t33 = 1.0./t4.^3;
dynamics2 = [dq;Tau.*t28+t28.*(t13.*1.03005e-2+t8.*t10.*2.0601e6-(t11.*4.1202e4)./(q.*2.0e6+2.0e2)-1.0./q.^3.*t30.^2.*(1.0./t29.^4.*t32.^2-1.0).*(C1.*2.0+C2.*t31.^2.*4.0))-dq.*t28.*(D-dq.*((t12.*(t13.*1.05e-2-t15+t8.*t10.*8.4e6+t23.*t33.*1.68e11))./2.0e1-(t5.*(t19.*1.05e-2+t21+t10.*t11.*8.4e6-t8.*t33.*8.4e10))./2.0e1))];
