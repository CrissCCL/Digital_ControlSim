clear all, close all, clc
%% discretizacion modelo
Ts=0.1;
gp=tf(20,[50 1]);
gpd=c2d(gp,Ts,'zoh');
%% discretizacion controlador
Kp=0.8;
Ti=9;
K0=Kp+Kp*Ts/(2*Ti);% parametros control PID discreto
K1=-Kp+Kp*Ts/(2*Ti); % parametros control PID discreto

%% simulacion sistema lazo cerrado por integracion directa
[num,den]=tfdata(gpd,'v');
t=0:Ts:60;
Ref=ones(1,length(t));
y1=0;
u1=0;
u2=0;
error1=0;
for i=1:length(t)
    %%%%% proceso%%%%%%
    y(i)=num(2)*u1;
    y(i)=y(i)-den(2)*y1;
    %%%% retroalimentacion %%%%%
    error=Ref(i)-y(i); % punto suma 
    Usim(i)=u1+K0*error+K1*error1; % accion de control

    %%%% saturacion %%%%%%
    if Usim(i)>100
        Usim(i)=100;
    end
     if Usim(i)<0
        Usim(i)=0;
    end
        
    y1=y(i);
    u1=Usim(i);
    error1=error;

end
subplot(2,1,1)
plot(t,y,'+',t,Ref,'--','MarkerSize', 4)
xlabel('tiempo (s)')
ylabel('Respuesta')
legend('Sim','Ref')
subplot(2,1,2)
plot(t,Usim,'+','MarkerSize', 4)
ylabel('potencia')
xlabel('tiempo (s)')