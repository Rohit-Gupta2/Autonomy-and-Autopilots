%Steady state rectilinear flight linearized about
%
%            state set point 
%
%V0=TAS=50 m/s
%H0=Altitude=330m
%gamma=flight angle=0 rad flight angle     
%flaps=0 rad
%alpha0=0.072255 rad   (0.07225516589006)             
% beta0=0 rad                  
% P0=0 rad/sec
% Q0=0 rad/sec
% R0=0 rad/sec
% TETA0=0.07225 rad    (0.07225516589006)                
% phi0=0 rad    
% n0=2058.5 rpm  (2058.505700174778)
% 
%  input set point
%
%      
% deltae0=-0.010574 rad   (-0.01057399250748)             
% deltaa0=0 rad
% deltar0=0 rad
% throttle0=0.4287 percentage throttle aperture (0.42869643410519)
%
%
%
%state: xlong=[ V alpha Q teta H n]'
%i.e. [TAS att.ang. pitch_rate elev.angle altitud. rpm]
Along =[

   -0.0447    5.6993         0   -9.8056         0    0.0004
   -0.0076   -1.4866    0.9399    0.0000         0   -0.0000
    0.0293  -12.1418   -7.9809         0         0   -0.0000
         0         0    1.0000         0         0         0
         0  -50.0000         0   50.0000         0         0
         0         0         0         0         0   -0.4462]
%ulong=[deltae throttle] i.e. [defless.elevator   percentage_throttle]
Blong =[

         0    1.7371
   -0.3067   -0.0025
  -40.6088   -0.2448
         0         0
         0         0
         0  706.5875]
%
% xlat=[ beta P  R phi  psi yaw angle]
Alat =[
   -0.1475    0.0722   -0.9974    0.1956         0
   -6.3380   -3.6229    0.5937         0         0
    2.9185   -0.4573   -0.6989         0         0
         0    1.0000    0.0724         0         0
         0         0    1.0026         0         0]
%ulat=[deltaa deltar] i.e. [defless.eileron.   defless.rudder]
Blat =[
   -0.0027    0.0427
   -5.6691    1.0274
   -0.1057   -2.4358
         0         0
         0         0]
%     
%  wind gusts distribution matrix -> dot(xlong)=Along*xlong+Blong*ulong+Elong*wlong
% with wlong=[gust_x  gust_z]'
%in simulator have to be defined an augmented matrix Blong_est=[Blong Elong]
%
%wind covariance Pwind=0.7
%se poi si vogliono filtrare i due rumori per ottenere vento correlato
%wind correlation ime constant
%tau_u=2.326;       longitudinal
%tau_w=0.9434;      vertical 
%
Elong =[

  -0.07224216819218   0.21477104529793
  -0.00954723493499  -0.03061702460982
   0.02878416744368  -0.24103859236200
                  0                  0
   0.07219231056339  -0.99739073100542
                  0                  0]
%with dot(xlat)=Alat*xlat+Blat*ulat+Elat*wlat
%aand wlat=[gust_y]
%Pwind=0.7
%tau_v=7.143;   
%
Elat =[
  -0.00295062123116
  -0.12676095004693
   0.05836991411251
                  0
                  0]


