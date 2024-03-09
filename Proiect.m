clearvars;
%% Date 
C = 5.75; % coeficient de curgere
u0 = 4.5; 
k = 0.025; % unul dintre coeficientii modelului de pompa -  sunt determinate experimental
% k caracterizeaza rezistenta la curgere a circuitului hidraulic (pg. 50)

% Restul de date
A = 332.5; % suprafata bazinului cm^2
k1 = 0.654; % restul de coeficienti ai pompei
k2 = -0.015;
k3 = -0.0006;

% gain-uri pompa
k11 = k2^2 + 4 * (k - k3) * k1;
k12 = 8 * (k - k3);
k13 = 2 * (k - k3);

k_eta = 8 * 10^(-5); % factor care intra in calculul randamentului pompei lamga debit si tensiunea de alimentare


%% Identificarea sistemului
% Conditii initiale 0 - treapta 0-4.5V
h0 = 10.22; %cm
q0 = 28.8; %cm3/s

% Treapta 4.5-5V
h = 12.99; %cm
q = 32.45; %cm3/s

kp = 1/3*(h - h0)/(u0 + 0.5 - u0);
val_stat = h0 + 0.63 * (h - h0);
Tp = 136.304;

Hp = tf(kp, [Tp 1]);

trcl = Tp/4;
Hcl = tf(1, [trcl 1]);

%% Control PI
Hr = minreal(1/Hp * Hcl/(1 - Hcl));
kr = 2.1661; Ti = kr * 1/0.01589;

trp = 4*(Tp + trcl); % timp de raspuns la perturbatie

% Var2:
Hr_v2 = pid(1/Hp * Hcl/(1 - Hcl));
kr_v2 = 2.17;
ki_v2 = 0.0159;

%% Feed Forward - perturbatie pe debitul de iesire
kpFF = (h - h0)/(q - q0);
kp_comp = (h - h0)/(5 - u0);
k_compensare = kpFF/kp_comp;

%% Cascada
% Hf1 - pentru elementul de executie
% Hf1 de la uc la iesirea filtrului
kf1 = (35.4 - q0)/(5 - u0);
valoare_stationara1 = 0.632*(35.35 - q0) + q0;
Tf1 = 0.76;
Hf1 = tf(kf1, [Tf1 1]);
T01 = Tf1/4;
H01 = tf(1, [T01 1]);
kr1 = Tf1/(kf1*T01);
Tr1 = Tf1;

%V2:
Hri2 = pid(1/Hf1*H01/(1-H01));
kri2 = 0.303;
ki2 = 0.399;


% Hf2
% functia de transfer de la iesirea pompei la iesirea senzorului
kf2 = ((h - h0)/3)/(q-q0);
valoare_stationara2 = 0.632*(h-h0)+h0;
Tf2 = 258.25;
Hf2 = tf(kf2, [Tf2 1]);
% tr = 136;
T02 = Tf2/12;
H02 = tf(1, [T02 1]);
kr2 = Tf2/(kf2*T02);
Tr2 = Tf2;

% V2
Hre2 = pid(1/Hf2*H02/(1-H02));
kre2 = 47.4;
ki2 = 0.184;