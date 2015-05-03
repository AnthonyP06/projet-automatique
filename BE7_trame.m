clear all;
close all;

%**************************************************************************
% Satellite antenna position control system
%**************************************************************************
% Author : Caroline Berard - Joel Bordeneuve
% octobre 2014
%**************************************************************************

%%
%**************************************************************************
% Initialisation data 
%**************************************************************************
Re = 2;         % Resistance de l'induit (ohms)
Le = 0.002;     % Inductance de l'induit (H)
Ke = 0.2;       % (V/rad/s)
Kg = 0.2;       % (V/rad/s)
Jt = 0.002;     % Inertie (m^2.kg)
N = 300;        % Rapport de reduction (_)
Kci = 1;        % Capteur de courant (_)
Kcv = 0.0314;   % Capteur de vitesse (V/rad/s)
Kcp = 1;        % Capteur de position (_)

%%
%**************************************************************************
% Simulation data
%**************************************************************************
pert = 5;               % amplitude perturbation de couple
tfin = 10;              % duree simulation
tf1 = 5;                % debut simulation perturbation
t = 0:0.01:tfin;        % vecteur temps
t1 = 0:0.01:tf1;        
t2 = tf1+0.01:0.01:tfin;
% input signal: 1rst is input voltage, 2nd is disturbance
E = [100*ones(size(t')) [zeros(size(t1'));pert*ones(size(t2'))]];


%%
%**************************************************************************
% Analyse de la boucle ouverte
%**************************************************************************

% Representation d'etat du systeme
    sys = linmod('open_loop_system');
    % Matrice de transfert entre la commande en entree et les differentes sorties
    [num, den] = ss2tf(sys.a, sys.b, sys.c, sys.d, 1);
    % Fonction de transfert entre entree et sortie 'Position'
    F = tf(num(1,:), den(1,:)); 

    
% Etude de l'observabilite et de commandabilite
    % Matrice d'observabilite
    O = obsv(sys.a, sys.c);
    rank(O);
    
    % Matrice de commandabilite
    C = ctrb(sys.a, sys.b);
    rank(C);

    
% Etude de la stabilite
    % Calcul des valeurs propres de la matrice d'etat A du systeme
    eig(sys.a);

    
% Trace des graphes
    % Diagramme de Bode
    h1 = figure('Name', 'Diagramme de Bode du systeme en boucle ouverte');
    bode(F);
    grid on;
    xlabel('Pulsation');
    title('Diagramme de Bode du systeme en boucle ouverte');
    saveas(h1, '.\Compte_Rendu\Figures\BodeBO', 'jpg')
    
    % Diagramme de Bode avec marges de gain et de phase
    h2 = figure('Name', 'Marges de gain et de phase du systeme en boucle ouverte');
    margin(F);
    xlabel('Pulsation');
    title('Marges de gain et de phase du systeme en boucle ouverte');
    saveas(h2, '.\Compte_Rendu\Figures\MargesGainPhaseBO', 'jpg');
    
    % Diagramme de Nyquist
    h3 = figure('Name', 'Diagramme de Nyquist du systeme en boucle ouverte');
    nyquist(F);
    grid on;
    xlabel('Axe reel');
    ylabel('Axe imaginaire');
    title('Diagramme de Nyquist du systeme en boucle ouverte');
    saveas(h3, '.\Compte_Rendu\Figures\NyquistBO', 'jpg');
    
    % Diagramme de Black-Nichols
    h4 = figure('Name', 'Diagramme de Black-Nichols du systeme en boucle ouverte');
    nichols(F);
    grid on;
    xlabel('Phase Boucle Ouverte');
    ylabel('Gain Boucle Ouverte');
    title('Diagramme de Black-Nichols du systeme en boucle ouverte');
    saveas(h4, '.\Compte_Rendu\Figures\NicholsBO', 'jpg');

% Marges de phase et de gain
M = allmargin(F);
    
% Simulation temporelle faite sur le fichier SimuLink


%%
%**************************************************************************
% Reglage par synthese frequentielle 
%**************************************************************************

% Trace du lieu des racines (lieu d'Evans)
    h5 = figure('Name', 'Lieu des racines');
    rlocus(F);
    xlabel('Axe reel');
    ylabel('Axe imaginaire');
    title('Lieu des racines');
    saveas(h5, '.\Compte_Rendu\Figures\LieuRacines', 'jpg');

    
% Reglage du gain pour stabiliser le systeme
    % K de 0 a +inf
    %rltool(F);
    
    % K de -inf a 0
    %rltool(-F);
        % On constate que quelque soit la valeur de K (positive ou negative), on ne peut pas stabiliser le
        % systeme. Il faut donc stabiliser le systeme par une methode differente.
    
        
% Reglage de l'avance de phase
        % A l'aide du theoreme de la valeur initiale, on a:
        % lim(p->infinity) pK*((1+aTp)/(1+Tp))*10*pi/180p =300
        % a*K = 300/(10*pi/180)
        % D'ou
    % Expression du produit aK_av
        aK_av = 300/(10*pi/180);
    
    % Fonction de transfert en B.O. multipliee par le gain aK_av
        F_av_non_cor = aK_av*F;
    
    % Trace du nouveau diagramme de Bode et observation des nouvelles
    % marges (gain et phase)
        h6 = figure('Name', 'Marges de gain et de phase du systeme en boucle ouverte corrige');
        margin(F);
        xlabel('Pulsation');
        title('Marges de gain et de phase du systeme en boucle ouverte corrige');
        saveas(h6, '.\Compte_Rendu\Figures\MargesGainPhaseBO_av', 'jpg');
    
    % Calcul des parametres necessaires au reglage de l'avance de phase
        M_av = allmargin(F_av_non_cor);
        % On trouve 32 degre de marge de phase. On considere que le systeme est
        % stable a partir de 45 degres. Il faut donc un PhD = (45-32) =13
        % degres.
        % On majore le PhD de 20% pour avoir une marge suffisante d'ou PhD
        % = 16 degres.
        % On inverse alors la relation du cours pour trouver a.
        % PhD = arcsin((a-1)/(a+1));
        % a = 1,76 => le vrai K_av du debut vaut aK/1,76 = (180*300)/(1,76*10*pi) = 976 
        % On trouve graphiquement Wc = 19 rad/s et T_av = 0.04s
        
        Wc = M_av.PMFrequency;              % Pulsation (rad/s)
        PhD = (45 - M_av.PhaseMargin)*1.2;  % Angle PhD majore de 20% (deg)
        a = (1+sind(PhD))/(1-sind(PhD));    % Parametre du correcteur
        T_av = 1/(Wc*sqrt(a));              % Temps caracteristique (s)
        K_av = aK_av/a;                     % Gain reel
        
        num_av = [a*T_av 1];                % Numerateur de la fct de transfert du correcteur avance de phase
        den_av = [T_av 1];                  % Denominateur de la fct de transfert du correcteur avance de phase
        F_av = K_av*tf(num_av, den_av)*F;   % Nouvelle fonction de transfert corrigee par l'avance de phase
    
    % Simulations temporelles avec prise en compte de la perturbation
        %[tsim, X, y] = sim('.\Simulink_prof\closed_loop_system_frequential_av', tfin, [], [t' E]);
        %figure(1)
        %subplot(2,1,1)
        %plot(tsim, y(:, 2), 'b');
        %hold on
        %title('position');
        %subplot(2,1,2);
        %plot(tsim, y(:,2), 'r');
        %hold on
        %title('commande');
        
% Reglage du proportionnel integrateur(PI)
    % Calcul des parametres necessaires au reglage du PI
        M2 = allmargin(F_av);
        
    % Recuperation des grandeurs necessaires
        W_pi = M2.PMFrequency;
        T_pi = 10/W_pi;
        
    % Fonction de transfert du PI
        num_pi = [T_pi 1];
        den_pi = [T_pi 0];
        F_pi = F_av*tf(num_pi, den_pi);
        
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reglage par retour d'etat
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Definition des nouveaux poles
    Pc = [-10-10*1i -10+10*1i -20]'; 
    
% Calcul de la matrice K de retour d'etat
    K = place(sys.a, sys.b(:,1), Pc);
    
% Calcul du gain statique correctif pour obtenir la position voulue
    % Systeme en BF du retour d'etat
    sys_bf = linmod('close_loop_system_retour_etat');
    
    % Matrice des gains statiques pour toutes les couples entrees/sorties
    MK_stat = dcgain(sys_bf.a, sys_bf.b, sys_bf.c, sys_bf.d);
    
    % Gain statique pour entree commande (1) et sortie position (1)
    K_stat = MK_stat(1,1);
    
% Ajout d'un effet integral
    % Definition des nouveaux poles
    Po = [-10-10*1i -10+10*1i -20 -235]';
    
    % Calcul de la nouvelle representation a 4 etats et de la boucle
    % ouverte (cas avec la correction integrale)
    sys_int = linmod('open_loop_system_retour_etat_int');
    [num_int den_int] = ss2tf(sys_int.a, sys_int.b, sys_int.c, sys_int.d, 1);
    F_int = tf(num_int(1,:), den_int(1,:));
    
    % Calcul de la nouvelle matrice K_int de retour d'etat (4 etats)
    K_int = place(sys_int.a, sys_int.b(:,1), Po);
    
    % Calcul du gain statique correctif pour obtenir la position voulue
        % Systeme en BF du retour d'etat
        sys_bf_int = linmod('close_loop_system_retour_etat_int');
    
        % Matrice des gains statiques pour toutes les couples entrees/sorties
        MK_stat_int = dcgain(sys_bf_int.a, sys_bf_int.b, sys_bf_int.c, sys_bf_int.d);
    
        % Gain statique pour entree commande (1) et sortie position (1)
        K_stat_int = MK_stat_int(1,1);
        
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reglage de l'estimateur
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Definition des poles de la matrice de gain d'observation, plus rapides
% que la dynamique du systeme reel
    Po = [-20+20*1i -20-20*1i -40]';
 
% Matrice de retour d'etat
    G = place((sys.a)', (sys.c(1,:))', Po);
