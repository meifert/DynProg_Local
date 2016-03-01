if 0
global LadungAnfang Pel Verbrauch Charge Mv DPErgebnispfad T0 DZ Zeit Revs
clear L PeOpt R Charge Mv Pel 
DPErgebnispfad='C:\Mondemo\DynProg\Results\';
%TestName='ECERun3'
datestr(now)
%
% Fahrzeugdatei laden
load MondemoDatei
% Trajektorien laden
load MondemoTraj          % NEDC Trajectory
%load MondemoECETraj
%
NeueTraj
%
N=length(Mp);             % Zahl Zeitpunkte
%  Trakektorien:
%                 Time
%                 Speed
%                 Clutch
%                 Mb
%                 Mp
%                 DZ
%                 Revs   
%
% Set DP Variables
CC=10000;
Pload=200;
%Xmin=-3;
%Xmax= 3;
%ZahlZus=21;
QS=(Xmax-Xmin)/(ZahlZus-1);                           % Quantisierungsstufenzahl
X=[Xmin:QS:Xmax];
XmidIndex=interp1(X,[1:length(X)],0,'nearest');       % Index von Anfangsladezustand
USchritt=1;
Umax=2100;
Kmayer=1;                                             % Gewicht vom Mayerischen Gütemaß
%Klagrange=1;
%E=1;
% Restkostenmatrix initializieren
R=CC*ones(length(X),N);                               % Restkostenmatrix mit grosser Zahl belegen
R(XmidIndex,N)=0;
end
%
% Restkostmatrix aufbauen
%for k=N-1:-1:1
for k=24:-1:1
    OptFlag=0;
    T0=k-1                                           % Zeitpunkt beim Anfang von Schritt k
    Geschwindigkeit=Speed(k)
    if Speed(k)>0                                     % Fahrzeug bewegt sich bei Anfang von Schritt k
        if Clutch(k)>0.99                          % Kupplung starr
            if Mb(k)==0                               % kein Bremsmoment
                %%%%%%%%%%%%%%%%%%%%%% Schleife über Antriebspunkte
                for n=1:length(X)                     % Schleife über Zustandsindex
                    for Pel=0:USchritt*DZ(k)/9.55:Umax % Schleife über elektische Leistung
                        Mv=Mp(k);
                        LadungAnfang=X(n);
                        sim('Fahrzeug')
                        Kosten=(Klagrange*Verbrauch(length(Verbrauch)))+(Kmayer*interp1(X,R(:,k+1),Charge(length(Charge)))).^E;
                        %
                        if Kosten<R(n,k)              % Eingangsgrößen verursachen niedrigsten Kosten
                            R(n,k)=Kosten;            % neuer Wert in Restkostenmatrix scheiben
                            OptFlag=1;
                        end
                    end
                end
                %
            else %%%%%%%%%%%%%%%%%%%%% Schleife über Bremspunkte mit starrer Kupplung (Regen): Mb > 0
                %for n=1:length(X)                     % Schleife über Zustandsindex
                    Pel=Pload;
                    Mv=Mp(k);
                %    LadungAnfang=X(n);
                     LadungAnfang=0;
                    sim('Fahrzeug')
                %    Kosten=(Klagrange*Verbrauch(length(Verbrauch)))+(Kmayer*interp1(X,R(:,k+1),Charge(length(Charge))))^E;
                R(:,k)=(Klagrange*Verbrauch(length(Verbrauch)))+(Kmayer*interp1(X,R(:,k+1),X'+Charge(length(Charge)))).^E;
                R(isnan(R(:,k)),k)=CC;
                %    %
                %    if Kosten<R(n,k)                  % Eingangsgrößen verursachen niedrigsten Kosten
                %        R(n,k)=Kosten;                % neuer Wert in Restkostenmatrix scheiben
                        OptFlag=1;
                %    end
                % end
            end % Mb-Test
            %
        else %%%%%%%%%%%%%%%%%%%%% Kupplung offen: Clutch < 0.99
            %for n=1:length(X)                         % Schleife über Zustandsindex
                Pel=Pload;
                Mv=Mp(k);
                LadungAnfang=0; %X(n);
                sim('Fahrzeug')
            %    Kosten=(Klagrange*Verbrauch(length(Verbrauch)))+(Kmayer*interp1(X,R(:,k+1),Charge(length(Charge))))^E;
            R(:,k)=(Klagrange*Verbrauch(length(Verbrauch)))+(Kmayer*interp1(X,R(:,k+1),X'+Charge(length(Charge)))).^E;
            R(isnan(R(:,k)),k)=CC;
            %    %
            %    if Kosten<R(n,k)                      % Eingangsgrößen verursachen niedrigsten Kosten
            %        R(n,k)=Kosten;                    % neuer Wert in Restkostenmatrix scheiben
                    OptFlag=1;
            %    end
            % end
        end % Clutch-Test
    else %%%%%%%%%%%%%%%%%%%%% Fahrzeug steht
        %for n=1:length(X)                             % Schleife über Zustandsindex
            Pel=0;
            Mv=Mp(k);
            LadungAnfang=0; %X(n);
            sim('Fahrzeug')
        %    %DeltaSOC=Charge(length(Charge))
        %    Kosten=(Klagrange*Verbrauch(length(Verbrauch)))+(Kmayer*interp1(X,R(:,k+1),Charge(length(Charge))))^E;
        %(Kmayer*interp1(X,R(:,k+1),X+Charge(length(Charge)))).^E
        R(:,k)=(Klagrange*Verbrauch(length(Verbrauch)))+(Kmayer*interp1(X,R(:,k+1),X'+Charge(length(Charge)))).^E;
        R(isnan(R(:,k)),k)=CC;
        %    %
        %    if Kosten<R(n,k)                          % Eingangsgrößen verursachen niedrigsten Kosten
        %        R(n,k)=Kosten;                        % neuer Wert in Restkostenmatrix scheiben
                OptFlag=1;
        %    end
        % end
    end % Speed-Test
    %
    if OptFlag==0
        disp(' ')
        disp('No optimum found')
        Time_Step=k
        [XX YY]=meshgrid(1:N,X);
        figure;mesh(XX,YY,R);grid on;xlabel('Time Step');ylabel('Delta SOC');zlabel('Cost')
        datestr(now)
        pause
    end
end % Schleife über Zeitschritte
%
DPErgebnis.R=R;
DPErgebnis.Xmin=Xmin;
DPErgebnis.Xmax=Xmax;
DPErgebnis.X=X;
DPErgebnis.N=N;
DPErgebnis.ZahlZus=ZahlZus;
DPErgebnis.Kmayer=Kmayer;
DPErgebnis.Klagrange=Klagrange;
DPErgebnis.E=E;
DPErgebnis.USchritt=USchritt;
%
Command=['save ' strcat(strcat(DPErgebnispfad,TestName),' DPErgebnis')]
%
try
    eval(Command);
catch
    disp('Save not possible.')
end
%
try
    [XX YY]=meshgrid(1:N,X);
    figure;mesh(XX,YY,R);grid on;xlabel('Time Step');ylabel('Delta SOC');zlabel('Cost');title('Value Function Matrix')
catch
    disp('Plot not possible.')
end
%
Command=['print -djpeg ' strcat(DPErgebnispfad,TestName)]
%
try
    eval(Command)
catch
    disp('Print save not possible.')
end
%
disp(' ')
disp('Restkostenmatrix aufgebaut')
datestr(now)
%
DPVorwaerts                                       % Vorwärtssimulation
