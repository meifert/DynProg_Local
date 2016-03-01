global Mv cum_fuel SlipStick GangZahl Mb v_veh0 Pe Me Charge w_axle0 a P_Load ZeitSchritt SimName 
global fc_max_trq fc_trq_scale fc_map_spd fc_spd_scale
%
disp('Berechnung von Restkostenmatrix und optimaler Trajektorie ohne Gangwahl...')
%
EndZeit=1
CC=10000                                % Anfangsbelegung von Restkostenmatrix
%
StartTime=datestr(now)
%
SimName='MondeoDP';
%
% Workspace vorbereiten
%
% Fahrzeug laden
load_parms_offline                      % Mondeo-Data laden
%
% Batterieparameter laden
MondeoBattData
%Battdata;                              % Batteriedata
%
% Powernetleistung laden
P_Load=1000
%
% Zyklus laden
load MondeoZyklus                       % Vektoren laden: Zeit, Geschwindigkeit, Gang, Mgef, Drehzahl
%
N=max(find(Geschwindigkeit>0))+1;
T_Zyk=Zeit(size(Zeit,1))-1;             % Zykluslänge in Sekunden
ZeitSchritt=Zeit(2)-Zeit(1);            % Zeitschrittlänge in Sekunde
%
% Zustandsgitter feststellen
% Zustandsvariable X --> Ladungszustand zwischen -5% und +5% der Nennkapazität
Xmin=-5.0;                              % untere Zustandsbeschränkungen für X
Xmax=5.0;                               % obere Zustandsbeschränkungen für X
ZahlZus=21;                             % Anzahl quantisierter Zustände
QS=(Xmax-Xmin)/(ZahlZus-1);             % Quantisierungsstufenzahl
X=[Xmin:QS:Xmax];                       % Zuständsgitter X2 definieren
XmidIndex=interp1(X,[1:length(X)],0,'nearest') % Index von Null-Prozent Ladungszustand
%
if XmidIndex>1
    X1Index=XmidIndex-1
    X2Index=XmidIndex+1
else
    X1Index=1
    X2Index=length(X)
end
%
Xf=0;                                   % Endzuständ des Zyklus: Ladungszustand=0
%
% Steuervektoren festellen
% Eingangsgröße: U --> elektrische Leistung in W 
Umin=0;
Umax=5000;
ZahlU=11;                               % Anzahl quantsierter Steuergrößen: Leistungswerte 0, 500, 1000, 1500, 2000,... 5000
QSU=(Umax-Umin)/(ZahlU-1);              % Quantisierungsstufe für Eingangsgröße U
U=[Umin:QSU:Umax];                      % Steuervektor U2=Me definieren
%
% dynamische Programmierung durchführen
%
R=CC*ones(size(X,2),N);                 % Restkostenmatrix mit grosser Zahl belegen
%
% Letzter Zeitschritt bearbeiten; Endgeschwindigkeit ist immer Null.
% Es wird angenommen, daß die Kupplung öffen ist.
k=N;
GangZahl=Gang(N);                       % letzter Gang im Zyklus
Vel=Geschwindigkeit(N);                 % letzte Geschwindigkeit
GeschAnfang=Geschwindigkeit(N-1)/3.6;   % vorletzte Geschwindigkeit
%Steig=Steigung(N);                     % letzter Steigungswinkel
letztesMoment=Mgef(N);
%
Count=1;                                % Waitbar-Index
WBHdl=[];                               % Waitbar-Handle
%
MeMax=-EGrenze(Drehzahl(N-1))       % maximales elektrisches Bremsmoment für Enddrehzahl 
%
for n=X1Index:X2Index               % Schleife über Zustandsgröße X bzw. Ladungszustand
    LadungAnfang=X(n);              % Anfangszustand von X bzw. Ladungszustand
    %                               
    Mb=Mgef(N)
    Mv=0;                           % Verbrennungsmotormoment nicht vorhanden
    Pe=0;
    SlipStick=0;                    % Kupplung getrennt
    sim(SimName);                   % Simulation mit gegebenen Zustands- und Eingangsvariablen durchführen
    %
    XEnde=Charge(length(Charge));   % Endzustände nach Simulation bei k=N
    %
    % Mayersches Gütemaß berechnen
    Kosten=100*(abs(XEnde-Xf));     % Vergleich mit gewünchten Endzuständen
    %
    if Kosten<R(n,k)                % Eingangsgrößen verursachen niedriger Kosten
        R(n,k)=Kosten;              % neue Kosten
        %U1Opt(n,k)=Mv;              % optimales Verbrennungsmotormoment
        %U2Opt(n,k)=Me;              % optimales elektrisches Moment
    end
end
%
% Restliche Zeitschritte bearbeiten
%
ZahlBerechnungen=N-2
for k=(N-1):-1:2                    % Rückwärtsschleife über alle restliche Zeitschritte
    Vel=Geschwindigkeit(k);         % Soll-Geschwindigkeit von Punkt k
    if Vel>0                        % Fahrzeug bewegt sich und steht nicht
        MeMax=1;                    % maximales Bremsmoment~=0
        GeschAnfang=Geschwindigkeit(k-1)/3.6;
        GangZahl=Gang(k);           % Gang für Zeitpunkt k
        Count=0;                    % Index für Waitbar
        OptFlag=0;                  % OptFlag=0 --> niedrigere Kosten gefunden; OptFlag=1 --> keine niedrigere Kosten gefunden
        %
        % Schleifen für Verzögerungspunkte
        if Mgef(k)<0                % Verzögerung oder Stillstandpunkt
            for n=1:size(X,2)       % Schleife über Zustandsgröße X Ladungszustand
                LadungAnfang=X(n);  % Anfangszustand von Ladungszustand
                %for m=1:size(U,2)  % Schleife über elektrisches Moment
                %
                %delete(WBHdl)
                %WBHdl=waitbar(Count/(size(X,2)),strcat('Verzögerungspunkt...',num2str(k)));
                %drawnow            % Waitbar aktuellizieren
                Count=Count+1;
                %
                %DZ=Drehzahl(k)
                if k~=N
                    MeMax=max([-EGrenze(Drehzahl(k-1));-EGrenze(Drehzahl(k));-EGrenze(Drehzahl(k+1))]); % maximales (wenig negatives) elektrisches Bremsmoment für Enddrehzahl 
                else
                    MeMax=max([-EGrenze(Drehzahl(k-1));-EGrenze(Drehzahl(k))]);
                end
                %
                if Drehzahl(k)>900  % Starre Verbindung zwischen Verbrennungsmotor und Antriebsstrang
                    SlipStick=1;
                    if Mgef(k)<MeMax
                        Me=MeMax;
                        Mb=Mgef(k)-MeMax;
                    else
                        Me=Mgef(k);
                        Mb=0;
                    end
                    Pe=-Me*((Drehzahl(k-1)+Drehzahl(k))/2)/9.55;
                else                % Kupplung trennt Verbrennungsmotor von Antriebsstrang
                    SlipStick=0;
                    Pe=0;           % Keine Generation bei getrennter Kupplung
                end
                %
                Mv=0;               % Verbrennungsmotormoment nicht vorhanden
                %
                sim(SimName)        % Simulation mit gegebenen Zustands- und Eingangsvariablen durchführen
                %
                XEnde=Charge(length(Charge)); % Endzustand
                %
                % Lagrangesches Gütemaß
                % disp('Lagrange')
                Kosten=0*100*(abs(XEnde-Xf)); % Vergleich mit gewünchten Endzuständen
                %
                % Restkosten addieren
                R_Rest=interp1(X,R(:,k+1),XEnde);
                Kosten=Kosten+R_Rest;
                %
                if MeMax~=0         % Drehzahl innerhalb Betriebsbereich der E-Maschine
                    if Kosten<R(n,k)       % Eingangsgrößen verursachen niedriger Kosten
                        %disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!neue Kosten!!!!!!!!!!!!!!!!!!!!!!!')
                        R(n,k)=Kosten;     % neue Kosten
                        %U1Opt(n,k)=Mv;    % optimales Verbrennungsmotormoment
                        %U2Opt(n,k)=Me;    % optimales elektrisches Moment
                        %U3Opt(n,k)=Mb;    % optimales Bremsmoment
                        OptFlag=1;
                    end
                else                % Drehzahl ausserhalb Betriebsbereich der E-Maschine
                    R(n,k)=CC;
                    OptFlag=1;
                end
                %end
            end
            %
        else                        % Mgefordert > 0; Schleifen über Antriebspunkte
            for n=1:size(X,2)       % Schleife über Zustandsgröße X bzw. Ladungszustand
                LadungAnfang=X(n);  % Anfangszustand von X bzw. Ladungszustand
                for m=1:size(U,2)   % Schleife über elektrische Leistung
                    %
                    %delete(WBHdl)
                    %WBHdl=waitbar(Count/(size(X,2)*size(U,2)),strcat('Antriebspunkt...',num2str(k)));
                    %drawnow        % Waitbar aktuellizieren
                    Count=Count+1;
                    %
                    Pe=U(m);        % elektrische Leistung
                    Me=-2*9.55*Pe/(Drehzahl(k-1)+Drehzahl(k));
                    Mv=Mgef(k)-Me;  % Verbrennungsmotormoment
                    Mb=0;           % Bremsmoment
                    %
                    MeMax=min([EGrenze(Drehzahl(k-1));EGrenze(Drehzahl(k));EGrenze(Drehzahl(k+1))]); % positives maximales Moment
                    %
                    if Mv>0 & 9.55*Pe/Drehzahl(k)<=MeMax
                        sim(SimName)% Simulation mit gegebenen Zustands- und Eingangsvariablen durchführen
                        %
                        % Lagrangesches Gütemaß
                        % Ladezustandkostenfaktor: 100
                        XEnde=Charge(length(Charge));  % Endzustände nach Simulation bei Zeitpunkt k
                        Kosten=0*100*(abs(XEnde-Xf));    % Vergleich mit gewünchten Endzuständen
                        %
                        if 0
                            if EGM(length(EGM))~=0 & ~isnan(EGM(length(EGM))) & EGM(length(EGM))>=0
                                Kosten=Kosten+(EGM(length(EGM))); % elektrischer Gütemaß zu Kosten addieren
                            else
                                Kosten=Kosten+CC;
                            end
                        end
                        %
                        %Kosten=Kosten+(VGM(length(VGM))); % Verbrennungsmotor-Gütemaß zu Kosten addieren
                        Kosten=Kosten+(cum_fuel(length(cum_fuel))); % Verbrennungsmotor-Gütemaß zu Kosten addieren
                        %
                        %Restkosten addieren
                        R_Rest=interp1(X,R(:,k+1),XEnde);
                        Kosten=Kosten+R_Rest;
                        %
                        if MeMax~=0           % Drehzahl innerhalb Betriebsbereich der E-Maschine
                            if Kosten<R(n,k)   % Eingangsgrößen verursachen niedriger Kosten
                                %disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!neue Kosten!!!!!!!!!!!!!!!!!!!!!!!')
                                R(n,k)=Kosten;  % neue Kosten
                                %U1Opt(n,k)=Mv; % optimales Verbrennungsmotormoment
                                %U2Opt(n,k)=Me; % optimales elektrisches Moment
                                %U3Opt(n,k)=Mb; % optimales Bremsmoment
                                OptFlag=1;
                            end
                        else                  % Drehzahl ausserhalb Betriebsbereich der E-Maschine
                            R(n,k)=CC;
                            OptFlag=1;
                        end
                    end 
                end
            end
        end
        if OptFlag==0
            disp('Keiner optimaler Punkt gefunden!')
            ZeitZahl=k
            Gefordertes_Moment=Mgef(k)
            EndDrehzahl=Drehzahl(k)
            EndGeschwindigkeit=Geschwindigkeit(k)
            MaxBremsMoment=MeMax
            pause
        end
        %
    else % Vel=0 --> Stillstandpunkt
        for n=X1Index:X2Index               % Schleife über Zustandsgröße X bzw. Ladungszustand
            LadungAnfang=X(n);              % Anfangszustand von X bzw. Ladungszustand
            %                               
            Mb=0;
            Mv=0;                           % Verbrennungsmotormoment nicht vorhanden
            Pe=0;
            SlipStick=0;                    % Kupplung getrennt
            sim(SimName);                   % Simulation mit gegebenen Zustands- und Eingangsvariablen durchführen
            %
            XEnde=Charge(length(Charge));   % Endzustände nach Simulation bei k=N
            %
            %Restkosten addieren
            R_Rest=interp1(X,R(:,k+1),XEnde);
            Kosten=R_Rest;
            %
            if Kosten<R(n,k)                % Eingangsgrößen verursachen niedriger Kosten
                R(n,k)=Kosten;              % neue Kosten
                %U1Opt(n,k)=Mv;              % optimales Verbrennungsmotormoment
                %U2Opt(n,k)=Me;              % optimales elektrisches Moment
            end
        end
    end
end
delete(WBHdl)
StartTime
datestr(now)
%
%save N:\common\ft2e_stg\Eifert\Work\RK R
try
    Kommand=['save RK R']
    eval(Kommand)
end
%
try
    Command=['save a:\RK R']
    eval(Command)
end
%
% Vorwärtssimulation durchführen und Trajektorie plotten
[Mv Me Mb]=VorDP(Geschwindigkeit,Mgef,Faktor,Steigung,Zeit,Gang,Drehzahl,N,U,X,SimName,R);
%
StartTime
datestr(now)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Mv, Me, Mb]=VorDP(Geschwindigkeit,Mgef,Faktor,Steigung,Zeit,Gang,Drehzahl,N,U,X,SimName,R);
% Die Funktion VorDP() führt eine Vorwärtssimulation für dynamische Programmierung mit
% einer Restkostenmatrix durch und plottet die Trajektorien von Verbrennungs-, Brems- 
% und elektrischen Momenten.
%    Faktor: Vektor von elektrischen Lastfaktoren; Dateiname 'LastFaktoren'
%    N: berechnet in Hauptroutine mit der Formel N=max(find(Geschwindigkeit>0))+1;
%    U: Vektor von Werten von elektrischen Moment
%    X: Vektor von Verten von Ladungszustand
%    SimName: String mit Name des Simulink-Modells
%    R: Restkostenmatrix
% global Mb Me Mv Zeit ELF
% global GeschAnfang LadungAnfang Vel VGM EGM GangZahl ZeitSchritt
% global VFlaechenData CO2FlaechenData COFlaechenData HCFlaechenData NOxFlaechenData GMFlaechenData EAFlaechenData EBFlaechenData
% global i_diff i_1 i_2 i_3 i_4 i_5 T_Mot r_rad J_4rad m_eff c_r g c_w A_f J_Mot rho_L
% global B_Sp B_Kap ess_tmp ess_soc ess_voc ess_r_dis ess_r_chg ess_module_num Steig
% global EndZeit VerbrauchAnfang SchrittVer DurchInvsWirkAnfang DurchInvsWirk DurchEnergie DurchEnergieAnfang
global Mv cum_fuel SlipStick GangZahl Mb v_veh0 Pe Me Charge w_axle0 a P_Load ZeitSchritt
%
MvOpt=[];
MeOpt=[];
MbOpt=[];
PeOpt=[];
%
LadungAnfang=0;                  % Anfangsladungszustand
InitialCharge=0;
GeschAnfang=0;                   %
VerbrauchAnfang=0;
DurchInvsWirkAnfang=0;           %
DurchEnergieAnfang=0;
Vel=1;                           %
WBHdl=[];
KOZFlag=0;                       % Flag für keinen optimalen Zustand
%
for k=2:N                        % Schleife über Beschleunigungspunkte
    %
    v_veh0=Geschwindigkeit(k-1); % Anfangsgeschwindigkeit vor Beschleunigung
    M=Mgef(k)
    ELF=Faktor(k);               % elektrischer Lastfaktor
    %Steig=Steigung(k);          % Steigungswinkel in Grad
    EndZeit=k*ZeitSchritt;       % Endzeit von ZeitSchritt
    L(k)=LadungAnfang;
    %
    GangZahl=Gang(k-1);
    %
    if Geschwindigkeit(k)>0 %Geschwindigkeit(k+1)>0; Zielgeschwindigkeit ist größer als Null
        Kosten=CC;
        %
        if Mgef(k)>0                % Antriebsmoment
            disp('Antriebspunkt')
            SlipStick=1;
            %
            for m=1:size(U,2)        % Schleife über U2 bzw. elektrische Leistung
                %
                %delete(WBHdl)
                %WBHdl=waitbar(m/size(U,2),strcat('Vorwärtssimulation...',num2str(k)));
                %drawnow               % Waitbar aktuellizieren
                %
                Pe=U(m);              % elektrisches Moment
                Me=-2*9.55*Pe/(Drehzahl(k-1)+Drehzahl(k));
                Mv=Mgef(k)-Me;        % Verbrennungsmotormoment
                Mb=0;                 % Bremsmoment
                %
                if k~=N
                    MeMax=min([EGrenze(Drehzahl(k-1));EGrenze(Drehzahl(k));EGrenze(Drehzahl(k+1))]);
                    MvMax=min([VGrenze(Drehzahl(k-1));VGrenze(Drehzahl(k));VGrenze(Drehzahl(k+1))]);
                else
                    MeMax=min([EGrenze(Drehzahl(k-1));EGrenze(Drehzahl(k))]);
                    MvMax=min([VGrenze(Drehzahl(k-1));VGrenze(Drehzahl(k))]);
                end
                %
                %if Mv>0 & Mv<=GMFlaechenData.MAXY & Me<=MeMax & Me>=-MeMax % gefordertes Verbrennungsmotormoment innerhalb möglichen Bereich
                if Mv>0 & Mv<=MvMax & Me<=MeMax & Me>=-MeMax
                    sim(SimName)
                    XEnde=Charge(length(Charge));  % Endzustand nach Simulation bei Zeitpunkt k
                    R_Rest=interp1(X,R(:,k),XEnde);
                    %
                    if R_Rest<Kosten
                        Kosten=R_Rest;
                        U1Opt=Mv;                          % Verbrennungsmotormoment
                        U2Opt=Me;                          % elektrisches Moment
                        U3Opt=0;                           % Bremsmoment
                        U4Opt=Pe;                          % elektrische Leistung
                        InitialCharge=XEnde;               % Anfangsladungszustand für nächsten Punkt
                        InitialSpeed=Speed(length(Speed)); % Anfangsgeschwindigkeit für nächsten Punkt
                    end
                end
            end
            %
            if Kosten>=CC
                KOZFlag=1;
                disp('Kein optimaler Zustand gefunden')
                %pause
                U1Opt=Mgef(k)
                U2Opt=0
                U3Opt=0
                U4Opt=0
            end
            %
            MvOpt=[MvOpt;U1Opt];
            MeOpt=[MeOpt;U2Opt];
            MbOpt=[MbOpt;U3Opt];
            PeOpt=[PeOpt;U4Opt];
            %
            LadungAnfang=InitialCharge;             % Anfangsladungszustand für nächsten Punkt
            %GeschAnfang=InitialSpeed               % Anfangsgeschwindigkeit für nächsten Punkt
            %VerbrauchAnfang=InitialVerbrauch;       % Anfangsverbrauch
            %DurchInvsWirkAnfang=InitialInvsWirk;    % Anfangs-Durchschnittinverswirkungsgrad
            %DurchEnergieAnfang=InitialDurchEnergie; % Anfangs-Durchschnittsenergie
        else  % aktuelles Moment ist Verzögerungsmoment
            disp('Bremspunkt')
            Mv=0;
            %
            MeMax=max([-EGrenze(Drehzahl(k-1));-EGrenze(Drehzahl(k));-EGrenze(Drehzahl(k+1))]); % maximales elektrisches Bremsmoment für Enddrehzahl 
            %                          
            if Drehzahl(k)>900  % Starre Verbindung zwischen Verbrennungsmotor und Antriebsstrang
                SlipStick=1;
                if Mgef(k)<MeMax
                    Me=MeMax;
                    Mb=Mgef(k)-MeMax;
                    %
                    U2Opt=MeMax;              % optimales elektrisches Moment
                    U3Opt=Mgef(k)-MeMax;      % optimales Bremsmoment
                else
                    Me=Mgef(k);
                    Mb=0;
                    %
                    U2Opt=Mgef(k);            % optimales elektrisches Moment
                    U3Opt=0                   % optimales Bremsmoment
                end
                Pe=-Me*((Drehzahl(k-1)+Drehzahl(k))/2)/9.55;
            else                              % Kupplung trennt Verbrennungsmotor von Antriebsstrang
                SlipStick=0;
                Pe=0;                         % Keine Generation bei getrennter Kupplung
            end
            %
            U1Opt=0;                          % optimales Verbrennungsmotormoment
            U4Opt=Pe;
            %
            sim(SimName)
            %
            XEnde=Charge(length(Charge));     % Endzustand nach Simulation bei Zeitpunkt k
            LadungAnfang=XEnde;               % Anfangsladungszustand für nächsten Punkt
            %GeschAnfang=Speed(length(Speed)) % Anfangsgeschwindigkeit für nächsten Punkt
            %DurchInvsWirkAnfang=DurchInvsWirk(length(DurchInvsWirk)); % Anfangs-Durchschnittinverswirkungsgrad
            %
            MvOpt=[MvOpt;U1Opt];
            MeOpt=[MeOpt;U2Opt];
            MbOpt=[MbOpt;U3Opt];
            PeOpt=[PeOpt;U4Opt];
        end
    else % Null gefordertes Moment --> Stillstandpunkt 
        disp('Stillstandpunkt')
        SlipStick=0;
        %
        Mv=0;
        Me=0;
        Mb=0;
        Pe=0;
        %
        sim(SimName)
        %
        XEnde=Charge(length(Charge));     % Endzustand nach Simulation bei Zeitpunkt k
        LadungAnfang=XEnde;               % Anfangsladungszustand für nächsten Punkt
        %
        MvOpt=[MvOpt;0];
        MeOpt=[MeOpt;0];
        MbOpt=[MbOpt;0];
        PeOpt=[PeOpt;0];
    end
end
%
Mv=MvOpt;
Me=MeOpt;
Mb=MbOpt;
Pe=PeOpt;
Rows=size(Mv,1);
Zeit=[0:Rows-1]';
Geschwindigkeit=Geschwindigkeit(1:Rows);
Gang=Gang(1:Rows);
Drehzahl=Drehzahl(1:Rows);
Steigung=Steigung(1:Rows);
%
%save C:\MATLABR11\work\Opt_S400City1 Mv Me Mb Zeit Geschwindigkeit Gang Drehzahl
delete(WBHdl)
%
if KOZFlag==1
    disp('Punkt oder Punkte mit keinem optimalen Zustand sind vorhanden!')
end
%
hold off
plot(Zeit,Mv,'g')
hold on
grid on
plot(Zeit,Pe,'m')
plot(Zeit,Mb,'r')
xlabel('Zeit in Sekunde')
ylabel('Drehmoment Nm')
figure
plot(L)
xlabel('Zeit in Sekunde')
ylabel('Prozent Ladungszustand')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [MeMax]=EGrenze(Drehzahl)
global EAFlaechenData
%
if Drehzahl<=EAFlaechenData.MAXX  % Drehzahl innerhalb Fläche
    XKoordinate=1+ceil((Drehzahl-EAFlaechenData.MINX)/EAFlaechenData.XInterp);
    %
    if ~isempty(find(EAFlaechenData.Flaeche(:,XKoordinate)==0)) % Es gibt Nulls in Spalte
        MaxYKoordinate=min(find(EAFlaechenData.Flaeche(:,XKoordinate)==0))-2;
        MeMax=EAFlaechenData.MINY+(MaxYKoordinate*EAFlaechenData.YInterp);
    else
        MeMax=EAFlaechenData.MAXY;
    end
    %
else
    %MeMax=-1;
    MeMax=0;
    %disp('Drehzahl außerhalb Fläche');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [MvMax]=VGrenze(Drehzahl)
global fc_map_spd fc_spd_scale fc_max_trq fc_trq_scale
%
if Drehzahl >= min(30*fc_map_spd*fc_spd_scale/pi) &  Drehzahl <= max(30*fc_map_spd*fc_spd_scale/pi) % Drehzahl innerhalb Fläche
   MvMax=interp1(30*fc_map_spd*fc_spd_scale/pi,fc_max_trq*fc_trq_scale,Drehzahl)
else
   MvMax=-1;
   disp('Drehzahl außerhalb Fläche')
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%