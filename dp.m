%global VerbrauchFLpfad CO2FLpfad COFLpfad HCFLpfad NOxFLpfad EFlaechenpfad RKMatrizenpfad
%global Trajektoriepfad Fahrzeugpfad DPTrajektoriepfad GMFlaechenpfad Mb Me Mv Zeit ELF
%global GeschAnfang LadungAnfang Vel VGM EGM B_Sp B_Kap ZeitSchritt GangZahl Gang Geschwindigkeit
%global VFlaechenData CO2FlaechenData COFlaechenData HCFlaechenData NOxFlaechenData GMFlaechenData EAFlaechenData EBFlaechenData
%global i_diff i_1 i_2 i_3 i_4 i_5 T_Mot r_rad J_4rad m_eff c_r g c_w A_f J_Mot rho_L
%global ess_tmp ess_soc ess_voc ess_r_dis ess_r_chg ess_module_num Steig Steigung R
%global EndZeit VerbrauchAnfang DurchInvsWirkAnfang DurchEnergieAnfang
global Mv cum_fuel SlipStick GangZahl Mb v_veh0 Pe Me Charge w_axle0 a P_Load ZeitSchritt
%
disp('Berechnung von Restkostenmatrix und optimaler Trajektorie ohne Gangwahl...')
%
EndZeit=1
%VerbrauchAnfang=0
%DurchInvsWirkAnfang=0
%DurchEnergieAnfang=0
CC=10000                   % Anfangsbelegung von Restkostenmatrix
%
StartTime=datestr(now)
%
% Workspace vorbereiten
%
% Fahrzeug laden
load_parms_offline         % Mondeodata laden
%FahrzeugFile=GetActiveItem('Listbox4');
%L=size(FahrzeugFile,2);
%FahrzeugFile=FahrzeugFile(1:L-2)
%eval(FahrzeugFile)
%
Go=0;
if Go==1
    %
    % Gütemaßfläche laden
    FlaechenData=LoadSelectedFile(GMFlaechenpfad,'Listbox3'); 
    %load (strcat(GMFlaechenpfad,'OM628gm2'))
    %FlaechenData.Flaeche(find(isnan(FlaechenData.Flaeche)))=(1e+10);
    FlaechenData.Flaeche(find(isnan(FlaechenData.Flaeche)))=C;
    GMFlaechenData=FlaechenData
    %
    % minimale Verbrennungsbetriebsgeschwindigkeit feststellen
    %Gmin=(3*pi*r_rad*VFlaechenData.MINX)/(25*i_1*i_diff);
    Gmin=0;
    %
    % Dateien für elektrische Maschine laden
    %load UQM1_FL
    FlaechenData=LoadSelectedFile(EFlaechenpfad,'Listbox2')
    FD=FlaechenData;
    Reihe=FlaechenData.Flaeche(4,:);
    Reihe(find(Reihe==0))=50;
    %
    if get(findobj(gcbf,'Tag','Radiobutton1'),'Value')==1
        for n=1:3
            %FlaechenData.Flaeche(n,:)=Reihe;
            FD.Flaeche(n,:)=Reihe;
        end
    end
    %
    %FlaechenData.Flaeche(find(isnan(FlaechenData.Flaeche)))=0;
    FD.Flaeche(find(isnan(FlaechenData.Flaeche)))=0;
    %
    EAFlaechenData=FD      % elektrische Antriebsfläche
    EBFlaechenData=FD      % elektrische Bremsfläche
    %
    %EBFlaechenData=FlaechenData      % elektrische Bremsfläche
    %DZWerte=[FlaechenData.MINX:FlaechenData.XInterp:FlaechenData.MAXX];
    %DMMWerte=[FlaechenData.MINY:FlaechenData.YInterp:FlaechenData.MAXY];
    %[XX,YY]=meshgrid(DZWerte,DMMWerte);
end
%
% Batterieparameter laden
MondeoBattData
%B_Kap=100;                          % Batterie-Kapazität in Ah
%B_Kap=str2num(get(findobj(gcbf,'Tag','EditText4'),'String')); % Batterie-Kapazität in Ah
%B_Sp=42;                            % Batterie-Spannung in V
%B_Sp=str2num(get(findobj(gcbf,'Tag','EditText9'),'String')); % Batterie-Spannung in V
%
%Battdata;                            % Batteriedata
%
% Powernetleistung laden
P_Load=1000
if 0
    if get(findobj(gcbf,'Tag','Radiobutton2'),'Value')==1  % Netzströme einbeziehen
        load LastFaktor
        SimName='Fahrzeug8a'; % elektrischer Lastfaktor-->Faktor laden
    else
        Faktor=zeros(2000,1);
        SimName='Fahrzeug8';
    end
end
%
% Zyklus laden
load MondeoZyklus                    % Vektoren laden: Zeit, Geschwindigkeit, Gang, Mgef
%load S400City1                      % Vektoren laden: Zeit, Geschwindigkeit, Gang, Mgef
%LoadSelectedFile(Trajektoriepfad,'Listbox5');
if 0
    Steigung=[];                         
    ZyklusName=GetActiveItem('Listbox5');
    L=size(ZyklusName,2);
    ZyklusName=ZyklusName(1:L-4);
    Command=['load 'ZyklusName]
    eval(Command)
    %
    %Gang(1)=1;
    Gang(find(Gang==0))=1;
    %
    %N=size(Zeit,1);
end
%
if isempty(Steigung)                % Steigungsvektor nicht vorhanden
    Steigung=zeros(size(Zeit));
end
N=max(find(Geschwindigkeit>0))+1;
%N=23
T_Zyk=Zeit(size(Zeit,1))-1;         % Zykluslänge in Sekunden
ZeitSchritt=Zeit(2)-Zeit(1);        % Zeitschrittlänge in Sekunde
%
% Zustandsgitter feststellen
% Zustandsvariable X --> Ladungszustand zwischen -5% und +5% der Nennkapazität
Xmin=-5.0;                          % untere Zustandsbeschränkungen für X
Xmax=5.0;                           % obere Zustandsbeschränkungen für X
%Xmax=str2num(get(findobj(gcbf,'Tag','EditText5'),'String'));    % obere Zustandsbeschränkungen für X
%Xmin=str2num(get(findobj(gcbf,'Tag','EditText6'),'String'));    % untere Zustandsbeschränkungen für X
ZahlZus=21;                         % Anzahl quantisierter Zustände
%ZahlZus=str2num(get(findobj(gcbf,'Tag','EditText7'),'String')); % Anzahl quantisierter Zustände
QS=(Xmax-Xmin)/(ZahlZus-1);          % Quantisierungsstufenzahl
X=[Xmin:QS:Xmax];                    % Zuständsgitter X2 definieren
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
Xf=0;                               % Endzuständ des Zyklus: Ladungszustand=0
%
% Steuervektoren festellen
% Eingangsgröße: U --> elektrisches Moment in Nm zwischen -EBFlaechenData.MAXY und EAFlaechenData.MAXY
%U2min=-EBFlaechenData.MAXY;         % untere Steuergrößenbeschränkung: maximales elektrisches Bremsmoment
%U2maGo=EAFlaechenData.MAXY;          % obere Steuergrößenbeschränkung: maximales elektrisches Antriebsmoment
Umin=0;
Umax=5000;
%Umax=str2num(get(findobj(gcbf,'Tag','EditText1'),'String')); % obere Steuergrößenbeschränkung: maximales elektrisches Antriebsmoment
%Umin=str2num(get(findobj(gcbf,'Tag','EditText2'),'String')); % untere Steuergrößenbeschränkung: maximales elektrisches Bremsmoment
ZahlU=11;                          % Anzahl quantsierter Steuergrößen: Leistungswerte 0, 500, 1000, 1500, 2000,... 5000
%ZahlU=str2num(get(findobj(gcbf,'Tag','EditText3'),'String')); % Anzahl quantsierter Steuergrößen
QSU=(Umax-Umin)/(ZahlU-1);          % Quantisierungsstufe für Eingangsgröße U
U=[Umin:QSU:Umax];                  % Steuervektor U2=Me definieren
%
% dynamische Programmierung durchführen
%
%R=(1e+10)*ones(size(X,2),N);        % Restkostenmatrix mit grosser Zahl belegen
R=CC*ones(size(X,2),N);        % Restkostenmatrix mit grosser Zahl belegen
%
% letzter Zeitschritt bearbeiten; Endgeschwindigkeit ist immer Null
k=N;
GangZahl=Gang(N);                       % letzter Gang im Zyklus
Vel=Geschwindigkeit(N);                 % letzte Geschwindigkeit
GeschAnfang=Geschwindigkeit(N-1)/3.6;   % vorletzte Geschwindigkeit
%Steig=Steigung(N);                     % letzter Steigungswinkel
letztesMoment=Mgef(N);
%
ZahlBerechnungen=3*1                    % Zahl Schleifen
Count=1;                                % Waitbar-Index
WBHdl=[];                               % Waitbar-Handle
%
Go=1;
if Go==1
    MeMax=-EGrenze(Drehzahl(N-1))       % maximales elektrisches Bremsmoment für Enddrehzahl 
    %
    for n=X1Index:X2Index               % Schleife über Zustandsgröße X bzw. Ladungszustand
        LadungAnfang=X(n);              % Anfangszustand von X bzw. Ladungszustand
        %                               % Es wird angenommen, daß Brems- und Antriebsflächen symmetrisch sind.
        if Mgef(N)<MeMax                % gefordertes Moment ist weniger (mehr negativ) als maximales negatives elektrisches Moment
            Me=MeMax;
            Mb=Mgef(N)-MeMax;
        else
            Me=Mgef(N);                 % elektrisches Moment
            Mb=0;
        end
        %
        Mv=0;                           % Verbrennungsmotormoment nicht vorhanden
        ELF=Faktor(N);
        sim(SimName);                   % Simulation mit gegebenen Zustands- und Eingangsvariablen durchführen
        %
        XEnde=Charge(length(Charge));   % Endzustände nach Simulation bei k=N
        %
        % Mayersches Gütemaß berechnen
        Kosten=0*100*(abs(XEnde-Xf));     % Vergleich mit gewünchten Endzuständen
        %
        % Lagrangesches Gütemaß
        %egmtest=EGM(length(EGM))
        %dz=Drehzahl(k)
        %MeWirkWert=interp2(XX,YY,EAFlaechenData.Flaeche,Drehzahl(k-1),-Me)
        %
        if 0
            if EGM(length(EGM))~=0 & ~isnan(EGM(length(EGM)))
                Kosten=Kosten+EGM(length(EGM)); % elektrischer Gütemaßen zu Kosten addieren
            else
                %Kosten=Kosten+(1e+10);
                Kosten=Kosten+CC;
            end
            %
            if Kosten<R(n,k)              % Eingangsgrößen verursachen niedriger Kosten
                R(n,k)=Kosten;             % neue Kosten
                U1Opt(n,k)=Mv;             % optimales Verbrennungsmotormoment
                U2Opt(n,k)=Me;             % optimales elektrisches Moment
            end
        end
        %
        %delete(WBHdl)
        %WBHdl=waitbar(Count/ZahlBerechnungen,strcat('letzter Zeitschritt...',num2str(Count)));
        %drawnow                       % Waitbar aktuellizieren
        Count=Count+1;
    end
end
%
delete(WBHdl)                       % Delete Waitbar
%
% restliche Zeitschritte bearbeiten
%
WBHdl=[];
Go=1;
if Go==1
    ZahlBerechnungen=N-2
    for k=(N-1):-1:2                 % Rückwärtsschleife über alle restliche Zeitschritte
        Vel=Geschwindigkeit(k);       % Soll-Geschwindigkeit von Punkt k
        ELF=Faktor(k);                % elektrischer Lastfaktor
        %Steig=Steigung(k);            % Steigungswinkel in Grad
        if Vel>0                      % Fahrzeug bewegt sich und steht nicht
            MeMax=1;                   % maximales Bremsmoment~=0
            GeschAnfang=Geschwindigkeit(k-1)/3.6;
            GangZahl=Gang(k);          % Gang für Zeitpunkt k
            Count=0;                   % Index für Waitbar
            OptFlag=0;                 % OptFlag=0 --> niedrigere Kosten gefunden; OptFlag=1 --> keine niedrigere Kosten gefunden
            %
            % Schleifen für Verzögerungspunkte
            if Mgef(k)<0               % Verzögerung oder Stillstandpunkt
                for n=1:size(X,2)       % Schleife über Zustandsgröße X Ladungszustand
                    LadungAnfang=X(n);   % Anfangszustand von Ladungszustand
                    %for m=1:size(U,2)   % Schleife über elektrisches Moment
                    %
                    %delete(WBHdl)
                    %WBHdl=waitbar(Count/(size(X,2)),strcat('Verzögerungspunkt...',num2str(k)));
                    %drawnow             % Waitbar aktuellizieren
                    Count=Count+1;
                    %
                    %Me=U(m);            % elektrisches Moment
                    %
                    %DZ=Drehzahl(k)
                    if k~=N
                        MeMax=max([-EGrenze(Drehzahl(k-1));-EGrenze(Drehzahl(k));-EGrenze(Drehzahl(k+1))]); % maximales (wenig negatives) elektrisches Bremsmoment für Enddrehzahl 
                    else
                        MeMax=max([-EGrenze(Drehzahl(k-1));-EGrenze(Drehzahl(k))]);
                    end
                    %
                    if Mgef(k)<MeMax
                        Me=MeMax;
                        Mb=Mgef(k)-MeMax;
                    else
                        Me=Mgef(k);
                        Mb=0;
                    end
                    %                              % Es wird angenommen, daß Brems- und Antriebsflächen symmetrisch sind.
                    %if Mgef(k)<=Me & Me<0          % gefordertes Moment ist weniger (mehr negativ) als oder gleich das maximale negatives elektrisches Moment
                    %Mb=Mgef(k)-Me;
                    %
                    %MeWirkWert=interp2(XX,YY,EAFlaechenData.Flaeche,Drehzahl(k),-Me);
                    Mv=0;                % Verbrennungsmotormoment nicht vorhanden
                    %
                    sim(SimName)     % Simulation mit gegebenen Zustands- und Eingangsvariablen durchführen
                    %
                    XEnde=Charge(length(Charge)); % Endzustand
                    %
                    % Lagrangesches Gütemaß
                    % disp('Lagrange')
                    % Ladezustandkostenfaktor: 100
                    Kosten=0*100*(abs(XEnde-Xf)); % Vergleich mit gewünchten Endzuständen
                    %egmtest=EGM(length(EGM))
                    %
                    if 0
                        if EGM(length(EGM))~=0 & ~isnan(EGM(length(EGM))) & EGM(length(EGM))>=0
                            Kosten=Kosten+EGM(length(EGM)); % elektrischer Gütemaß zu Kosten addieren
                        else
                            %testegm=EGM(length(EGM))
                            %Kosten=Kosten+(1e+10);
                            Kosten=Kosten+CC;
                        end
                    end
                    %
                    % Restkosten addieren
                    R_Rest=interp1(X,R(:,k+1),XEnde);
                    Kosten=Kosten+R_Rest;
                    %
                    if MeMax~=0              % Drehzahl innerhalb Betriebsbereich der E-Maschine
                        if Kosten<R(n,k)      % Eingangsgrößen verursachen niedriger Kosten
                            %disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!neue Kosten!!!!!!!!!!!!!!!!!!!!!!!')
                            R(n,k)=Kosten;     % neue Kosten
                            %U1Opt(n,k)=Mv;    % optimales Verbrennungsmotormoment
                            %U2Opt(n,k)=Me;    % optimales elektrisches Moment
                            %U3Opt(n,k)=Mb;    % optimales Bremsmoment
                            OptFlag=1;
                        end
                    else                     % Drehzahl ausserhalb Betriebsbereich der E-Maschine
                        R(n,k)=R(n,k+1);
                        OptFlag=1;
                    end
                    %end
                end
                %
            else % Mgefordert > 0; Schleifen über Antriebspunkte
                for n=1:size(X,2)          % Schleife über Zustandsgröße X bzw. Ladungszustand
                    LadungAnfang=X(n);      % Anfangszustand von X bzw. Ladungszustand
                    for m=1:size(U,2)       % Schleife über elektrisches Moment
                        %
                        %delete(WBHdl)
                        %WBHdl=waitbar(Count/(size(X,2)*size(U,2)),strcat('Antriebspunkt...',num2str(k)));
                        %drawnow              % Waitbar aktuellizieren
                        Count=Count+1;
                        %
                        Me=U(m);             % elektrisches Moment
                        Mv=Mgef(k)-Me;       % Verbrennungsmotormoment
                        Mb=0;                % Bremsmoment
                        %
                        %DZ=Drehzahl(k);
                        MeMax=min([EGrenze(Drehzahl(k-1));EGrenze(Drehzahl(k));EGrenze(Drehzahl(k+1))]);
                        %MeWirkWert=interp2(XX,YY,EAFlaechenData.Flaeche,Drehzahl(k),Me);
                        %
                        if Mv>0 & Mv<=GMFlaechenData.MAXY & Me<=MeMax & Me>=-MeMax
                            sim(SimName)  % Simulation mit gegebenen Zustands- und Eingangsvariablen durchführen
                            %
                            % Lagrangesches Gütemaß
                            % Ladezustandkostenfaktor: 100
                            XEnde=Charge(length(Charge));  % Endzustände nach Simulation bei Zeitpunkt k
                            Kosten=0*100*(abs(XEnde-Xf));    % Vergleich mit gewünchten Endzuständen
                            %egmtest=EGM(length(EGM))
                            %dz=Drehzahl(k)
                            %MeWirkWert=interp2(XX,YY,EAFlaechenData.Flaeche,Drehzahl(k),Me)
                            if 0
                                if EGM(length(EGM))~=0 & ~isnan(EGM(length(EGM))) & EGM(length(EGM))>=0
                                    Kosten=Kosten+(EGM(length(EGM))); % elektrischer Gütemaß zu Kosten addieren
                                else
                                    %Kosten=Kosten+(1e+10);
                                    Kosten=Kosten+CC;
                                end
                            end
                            %
                            %if VGM(length(VGM))~=0
                            %Kosten=Kosten+(VGM(length(VGM))); % Verbrennungsmotor-Gütemaß zu Kosten addieren
                            Kosten=Kosten+(cum_fuel(length(cum_fuel))); % Verbrennungsmotor-Gütemaß zu Kosten addieren
                            %else
                            %   Kosten=Kosten+(1e+10);
                            %end
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
                                R(n,k)=R(n,k+1);
                                OptFlag=1;
                            end
                        end 
                    end
                end
            end
            if OptFlag==0
                ZeitZahl=k
                Gefordertes_Moment=Mgef(k)
                EndDrehzahl=Drehzahl(k)
                EndGeschwindigkeit=Geschwindigkeit(k)
                MaxBremsMoment=MeMax
                pause
            end
            %
        else % Geschwindigkeit=0 --> Stillstandpunkt
            if 0 %get(findobj(gcbf,'Tag','Radiobutton2'),'Value')==0 % Netzstrom nicht berücksichtigen
                R(:,k)=R(:,k+1);
            else
                for n=1:size(X,2)          % Schleife über Zustandsgröße X bzw. Ladungszustand
                    LadungAnfang=X(n);      % Anfangszustand von X bzw. Ladungszustand
                    for m=1:size(U,2)       % Schleife über elektrisches Moment
                        %
                        %delete(WBHdl)
                        %WBHdl=waitbar(Count/(size(X,2)*size(U,2)),strcat('Antriebspunkt...',num2str(k)));
                        %drawnow              % Waitbar aktuellizieren
                        Count=Count+1;
                        %
                        Me=U(m);             % elektrisches Moment
                        Mv=Mgef(k)-Me;       % Verbrennungsmotormoment; Mgef=0
                        Mb=0;                % Bremsmoment
                        %
                        %DZ=Drehzahl(k);
                        MeMax=min([EGrenze(Drehzahl(k-1));EGrenze(Drehzahl(k));EGrenze(Drehzahl(k+1))]);
                        %MeWirkWert=interp2(XX,YY,EAFlaechenData.Flaeche,Drehzahl(k),Me);
                        %
                        if Mv>0 & Mv<=GMFlaechenData.MAXY & Me<=MeMax & Me>=-MeMax
                            sim(SimName)  % Simulation mit gegebenen Zustands- und Eingangsvariablen durchführen
                            %
                            % Lagrangesches Gütemaß
                            % Ladezustandkostenfaktor: 100
                            XEnde=Charge(length(Charge));  % Endzustände nach Simulation bei Zeitpunkt k
                            Kosten=0*100*(abs(XEnde-Xf));  % Vergleich mit gewünchten Endzuständen
                            %egmtest=EGM(length(EGM))
                            %dz=Drehzahl(k)
                            %MeWirkWert=interp2(XX,YY,EAFlaechenData.Flaeche,Drehzahl(k),Me)
                            if 0
                                if EGM(length(EGM))~=0 & ~isnan(EGM(length(EGM))) & EGM(length(EGM))>=0
                                    Kosten=Kosten+(EGM(length(EGM))); % elektrischer Gütemaß zu Kosten addieren
                                else
                                    %Kosten=Kosten+(1e+10);
                                    Kosten=Kosten+CC;
                                end
                            end
                            %
                            %if VGM(length(VGM))~=0
                            %Kosten=Kosten+(VGM(length(VGM))); % Verbrennungsmotor-Gütemaß zu Kosten addieren
                            Kosten=Kosten+(cum_fuel(length(cum_fuel))); % Verbrennungsmotor-Gütemaß zu Kosten addieren
                            %else
                            %   Kosten=Kosten+(1e+10);
                            %end
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
                                R(n,k)=R(n,k+1);
                                OptFlag=1;
                            end
                        end 
                    end
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
LadungAnfang=0;                % Anfangsladungszustand
InitialCharge=0;
GeschAnfang=0;
VerbrauchAnfang=0;
DurchInvsWirkAnfang=0;
DurchEnergieAnfang=0;
Vel=1;
WBHdl=[];
KOZFlag=0;                     % Flag für keinen optimalen Zustand
%
for k=2:N                      % Schleife über Beschleunigungspunkte
   %
   if 0 %k==N
      if get(findobj(gcbf,'Tag','Radiobutton2'),'Value')==0
         Fahrzeug8
      else                     % Netzströme einbeziehen
         Fahrzeug8a
      end
   end
   %
   v_veh0=Geschwindigkeit(k-1); % Anfangsgeschwindigkeit vor Beschleunigung
   M=Mgef(k)
   ELF=Faktor(k);              % elektrischer Lastfaktor
   %Steig=Steigung(k);          % Steigungswinkel in Grad
   EndZeit=k*ZeitSchritt;      % Endzeit von ZeitSchritt
   L(k)=LadungAnfang;
   %
   GangZahl=Gang(k-1);
   %
   if Geschwindigkeit(k)>0 %Geschwindigkeit(k+1)>0; Zielgeschwindigkeit ist größer als Null
      GangZahl=Gang(k-1);
      Kosten=1e+10;
      %
      if Mgef(k)>0                % Antriebsmoment
         disp('Antriebspunkt')
         %
         for m=1:size(U,2)        % Schleife über U2 bzw. elektrisches Moment
            %
            %delete(WBHdl)
            %WBHdl=waitbar(m/size(U,2),strcat('Vorwärtssimulation...',num2str(k)));
            %drawnow               % Waitbar aktuellizieren
            %
            Me=U(m);              % elektrisches Moment
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
                  InitialCharge=XEnde;               % Anfangsladungszustand für nächsten Punkt
                  InitialSpeed=Speed(length(Speed)); % Anfangsgeschwindigkeit für nächsten Punkt
                  %
                  %InitialVerbrauch=SchrittVer(length(SchrittVer));
                  %InitialInvsWirk=DurchInvsWirk(length(DurchInvsWirk));
                  %InitialDurchEnergie=DurchEnergie(length(DurchEnergie));
               end
            end
         end
         %
         %Mee=U2Opt
         %Mvv=U1Opt
         %Mbb=U3Opt
         %
         if Kosten>=1e+10
            KOZFlag=1;
            disp('Kein optimaler Zustand gefunden')
            %pause
            U1Opt=Mgef(k)
            U2Opt=0
            U3Opt=0
         end
         %
         MvOpt=[MvOpt;U1Opt];
         MeOpt=[MeOpt;U2Opt];
         MbOpt=[MbOpt;U3Opt];
         %
         LadungAnfang=InitialCharge;             % Anfangsladungszustand für nächsten Punkt
         %GeschAnfang=InitialSpeed               % Anfangsgeschwindigkeit für nächsten Punkt
         %VerbrauchAnfang=InitialVerbrauch;       % Anfangsverbrauch
         %DurchInvsWirkAnfang=InitialInvsWirk;    % Anfangs-Durchschnittinverswirkungsgrad
         %DurchEnergieAnfang=InitialDurchEnergie; % Anfangs-Durchschnittsenergie
      else  % aktuelles Moment ist Verzögerungsmoment
         disp('Bremspunkt')
         U1Opt=0                 % optimales Verbrennungsmotormoment
         Mv=0;
         %
         MeMax=max([-EGrenze(Drehzahl(k-1));-EGrenze(Drehzahl(k));-EGrenze(Drehzahl(k+1))]); % maximales elektrisches Bremsmoment für Enddrehzahl 
         %                           % Es wird angenommen, daß Brems- und Antriebsflächen symmetrisch sind.
         if Mgef(k)<MeMax            % gefordertes Moment ist weniger (mehr negativ) als maximales negatives elektrisches Moment
            Me=MeMax;
            Mb=Mgef(k)-MeMax;
            %
            U2Opt=MeMax;             % optimales elektrisches Moment
            U3Opt=Mgef(k)-MeMax;     % optimales Bremsmoment
         else
            Me=Mgef(k);              % elektrisches Moment
            Mb=0;
            %
            U2Opt=Mgef(k);           % optimales elektrisches Moment
            U3Opt=0                  % optimales Bremsmoment
         end
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
      end
   else % Null gefordertes Moment --> Stillstandpunkt 
      disp('Stillstandpunkt')
      %
      if get(findobj(gcbf,'Tag','Radiobutton2'),'Value')==1 % Netzströme berücksichtigen
         Mv=0;
         Me=0;
         Mb=0;
         %
         sim(SimName)
         %
         XEnde=Charge(length(Charge));     % Endzustand nach Simulation bei Zeitpunkt k
         LadungAnfang=XEnde;               % Anfangsladungszustand für nächsten Punkt
      end
      %
      MvOpt=[MvOpt;0];
      MeOpt=[MeOpt;0];
      MbOpt=[MbOpt;0];
   end
end
%
Mv=MvOpt;
Me=MeOpt;
Mb=MbOpt;
Rows=size(Mv,1);
Zeit=[0:Rows-1]';
Geschwindigkeit=Geschwindigkeit(1:Rows);
Gang=Gang(1:Rows);
Drehzahl=Drehzahl(1:Rows);
Steigung=Steigung(1:Rows);
%
set(gcbf,'UserData',[Mv Me Mb Zeit Geschwindigkeit Gang Drehzahl Steigung])
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
plot(Zeit,Me,'m')
plot(Zeit,Mb,'r')
xlabel('Zeit in Sekunde')
ylabel('Drehmoment Nm')
hold off
figure
plot(L)
xlabel('Zeit in Sekunde')
ylabel('Prozent Ladungszustand')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%