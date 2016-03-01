% Vorwärtssimulation durchführen
clear L Ch Ver Mode NOTS
LadungAnfang=0;
PeOpt=[];
L=[];
NOTS=[];
Ch=zeros(N,1);
Ver=zeros(N,1);
Mode=zeros(N,1);
Flag=0;
%
for k=1:N-1                                        % Schleife über Zeitschritten
    OptFlag=0;
    T0=k-1                                         % Zeitpunkt beim Anfang von Schritt k
    L=[L;LadungAnfang];                            % relativer Ladezustand
    if Speed(k)>0                                  % Fahrzeug bewegt sich bei Zeitpunkt k
        if Clutch(k)>0.99                          % Kupplung starr
            if Mb(k)==0 & Mp(k)>0                  % kein Bremsmoment
                Mode(k)=1;
                OptKosten=100*CC*CC;
                %%%%%%%%%%%%%%%%%%%%%% Schleife über Antriebspunkte
                if T0>905 & T0<960
                    Pel=1700;
                    Mv=Mp(k);
                    sim('Fahrzeug')
                    OptFlag=1;
                    Uopt=Pel;
                    InitialCharge=Charge(length(Charge));
                    OptKosten=Kosten;
                    Ch(k)=Charge(length(Charge));
                    Ver(k)=Verbrauch(length(Verbrauch));
                else
                    if T0>1035 & T0<1130
                        Pel=0;
                        Mv=Mp(k);
                        sim('Fahrzeug')
                        OptFlag=1;
                        Uopt=Pel;
                        InitialCharge=Charge(length(Charge));
                        OptKosten=Kosten;
                        Ch(k)=Charge(length(Charge));
                        Ver(k)=Verbrauch(length(Verbrauch));
                    else
                       
                        for Pel=0:USchritt*DZ(k)/9.55:Umax % Schleife über elektische Leistung
                            Mv=Mp(k);
                            sim('Fahrzeug')
                            Kosten=(Klagrange*Verbrauch(length(Verbrauch)))+(Kmayer*interp1(X,R(:,k+1),Charge(length(Charge))));
                            %
                            if Kosten<OptKosten
                                OptFlag=1;
                                Uopt=Pel;
                                InitialCharge=Charge(length(Charge));
                                OptKosten=Kosten;
                                Ch(k)=Charge(length(Charge));
                                Ver(k)=Verbrauch(length(Verbrauch));
                            end
                        end
                    end
                end
            else %%%%%%%%%%%%%%%%%%%%% Schleife über Bremspunkte mit starrer Kupplung (Regen): Mb > 0
                Pel=Pload;
                Mv=Mp(k);
                sim('Fahrzeug')
                OptFlag=1;
                Uopt=Pload;
                InitialCharge=Charge(length(Charge));
                Mode(k)=2;
            end % Mb-Test
            %
        else %%%%%%%%%%%%%%%%%%%%% Kupplung offen: Clutch < 0.99
            Pel=Pload;
            Mv=Mp(k);
            sim('Fahrzeug')
            OptFlag=1;
            Uopt=Pload;
            InitialCharge=Charge(length(Charge));
            Mode(k)=3;
        end % Clutch-Test    
    else %%%%%%%%%%%%%%%%%%%%% Fahrzeug steht
        Pel=0;
        Mv=Mp(k);
        sim('Fahrzeug')
        OptFlag=1;
        Uopt=0;
        InitialCharge=Charge(length(Charge));
        Mode(k)=4;
    end % SPeled-Test
    %
    if OptFlag==0
        disp(' ')
        disp('No optimum found')
        Time_Step=k
        Flag=1
        NOTS=[NOTS;Time_Step]
    end
    %
    OptFlag=0;                                 % Reset OptFlag
    PeOpt=[PeOpt;Uopt];
    LadungAnfang=InitialCharge;
end % Schleife über Zeitschritte
%L(N)=0;
%
if Flag==1
    disp('End reached; no optimum found.')
    disp('Time steps:')
    NOTS
end
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
%
DPErgebnis.L=L;
DPErgebnis.PeOpt=PeOpt;
DPErgebnis.Flag=Flag;
%
Command=['save ' strcat(strcat(DPErgebnispfad,TestName),' DPErgebnis')]
%
try
    eval(Command);
catch
    disp('Save not possible.')
end
%
figure;plot([0:(length(PeOpt)-1)],PeOpt);grid on;xlabel('Time in Seconds');ylabel('Pe in Watts');title('Optimal Power Trajectory')
Command=['print -djpeg ' strcat(strcat(DPErgebnispfad,TestName),'_P_Opt')]
eval(Command)
figure;plot([0:(length(L)-1)],L);grid on;xlabel('Time in Seconds');ylabel('Delta SOC');title('Optimal SOC Trajectory')
Command=['print -djpeg ' strcat(strcat(DPErgebnispfad,TestName),'_SOC_Opt')]
eval(Command)
figure;plot([0:(length(Mode)-1)],Mode);grid on; ylabel('Mode')
figure;plot([0:(length(Ver)-1)],Ver);grid on; ylabel('Verbrauch')
figure;plot([0:(length(Ch)-1)],Ch);grid on; ylabel('Ch')