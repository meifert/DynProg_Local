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
                
                 Pel=Pload; % Schleife über elektische Leistung
                    Mv=Mp(k);
                    sim('Fahrzeug')
                    %
                    
                        OptFlag=1;
                        Uopt=Pel;
                        InitialCharge=Charge(length(Charge));
                        OptKosten=Kosten;
                        Ch(k)=Charge(length(Charge));
                        Ver(k)=Verbrauch(length(Verbrauch));
                
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
        Pel=Pload;
            Mv=Mp(k);
            sim('Fahrzeug')
            %
                OptFlag=1;
                Uopt=Pel;
                InitialCharge=Charge(length(Charge));
                OptKosten=Kosten;
                Ch(k)=Charge(length(Charge));
                Ver(k)=Verbrauch(length(Verbrauch));
            
        Mode(k)=4;
    end % Speed-Test
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
KraftstoffVerbrauch=sum(Ver)
beep