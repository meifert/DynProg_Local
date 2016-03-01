k=920
T0=919
sx=length(X)
OptFlag=0
OptKosten=100*100000;
LadungAnfang=-1.8
                %%%%%%%%%%%%%%%%%%%%%% Schleife über Antriebspunkte
                for Pel=0:USchritt*DZ(k)/9.55:Umax % Schleife über elektische Leistung
                    Power=Pel
                    Mv=Mp(k);
                    sim('Fahrzeug')
                    Kosten=(Klagrange*Verbrauch(length(Verbrauch)))+(Kmayer*interp1(X,R(:,k+1),Charge(length(Charge))))
                    V=Verbrauch(length(Verbrauch))
                    MM=Kmayer*interp1(X,R(:,k+1),Charge(length(Charge)))
                    %
                    if Kosten<OptKosten
                        OptFlag=1;
                        Uopt=Pel
                        InitialCharge=Charge(length(Charge));
                        OptKosten=Kosten;
                        Ch(k)=Charge(length(Charge));
                        Ver(k)=Verbrauch(length(Verbrauch));
                    end
                end