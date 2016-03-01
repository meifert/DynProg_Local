function [Rneu]=RBerech(P,M,SOCVektor,k,Klagrange,Kmayer,E,RKM)
global Pel Mv Charge Verbrauch LadungAnfang
Rneu=RKM;
Pel=P;
Mv=M
LadungAnfang=0;
sim('Fahrzeug')
%Kosten=(Klagrange*Verbrauch(length(Verbrauch)))+(Kmayer*interp1(X,R(:,k+1),Charge(length(Charge))))^E;
Rneu(:,k)=(Klagrange*Verbrauch(length(Verbrauch)))+(Kmayer*interp1(SOCVektor,RKM(:,k+1),SOCVektor+Charge(length(Charge)))).^E;