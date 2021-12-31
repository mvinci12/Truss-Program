load('RealTruss1_MarkDeaFelix_A3.mat');

%making matrix Ac
Lengths = CalcLength(X,Y,C);
A = MakeMatrixA(C,X,Y,Lengths,Sx,Sy);

%solving for Tensions
Ainv = inv(A);
T = Ainv*L;

%meters to inches
%Lengths = Lengths*39.37;

%Pcrit / Buckling Strength
for i = 1:length(Lengths)
    Pcrit(i) = 2990/(Lengths(i)^2);
end

%creating SR
for i = 1:length(Lengths)
    SR(i) = -(T(i)/(2990/(Lengths(i))^2));
end

load = sum(L);

Wfail = load / (max(SR));              %max theoretical load
maximumSR = max(SR); %maxSR

%which number fails
memfail = find(SR==maximumSR);      %which member fails

%total cost
[r c] = size(C);
Cost = (10*r) + (1*sum(Lengths));

%load to cost ratio
loadtocost = Wfail / Cost;

%lets print
fprintf('EK301, Section A3, Group The Buckling Bucklers: Mark V, Dea T, Felix F,11/23/2020 \n\n'); 
fprintf('Load: %.2f oz \n\n',max(L));
fprintf('Member forces in Ounces for LIVE LOAD \n');

%prints members and their tensions/compressions
for i = 1:c
    if T(i) < 0
        word = "(C)";
    elseif T(i) > 0
        word = "(T)";
    elseif T(i) == 0
        word = "";
    end
    
    fprintf('m%d: %.3f %s \n',i,abs(T(i)),word);

end

fprintf('\n');

%print reaction forces
fprintf('Reaction forces in Ounces for LIVE LOAD: \n');
[rx cx] = size(Sx);
[ry cy] = size(Sy);

countx = 0;
for i = 1:rx
    for j = 1:cx
        if Sx(i,j) == 1
            countx = countx + 1;
        end
    end
end

county = 0;
for i = 1:ry
    for j = 1:cy
        if Sy(i,j) == 1
            county = county + 1;
        end
    end
end

%prints Sx and Sy

for i = 1:countx
    fprintf('Sx%d: %.2f \n',i,T(length(Lengths)+i));
end

for j = 1:county
    fprintf('Sy%d: %.2f \n',j,T(length(Lengths)+i+j));
end


fprintf('\nCost of truss: $%.2f\n',Cost);
fprintf('Theoretical max load/cost ratio in oz/$: %.4f\n\n\n',loadtocost);


%PRINTING OUR CM
fprintf('Our Cm for our members \n');



%prints members and their Cms
for i = 1:c
    
    fprintf('m%d: %.3f \n',i,abs(T(i))/max(L));

end

fprintf('\n\n');


%%PRINTING WDEAD
%solving for Tensions
Ainv = inv(A);
Tdead = Ainv*Ldead;

fprintf('Member forces in Ounces for the DEAD LOAD \n');

%prints members and their tensions/compressions
for i = 1:c
    if Tdead(i) < 0
        word = "(C)";
    elseif Tdead(i) > 0
        word = "(T)";
    elseif Tdead(i) == 0
        word = "";
    end
    
    fprintf('m%d: %.3f %s \n',i,abs(Tdead(i)),word);

end

fprintf('\n');

%print reaction forces
fprintf('Reaction forces in Ounces for DEAD LOAD: \n');
[rx cx] = size(Sx);
[ry cy] = size(Sy);

countx = 0;
for i = 1:rx
    for j = 1:cx
        if Sx(i,j) == 1
            countx = countx + 1;
        end
    end
end

county = 0;
for i = 1:ry
    for j = 1:cy
        if Sy(i,j) == 1
            county = county + 1;
        end
    end
end

%prints Sx and Sy

for i = 1:countx
    fprintf('Sx%d: %.2f \n',i,Tdead(length(Lengths)+i));
end

for j = 1:county
    fprintf('Sy%d: %.2f \n',j,Tdead(length(Lengths)+i+j));
end

%REAL LOAD DATA

Treal = T + Tdead;

%creating SR
for i = 1:length(Lengths)
    SRnom(i) = -(Treal(i)/(2990/(Lengths(i))^2));
    SRweak(i) = -(Treal(i)/(2990/(Lengths(i))^2 - 20));
    SRstrong(i) = -(Treal(i)/(2990/(Lengths(i))^2 + 20));
end

Lreal = L + Ldead;
loadreal = sum(Lreal);

Wfailnom = loadreal / (max(SRnom));%max theoretical load
Wfailweak =  loadreal / (max(SRweak));
Wfailstrong =  loadreal / (max(SRstrong));
maximumSRreal = max(SRnom); %maxSR

%which number fails
memfailreal = find(SRnom==maximumSRreal);      %which member fails

%Printing our Wnom, Wstrong, Wweak
fprintf('\nWlNom: %.2foz\n',Wfailnom);
fprintf('Wlweak: %.2foz\n',Wfailweak);
fprintf('Wlstrong: %.2foz\n',Wfailstrong);



%FUNCTIONS
function Lengths = CalcLength(X,Y,C)
    
    [~,members] = size(C);
    Lengths = zeros(members,1);
    
    for i = 1:members
        points = find(C(:,i));
        p1 = points(1);
        p2 = points(2);
        
        deltaX = X(p2) - X(p1);
        deltaY = Y(p2) - Y(p1);
        
        Lengths(i) = sqrt(power(deltaX,2) + power(deltaY,2));
    end
end

function A = MakeMatrixA(C, X, Y, Lengths,Sx,Sy)
    
[joints,members] = size(C);
Ax = zeros(joints,members);
Ay = zeros(joints,members);

for i = 1:members
    points = find(C(:,i));
    p1 = points(1);
    p2 = points(2);
    length = Lengths(i);
    
    Ax(p1,i) = (X(p2)-X(p1))/length;
    Ax(p2,i) = (X(p1)-X(p2))/length;
    Ay(p1,i) = (Y(p2)-Y(p1))/length;
    Ay(p2,i) = (Y(p1)-Y(p2))/length;
    
end

A = [Ax Sx; Ay Sy];
end












