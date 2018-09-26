function guess = guess_solution(IC,FC)
global N simTime
guess=IC;
dec= length(IC);
element=[];
for i = 1:N-1
    for j=1:length(IC)-1
        if j == 1 || j == 2 || j ==3 || j == 7 || j == 8 || j == 9
            element(j)=(FC(j)-IC(j))*i/(N-1)+IC(j);
        else 
            element(j) = 2*rand(1,1);
        end
    end
    element = [element, simTime/(N-1)];
    guess = [guess; element'];
    element =[];
end
guess(end) = 0;
%show_robot(guess,dec);
end
