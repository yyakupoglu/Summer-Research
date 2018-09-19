function guess = guess_solution(IC,FC)
global N simTime
guess=IC;
dec= length(IC);
element=[];
for i = 1:N-1
    for j=1:length(IC)-1
        element(j)=(FC(j)-IC(j))*i/(N-1)+IC(j);
    end
    element = [element, simTime/(N-1)];
    guess = [guess; element'];
    element =[];
end
guess(end) = 0;
%show_robot(guess,dec);
end
