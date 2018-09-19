function guess = guess_solution(IC,FC)
global N
guess=IC;
for i = 1:N-1
    for j=1:length(IC)
        element(j)=(FC(j)-IC(j))*i/(N-1)+IC(j);
    end
    guess=[guess; element'];
end
    
end
