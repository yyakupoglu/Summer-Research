function guess = guess_solution(IC,FC,knotPoints)
guess=IC;
for i = 1:knotPoints-1
    for j=1:length(IC)
        element(j)=(FC(j)-IC(j))*i/(knotPoints-1)+IC(j);
    end
    guess=[guess; element'];
end
    
end
