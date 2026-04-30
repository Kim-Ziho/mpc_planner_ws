function RandomizedVals

%{
choiceVals = [2.0 4.0 6.0];

for a=1:5
    
    if rand < 0.5
        n = 1;
    else
        n = 2;
    end
    
    thisVec = choiceVals(randperm(3));
    thisVec = thisVec(1:n);
    
    disp(thisVec);
    
end
%}

disp([2.0+ran 6.0+ran]);
disp([2.0+ran 6.0+ran]);
disp([2.0+ran 6.0+ran]);
disp([4.0+ran]);
disp([4.0+ran]);



function abc = ran
abc = 2*rand - 1.0;
