function GenerateRandomObs(n)
XX = 1000;
YY = 1000;

figure;
hold on;
axis equal;
axis([0 XX 0 YY]);

forbiddenArea = {};

while length(forbiddenArea) < n
    
    while (true)
        % Rectangle
        xc = rand*XX;
        yc = rand*YY;
        w = rand*300 + 100;
        h = rand*300 + 100;
        corners = [xc-w/2, yc-h/2,  xc+w/2, yc+h/2];
        isOk = true;
        for b=1:length(forbiddenArea)
            if ~( ((corners(1)<forbiddenArea{b}(1) && corners(3)<forbiddenArea{b}(1)) || ...
                    (corners(1)>forbiddenArea{b}(3) && corners(3)>forbiddenArea{b}(3)) || ...
                    (corners(2)<forbiddenArea{b}(2) && corners(4)<forbiddenArea{b}(2)) || ...
                    (corners(2)>forbiddenArea{b}(4) && corners(4)>forbiddenArea{b}(4))) ...
                    && ...
                    (all(corners>1) && all(corners<([XX YY XX YY]-1))) )
                isOk = false;
                break;
            end
        end
        if (isOk)
            patch([corners(1), corners(1), corners(3), corners(3)], [corners(2), corners(4), corners(4), corners(2)], [1 0 0]);
            disp(sprintf('Rectangle: %f %f   %f %f', corners(1), corners(2), corners(3), corners(4)));
            disp(sprintf('HomotopyCriticalPt: %f %f', xc, yc));
            forbiddenArea{end+1} = corners;
            break;
        end
    end
    
    while (true)
        % Circle
        xc = rand*XX;
        yc = rand*YY;
        r = rand*100 + 100;
        corners = [xc-r, yc-r,  xc+r, yc+r];
        isOk = true;
        for b=1:length(forbiddenArea)
            if ~( ((corners(1)<forbiddenArea{b}(1) && corners(3)<forbiddenArea{b}(1)) || ...
                    (corners(1)>forbiddenArea{b}(3) && corners(3)>forbiddenArea{b}(3)) || ...
                    (corners(2)<forbiddenArea{b}(2) && corners(4)<forbiddenArea{b}(2)) || ...
                    (corners(2)>forbiddenArea{b}(4) && corners(4)>forbiddenArea{b}(4))) ...
                    && ...
                    (all(corners>1) && all(corners<([XX YY XX YY]-1))) )
                isOk = false;
                break;
            end
        end
        if (isOk)
            patch([corners(1), corners(1), corners(3), corners(3)], [corners(2), corners(4), corners(4), corners(2)], [0 1 0]);
            disp(sprintf('Circle: %f %f   %f', xc, yc, r));
            disp(sprintf('HomotopyCriticalPt: %f %f', xc, yc));
            forbiddenArea{end+1} = corners;
            break;
        end
    end
    
end
