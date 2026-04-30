function GenerateMap

MapSize = [100, 100];
TheMap = false(MapSize);

Rect{1} = [20 40; 30 80];
Rect{2} = [80 60; 70 20];
Rect{3} = [30 40; 45 60];
Rect{4} = [55 40; 70 60];

for r = 1:length(Rect)
    
    for x = Rect{r}(1,1):sign(Rect{r}(2,1)-Rect{r}(1,1)):Rect{r}(2,1)
        for y = Rect{r}(1,2):sign(Rect{r}(2,2)-Rect{r}(1,2)):Rect{r}(2,2)
            
            TheMap(x,y) = true;
            
        end
    end
    
end

figure;
imagesc(TheMap');
axis equal;
axis xy;

pause;

fid = fopen('GenerateMap.map', 'w');

for y = 1:MapSize(2)
    for x = 1:MapSize(1)
        if TheMap(x,y)
            fprintf(fid,' 255');
        else
            fprintf(fid,'   0');
        end
    end
    fprintf(fid,'\n');
end

fclose(fid);
