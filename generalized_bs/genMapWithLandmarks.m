function [map]=genMapWithLandmarks(sampling_mode)
    if  strcmp(sampling_mode,'grid')
        [x,y]=meshgrid([1:1:10]',[1:1:10]');
        map=[reshape(x,100,1) reshape(y,100,1)];
    elseif  strcmp(sampling_mode,'uniform_random')
        [x,y]=meshgrid(1 + (10-1)*rand(10,1),1 + (10-1)*rand(10,1));
        map=[reshape(x,100,1) reshape(y,100,1)];
    end
end
