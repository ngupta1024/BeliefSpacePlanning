function rand_num=rand_gen(mean,var)
    % everything here is scalar
    rand_num=mean+sqrt(var).*randn(1);
end