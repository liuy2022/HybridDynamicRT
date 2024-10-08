function [index_row,index_col,max_val] = fun_get_index_of_the_maxmum_value_in_a_matrix(A)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
[max_each_col,vec_row] =  max(A,[],1);


[max_val,index_col] =  max(max_each_col);

index_row = vec_row(index_col);


end