function [iHT] = invHT(HT)

R_Transpose = transpose(HT(1:3, 1:3));
t = HT(1:3,4);

iHT=eye(4);
iHT(1:3,1:3) = R_Transpose;
iHT(1:3,4) = -R_Transpose*t;

end