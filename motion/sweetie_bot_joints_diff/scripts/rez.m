A=dlmread("A.out", " ");
B=dlmread("B.out", " ");


plot(B(:,1), B(:,3), "*-;input;", A(:,1), A(:,3), "*-;output;")

figure;
plot(B(:,1), B(:,4), "*-;input;", A(:,1), A(:,4), "*-;output;")

figure;
plot(B(:,1), B(:,5), "*-;d input;", A(:,1), A(:,5), "*-;d output;")

figure;
plot(B(:,1), B(:,6), "*-;d input;", A(:,1), A(:,6), "*-;d output;")

figure;

plot(B(:,1), [0; diff(B(:,5))]/0.0224, "*-;d2 input;", A(:,1), A(:,7), "*-;d2 output;")

figure;

plot(B(:,1), [0; diff(B(:,6))]/0.0224, "*-;d2 input;", A(:,1), A(:,8), "*-;d2 output;")

