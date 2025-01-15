function [X,Y] = cov2elli(x,P,ns,NP)

persistent circle

if isempty(circle)
    alpha = 2*pi/NP*(0:NP);
    circle = [cos(alpha);sin(alpha)];
end

C = chol(P)';
ellip = ns*C*circle;

% output ready for plotting (X and Y line vectors)
X = x(1)+ellip(1,:);
Y = x(2)+ellip(2,:);


