clear all;
clc;
close all;

p = 0:1e-3:1;
w_of_p = @(x) w_of_prh(x,-5.,-5.);
W = w_of_p(p);
W_prime = diff(W,1,1);
W_prime_norm = W_prime ./ sqrt(sum(W_prime .* W_prime,2));
s_hat = W_prime_norm(1,:)
pitch = atan2(-s_hat(3), sqrt(s_hat(1)*s_hat(1) + s_hat(2)*s_hat(2)))
yaw = atan2(s_hat(2), s_hat(1))

w_of_p = @(x) w_of_prh(x,0.,-5.);
W = w_of_p(p);
W_prime = diff(W,1,1);
W_prime_norm = W_prime ./ sqrt(sum(W_prime .* W_prime,2));
s_hat = W_prime_norm(1,:)
pitch = atan2(-s_hat(3), sqrt(s_hat(1)*s_hat(1) + s_hat(2)*s_hat(2)))
yaw = atan2(s_hat(2), s_hat(1))
% 
% w_of_p = @(x) w_of_prh(x,0,0);
% W = w_of_p(p);
% W_prime = diff(W,1,1);
% W_prime_norm = W_prime ./ sqrt(sum(W_prime .* W_prime,2));
% s_hat = W_prime_norm(1,:);
% pitch = atan2(-s_hat(3), sqrt(s_hat(1)*s_hat(1) + s_hat(2)*s_hat(2)))
% yaw = atan2(s_hat(2), s_hat(1))
% 
% 
% w_of_p = @(x) w_of_prh(x,5,5);
% W = w_of_p(p);
% W_prime = diff(W,1,1);
% W_prime_norm = W_prime ./ sqrt(sum(W_prime .* W_prime,2));
% s_hat = W_prime_norm(end/2,:);
% pitch = atan2(-s_hat(3), sqrt(s_hat(1)*s_hat(1) + s_hat(2)*s_hat(2)))
% yaw = atan2(s_hat(2), s_hat(1))
% 
% w_of_p = @(x) w_of_prh(x,-5,5);
% W = w_of_p(p);
% W_prime = diff(W,1,1);
% W_prime_norm = W_prime ./ sqrt(sum(W_prime .* W_prime,2));
% s_hat = W_prime_norm(end,:);
% pitch = atan2(-s_hat(3), sqrt(s_hat(1)*s_hat(1) + s_hat(2)*s_hat(2)))
% yaw = atan2(s_hat(2), s_hat(1))