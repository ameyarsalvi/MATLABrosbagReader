function gap = getSimGap(w_sim,w_real,V)


omega = w_sim;
vel = V;
for i=1:length(omega)
    d_sim(i) = vel/omega(i);
end

clear omega

omega = w_real;
vel = V;
for i=1:length(omega)
    d_real(i) = vel/omega(i);
end

error_d = d_sim - d_real;
mean_error = mean(error_d);
pd_error = fitdist(error_d','normal');
pdf_gap = pdf(pd_error,error_d');

gap.x = error_d;
gap.y = pdf_gap;
gap.mean = mean_error;
gap.dist = error_d;

end