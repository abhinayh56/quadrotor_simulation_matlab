function rpm = pwV_to_rpm(pw,V)

a2   = 0.039050514422000;
b2   = 3.513433765980000;
a1b1 = -0.711422633380000;
a1   = 11.499308216548000;
b1   = -72.304474753421999;
c0   = 1450.850740639114;

A = a2;
B = a1b1*V + a1;
C = b2*V^2 + b1*V + c0 - pw;

D = B^2 - 4*A*C;

if(D<0)
    rps = 0;
    rpm = rps*60;
end

if(D==0)
    rps = (-B)/(2*A);
    rpm = rps*60;
end

if(D>0)
    rps = (-B + sqrt(D))/(2*A);
    rpm = rps*60;
end

end

