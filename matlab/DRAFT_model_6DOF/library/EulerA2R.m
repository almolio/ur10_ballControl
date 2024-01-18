function R = EulerA2R( phi,theta,psi )

    Hz = RotZ(phi);
    Hy = RotZ(theta);
    Hx = RotX(psi);
    R = Hz*Hy*Hx;
    
end