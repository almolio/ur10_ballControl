function H_stack = absTransWorld_links(DH, T0_W)
    dof = size(DH, 1);
    H_stack = sym(zeros(4,4,dof + 1));
    H_stack(:,:,1) = sym(eye(4));
    H_stack(:,:,1) = T0_W;
    
    for i = 2 : dof + 1
        H_stack(:,:,i) = H_stack(:,:,i-1)*relativeTrans(DH(i-1,:));
    end
    
end