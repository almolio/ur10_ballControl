function H_stack = absTransWorld_cms(DH_cms, H_stack_links)
    dof = size(DH_cms, 1);
    H_stack = sym(zeros(4,4,dof));
    
    for i = 1 : dof
        H_stack(:,:,i) = H_stack_links(:,:,i)*relativeTrans(DH_cms(i,:));
    end
    
end  