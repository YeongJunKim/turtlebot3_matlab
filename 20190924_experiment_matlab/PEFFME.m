function state_hat = PEFFME(F_array,H_array,y_tilde_array,u_tilde_array,M)
    A_big = Big_A(F_array,H_array,M);
    B_big = Big_B(F_array,H_array,M);
    C_big = Big_C(F_array,M);
    
    F0 = F_function(F_array,0,1,M);
    
    L = F0 / (A_big' * A_big) * A_big';
    M = -L * B_big + C_big;
    
    state_hat = L * reshape(y_tilde_array,[],1) + M * reshape(u_tilde_array,[],1);
end

function output = Big_A(F_array,H_array,M)
    m = size(H_array(:,:,1),1);
    n = size(H_array(:,:,1),2);
    
    output = zeros(M*m,n);
    
    for i = 1:M
        output(m*(i-1)+1:m*i,:) = Gamma_function(F_array,H_array,i,1,1);
    end
end

function output = Big_B(F_array,H_array,M)
    m = size(H_array(:,:,1),1);
    n = size(H_array(:,:,1),2);
    
    output = zeros(M*m,M*n);
    
    for i = 1:M-1
        for j = 1:M-1
            row_interval = m*i+1:m*(i+1);
            col_interval = n*(j-1)+1:n*j;
            
            output(row_interval,col_interval) = Gamma_function(F_array,H_array,i,j,0);
        end
    end
end

function output = Big_C(F_array,M)
    n = length(F_array(:,:,1));
    output = zeros(n,M*n);
    
    for i = 1:M
        output(:,n*(i-1)+1:n*i) = F_function(F_array,i,1,M);
    end
end

function output = Gamma_function(F_array,H_array,a,b,c)
    if a > b
        F_tmp = F_array(:,:,a-c);
        for i = 2:a-b
            F_tmp = F_tmp * F_array(:,:,a+1-c-i);
        end
        output = H_array(:,:,a+1-c) * F_tmp;
    elseif a == b
        output = H_array(:,:,a+1-c);
    else
        output = zeros(size(H_array(:,:,1)));
    end
end

function output = F_function(F_array,a,c,M)
    n = length(F_array(:,:,1));
    output = eye(n);
    
    if a < M
        for i = 1:M-a
            output = F_array(:,:,a+1+i-c) * output;
        end
    end
end