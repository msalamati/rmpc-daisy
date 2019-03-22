function RMPC3(delta,eps)
    %clc
    %clear all
    close all
    format long
    addpath(genpath('/home/roccosalvia/Documents/MATLAB/YALMIP-master'))
    addpath(genpath('/home/roccosalvia/Documents/MATLAB/MATLAB_files'))
    addpath(genpath('/home/roccosalvia/Documents/MATLAB/MPT/tbxmanager'))
    %%
    %Inverted Pendulum example
    delta=.1;
    eps=.001;
    g = 9.81;
    m = .344;
    b = 0.48;
    L = 1.703;
    A=[0 1;g/L -b/(m*L^2)];
    B=[0;1/(m*L^2)];
    C=[1 0;0 1];
    D=[0;0];
    KK=place(A,B,[-.5,-.1]);
    A=A-B*KK;
    Ts = 0.1;
    W_x_bounds = [-delta,delta]*1;
    U_x_bounds = [-10,10]; % 
    N = 1; 
    Y_x_Limit_orig = [pi;pi/8]; % 
    Q = 100*eye(2); 
    R = 0.1; 
    Y_ref = [.4;0]; 
    Q_L=1;
    R_L=1;
    %eps=0.001;%tube width x .5


 
    %% Discretizing the sys
    sys = ss(A,B,C,D);
    sys_d = c2d(sys,Ts,'zoh');
    %Define Additive Uncertain Model
    E = 1*B;
    add_usys_d = add_uss(A,B,C,D,E,Ts);


    

    %%  MultiParametric Approximate Closed Loop MinMax Solution

    x_state = sdpvar(length(add_usys_d.matrices.A),1);
    norm_type = Inf;
    [sol_x_mp,ValueFunction_x,MP_SolutionOut] = MP_CL_MinMax_SDPrelax(add_usys_d, x_state, Y_ref, Y_x_Limit_orig,U_x_bounds,W_x_bounds,Q,R,N,Q_L,R_L,norm_type);
    %  Simulate in time
    flag_sim = 1;
    if(flag_sim == 1)
        x_state_init = .5*ones(length(add_usys_d.matrices.A),1); 
        time_sec = 10; % 10 secs
        [y,u_ctrl_seq,t] = simulate_Multiparametric_Approximate_ClosedLoop_MinMax(add_usys_d,sol_x_mp,x_state_init,time_sec,MP_SolutionOut.Optimizer_x,W_x_bounds,x_state,Ts);
    end
    %%  Export and Test Real-Time Controller
    ExplicitRobustMPC_obj = Assemble_ExplicitRobustMPC_obj(sol_x_mp,add_usys_d,U_x_bounds,Y_x_Limit_orig,Ts);
    %%
    %   1. validator:
    sol_x_mp{1}.Ai = cell(1,length(sol_x_mp{1}.Ai));
    sol_x_mp{1}.Bi = sol_x_mp{1}.Fi;
    sol_x_mp{1}.Ci = sol_x_mp{1}.Gi;
    Optimizer = pwf(sol_x_mp{1},x_state,'general');
    %   derive the matrices for real-time execution
    [H,K,F,G,Nc] = GetRobustMPC_Matrices(sol_x_mp{1});
    Nx = length(add_usys_d.matrices.A);
    %%
    close all
    xxx=sdpvar(size(A,1),1);
    nn=0;
    mm=0;
    Nc_acc=cumsum(Nc);
    N_nei=1;
    %reshape H, K for demonstration purposes!
    H_line=zeros(size(Nc,1),max(Nc)*size(A,1));
    H_line=zeros(size(Nc,1),max(Nc));
    e_corner=0;
    e_border=0;
    for i=1:1:size(Nc,1)
        if i~=1
            H_line(i,1:Nc(i)*size(A,1))=reshape(H(Nc_acc(i-1)+1:Nc_acc(i),:)',[1,Nc(i)*size(A,1)]);
            K_line(i,1:Nc(i))=reshape(K(Nc_acc(i-1)+1:Nc_acc(i),:)',[1,Nc(i)*1]);
        else
            H_line(i,1:Nc(i)*size(A,1))=reshape(H(1:Nc_acc(i),:)',[1,Nc(i)*size(A,1)]);
            K_line(i,1:Nc(i))=reshape(K(1:Nc_acc(i),:)',[1,Nc(i)*1]);
        end

    end
    %Finding neighbouring regions
    for i=1:1:size(Nc,1)%Loop over all the regions
        ii1=H(nn+1:nn+Nc(i),:);%H partition for region 'i'
        jj1=K(nn+1:nn+Nc(i),:);%K partition for region 'i'
        V=[];
        V=lcon2vert(ii1,jj1,[],[]);%set of all vertices of region 'i'
        
        %First compute the maximum error around vertices
        for v_ind=1:1:size(V,1) 
            corner_set(1,:)=[V(v_ind,1)+eps,V(v_ind,2)+eps];
            corner_set(2,:)=[V(v_ind,1)+eps,V(v_ind,2)-eps];
            corner_set(3,:)=[V(v_ind,1)-eps,V(v_ind,2)+eps];
            corner_set(4,:)=[V(v_ind,1)-eps,V(v_ind,2)-eps];
            
            for rest_reg_ind=1:1:size(Nc,1)
                if rest_reg_ind~=i
                    H_set_rest_reg_ind=[];
                    K_set_rest_reg_ind=[];
                    if rest_reg_ind==1
                        H_set_rest_reg_ind=H(1:Nc_acc(1),:);
                        K_set_rest_reg_ind=K(1:Nc_acc(1),1);
                    else
                        H_set_rest_reg_ind=H(Nc_acc(rest_reg_ind-1)+1:Nc_acc(rest_reg_ind),:);
                        K_set_rest_reg_ind=K(Nc_acc(rest_reg_ind-1)+1:Nc_acc(rest_reg_ind),1);
                    end
                    V_rest_reg_ind=lcon2vert(H_set_rest_reg_ind,K_set_rest_reg_ind,[],[]);
                    for v_rest_reg_ind=1:1:size(V_rest_reg_ind,1) 
                        if norm(V_rest_reg_ind(v_rest_reg_ind,:)-V(v_ind,:))<1e-5
                            for corner_ind=1:1:size(corner_set,1)
                                e_corner_ij=abs((F(i,:)-F(rest_reg_ind,:))*corner_set(corner_ind,:)'+G(i)-G(rest_reg_ind));
                                if e_corner_ij>e_corner
                                    e_corner=e_corner_ij;
                                end
                            end
                        end
                    end
                            
                end
                clear H_set_rest_reg_ind K_set_rest_reg_ind V_rest_reg_ind
            end
        end
                
        
        num_neigh(i)=0;%
        ver_ind=1;
        for j=1:1:Nc(i)%go over the all borders around region 'i'
            l=1;
            for k=1:1:size(V,1)
                if abs(ii1(j,:)*V(k,:)'-jj1(j))<1e-5
                    v1(j,l,:)=V(k,:);%the l^{th} vertex coordinate for the j^{th} border of i^{th} region
                    vv(l,:)=V(k,:);%same as above stored in 3D matrix
                    l=l+1;%
                end
            end
            for ii=1:1:size(A,1)%Compute the range for j^{th} borders for i^{th} region
                minn1(1,ii)=min(vv(:,ii));
                maxx1(1,ii)=max(vv(:,ii));
            end
            %To update range for i^{th} region
            if j==1
                minn_reg=minn1;
                maxx_reg=maxx1;
            else
                for qq=1:1:size(A,1)
                    if minn_reg(qq)>minn1(1,qq)
                        minn_reg(qq)=minn1(1,qq);
                    end
                    if maxx_reg(qq)<maxx1(1,qq)
                        maxx_reg(qq)=maxx1(1,qq);
                    end
                end
            end
            clear vv
            gg=[H(nn+j,:),K(nn+j)];%putting the equation of j^{th} border in the i^{th} region in a row

            for cc=1:1:size(Nc,1)%for all the regions except:
                if cc~=i%the i^{th} region,



                    if cc==1%initialize the counter 
                        mm=0;
                    else
                        mm=Nc_acc(cc-1);
                    end
                    %The set of ineqs corresponding to the cc^{th} region
                    ii2=H(mm+1:mm+Nc(cc),:);%H partition for region 'cc'
                    jj2=K(mm+1:mm+Nc(cc),:);%K partition for region 'cc'
                    V2=lcon2vert(ii2,jj2,[],[]);%The set of verticec corresponding to the cc^{th} region




                    for dd=1:1:Nc(cc)%for all the borders constructing cc^{th} reg

                        l2=1;
                        for k2=1:1:size(V2,1)
                            if abs(ii2(dd,:)*V2(k2,:)'-jj2(dd))<1e-5
                                vv2(l2,:)=V2(k2,:);%the l2^{th} vertex coordinate for the dd^{th} border of cc^{th} region
                                v2(k2,l2,:)=V(k,:);%the l2^{th} vertex coordinate for the dd2^{th} border of cc^{th} region
                                l2=l2+1;
                            end
                        end
                        for iii=1:1:size(A,1)%Compute the range of dd^{th} border for cc^{th} region
                            minn2(1,iii)=min(vv2(:,iii));
                            maxx2(1,iii)=max(vv2(:,iii));
                        end
                        clear vv2 



                        overlap_ind=1;%initialize overlap index (as prerequisite for border detection)at true!
                        minn_diff=minn1-minn2+1e-5;
                        maxx_diff=maxx1-maxx2+1e-5;
                        for iii=1:1:size(A,1)%Check whether overlap condition is violated or not!
                            if ((minn_diff(iii)>0 && maxx_diff(iii)>0) || (minn_diff(iii)<1e-5 && maxx_diff(iii)<1e-5)) && abs(maxx_diff(iii)+minn_diff(iii))>1e-2
                            %if sign(minn_diff(iii)*maxx_diff(iii))>0 && abs(minn_diff(iii))>1e-5  && abs(maxx_diff(iii))>1e-5
                                overlap_ind=0;
                            end
                        end




                        if cc==1
                            hh=[gg;[H(dd,:),K(dd)]];%
                            if rank(hh([1,2],:),1e-6)<2 && overlap_ind==1
                                neigh_mat(N_nei,:)=[round(i)-1,round(cc)-1,gg];

                                %vertex coordinate computation for the border between i^{th} and cc^{th} regions as
                                %intersection of a set of lines!
%                                 H_intersec(1,:)=hh(1,1:size(A,1));
%                                 H_intersec(2,:)=-hh(1,1:size(A,1));
%                                 K_intersec(1,1)=hh(1,end)+eps;
%                                 K_intersec(2,1)=-hh(1,end)+eps;
%                                 zzz=1;
%                                 for zz=1:1:2*size(A,1)
%                                     if mod(zz,2)==1
%                                         H_intersec(zz+2,zzz)=1;
%                                         K_intersec(zz+2,1)=min(maxx1(zzz),maxx2(zzz));
%                                     else
%                                         H_intersec(zz+2,zzz)=-1;
%                                         K_intersec(zz+2,1)=-max(minn1(zzz),minn2(zzz));
%                                     end
%                                     if mod(zz,2)==0
%                                         zzz=zzz+1;
%                                     end
%                                 end
%                                 ver_intersec=lcon2vert(H_intersec,K_intersec,[],[]);%The set of verticec corresponding to the border betweencc^{th} and i^{th} regions  

%                                 [M1,ID1]=max([minn1(1),minn2(1)]);
%                                 [M2,ID2]=min([maxx1(1),maxx2(1)]);
%                                 for oo=1:1:1*size(A,1)
%                                     
%                                     if ID1==1
%                                         Ver=[minn1(ID1),maxx1(ID1)];
%                                     else
%                                         Ver=[minn2(ID2),maxx2(ID2)];
%                                     end
%                                     ver_intersec(oo,:)=Ver;
%                                 end
                                    
                                H_intersec(1,:)=hh(1,1:size(A,1));
                                K_intersec(1,1)=hh(1,end);
                                zzz=1;
                                ver_ind=1;
                                for zz=1:1:2*size(A,1)
                                    H_intersec(2,:)=zeros(1,size(A,1));
                                    if mode(zz,2)==1
                                        H_intersec(2,zzz)=1;
                                        K_intersec(2,1)=min(maxx1(zzz),maxx2(zzz));
                                    else
                                        H_intersec(2,zzz)=1;
                                        K_intersec(2,1)=max(minn1(zzz),minn2(zzz));
                                    end
                                    if mod(zz,2)==0
                                        zzz=zzz+1;
                                    end
                                    ver_intersec(ver_ind,:)=lcon2vert([],[],H_intersec,K_intersec);
                                    if ver_intersec(ver_ind,1)>max(minn1(1),minn2(1))-1e-5 &&...
                                            ver_intersec(ver_ind,1)<min(maxx1(1),maxx2(1))+1e-5&&...
                                            ver_intersec(ver_ind,2)>max(minn1(2),minn2(2))-1e-5 &&...
                                            ver_intersec(ver_ind,2)<min(maxx1(2),maxx2(2))+1e-5
                                        ver_ind=ver_ind+1;
                                        for kkk=1:1:zz-1
                                            if norm(ver_intersec(ver_ind-1,:)-ver_intersec(kkk,:))<1e-5
                                                ver_intersec(ver_ind-1,:)=[];
                                                ver_ind=ver_ind-1;
                                                break
                                            end
                                        end
                                    else
                                        ver_intersec(ver_ind,:)=[];
                                    end
                                end
                                ver_intersec_line(N_nei,1:size(ver_intersec,1)*size(ver_intersec,2))=reshape(ver_intersec',[1,size(ver_intersec,1)*size(ver_intersec,2)]);%reshape the above line for demonstration!
                                %update the maximum error over borders
                                line_corner_set(1,:)=ver_intersec(1,:)+[0,eps];
                                line_corner_set(2,:)=ver_intersec(1,:)+[0,-eps];
                                line_corner_set(3,:)=ver_intersec(2,:)+[0,eps];
                                line_corner_set(4,:)=ver_intersec(2,:)+[0,-eps];
                                for line_corner_ind=1:1:size(line_corner_set,1)
                                    e_border_ij=abs((F(i,:)-F(cc,:))*line_corner_set(line_corner_ind,:)'+G(i)-G(cc));
                                    if e_border_ij>e_border
                                        e_border=e_border_ij;
                                    end
                                    
                                end
                                clear ver_intersec H_intersec K_intersec
                                N_nei=N_nei+1;%update total number of neighbours
                                num_neigh(i)=num_neigh(i)+1;%update number of neighbours for i^{th} region
                            end
                        else
                            hh=[gg;[H(dd+Nc_acc(cc-1),:),K(dd+Nc_acc(cc-1))]];%
                            if rank(hh([1,2],:),1e-6)<2 && overlap_ind==1
                                neigh_mat(N_nei,:)=[round(i)-1,round(cc)-1,gg];


                                %vertex coordinate computation
%                                 H_intersec(1,:)=hh(1,1:size(A,1));
%                                 H_intersec(2,:)=-hh(1,1:size(A,1));
%                                 K_intersec(1,1)=hh(1,end)+eps;
%                                 K_intersec(2,1)=-hh(1,end)+eps;
%                                 zzz=1;
%                                 for zz=1:1:2*size(A,1)
%                                     if mod(zz,2)==1
%                                         H_intersec(zz+2,zzz)=1;
%                                         K_intersec(zz+2,1)=min(maxx1(zzz),maxx2(zzz));
%                                     else
%                                         H_intersec(zz+2,zzz)=-1;
%                                         K_intersec(zz+2,1)=-max(minn1(zzz),minn2(zzz));
%                                     end
%                                     if mod(zz,2)==0
%                                         zzz=zzz+1;
%                                     end
%                                 end
%                                 ver_intersec=lcon2vert(H_intersec,K_intersec,[],[]);%The set of verticec corresponding to the border betweencc^{th} and i^{th} regions  


%                                 [M1,ID(1)]=max([minn1(1),minn2(1)]);
%                                 [M2,ID(2)]=min([maxx1(1),maxx2(1)]);
%                                 for oo=1:1:1*size(A,1)                                    
%                                     if ID(1)==1
%                                         Ver=[minn1(ID(1)),maxx1(ID(1))];
%                                     else
%                                         Ver=[minn2(ID(2)),maxx2(ID(2))];
%                                     end
%                                     ver_intersec(oo,:)=Ver;
%                                 end


                                H_intersec(1,:)=hh(1,1:size(A,1));
                                K_intersec(1,1)=hh(1,end);
                                zzz=1;
                                ver_ind=1;
                                for zz=1:1:2*size(A,1)
                                    H_intersec(2,:)=zeros(1,size(A,1));
                                    if mode(zz,2)==1
                                        H_intersec(2,zzz)=1;
                                        K_intersec(2,1)=min(maxx1(zzz),maxx2(zzz));
                                    else
                                        H_intersec(2,zzz)=1;
                                        K_intersec(2,1)=max(minn1(zzz),minn2(zzz));
                                    end
                                    if mod(zz,2)==0
                                        zzz=zzz+1;
                                    end
                                    ver_intersec(ver_ind,:)=lcon2vert([],[],H_intersec,K_intersec);
                                    if ver_intersec(ver_ind,1)>max(minn1(1),minn2(1))-1e-5 &&...
                                            ver_intersec(ver_ind,1)<min(maxx1(1),maxx2(1))+1e-5&&...
                                            ver_intersec(ver_ind,2)>max(minn1(2),minn2(2))-1e-5 &&...
                                            ver_intersec(ver_ind,2)<min(maxx1(2),maxx2(2))+1e-5
                                        ver_ind=ver_ind+1;
                                        for kkk=1:1:zz-1
                                            if norm(ver_intersec(ver_ind-1,:)-ver_intersec(kkk,:))<1e-6
                                                ver_intersec(ver_ind-1,:)=[];
                                                ver_ind=ver_ind-1;
                                                break
                                            end
                                        end
                                    else
                                        ver_intersec(ver_ind,:)=[];
                                    end
                                end
                                ver_intersec_line(N_nei,1:size(ver_intersec,1)*size(ver_intersec,2))=reshape(ver_intersec',[1,size(ver_intersec,1)*size(ver_intersec,2)]);
                                %update the maximum error over borders
                                line_corner_set(1,:)=ver_intersec(1,:)+[0,eps];
                                line_corner_set(2,:)=ver_intersec(1,:)+[0,-eps];
                                line_corner_set(3,:)=ver_intersec(2,:)+[0,eps];
                                line_corner_set(4,:)=ver_intersec(2,:)+[0,-eps];
                                for line_corner_ind=1:1:size(line_corner_set,1)
                                    e_border_ij=abs((F(i,:)-F(cc,:))*line_corner_set(line_corner_ind,:)'+G(i)-G(cc));
                                    if e_border_ij>e_border
                                        e_border=e_border_ij;
                                    end
                                    
                                end
                                clear ver_intersec H_intersec K_intersec
                                N_nei=N_nei+1;
                                num_neigh(i)=num_neigh(i)+1;
                            end
                        end
                        clear ver_intersec

                    end
                end
            end


        end
        %Finding bounds for all regions
        eee=1;
        for ee=1:1:2*size(A,1)
            if mod(ee,2)==1
                X(i,ee)=min(minn1(eee));
            else
                X(i,ee)=max(maxx1(eee));
            end
            if mod(ee,2)==0
                eee=eee+1;
            end
        end
        X(i,1:4)=[minn_reg(1),maxx_reg(1),minn_reg(2),maxx_reg(2)];%just for 2D
        nn=nn+Nc(i);            
    end 





    %plot regions only in 2D    
    if size(A,1)==2
        nnn=0;
        for i1=1:1:size(Nc,1)
            ii11=H(nnn+1:nnn+Nc(i1),:);
            jj11=K(nnn+1:nnn+Nc(i1),:);
            nnn=nnn+Nc(i1);
            plot(ii11*xxx<=jj11);
            hold on
        end
    end
    %computing line ranges
    nn=0;
    line_ind_glob=1;
    min_line=[];
    for reg=1:1:size(Nc,1)
        ineq_set_left=[];
        ineq_set_right=[];
        ineq_set_left=H(nn+1:nn+Nc(reg),:);%H partition for region 'reg'
        ineq_set_right=K(nn+1:nn+Nc(reg),:);%K partition for region 'reg'
        V_reg=[];
        V_reg=lcon2vert(ineq_set_left,ineq_set_right,[],[]);%set of all vertices of region 'reg'
        min_line=[];
        max_line=[];
        for line_ind=1:1:size(ineq_set_left,1)
            for ver_ind=1:1:size(V_reg,1)
                if abs(ineq_set_left(line_ind,:)*V_reg(ver_ind,:)'-ineq_set_right(line_ind))<1e-6
                    if size(min_line,1)<line_ind
                        for line_dim=1:1:size(A,1)
                            min_line(line_ind,line_dim)=V_reg(ver_ind,line_dim);
                            max_line(line_ind,line_dim)=V_reg(ver_ind,line_dim);
                        end
                    else
                        for line_dim=1:1:size(A,1)
                            if V_reg(ver_ind,line_dim)<min_line(line_ind,line_dim)
                                min_line(line_ind,line_dim)=V_reg(ver_ind,line_dim);
                            end
                            if V_reg(ver_ind,line_dim)>max_line(line_ind,line_dim)
                                max_line(line_ind,line_dim)=V_reg(ver_ind,line_dim);
                            end
                        end
                    end
                    
                    
                end
            end

                
            for dim=1:1:size(A,1)
                if dim==1
                    
                    LINE_RANGE(line_ind_glob,1:2+size(A,1)+1)=[ineq_set_left(line_ind,:),ineq_set_right(line_ind),min_line(line_ind,dim),max_line(line_ind,dim)];
                    pointer=4+size(A,1);
                else
                    LINE_RANGE(line_ind_glob,pointer:pointer+1)=[min_line(line_ind,dim),max_line(line_ind,dim)];
                    pointer=pointer+2;
                end
            end
            line_ind_glob=line_ind_glob+1;
        end
        
         nn=nn+Nc(reg);   
    end 
    
    %%
    %Print outputs
    clc
    printMatrix(F,'F')
    printMatrix(G,'G')
    printMatrix(H_line,'H')
    printMatrix(K_line,'K') 
    printMatrix(X,'X')
    printMatrix(neigh_mat,'neigh_mat')
    printMatrix(ver_intersec_line,'Vertices')
    printMatrix(max(e_corner,e_border),'error_ij')
    printMatrix(Nc_acc-ones(size(Nc,1),1),'region_pointer')
    printMatrix(LINE_RANGE,'LINE_RANGE')
    clear all
     
end
     



%%
%function for printing 
function printMatrix(matrix, name)
    fprintf('%s = \n\n\n',name)
    for row=1:size(matrix,1)
        fprintf(' %s \n',sprintf(' %.15f ', matrix(row,:)))
    end
    fprintf('\n')
end   

%%

