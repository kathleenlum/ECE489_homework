function [F_ext p_ee] = fcn_ExternalForce(t,X,p)
%Function returns the external force and the point of contact pc

%Robot parameters
params = p.params;

%Initialize external force
F_ext = [0; 0; 0];
p_ee = [0; 0; 0];

%Tool position
d = 2.65*0.0254;
h = 6*0.0254;
dia = 0.75*0.0254;
p4 = fcn_p4(X(1:3,1),params);
J4 = fcn_J4(X(1:3,1),params);
dp4 = J4*X(4:6,1);
T01 = fcn_T01(X(1:3,1),params);
p_ee = [p4; 0] + T01*[d; 0; -h/2; 1];
p_ee = p_ee(1:3,1);

%Rigid surface stiffnes for soft contact model
K_surf = 5000;
B_surf = K_surf/10;

%Soft contact model for hole in task 1:
if((p_ee(1)>=-0.05)&&(p_ee(1)<=0.05))
    if((p_ee(2)>=0.35)&&(p_ee(2)<=0.45))
        if(p_ee(3)<=0.1)
        Fz = K_surf*(0.1 - p_ee(3)) - B_surf*dp4(3);
        F_ext = [0; 0; Fz];
        
        r = norm([0; 0.4] - p_ee(1:2,1));
        if(r < 0.0254)
            Fxy = K_surf*([0; 0.4] - p_ee(1:2,1)) - B_surf*dp4(1:2);
            F_ext = [Fxy; 0]; 
            if(p_ee(3)<=0.0492)
                Fz = K_surf*(0.0492 - p_ee(3)) - B_surf*dp4(3);
                F_ext = F_ext + [0; 0; Fz];
            end
        end
        end
    end
end

%Soft contact model for narrow gap in task 2:
w = 1.5*0.0254; %gap width
if(p_ee(3)<=0.2)
   if((p_ee(1)>=0.35+w/2-0.1)&&(p_ee(1)<=0.35+w/2+0.1))
       if((p_ee(2)>=0)&&(p_ee(2)<=0.1))
           Fy = K_surf*(0.1 - p_ee(2)) - B_surf*dp4(2);
           F_ext = [0; Fy; 0]; 
       end
   end
   if((p_ee(1)>=0.35-w/2)&&(p_ee(1)<=0.35+w/2))
       if((p_ee(2)>=-0.1)&&(p_ee(2)<=0.1))
           Fx = K_surf*(0.35 - p_ee(1)) - B_surf*dp4(1);
           F_ext = [Fx; 0; 0];
       end
   end
end
   

%Soft contact model for buttom in task 3:
if((p_ee(1)>=-0.05)&&(p_ee(1)<=0.05))
    if((p_ee(2)>=-0.45)&&(p_ee(2)<=-0.35))
        if(p_ee(3)<=0.3)
        Fz = 1000*(0.3 - p_ee(3)) - 100*dp4(3);
        F_ext = [0; 0; Fz];
        end        
    end
end

end