classdef localization < handle
   properties
       %% init
       P, Q, R, firstRun;
       f,h,jac_f,jac_h;
       %% variable area
       u;
       z;       
       x_hat;
       
       alpha;
   end
   methods
       %% function area
       function localization_init(obj, f_, h_, jac_f_, jac_h_, P_, Q_, R_)
          obj.f = f_;
          obj.h = h_;
          obj.jac_f = jac_f_;
          obj.jac_h = jac_h_;
          obj.P = P_;
          obj.Q = Q_;
          obj.R = R_;
       end
       
   end
end