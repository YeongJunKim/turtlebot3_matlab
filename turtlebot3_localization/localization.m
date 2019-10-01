classdef localization < handle
   properties 
       %% variable area
       f,h,jac_f,jac_h;
   end
   methods
       %% function area
       function localization_init(f_,h_,jac_f_,jac_h_)
          f = f_;
          h = h_;
          jac_f = jac_f_;
          jac_h = jac_h_;
       end
       
   end
end