function [T04] = fcn_T04(q,p)

T04 = zeros(4,4);

  T04(1,1)=cos(q(1))*cos(q(2))*cos(q(3)) - cos(q(1))*sin(q(2))*sin(q(3));
  T04(1,2)=- cos(q(1))*cos(q(2))*sin(q(3)) - cos(q(1))*cos(q(3))*sin(q(2));
  T04(1,3)=-sin(q(1));
  T04(1,4)=p(4)*(cos(q(1))*cos(q(2))*cos(q(3)) - cos(q(1))*sin(q(2))*sin(q(3))) + p(3)*cos(q(1))*cos(q(2));
  T04(2,1)=cos(q(2))*cos(q(3))*sin(q(1)) - sin(q(1))*sin(q(2))*sin(q(3));
  T04(2,2)=- cos(q(2))*sin(q(1))*sin(q(3)) - cos(q(3))*sin(q(1))*sin(q(2));
  T04(2,3)=cos(q(1));
  T04(2,4)=p(3)*cos(q(2))*sin(q(1)) - p(4)*(sin(q(1))*sin(q(2))*sin(q(3)) - cos(q(2))*cos(q(3))*sin(q(1)));
  T04(3,1)=- cos(q(2))*sin(q(3)) - cos(q(3))*sin(q(2));
  T04(3,2)=sin(q(2))*sin(q(3)) - cos(q(2))*cos(q(3));
  T04(3,3)=0;
  T04(3,4)=p(2) - p(4)*(cos(q(2))*sin(q(3)) + cos(q(3))*sin(q(2))) - p(3)*sin(q(2));
  T04(4,1)=0;
  T04(4,2)=0;
  T04(4,3)=0;
  T04(4,4)=1;

 