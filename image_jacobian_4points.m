function L=image_jacobian_4points(s,Z)
  
  x=s(1); y=s(2);
  L1=[-1/Z,  0,  x/Z,  x*y,  -(1+x*x),  y;
        0,   -1/Z, y/Z, (1+y*y), -x*y, -x];
  x=s(3); y=s(4);
  L2=[-1/Z,  0,  x/Z,  x*y,  -(1+x*x),  y;
        0,   -1/Z, y/Z, (1+y*y), -x*y, -x];
  x=s(5); y=s(6);
  L3=[-1/Z,  0,  x/Z,  x*y,  -(1+x*x),  y;
        0,   -1/Z, y/Z, (1+y*y), -x*y, -x];
  x=s(7); y=s(8);
  L4=[-1/Z,  0,  x/Z,  x*y,  -(1+x*x),  y;
        0,   -1/Z, y/Z, (1+y*y), -x*y, -x];
        
  L=[L1;L2;L3;L4];
   
endfunction
