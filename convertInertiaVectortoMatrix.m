function I=convertInertiaVectortoMatrix(i)
I=[i(1) i(4) i(5);
   i(4) i(2) i(6);
   i(5) i(6) i(3)];
end
