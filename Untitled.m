j = 1;
for i=1:length(ye)
   if(mod(ye(i,1), 0.01) == 0)
      freq(j) = ye(i,2);  
      j = j +1;
   end
    
end
freq = freq(1:10000)';
freq(10001) = 0;