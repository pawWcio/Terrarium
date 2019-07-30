function Frame = frame_fcn(RECEIVE_DATA)

buffer=length(RECEIVE_DATA);
FRAME_SIZE=14;
start_code=hex2dec('40');
end_code=hex2dec('80');

for i=1:(buffer-FRAME_SIZE)
   if( (RECEIVE_DATA(3,i) == start_code) && (RECEIVE_DATA(3,i+(FRAME_SIZE-1)) == end_code) ) 
       for j=1:FRAME_SIZE
           Frame(j) = RECEIVE_DATA(3,i + j-1);
       end
       i
       break
   end
end

end