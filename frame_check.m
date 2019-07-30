function [frame] = frame_fcn(RECEIVE_DATA)

buffer=50;
FRAME_SIZE=21;
start_code=hex2dec('40');
end_code=hex2dec('80');

for i=1:(buffer-FRAME_SIZE)
   if( RECEIVE_DATA[i] == start_code && RECEIVE_DATA[i+FRAME_SIZE] == end_code ) 
      frame[i] += RECEIVE_DATA[i];
   end
end

end