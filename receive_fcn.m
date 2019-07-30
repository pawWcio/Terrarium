function [one, two, three, temperature, humidity, fan, time, bulb, fogger] = receive_fcn(frame_raw)
frame = cast(frame_raw,'uint8');
one=typecast(frame(2),'uint8');
two=typecast(frame(3:4),'uint16');
three=typecast(frame(5:8),'uint32');
temperature=typecast(frame(9:12),'single');
humidity=typecast(frame(13:16),'single');
fan=typecast(frame(17),'uint8');
time=typecast(frame(18),'uint8');
bulb=typecast(frame(19),'uint8');
fogger=typecast(frame(20),'uint8');

end