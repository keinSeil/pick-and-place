clc
tic
% Connect to Arduino
s = serialport("COM3", 115200); % Define arduino serialport object
pause(2);
toc

% Define three strings to send to Arduino
% vector = ["90.0", "55.7", "70"];
% vector = ["90.0", "55.7", "70.0"];
% vector = ["120", "140", "50"];


% v = [120, 140, 50];
v = [260.1974  200.5655   120]
vector = string(v)

tic
% Send strings to Arduino
for i = 1:length(vector)
    write(s, vector(i), "string")
    write(s, '\n', "char") % Send a newline character to indicate the end of the string
    pause(.5);
end
toc 

%!!! Reading these strings is how I confirmed that the correct values were
%send to arduino!!!

tic
% Read strings from Arduino
receivedStrings = strings(1, length(vector));
for i = 1:length(vector)
    receivedStrings(i) = read(s, strlength(vector(i)), "string");
    read(s, 2, "char"); % Read the newline character to clear it from the buffer
end
toc

disp("Received Strings:")
disp(receivedStrings);

% Close the serial connection
delete(s);
clear s;

str2double(receivedStrings)
