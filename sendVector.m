function response = sendVector(device, vector)
    % Send vector to Arduino
    write(device, vector, 'uint16')
    pause(2);

    % Read the response from the Arduino
    if device.NumBytesAvailable > 0
        numbytes = device.NumBytesAvailable;
        response = read(device, 3, 'uint16');
    end
    
    disp("Number of Available Bytes:")
    disp(numbytes)
    disp("Hans' Response:")
    disp(response)

end

