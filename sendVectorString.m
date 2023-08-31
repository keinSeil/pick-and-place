function response = sendVectorString(device, vector)
    % Convert the vector to a string, separated by commas
    vector_str = sprintf('%u %u %u \n', vector(1), vector(2), vector(3))
    
    % Send the vector string to Arduino
    write(device, vector_str, 'string')
    pause(2);    

    % % Read the response from the Arduino
    % if device.NumBytesAvailable > 0
    %     response = read(device, 3, 'uint16_t');
    % end

    % Read the response from the Arduino
    response_data = '';
    while device.NumBytesAvailable > 0
        c = read(device, 1, 'char');
        if c ~= newline
            response_data = [response_data, c];
        else
            break;
        end
    end

    % Parse the received response_data string
    response = sscanf(response_data, '%u,%u,%u');

    disp("Hans' Response:")
    disp(response)

end

