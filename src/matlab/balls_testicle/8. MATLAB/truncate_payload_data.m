function [time, pitch] = truncate_payload_data(data)

% pitch = data.Pitch(7848:(7848 + 31));
roll = data.Roll(7848:(7848 + 31));
% yaw = data.Yaw(7848:(7848 + 31));

% pitch = fillmissing(pitch, "linear");
pitch = fillmissing(roll, "linear");
% yaw = fillmissing(yaw, "linear");

time = linspace(0, 18, length(pitch));
end

