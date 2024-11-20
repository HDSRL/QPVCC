function [robo] = parseData(filenames)
    % This function will parse a data file from the low-level controller.
    % Pass in the full filename to the data file and a structure will
    % be returned with the parsed data. 
    % 
    
    gearReduction = repmat([1,1,1],1,4);

    robo.raw = csvread(filenames{1});
    robo.time = robo.raw(:,1);
    robo.outputs = robo.raw(:,2:13);
    robo.torques = robo.raw(:,14:25)./gearReduction;
    robo.pos = robo.raw(:,26:43);
    robo.vel = robo.raw(:,44:61);
    robo.hd = robo.raw(:,62:73);
    robo.dhd = robo.raw(:,74:85);
    robo.ddhd = robo.raw(:,86:97);
    robo.cmdVel = robo.raw(:,98:109);
    robo.forceMPC = robo.raw(:,110:121);
    robo.doutputs = robo.raw(:,122:133);
    robo.V = robo.raw(:,134);
    robo.dV = robo.raw(:,135);
    robo.force = robo.raw(:,136:139);
    robo.QPforce = robo.raw(:,140:151);

end

