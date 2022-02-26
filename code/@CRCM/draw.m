function draw(obj)
% Draws the chain

    % Delete any existing objects
    try
        obj.plotDelete();
    catch
    end
    
    obj.hanbase = patch(obj.linkbase);
    for i = 1:length(obj.link)
        obj.han{i} = patch(obj.link{i});
    end
end