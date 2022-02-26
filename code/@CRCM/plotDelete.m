function plotDelete(obj)
% Delete plot objects

    delete(obj.hanbase)
    for i = 1:length(obj.han)
        delete(obj.han{i});
    end
end