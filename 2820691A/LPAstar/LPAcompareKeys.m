function res = LPAcompareKeys(key1, key2)
%Node expansion decision
    res = true;

    if key1(1) > key2(1)
        res = false;
    elseif key1(1) == key2(1)

        if key1(2) >= key2(2)
            res = false;
        end

    end

end
