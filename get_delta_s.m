function delta_s = get_delta_s(s, s_nom)
    delta_s = s-s_nom;

    if delta_s(7)>pi
        delta_s(7)=delta_s(7)-2*pi;
    end
    if delta_s(8)>pi
        delta_s(8)=delta_s(8)-2*pi;
    end
    if delta_s(9)>pi
        delta_s(9)=delta_s(9)-2*pi;
    end
    if delta_s(7)<-pi
        delta_s(7)=delta_s(7)+2*pi;
    end
    if delta_s(8)<-pi
        delta_s(8)=delta_s(8)+2*pi;
    end
    if delta_s(9)<-pi
        delta_s(9)=delta_s(9)+2*pi;
    end
end