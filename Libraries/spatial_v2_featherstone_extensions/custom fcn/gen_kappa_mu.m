function model = gen_kappa_mu(model)
    if(isfield(model, 'kappa'))
        model = rmfield(model, 'kappa') ;
    end
    if(isfield(model, 'mu'))
        model = rmfield(model, 'mu') ;
    end
    
    model.kappa{find(model.parent==0)} = find(model.parent==0) ;
    for i=find(model.parent ~= 0)%2:model.NB
        model.kappa{i} = [model.kappa{model.parent(i)} i] ;
    end

    model.mu{1} = [] ;
    for i=find(model.parent ~= 0)%2:model.NB
        model.mu{i} = find(model.parent == i) ;
    end
end