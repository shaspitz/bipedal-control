% Combines a link's inertial properties with its parents
function mod_model = combine_DOF(model, q, b_list)
    remain_idx = setdiff(1:model.NB, b_list) ;

    mod_model = model ;
    mod_model.map.fullidx = model.idx ;
    % q = Mtofull * q_new + c_tofull
    mod_model.map.tofull = eye(model.NB) ;
    mod_model.map.tofull = mod_model.map.tofull(:,remain_idx) ;
    mod_model.map.c_tofull = 0*q ;
    mod_model.map.c_tofull(b_list) = q(b_list) ;
    % q_new = Mfromfull * q
    mod_model.map.fromfull = eye(model.NB) ;
    mod_model.map.fromfull = mod_model.map.fromfull(remain_idx,:) ;

    
    % Sort b based on size of chain from root - highest first so that we
    % combine from fartherest from the base
    base_dist = zeros(model.NB, 1) ;
    for j=1:model.NB
        base_dist(j) = length(model.kappa{j}) ;
    end
    b_ordered = flipdim(sortrows([b_list base_dist(b_list)], 1), 1) ;
    b_ordered = b_ordered(:,1) ;
    

    for k=1:length(b_ordered)
        b = b_ordered(k) ;
        p = model.parent(b) ;
    
        [XJ, S] = jcalc( mod_model.jtype{b}, q(b) );
        b_X_p = XJ * mod_model.Xtree{b} ;
        p_X_b_star = b_X_p' ;

        mod_model.I{p} = mod_model.I{p} + p_X_b_star * mod_model.I{b} * b_X_p ;
        
        % Remove child node of parent corresponding to current b
        ch = mod_model.mu{p} ;
        ch(ch == b) = [] ;
        mod_model.mu{p} = ch ;

        % Add children of b to children of p
        for j=mod_model.mu{b}
            mod_model.Xtree{j} = mod_model.Xtree{j}*XJ*mod_model.Xtree{b} ;
            mod_model.parent(j) = p ;
            mod_model.mu{p} = [mod_model.mu{p} j] ;
        end

        % Effectively delete current element  [perhaps not required]
        mod_model.Xtree{b} = eye(6) ;
        q(b) = 0 ;
        mod_model.I{b} = mcI(0, zeros(3,1), zeros(3,3)) ;
    end
    
    
    % Delete stuff
    mod_model.I = {mod_model.I{remain_idx}} ;
    mod_model.jtype = {mod_model.jtype{remain_idx}} ;
    mod_model.Xtree = {mod_model.Xtree{remain_idx}} ;
    if(isfield(mod_model,'jnames'))
        mod_model.jnames = {mod_model.jnames{remain_idx}} ;
    end
    

    % Update indices
    names = model.link_names ; %fieldnames(mod_model.idx) ;
    values = model.link_id ; %cell2mat(struct2cell(mod_model.idx)) ;
    for k=1:length(b_ordered)
        b = b_ordered(k) ;
        ind = find(mod_model.parent > b) ;
        mod_model.parent(ind) = mod_model.parent(ind) - 1 ;
        values(ind) = values(ind) - 1 ;
    end
    mod_model.parent(b_ordered) = [] ;
    names = names(remain_idx) ;
    values = 1:length(remain_idx) ; % ASSUMES ORIGINAL LIST WAS ORDERED values(remain_idx) ;
    mod_model = rmfield(mod_model, 'idx') ;
    for j=1:length(names)
        eval(['mod_model.idx.' names{j} '=' num2str(values(j)) ';']) ;
    end
    mod_model.link_names = names ;
    mod_model.link_id = values ;
    

    % Regenerate children with modified indices
    mod_model.NB = mod_model.NB - length(b_list);
    mod_model = gen_kappa_mu(mod_model) ;
    
    % Update mod_model.idx

     mod_model = model_app(mod_model);
end





% 
% function mod_model = combine_DOF(model, q, b)
%     % Combines with parent
%     p = model.parent(b) ;
%     
%     [XJ, S] = jcalc( model.jtype{b}, q(b) );
%     b_X_p = XJ * model.Xtree{b} ;
%     p_X_b_star = b_X_p' ;
%     
%     mod_model = model ;
%     mod_model.I{p} = mod_model.I{p} + p_X_b_star * model.I{b} * b_X_p ;
%     
%     for j=model.mu{b}
%         mod_model.Xtree{j} = mod_model.Xtree{j}*XJ*mod_model.Xtree{b} ;
%     end
%     
%     ind = [1:(b-1), b+1:model.NB] ;
%     mod_model.I = {mod_model.I{ind}} ;
%     mod_model.jtype = {mod_model.jtype{ind}} ;
%     mod_model.Xtree = {mod_model.Xtree{ind}} ;
%     for j=model.mu{b}
%         mod_model.parent(j) = p ;
%     end
%     
%     ind = find(mod_model.parent > b) ;
%     mod_model.parent(ind) = mod_model.parent(ind) - 1 ;
%     mod_model.parent(b) = [] ;
%     
%     mod_model.NB = mod_model.NB - 1;
%     mod_model = rmfield(mod_model, 'kappa') ;
%     mod_model = rmfield(mod_model, 'mu') ;
%     mod_model = gen_kappa_mu(mod_model) ;
% end