function [chans] = gramSchmDarpa_COB(X, Z, mvts, maxChans)

%This function finds channels using the forward selection (gram-schmidt
%orthogonalization procedure). (modified 3/27/18 by MRB for COB Nomad compilation)

%X: Signal being estimated, NxM, N movements and M samples.
%Z: Estimators, KxM, K channels and M samples.
%mvts: A vector holding information about which of the N movements in X 
%should be used
%maxChans: The maximum number of channels that should be chosen


% Outputs
%chans: Cell of vectors of channels for each DOF
%loss: Cell of vectors of stopping criteria values. The best number of
%channels is the one before the minimum index.

% Defaults from JN code
% corrKineThresh = 0.3; %threshold: A minimum correlation coefficient threshold (with the kinematic) to be considered a good channel.
% windowType = 'all'; %windowType: 'singMvt' uses the window around each DOF to find the correlation and 'all' uses the entire window in X, for pre-selecting potential channels.
% possChans = corrChanSelZeroMeanDarpa_COB(X,Z,1:size(X,1),corrKineThresh,size(Z,1),windowType); % only as a prefilter...needed with neural data issue, sometimes chooses bad channels
possChans = (1:size(Z,1));
fullZ = Z(possChans,:);  % throw out any neural channels that would throw off the channel selection
fullX = X(mvts,:)-repmat(mean(X(mvts,:),2),1,size(X,2)); %%% zero mean the kinematics
fullZ = fullZ-repmat(mean(fullZ,2),1,size(fullZ,2)); %%% zero mean the features
fullZ(isnan(fullZ)) = 0; %%% should check other norms for nans to be thorough

mu = zeros(size(fullX)); % initialize the estimate of the kinematic residual
% actives = []; % list of the best indxs chosen
actives = zeros(1,min(maxChans,numel(possChans))); % list of the best indxs chosen
% chans = zeros(1,min(maxChans,numel(possChans)));
newZ = fullZ./repmat(sqrt(sum(fullZ.^2,2)),1,size(fullZ,2)); %%%  normalization of features
for j = 1:min(maxChans,numel(possChans))
    
    resid = fullX-mu;
    residNorm = resid./repmat(sqrt(sum(resid.^2,2)),1,size(resid,2));
    SSE = resid(:)'*resid(:); %Calculate error, sum up each sq of residual
    
    corrs = newZ*residNorm'; % matrix of corr coeffs for each dof and feature
    corrs(isnan(corrs)) = 0;
    [val, indA] = sort(abs(corrs(:)), 'descend');
    [ind1, ind2] = ind2sub(size(corrs),indA);
    
    while(~isempty(find(actives==ind1(1)))) %find new channels, not active
%         ind1 = ind1(2:end);
        ind1 = circshift(ind1,-1,1);
    end
    ind = ind1(1);
    actives(j) = ind;
    
    wX =  resid/newZ(ind,:); % weights mapping the selected feature to the residual of the kinematics
    mu = mu + wX*newZ(ind,:); %Find new estimate (mu), which is the old plus what the current, weighted feature can add
    temp = newZ(ind,:);
    wZ = newZ/temp; % project onto every other feature (forward slash may be slower than the projection); solving newZ = w x temp
    newZ = newZ-wZ*temp; % orth every other feature to what you just did (remove that feature from every other); w is a regression so it can predict the next newZ
    newZ = newZ./repmat(sqrt(sum(newZ.^2,2)),1,size(newZ,2)); % normalize the new set of features
%     fprintf( 'Number of finite rows is %d\n', ...
%         numel( find( all( isfinite( newZ ), 2 ) ) ) );
%     fprintf( 'Number of non-zero rows is %d\n', ...
%         numel( any( ( newZ ~= 0 ), 2 ) ) );
%    fprintf( 'After selecting %dth channel, rank of features is %f\n', j, rank( newZ ) );
end

% j = j+1; %Final stopping criteria value
resid = fullX-mu;
SSE = resid(:)'*resid(:);

%     [~,temp] =  min(loss{i},[],2); %Find channel with best stopping crit
%     [ind] = max(temp);
%     ind = ind-1;

chans = possChans(actives);
end
