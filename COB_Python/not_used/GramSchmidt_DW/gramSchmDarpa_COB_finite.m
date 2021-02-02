function [chans] = gramSchmDarpa_COB_finite(X, Z, mvts, maxChans)

%This function finds channels using the forward selection (gram-schmidt
%orthogonalization procedure). (modified 3/27/18 by MRB for COB Nomad compilation)

%X: Signal being estimated, NxM, N movements and M samples.
%Z: Estimators, KxM, K channels and M samples.
%mvts: A vector holding information about which of the N movements in X
%should be used
%maxChans: The maximum number of channels that should be chosen
debug = 1;


% Outputs
%chans: Cell of vectors of channels for each DOF
%loss: Cell of vectors of stopping criteria values. The best number of
%channels is the one before the minimum index.

% Defaults from JN code
% corrKineThresh = 0.3; %threshold: A minimum correlation coefficient threshold (with the kinematic) to be considered a good channel.
% windowType = 'all'; %windowType: 'singMvt' uses the window around each DOF to find the correlation and 'all' uses the entire window in X, for pre-selecting potential channels.
% possChans = corrChanSelZeroMeanDarpa_COB(X,Z,1:size(X,1),corrKineThresh,size(Z,1),windowType); % only as a prefilter...needed with neural data issue, sometimes chooses bad channels
possChans = (1:size(Z,1));
nSamples = size( X, 2 );
fullZ = Z(possChans,:);  % throw out any neural channels that would throw off the channel selection
fullZ(isnan(fullZ)) = 0; %%% Zero out an NaN data
nFeatures = size( fullZ, 1 );
fullZ = fullZ-repmat(mean(fullZ,2),1,nSamples); %%% zero mean the features
fullX = X(mvts,:); % limit to movements trained
fullX = fullX-repmat(mean(fullX,2),1,nSamples); %%% zero mean the kinematics
nDoF = size( fullX, 1 );

% Initialization for looping
mu = 0*fullX; % initialize the estimate of the kinematic residual
resid = 0*fullX;
residNorm = 0*fullX;
estX = 0*fullX;
wX = ones( nDoF, 1 );
abscorrs = zeros( nFeatures, nDoF );
selectedFeatureData = ones( 1, nSamples );
jMax = min(maxChans,numel(possChans)); % Only loop over the possible number features
maxByFeature = ones(nFeatures,1);
% actives = []; % list of the best indxs chosen
actives = zeros(1,jMax); % list of the best indxs chosen
% chans = zeros(1,min(maxChans,numel(possChans)));
newZ = fullZ; %%%  normalization of features
for k = 1:nFeatures
    newZ(k,:) = newZ(k,:)/norm(newZ(k,:),2); % normalize the new set of features
    if( any( isnan( newZ(k,:) ) ) )
        newZ(k,:) = 0;
    end
end
wZ = zeros( nFeatures, 1 );
chanStatus = ones( nFeatures, 1 ); % Flag if channel available (1) or used (0)
for j = 1:jMax
    
    % Residual calculation
    resid(:) = fullX-mu;
    for k=1:nDoF
        residNorm(k,:) = resid(k,:)/norm(resid(k,:),2);
    end
    %     SSE = resid(:)'*resid(:); %Calculate error, sum up each sq of residual
    
    % Use correlation to find "best" feature for any DoF
    abscorrs(:) = ( newZ*residNorm' ); % matrix of corr coeffs for each dof and feature
    abscorrs(:) = abs( abscorrs );
    % NaN out any feataure that has been used before, leading to ignoring
    for k=1:(j-1)
        abscorrs( actives(k), : ) = NaN;
    end
    maxByFeature(:) = max( abscorrs, [], 2, 'omitnan' ); % best correlation for each feature cross DoF
    [ ~, ind ] = max( maxByFeature, [], 'omitnan' ); % best correlation for all
    actives(j) = ind;
    
    % Figure out which features are not current best
    if( ind == 1 )
        indNot = 2:nFeatures;
    elseif( ind == nFeatures )
        indNot = 1:(nFeatures-1);
    else
        indNot = [(1:(ind-1)),((ind+1):nFeatures)];
    end
    
    % Work with best feature data
    selectedFeatureData(:) = newZ(ind,:);
    chanStatus(ind) = 0;
    
    % Add to running sum of best estimate of X by regression
    wX(:) =  resid/selectedFeatureData; % weights mapping the selected feature to the residual of the kinematics
    estX(:) = wX*selectedFeatureData; % Estimate of residual from selected feature data
    mu(:) = mu + estX; %Find new estimate (mu), which is the old plus what the current, weighted feature can add
    
    % Figure out best
    wZ(ind) = 0;
    wZ(indNot) = newZ(indNot,:)/selectedFeatureData; % project onto every other feature (forward slash may be slower than the projection); solving newZ = w x temp
    if( debug == 1)
        subplot( 1, 3, 1 );
        semilogy( abs(wZ), 'ko' );
        title( sprintf( 'Loop %d, selected channel %d', j, ind ) );
    end
    newZ(:) = newZ-wZ*selectedFeatureData; % orth every other feature to what you just did (remove that feature from every other); w is a regression so it can predict the next newZ
    newZ(ind,:) = 0;
    for k = 1:nFeatures
        if( selectedFeatureData ~= 1 )
            newZ(k,:) = newZ(k,:)/norm(newZ(k,:),2); % normalize the new set of features
            if( any( isnan( newZ(k,:) ) ) )
                newZ(k,:) = 0;
            end
        end
    end
    
    % Status checks
    if( debug == 1)
        subplot( 1, 3, 2 );
        imagesc( newZ );colorbar;
        subplot( 1, 3, 3 );
        imagesc( fullX-mu );colorbar;
        pause(0.5);
        
        fprintf( 'Number of finite rows is %d\n', ...
            numel( find( all( isfinite( newZ ), 2 ) ) ) );
        fprintf( 'Number of non-zero rows is %d\n', ...
            numel( any( ( newZ ~= 0 ), 2 ) ) );
        fprintf( 'After selecting %dth channel, rank of features is %f\n', j, rank( newZ, 1E-3 ) );
    end
end

% j = j+1; %Final stopping criteria value
% resid = fullX-mu;
% SSE = resid(:)'*resid(:);

%     [~,temp] =  min(loss{i},[],2); %Find channel with best stopping crit
%     [ind] = max(temp);
%     ind = ind-1;

chans = possChans(actives);
end
