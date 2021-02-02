% magic numbers
M = 5000; % number of samples in time
N = 6; % number of movements
K = 520; % number of features
KUsed = 48; % Number of features used

% Build features randomly, with small bias and uniform variance
Z = normrnd( 0.1, 1, K, M );
% make some features more noisy
KNoise = unique( unidrnd( K, 10, 1 ) );
Z(KNoise,:) = normrnd( 0.1, 5, numel(KNoise), M );
% make some features have larger bias
KBias = unique( unidrnd( K, 10, 1 ) );
Z(KBias,:) = Z(KBias,:) + ...
    repmat( normrnd( 0, 5, numel(KBias), 1 ), 1, M );
% make group of 5 colinear
for k = 1:10:K
    k1 = k;
    k2 = min( K, k1+4 );
    kIndices = k1:k2;
    numK = numel( kIndices );
    mixing = NaN*ones( numK, numK );
    mixing(1,:) = normrnd( 0, 1, 1, numK );
    for m = 2:numK
        mixing(m,:) = mixing(1,:)*unifrnd( 0, 1, 1, 1 );
    end;clear m
    mixingNoise = 1E-5;
    Z( kIndices, : ) = mixing * Z( kIndices, : ) ...
        + unifrnd( -mixingNoise, mixingNoise, numK, M );
    clear k1 k2 kIndices numK mixing mixingNoise
end;clear k
% shuffle the rows of Z
[ ~, ishuffler ] = sort( normrnd( 0, 0, K, 1 ) );
Z = Z( ishuffler, : );clear shuffler ishuffler
fprintf( 'Rank of features = %d of %d possible\n', rank(Z,1e-3), K );

% Create actual beta values
beta = normrnd( 0, 1, N, K );
% push small values to zero
z =  abs(  beta(:) ) <= 0.05;beta(z) = 0;clear z
% force some other values to zero
betaZero = unique( unidrnd( numel(beta), fix(numel(beta)*0.33), 1 ) );
beta(betaZero) = 0;

% calculate X
X = beta * Z + normrnd( 0, 0.5, N, M );
% make some X values more noisy
X( 3, : ) = X( 3, : ) + normrnd( 0, 2, 1, M );

% Estimate these data
betaEst = X / Z;
Xest = betaEst * Z;
Xerror = X - Xest;

% Plot up the estimates
if( ~exist( 'fig1', 'var' ) )
    fig1 = figure;
else
    clf(fig1);
end
figure(fig1);
betaError = beta - betaEst;
subplot( 1, 3, 1 );
imagesc( abs(betaError) );
colorbar;
title( 'Absolute beta error' );
subplot( 1, 3, 2 );
imagesc( abs(betaError)./abs(beta) );
colorbar;
title( 'Absolute relative beta error' );
subplot( 1, 3, 3 );
boxplot( Xerror', 'Notch', 'on' );
title( 'Estimation error' );

% Run through original channel selector & make estimates with selected channels
if( ~exist( 'fig2', 'var' ) )
    fig2 = figure;
else
    clf(fig2);
end
figure(fig2);
chanOrginal = gramSchmDarpa_COB( X, Z, 1:N, KUsed );
chanOrginal = sort( chanOrginal );
betaEstGSOrginal = X / Z(chanOrginal,:);
betaEstGSOrginalFull = 0*beta;betaEstGSOrginalFull(:,chanOrginal) = betaEstGSOrginal;

XestGSOrginal = betaEstGSOrginalFull * Z;
XerrorGSOrginal = X - XestGSOrginal;

% Plot up the estimates
betaErrorGSOrginal = beta - betaEstGSOrginalFull;
subplot( 1, 3, 1 );
imagesc( abs(betaErrorGSOrginal) );
colorbar;
title( 'GS Orig. Absolute beta error' );
subplot( 1, 3, 2 );
imagesc( abs(betaErrorGSOrginal)./abs(betaErrorGSOrginal) );
colorbar;
title( 'GS Orig. Absolute relative beta error' );
subplot( 1, 3, 3 );
boxplot( XerrorGSOrginal', 'Notch', 'on' );
title( [ 'GS Orig. Estimation error with ' int2str(KUsed) ' features' ] );

% Run through modified channel selector & make estimates with selected channels
if( ~exist( 'fig3', 'var' ) )
    fig3 = figure;
else
    clf(fig3);
end
figure(fig3);
chanModified = gramSchmDarpa_COB_finite( X, Z, 1:N, KUsed );
chanModified = sort( chanModified );
betaEstGSModified = X / Z(chanModified,:);
betaEstGSModifiedFull = 0*beta;betaEstGSModifiedFull(:,chanModified) = betaEstGSModified;

XestGSModified = betaEstGSModifiedFull * Z;
XerrorGSModified = X - XestGSModified;

% Plot up the estimates
betaErrorGSModified = beta - betaEstGSModifiedFull;
subplot( 1, 3, 1 );
imagesc( abs(betaErrorGSModified) );
colorbar;
title( 'GS Mod. Absolute beta error' );
subplot( 1, 3, 2 );
imagesc( abs(betaErrorGSModified)./abs(betaErrorGSModified) );
colorbar;
title( 'GS Mod. Absolute relative beta error' );
subplot( 1, 3, 3 );
boxplot( XerrorGSModified', 'Notch', 'on' );
title( [ 'GS Orig. Estimation error with ' int2str(KUsed) ' features' ] );

% Compare results
if( ~exist( 'fig4', 'var' ) )
    fig4 = figure;
else
    clf(fig4);
end
figure(fig4)
for n = 1:N
    [ ~, p ] = kstest2( XerrorGSModified(n,:), XerrorGSOrginal(n,:) );
    subplot( 2, 3, n )
    cdfplot( XerrorGSModified(n,:) );hold on;cdfplot( XerrorGSOrginal(n,:) );hold off
    title( sprintf( 'DOF %d, Prob. of same = %5.3f', n, p ) );
end;clear n p h
chanDiff = setdiff( chanOrginal, chanModified );
if( isempty( chanDiff ) )
    fprintf( 'Original and modified selected same channels\n' );
else
    fprintf( 'Original and modified channels:\n' );
    for k = 1:KUsed
        if( chanOrginal(k) ~= chanModified(k) )
            fprintf( '\t%d\t%d\n', chanOrginal(k), chanModified(k) );
        end
    end;clear k
    pause
end;clear chanDiff

return




