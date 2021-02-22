%%
fname = 'DekaDriverOn_RightHand_ConnectFailure_candump-2021-02-22_115420.log';
% fname = 'DekaDriverOn_RightHand_ConnectSuccess_candump-2021-02-22_114603.log';
% fname = 'DekaDriverOn_LeftHand_ConnectSuccess_candump-2021-02-22_114431.log';
A = importdata(fname);


ts = cellfun(@str2double,regexp(A,'\(([\d+\.]+)\)','tokens','once'));

sync = ~cellfun(@isempty,regexp(A,'080#','once'));
m210 = ~cellfun(@isempty,regexp(A,'210#','once'));
m211 = ~cellfun(@isempty,regexp(A,'211#','once'));
m212 = ~cellfun(@isempty,regexp(A,'212#','once'));
m213 = ~cellfun(@isempty,regexp(A,'213#','once'));
m4AA = ~cellfun(@isempty,regexp(A,'4AA#','once'));

sync_idx = find(sync);

Sync = [sync_idx(1:end-1),sync_idx(2:end)];

for k=1:size(Sync,1)
    start_idx = Sync(k,1);
    end_idx = Sync(k,2);
    
    Sync(k,3) = sum(m210(start_idx:end_idx));
    Sync(k,4) = sum(m211(start_idx:end_idx));
    Sync(k,5) = sum(m212(start_idx:end_idx));
    Sync(k,6) = sum(m213(start_idx:end_idx));
end

% figure;
% plot(diff(ts(sync)))
% title(find(diff(ts(sync))>0.011))

figure;
plot(diff(ts(m4AA)))
% title(find(diff(ts(m240))>0.011))