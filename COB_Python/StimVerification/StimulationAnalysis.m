figure(1);
for index=1:32
    plot(abs(WF(index,:)));
    hold on;
    plot([50e3 * ((100-index)*10^-6)]*ones(1,2500))
    title(index)
    hold off;
    ylim=([5,0]);
    overStimulation(index,:) = abs(WF(index,:))- 50e3 * (100-index)*10^-6;
    
    pause(1);
 
end
for index = 1:32
expectedVoltages(index) = 50e3 * ((100-index)*10^-6);
end
overStimulation(overStimulation<0)=0;
maxOver = max(overStimulation,[],2);



amperageAtMaxOver = (maxOver'+expectedVoltages)/50000;
for index = 1:32
expectedAmperages(index) = ((100-index)*10^-6);
end

differenceExpectedRecorded = (amperageAtMaxOver-expectedAmperages)*1e6 %% in micro Amps

% save('stimulationDifference_1','differenceExpectedRecorded')
% save('stimulationDifference_2','differenceExpectedRecorded')
% save('stimulationDifference_3','differenceExpectedRecorded')
