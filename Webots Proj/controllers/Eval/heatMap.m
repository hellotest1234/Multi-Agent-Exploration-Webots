close all

% Read individual CSV files
dataRand = csvread('coverageRand.csv');
dataLevy = csvread('coverageLevy.csv');
dataFBE = csvread('coverageFBE.csv');
dataSBA = csvread('coverageSBA.csv');

dataFixedRand = csvread('coverageFixedRand.csv');
dataFixedLevy = csvread('coverageFixedLevy.csv');
dataFixedFBE = csvread('coverageFixedFBE.csv');
dataFixedSBA = csvread('coverageFixedSBA.csv');
cMap = interp1(linspace(0, 1, 3), [1 0 0; 1 1 0; 0 1 0], linspace(0, 1, 256));
% Concatenate along the third dimension to create a 3D array
dataRMAP = cat(3, dataRand, dataLevy, dataFBE, dataSBA);
titleRMAP = {'Random Map Random', 'Random Map Levy', 'Random Map MMAFBE','Random Map SBA'};
dataFMAP = cat(3, dataFixedRand, dataFixedLevy, dataFixedFBE,dataFixedSBA);
titleFMAP = {'Fixed Map Random', 'Fixed Map Levy', 'Fixed Map MMAFBE','Fixed Map SBA'};

% Plot heatmaps for dataRMAP
for i = 1:size(dataRMAP, 3)
    figure(i);
    heatmap(dataRMAP(:,:,i), 'Colormap', turbo, 'ColorbarVisible', 'on');
    caxis([0, 120]);
    xlabel('X-axis');
    ylabel('Y-axis');
    title(titleRMAP{i});
end


% Plot heatmaps for dataFMAP
for i = 1:size(dataFMAP, 3)
    figure(i + size(dataRMAP, 3));  % Use unique figure numbers
    heatmap(dataFMAP(:,:,i), 'Colormap', turbo, 'ColorbarVisible', 'on');
    caxis([0, 120]);
    xlabel('X-axis');
    ylabel('Y-axis');
    annotation(gcf,'rectangle',...
    [0.128906249999999 0.667735042735042 0.0627604166666657 0.0512820512820512],...
    'LineWidth',0.01,...
    'FaceColor',[0.650980392156863 0.650980392156863 0.650980392156863]);
annotation(gcf,'rectangle',...
    [0.130208333333333 0.260683760683761 0.316145833333333 0.0544871794871795],...
    'Color',[0.650980392156863 0.650980392156863 0.650980392156863],...
    'FaceColor',[0.650980392156863 0.650980392156863 0.650980392156863]);
annotation(gcf,'rectangle',...
    [0.319010416666665 0.669871794871795 0.0630208333333332 0.255341880341879],...
    'Color',[0.650980392156863 0.650980392156863 0.650980392156863],...
    'FaceColor',[0.650980392156863 0.650980392156863 0.650980392156863]);
annotation(gcf,'rectangle',...
    [0.319010416666665 0.669871794871795 0.0630208333333332 0.255341880341879],...
    'Color',[0.650980392156863 0.650980392156863 0.650980392156863],...
    'FaceColor',[0.650980392156863 0.650980392156863 0.650980392156863]);
annotation(gcf,'rectangle',...
    [0.633854166666665 0.719017094017094 0.0630208333333334 0.204594017094016],...
    'Color',[0.650980392156863 0.650980392156863 0.650980392156863],...
    'FaceColor',[0.650980392156863 0.650980392156863 0.650980392156863]);
annotation(gcf,'rectangle',...
    [0.695833333333333 0.311965811965812 0.0630208333333334 0.306623931623932],...
    'Color',[0.650980392156863 0.650980392156863 0.650980392156863],...
    'FaceColor',[0.650980392156863 0.650980392156863 0.650980392156863]);
annotation(gcf,'rectangle',...
    [0.821354166666666 0.719551282051282 0.0627604166666659 0.0512820512820511],...
    'LineWidth',0.01,...
    'FaceColor',[0.650980392156863 0.650980392156863 0.650980392156863]);
annotation(gcf,'rectangle',...
    [0.758333333333333 0.567307692307692 0.1265625 0.0512820512820513],...
    'Color',[0.650980392156863 0.650980392156863 0.650980392156863],...
    'LineWidth',0.01,...
    'FaceColor',[0.650980392156863 0.650980392156863 0.650980392156863]);
annotation(gcf,'rectangle',...
    [0.445270833333333 0.261752136752137 0.0625416666666666 0.357905982905983],...
    'Color',[0.650980392156863 0.650980392156863 0.650980392156863],...
    'LineStyle','none',...
    'FaceColor',[0.650980392156863 0.650980392156863 0.650980392156863]);

    title(titleFMAP{i});
end
% Make all figures default fullscreen
hFigures = findobj('Type', 'figure');
for i = 1:numel(hFigures)
    set(hFigures(i), 'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
end
