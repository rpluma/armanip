function [csvData] = runExperiment(numExperiment)

% 1. Run the model
    modelName = 'lab2'; 
    open_system(modelName);
    simOut = sim(modelName);

% 2. Compose the values to store in the csv

    qSet = simOut.q; % Dataset
    qSig = qSet{1}; % Signal
    qVal = qSig.Values;
    qDat = qVal.Data;
    qSqz = squeeze(qDat);
    
    qdotSet = simOut.qdot; % Dataset
    qdotSig = qdotSet{1}; % Signal
    qdotVal = qdotSig.Values;
    qdotDat = qdotVal.Data;
    qdotSqz = squeeze(qdotDat);
    
    qddotSet = simOut.qddot; % Dataset
    qddotSig = qddotSet{1}; % Signal
    qddotVal = qddotSig.Values;
    qddotDat = qddotVal.Data;
    qddotSqz = squeeze(qddotDat);

% 3. Write header to csv
    csvHeader = {"q1", "q2", "q1'", "q2'", "q1''", "q2''"};
    csvFileName = sprintf('experiment%d.csv', numExperiment);
    fid = fopen(csvFileName, 'w');
    fprintf(fid, "%s,", csvHeader{1:end-1});
    fprintf(fid, "%s ", csvHeader{end});
    fclose(fid);

% 4. Write data to csv
    csvData = [qSqz' qdotSqz' qddotSqz'];
    writematrix(csvData, csvFileName, 'WriteMode', 'append');


end