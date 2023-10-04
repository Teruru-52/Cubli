function independentColumns = extractIndependentColumns(matrix, n)
    [~, numColumns] = size(matrix);
    selectedIndices = [];
    
    while length(selectedIndices) < n
        % ランダムに列ベクトルを選択
        randomIndex = randi([1, numColumns]);
        
        % 既に選択済みの列ベクトルでないことを確認
        if ~ismember(randomIndex, selectedIndices)
            selectedIndices = [selectedIndices, randomIndex];
            
            % 選択済みの列ベクトルで行列を作成し，線形独立性を確認
            selectedMatrix = matrix(:, selectedIndices);
            if rank(selectedMatrix) < length(selectedIndices)
                selectedIndices = selectedIndices(1:end-1);  % 線形独立でない場合，選択を取り消す
            end
        end
    end
    
    independentColumns = matrix(:, selectedIndices);
end