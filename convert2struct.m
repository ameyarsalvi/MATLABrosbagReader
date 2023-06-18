function [str] = convert2struct(cellArray)
%CONVERT2STRUCT convert arrays of rosbag structures to a structure of
%   arrays
%   Detailed explanation goes here

if ~isstruct(cellArray{1})
    error('Input must be a cell array of structures');
end

% create helper functions to extract info from cell array of structures
extractScalar = @(str,field)(cellfun(@(x)(x.(field)),str));
extractChar = @(str,field)(cellfun(@(x)(string(x.(field))),str));
extractCell = @(str,field)(cellfun(@(x)({x.(field)}),str));

% copy over the fields as arrays (or as an array of cells)
fieldNames = fieldnames(cellArray{1});
for i=1:length(fieldNames)
    if ischar(cellArray{1}.(fieldNames{i}))
        str.(fieldNames{i}) = extractChar(cellArray,fieldNames{i});
    elseif isstruct(cellArray{1}.(fieldNames{i}))
        % structures -- recursively decompose further
        str.(fieldNames{i}) = convert2struct(extractCell(cellArray,fieldNames{i}));
    elseif length(cellArray{1}.(fieldNames{i})) == 1
        % scalar variable (i.e., array of lenght 1)
        str.(fieldNames{i}) = extractScalar(cellArray,fieldNames{i});
    else
        % matrices -- stick store as cell
        str.(fieldNames{i}) = extractCell(cellArray,fieldNames{i});
    end
end
end

